/*
  sink_rnr.c - slice sink driving an RNR Eco Motion 2.0 controller

  The simulator thread pushes slices into a small bounded queue. A dedicated
  I/O thread owns the transport: it reads status reports continuously, sends
  slices paced by the PC clock and runs the watchdog.

  Pacing: the controller acknowledges a slice as soon as it has buffered it,
  not when it has executed it, and it can buffer several seconds (observed by
  BreakoutCNC). Sending on acknowledgements alone lets the controller run
  seconds behind, so feed holds and stops would act late. Slices are therefore
  sent in real time, at most RNR_LEAD slices ahead of the PC clock; the
  acknowledgement count is only a safety limit.

  Frame sequence, as bench-tested by BreakoutCNC (to be checked against Mach3
  USB captures):
    connect : Configure, EStop, SetOutputs(0), SetPositions(0)
    arm     : PreReset, ClearEStop, Configure (sent before the first motion)
    run     : StartJog(X), JogSlice..., EndOfMotion(outputs) once all slices ran
    stop    : EStop (drops the controller's buffer); re-armed on the next motion

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sink_rnr.h"
#include "rnr_os.h"

#define RNR_QUEUE        6      // slices queued in the bridge (150 ms)
#define RNR_LEAD         3      // slices sent ahead of real time (75 ms)
#define RNR_MAX_UNACKED  (RNR_LEAD + 2)
#define WATCHDOG_MS      500    // no status report for that long = controller lost
#define REST_MS          50     // margin after the last slice has run before EndOfMotion
#define READ_TIMEOUT_MS  2
#define CONNECT_WAIT_MS  1000
#define SLICE_US         ((uint64_t)SLICE_MS * 1000u)

typedef struct {
    rnr_transport_t *t;
    rnr_thread_t thread;
    rnr_mutex_t lock;

    // shared, protected by lock
    slice_t queue[RNR_QUEUE];
    int qhead, qcount;
    rnr_status_t status;
    bool quit, stop_request, fault, run_open_shared;
    bool link_lost;             // no status any more: the machine position is unknown
    bool blocked;               // between stop() and resume(): pushed slices are dropped
    bool halted;                // the controller has taken the stop and reported its position since
    int32_t halted_pos[RNR_AXES];
    char fault_msg[160];

    // I/O thread only
    uint16_t seq;
    uint16_t arm_seq;           // sequence number of the ClearEStop that armed the controller
    bool armed, run_open;
    uint64_t busy_until;        // PC time at which the controller will have run every slice sent
    uint64_t last_status;
    uint16_t last_slice_seq;
    uint8_t outputs;
    uint32_t slices_sent;
    bool awaiting_halt;         // EStop sent, waiting for a status report that covers it
    uint16_t halt_seq;
    // Position check after each run: controller counters must equal start + steps sent
    int32_t run_start[RNR_AXES], run_sent[RNR_AXES];
    bool check_pending;
    uint16_t check_seq;
    uint32_t checks_ok, checks_bad;
} rnr_sink_t;

static int send_frame (rnr_sink_t *s, uint8_t buf[RNR_OUT_BYTES])
{
    rnr_put_le16(&buf[3], ++s->seq);

    return s->t->write(s->t, buf);
}

static int send_short (rnr_sink_t *s, rnr_opcode_t op, bool limit_override, uint8_t arg5, uint8_t arg6)
{
    uint8_t buf[RNR_OUT_BYTES];

    rnr_build_short(buf, op, 0, limit_override, arg5, arg6);

    return send_frame(s, buf);
}

static int send_configure (rnr_sink_t *s)
{
    uint8_t buf[RNR_OUT_BYTES];
    rnr_config_t cfg = { .pulse_khz = 100, .slice_ms = SLICE_MS, .step_dir_invert = 0 };

    rnr_build_configure(buf, 0, &cfg);

    return send_frame(s, buf);
}

static void set_fault (rnr_sink_t *s, const char *msg, bool link_lost)
{
    rnr_mutex_lock(&s->lock);
    if(link_lost)
        s->link_lost = true;
    if(!s->fault) {
        s->fault = true;
        snprintf(s->fault_msg, sizeof(s->fault_msg), "%s", msg);
        fprintf(stderr, "[rnr] FAULT: %s\n", msg);
    }
    s->qcount = 0;
    rnr_mutex_unlock(&s->lock);
}

// Immediate halt: the controller drops whatever it has buffered.
static void halt (rnr_sink_t *s)
{
    send_short(s, RnrOp_EStop, false, 0, 0);
    s->halt_seq = s->seq;
    s->awaiting_halt = true;
    s->armed = false;
    s->run_open = false;
    s->check_pending = false;
    s->busy_until = 0;
}

// Compares the controller's counters with the steps sent during the run that just ended.
static void check_run_position (rnr_sink_t *s, const rnr_status_t *st)
{
    bool ok = true;

    for(int a = 0; a < RNR_AXES; a++)
        ok = ok && st->live_steps[a] == s->run_start[a] + s->run_sent[a];

    if(ok)
        s->checks_ok++;
    else {
        s->checks_bad++;
        fprintf(stderr, "[rnr] POSITION MISMATCH after run: controller X=%d Y=%d Z=%d A=%d, expected X=%d Y=%d Z=%d A=%d\n",
                 st->live_steps[0], st->live_steps[1], st->live_steps[2], st->live_steps[3],
                 s->run_start[0] + s->run_sent[0], s->run_start[1] + s->run_sent[1],
                 s->run_start[2] + s->run_sent[2], s->run_start[3] + s->run_sent[3]);
    }
}

static bool arm (rnr_sink_t *s)
{
    if(send_short(s, RnrOp_PreReset, true, 0, 0) != RNR_IO_OK ||
        send_short(s, RnrOp_ClearEStop, true, 0, 0) != RNR_IO_OK)
        return false;
    s->arm_seq = s->seq;
    // After a reset the controller runs on default timing until Configure is sent again
    if(send_configure(s) != RNR_IO_OK)
        return false;
    s->armed = true;

    return true;
}

static void io_thread (void *arg)
{
    rnr_sink_t *s = arg;
    uint8_t rx[RNR_IN_BYTES];
    rnr_status_t st = {0};
    bool have_status = false;

    s->last_status = rnr_now_us();

    while(true) {

        int r = s->t->read(s->t, rx, READ_TIMEOUT_MS);
        uint64_t now = rnr_now_us();

        if(r == RNR_IO_OK && rnr_parse_status(rx, RNR_IN_BYTES, &st)) {
            have_status = true;
            s->last_status = now;
            rnr_mutex_lock(&s->lock);
            s->status = st;
            // First report that covers the EStop: this is where the machine stopped
            if(s->awaiting_halt && (int16_t)(st.ack_seq - s->halt_seq) >= 0) {
                s->awaiting_halt = false;
                s->halted = true;
                memcpy(s->halted_pos, st.live_steps, sizeof(s->halted_pos));
            }
            rnr_mutex_unlock(&s->lock);
            // Faults reported by the controller count once it has taken the arming frame
            if(s->armed && (int16_t)(st.ack_seq - s->arm_seq) >= 0) {
                if(st.fault_flags & RNR_FAULT_ESTOP)
                    set_fault(s, "controller reports e-stop", false);
                else if(st.fault_flags & RNR_FAULT_LIMIT)
                    set_fault(s, "controller reports a limit switch", false);
            }
            if(s->check_pending && (int16_t)(st.ack_seq - s->check_seq) >= 0 && st.motion_state == 0) {
                s->check_pending = false;
                check_run_position(s, &st);
            }
        } else if(r == RNR_IO_DISCONNECTED)
            set_fault(s, "USB connection to the controller lost", true);

        if(now - s->last_status > (uint64_t)WATCHDOG_MS * 1000u)
            set_fault(s, "controller stopped reporting status (watchdog)", true);

        rnr_mutex_lock(&s->lock);
        bool quit = s->quit, fault = s->fault, stop = s->stop_request;
        s->stop_request = false;
        rnr_mutex_unlock(&s->lock);

        if(stop || (fault && (s->armed || s->run_open))) {
            if(stop)
                fprintf(stderr, "[rnr] stop requested: e-stop sent to the controller\n");
            halt(s);
            rnr_mutex_lock(&s->lock);
            s->qcount = 0;
            rnr_mutex_unlock(&s->lock);
        }

        if(quit)
            break;
        if(fault || r == RNR_IO_DISCONNECTED)
            continue;

        // Send the slices that are due
        while(true) {
            if(s->busy_until > now + RNR_LEAD * SLICE_US - SLICE_US / 2)
                break;  // far enough ahead of real time
            if(have_status && (int16_t)(s->seq - st.ack_seq) > RNR_MAX_UNACKED)
                break;  // safety limit on unacknowledged frames

            slice_t slice;
            rnr_mutex_lock(&s->lock);
            bool got = s->qcount > 0;
            if(got) {
                slice = s->queue[s->qhead];
                s->qhead = (s->qhead + 1) % RNR_QUEUE;
                s->qcount--;
            }
            rnr_mutex_unlock(&s->lock);
            if(!got)
                break;

            if(!s->armed && !arm(s)) {
                set_fault(s, "cannot arm the controller", false);
                break;
            }
            if(!s->run_open) {
                if(send_short(s, RnrOp_StartJog, false, 0, 0) != RNR_IO_OK) {
                    set_fault(s, "cannot start a motion run", false);
                    break;
                }
                s->run_open = true;
                // The controller is at rest between runs: its counters are the run's start point
                memcpy(s->run_start, st.live_steps, sizeof(s->run_start));
                memset(s->run_sent, 0, sizeof(s->run_sent));
            }

            uint8_t buf[RNR_OUT_BYTES];
            rnr_build_slice(buf, RnrOp_JogSlice, 0, false, slice.steps, slice.outputs);
            if(send_frame(s, buf) != RNR_IO_OK) {
                set_fault(s, "slice write failed", false);
                break;
            }
            for(int a = 0; a < RNR_AXES; a++)
                s->run_sent[a] += slice.steps[a];
            s->last_slice_seq = s->seq;
            s->outputs = slice.outputs;
            s->slices_sent++;
            s->busy_until = (s->busy_until > now ? s->busy_until : now) + SLICE_US;
        }

        // Close the run once the controller has run out of slices
        rnr_mutex_lock(&s->lock);
        bool empty = s->qcount == 0;
        rnr_mutex_unlock(&s->lock);
        if(s->run_open && empty && now > s->busy_until + (uint64_t)REST_MS * 1000u &&
            have_status && (int16_t)(st.ack_seq - s->last_slice_seq) >= 0) {
            send_short(s, RnrOp_EndOfMotion, false, s->outputs, 0);
            s->run_open = false;
            s->check_pending = true;
            s->check_seq = s->seq;
        }

        rnr_mutex_lock(&s->lock);
        s->run_open_shared = s->run_open;
        rnr_mutex_unlock(&s->lock);
    }
}

static sink_status_t rnr_push (slice_sink_t *sink, const slice_t *slice)
{
    rnr_sink_t *s = sink->ctx;
    sink_status_t status = Sink_Ok;

    rnr_mutex_lock(&s->lock);
    if(s->blocked)
        status = Sink_Ok;   // stopped: the slice is dropped
    else if(s->fault)
        status = Sink_Error;
    else if(s->qcount == RNR_QUEUE)
        status = Sink_Full;
    else {
        s->queue[(s->qhead + s->qcount) % RNR_QUEUE] = *slice;
        s->qcount++;
    }
    rnr_mutex_unlock(&s->lock);

    return status;
}

static void rnr_get_position (slice_sink_t *sink, int32_t position[SLICE_AXES])
{
    rnr_sink_t *s = sink->ctx;

    rnr_mutex_lock(&s->lock);
    for(int a = 0; a < SLICE_AXES; a++)
        position[a] = a < RNR_AXES ? s->status.live_steps[a] : 0;
    rnr_mutex_unlock(&s->lock);
}

static void rnr_stop (slice_sink_t *sink)
{
    rnr_sink_t *s = sink->ctx;

    rnr_mutex_lock(&s->lock);
    s->stop_request = true;
    s->blocked = true;
    s->halted = false;
    s->qcount = 0;
    rnr_mutex_unlock(&s->lock);
}

static void rnr_resume (slice_sink_t *sink)
{
    rnr_sink_t *s = sink->ctx;

    rnr_mutex_lock(&s->lock);
    s->blocked = false;
    rnr_mutex_unlock(&s->lock);
}

static bool rnr_get_halted_position (slice_sink_t *sink, int32_t position[SLICE_AXES], uint32_t timeout_ms)
{
    rnr_sink_t *s = sink->ctx;
    uint64_t deadline = rnr_now_us() + (uint64_t)timeout_ms * 1000u;
    bool halted = false, lost = false;

    while(true) {
        rnr_mutex_lock(&s->lock);
        halted = s->halted;
        lost = s->link_lost;
        if(halted)
            for(int a = 0; a < SLICE_AXES; a++)
                position[a] = a < RNR_AXES ? s->halted_pos[a] : 0;
        rnr_mutex_unlock(&s->lock);
        if(halted || lost || rnr_now_us() >= deadline)
            break;
        rnr_sleep_ms(1);
    }

    return halted;
}

static void rnr_close (slice_sink_t *sink)
{
    rnr_sink_t *s = sink->ctx;
    uint64_t deadline = rnr_now_us() + 10000000u;   // let pending motion finish, 10 s at most

    while(rnr_now_us() < deadline) {
        rnr_mutex_lock(&s->lock);
        bool done = s->fault || (s->qcount == 0 && !s->run_open_shared);
        rnr_mutex_unlock(&s->lock);
        if(done)
            break;
        rnr_sleep_ms(10);
    }
    rnr_sleep_ms(30);   // let the I/O thread run the last position check

    rnr_mutex_lock(&s->lock);
    s->quit = true;
    rnr_mutex_unlock(&s->lock);
    rnr_thread_join(s->thread);

    // Leave the controller disarmed
    send_short(s, RnrOp_EStop, false, 0, 0);
    fprintf(stderr, "[rnr] closed: %u slices sent, position checks after runs: %u ok, %u mismatch\n",
             s->slices_sent, s->checks_ok, s->checks_bad);

    s->t->close(s->t);
    rnr_mutex_destroy(&s->lock);
    free(s);
    free(sink);
}

slice_sink_t *sink_rnr_open (rnr_transport_t *t)
{
    slice_sink_t *sink = calloc(1, sizeof(slice_sink_t));
    rnr_sink_t *s = calloc(1, sizeof(rnr_sink_t));
    uint8_t buf[RNR_OUT_BYTES], rx[RNR_IN_BYTES];
    static const int32_t zero[RNR_AXES] = {0};
    rnr_status_t st;
    bool got = false;

    if(!sink || !s) {
        free(s);
        free(sink);
        return NULL;
    }
    s->t = t;
    rnr_mutex_init(&s->lock);

    // Connect: configure, stay in e-stop until the first motion, outputs off, step counters = 0
    int r = send_configure(s);
    if(r == RNR_IO_OK)
        r = send_short(s, RnrOp_EStop, false, 0, 0);
    if(r == RNR_IO_OK)
        r = send_short(s, RnrOp_SetOutputs, false, 0, 0);
    if(r == RNR_IO_OK) {
        rnr_build_positions(buf, 0, zero);
        r = send_frame(s, buf);
    }

    for(uint64_t end = rnr_now_us() + CONNECT_WAIT_MS * 1000u; r == RNR_IO_OK && !got && rnr_now_us() < end; )
        got = t->read(t, rx, 20) == RNR_IO_OK && rnr_parse_status(rx, RNR_IN_BYTES, &st);

    if(!got || rnr_thread_start(&s->thread, io_thread, s) != 0) {
        fprintf(stderr, "[rnr] controller does not answer\n");
        t->close(t);
        rnr_mutex_destroy(&s->lock);
        free(s);
        free(sink);
        return NULL;
    }
    s->status = st;

    sink->name = t->name;
    sink->push = rnr_push;
    sink->get_position = rnr_get_position;
    sink->stop = rnr_stop;
    sink->resume = rnr_resume;
    sink->get_halted_position = rnr_get_halted_position;
    sink->close = rnr_close;
    sink->ctx = s;

    return sink;
}
