/*
  sink_rnr.c - slice sink driving an RNR Eco Motion 2.0 controller

  The simulator thread pushes slices into a small bounded queue. A dedicated
  I/O thread owns the transport: it reads status reports continuously, sends
  slices paced by the PC clock and runs the watchdog.

  Pacing: the controller acknowledges a slice as soon as it has buffered it,
  not when it has executed it, and it buffers at least 100 slices (Mach3 keeps
  about 0.5 s ahead). Sending on acknowledgements alone lets the controller run
  far behind, so feed holds would act late. Slices are therefore sent in real
  time, at most RNR_LEAD_MS ahead of the PC clock; the acknowledgement count
  is only a safety limit.

  Frame sequence, as Mach3 drives the controller (USB captures):
    connect : Configure, EStop, SpindlePwm(0), SetOutputs(0), SetPositions(0)
    arm     : PreReset, ClearEStop, SetPositions(current), Configure
              (before the first motion and after any e-stop)
    run     : GcodeSlice..., then EndOfMotion(outputs) once all slices ran;
              the controller goes Program (6) -> Ending (7) -> Ready (1)
    outputs : at rest, SpindlePwm(duty) then SetOutputs(OUT1..OUT4); during a run every
              slice carries the outputs (byte 25). The core changes outputs between
              moves only: a run is closed before an output switches, so that it switches
              once the motion before it has run, not RNR_LEAD_MS early
    reset   : Cancel: stops at once, drops the buffer, stays armed
    fault   : EStop (watchdog, USB, write failure): disarms
    e-stop  : the controller stops by itself on its e-stop input (state 0,
              fault flag 0x20) and must be re-armed: reported as e-stop

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sink_rnr.h"
#include "rnr_os.h"

#define RNR_LEAD_MS      75     // slices sent ahead of real time
#define RNR_LEAD         (RNR_LEAD_MS / SLICE_MS)
#define RNR_PREFILL_MS   40     // a run starts with that much motion queued (the simulator produces slices in ~20 ms bursts)
#define RNR_PREFILL      (RNR_PREFILL_MS / SLICE_MS)
#define RNR_QUEUE        (2 * RNR_LEAD)     // slices queued in the bridge
#define RNR_MAX_UNACKED  (RNR_LEAD + 25)    // status reports come every ~50 ms: acks lag up to 10 slices
#define WATCHDOG_MS      500    // no status report for that long = controller lost
#define REST_MS          50     // margin after the last slice has run before EndOfMotion
#define ESTOP_HOLD_MS    300    // an e-stop seen only as a disarm (press shorter than a status period) is held that long
#define READ_TIMEOUT_MS  2
#define CONNECT_WAIT_MS  1000
#define SLICE_US         ((uint64_t)SLICE_MS * 1000u)

typedef struct {
    rnr_transport_t *t;
    rnr_thread_t thread;
    rnr_mutex_t lock;
    rnr_options_t opt;

    // shared, protected by lock
    slice_t queue[RNR_QUEUE];
    int qhead, qcount;
    rnr_status_t status;
    bool quit, stop_request, fault, run_open_shared;
    bool link_lost;             // no status any more: the machine position is unknown
    bool blocked;               // between stop() and resume(): pushed slices are dropped
    bool halted;                // the controller has taken the stop and reported its position since
    int32_t halted_pos[RNR_AXES];
    bool estop_input;           // the controller reports its e-stop input active
    slice_outputs_t latest;     // outputs last set by the core
    uint64_t estop_until;       // e-stop seen as a disarm only: reported until then
    char fault_msg[160];

    // I/O thread only
    uint16_t seq;
    uint16_t arm_seq;           // sequence number of the Configure that ends the arming sequence
    bool armed, run_open;
    uint64_t busy_until;        // PC time at which the controller will have run every slice sent
    uint64_t first_queued;      // PC time at which slices were first seen queued with no run open (0: none)
    uint64_t last_status;
    uint16_t last_slice_seq;
    uint8_t outputs;            // OUT1..OUT4 last sent
    uint16_t pwm;               // spindle PWM duty last sent
    bool outputs_change;        // the run must close: the next slice switches an output
    uint32_t slices_sent, estops_seen;
    bool awaiting_halt;         // stop handled, waiting for a status report that covers it
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

static int send_short (rnr_sink_t *s, rnr_opcode_t op, uint8_t arg5)
{
    uint8_t buf[RNR_OUT_BYTES];

    rnr_build_short(buf, op, 0, arg5, 0);

    return send_frame(s, buf);
}

static int send_pwm (rnr_sink_t *s, uint16_t duty)
{
    uint8_t buf[RNR_OUT_BYTES];

    rnr_build_pwm(buf, 0, duty > RNR_PWM_PERIOD ? RNR_PWM_PERIOD : duty);
    s->pwm = duty;

    return send_frame(s, buf);
}

static uint8_t out_bit (uint8_t n)
{
    return n >= 1 && n <= 4 ? (uint8_t)(1u << (n - 1)) : 0;
}

// Core outputs -> controller outputs OUT1..OUT4
static uint8_t map_outputs (const rnr_sink_t *s, uint8_t flags)
{
    uint8_t out = 0;

    if(flags & SLICE_OUT_SPINDLE)
        out |= out_bit((flags & SLICE_OUT_SPINDLE_CCW) && s->opt.spindle_ccw_out ? s->opt.spindle_ccw_out : s->opt.spindle_out);
    if(flags & SLICE_OUT_FLOOD)
        out |= out_bit(s->opt.flood_out);
    if(flags & SLICE_OUT_MIST)
        out |= out_bit(s->opt.mist_out);

    return out;
}

// Switches the outputs while no run is open: spindle speed first, then the outputs, as Mach3
// does on M3 and M5.
static int apply_outputs (rnr_sink_t *s, slice_outputs_t out)
{
    int r = RNR_IO_OK;
    uint8_t outputs = map_outputs(s, out.flags);

    if(out.spindle_pwm != s->pwm)
        r = send_pwm(s, out.spindle_pwm);
    if(r == RNR_IO_OK && outputs != s->outputs) {
        s->outputs = outputs;
        r = send_short(s, RnrOp_SetOutputs, outputs);
    }

    return r;
}

// Spindle stopped and every output off (fault, exit), whatever the core has asked for
static void outputs_off (rnr_sink_t *s)
{
    if(s->pwm)
        send_pwm(s, 0);
    if(s->outputs) {
        s->outputs = 0;
        send_short(s, RnrOp_SetOutputs, 0);
    }
}

static int send_configure (rnr_sink_t *s)
{
    uint8_t buf[RNR_OUT_BYTES];
    rnr_config_t cfg = { .pulse_khz = 100, .slice_ms = SLICE_MS, .step_dir_invert = s->opt.step_dir_invert };

    rnr_build_configure(buf, 0, &cfg);

    return send_frame(s, buf);
}

static int send_positions (rnr_sink_t *s, const int32_t steps[RNR_AXES])
{
    uint8_t buf[RNR_OUT_BYTES];

    rnr_build_positions(buf, 0, steps, 0x0F);

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

// The machine is stopped (or about to be): forget the run in progress. The next status report
// that covers every frame sent so far gives the position where it stopped.
static void end_motion (rnr_sink_t *s)
{
    s->halt_seq = s->seq;
    s->awaiting_halt = true;
    s->run_open = false;
    s->first_queued = 0;
    s->check_pending = false;
    s->busy_until = 0;
    rnr_mutex_lock(&s->lock);
    s->qcount = 0;
    rnr_mutex_unlock(&s->lock);
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

// Same sequence as Mach3 when its RESET button is released. The step counters are set to their
// current value, so that the controller and the bridge keep the same origin.
static bool arm (rnr_sink_t *s, const rnr_status_t *st)
{
    if(send_short(s, RnrOp_PreReset, 0) != RNR_IO_OK ||
        send_short(s, RnrOp_ClearEStop, 0) != RNR_IO_OK ||
        send_positions(s, st->live_steps) != RNR_IO_OK ||
        send_configure(s) != RNR_IO_OK)
        return false;
    s->arm_seq = s->seq;
    s->armed = true;

    return true;
}

static void on_status (rnr_sink_t *s, const rnr_status_t *st, uint64_t now)
{
    bool covered = (int16_t)(st->ack_seq - s->seq) >= 0;    // every frame sent so far taken over
    bool estop_input = (st->fault_flags & RNR_FAULT_ESTOP) != 0;
    bool armed_seen = s->armed && (int16_t)(st->ack_seq - s->arm_seq) >= 0;

    // The controller stopped by itself on its e-stop input: it dropped its buffer and must be
    // re-armed. A press shorter than a status period shows only as the disarm.
    if(estop_input || (armed_seen && st->state == RnrState_EStop)) {
        if(s->armed) {
            s->estops_seen++;
            fprintf(stderr, "[rnr] controller e-stop%s: stopped at X=%d Y=%d Z=%d A=%d\n",
                     estop_input ? "" : " (seen as a disarm)",
                     st->live_steps[0], st->live_steps[1], st->live_steps[2], st->live_steps[3]);
            s->armed = false;
            end_motion(s);
            rnr_mutex_lock(&s->lock);
            s->estop_until = now + (uint64_t)ESTOP_HOLD_MS * 1000u;
            // Slices computed before the e-stop must never run once it is released: drop them
            // until the core has reset (the reset calls stop() then resume()).
            s->blocked = true;
            rnr_mutex_unlock(&s->lock);
        }
    }

    rnr_mutex_lock(&s->lock);
    s->status = *st;
    s->estop_input = estop_input;
    if(s->awaiting_halt && covered) {
        s->awaiting_halt = false;
        s->halted = true;
        memcpy(s->halted_pos, st->live_steps, sizeof(s->halted_pos));
    }
    rnr_mutex_unlock(&s->lock);

    if(armed_seen && (st->fault_flags & RNR_FAULT_LIMIT))
        set_fault(s, "controller reports a limit switch", false);

    if(s->check_pending && (int16_t)(st->ack_seq - s->check_seq) >= 0 && st->state == RnrState_Ready) {
        s->check_pending = false;
        check_run_position(s, st);
    }
}

static void io_thread (void *arg)
{
    rnr_sink_t *s = arg;
    uint8_t rx[RNR_IN_BYTES];
    rnr_status_t st = s->status;

    s->last_status = rnr_now_us();

    while(true) {

        int r = s->t->read(s->t, rx, READ_TIMEOUT_MS);
        uint64_t now = rnr_now_us();

        if(r == RNR_IO_OK && rnr_parse_status(rx, RNR_IN_BYTES, &st)) {
            s->last_status = now;
            on_status(s, &st, now);
        } else if(r == RNR_IO_DISCONNECTED)
            set_fault(s, "USB connection to the controller lost", true);

        if(now - s->last_status > (uint64_t)WATCHDOG_MS * 1000u)
            set_fault(s, "controller stopped reporting status (watchdog)", true);

        rnr_mutex_lock(&s->lock);
        bool quit = s->quit, fault = s->fault, stop = s->stop_request;
        s->stop_request = false;
        rnr_mutex_unlock(&s->lock);

        if(fault && s->armed) {
            // Faults disarm the controller: it must not run anything until the bridge restarts
            send_short(s, RnrOp_EStop, 0);
            outputs_off(s);
            s->armed = false;
            end_motion(s);
        } else if(stop) {
            // Core reset: stop at once and drop the buffer, like Mach3's Stop button. The
            // controller stays armed. Nothing to send if it is already stopped (e-stop).
            if(s->armed) {
                send_short(s, RnrOp_Cancel, 0);
                fprintf(stderr, "[rnr] stop requested: cancel sent to the controller\n");
            }
            end_motion(s);
        }

        if(quit)
            break;
        if(fault || r == RNR_IO_DISCONNECTED)
            continue;

        // Prefill: open a run only once enough motion is queued, or the motion is shorter than that
        rnr_mutex_lock(&s->lock);
        int queued = s->qcount;
        rnr_mutex_unlock(&s->lock);
        bool may_send = s->run_open;
        if(!s->run_open && queued) {
            if(!s->first_queued)
                s->first_queued = now;
            may_send = queued >= RNR_PREFILL || now - s->first_queued >= (uint64_t)RNR_PREFILL_MS * 1000u;
        }

        // Send the slices that are due
        while(true) {
            if(!may_send)
                break;
            if(s->busy_until > now + RNR_LEAD * SLICE_US - SLICE_US / 2)
                break;  // far enough ahead of real time
            if((int16_t)(s->seq - st.ack_seq) > RNR_MAX_UNACKED)
                break;  // safety limit on unacknowledged frames

            slice_t slice;
            rnr_mutex_lock(&s->lock);
            bool got = s->qcount > 0 && !s->estop_input;
            if(got)
                slice = s->queue[s->qhead];
            rnr_mutex_unlock(&s->lock);
            if(!got)
                break;

            if(!s->run_open && !(slice.steps[0] | slice.steps[1] | slice.steps[2] | slice.steps[3])) {
                // Empty slice with no run open (the core wakes the steppers for a block without
                // motion, e.g. M5 or M9 in a program): it would open a run for nothing
                rnr_mutex_lock(&s->lock);
                if(s->qcount > 0) {
                    s->qhead = (s->qhead + 1) % RNR_QUEUE;
                    s->qcount--;
                }
                rnr_mutex_unlock(&s->lock);
                s->first_queued = 0;    // the prefill delay starts with the first real slice
                break;
            }

            uint8_t outputs = map_outputs(s, slice.out.flags);
            if(s->run_open && outputs != s->outputs) {
                // An output switches between two moves (M3, M5, M8...: the core waits for the end of
                // motion before it changes them). Close the run and switch once every slice before
                // has run, so that the spindle does not start or stop RNR_LEAD_MS early.
                s->outputs_change = true;
                break;
            }

            if(!s->armed && !arm(s, &st)) {
                set_fault(s, "cannot arm the controller", false);
                break;
            }
            if(!s->run_open && (apply_outputs(s, slice.out) != RNR_IO_OK)) {
                set_fault(s, "output write failed", false);
                break;
            }
            if(slice.out.spindle_pwm != s->pwm && send_pwm(s, slice.out.spindle_pwm) != RNR_IO_OK) {
                // speed change during a run (S word, laser mode): sent ahead of its slice
                set_fault(s, "spindle PWM write failed", false);
                break;
            }

            rnr_mutex_lock(&s->lock);
            got = s->qcount > 0;    // stop() may have emptied the queue meanwhile
            if(got) {
                s->qhead = (s->qhead + 1) % RNR_QUEUE;
                s->qcount--;
            }
            rnr_mutex_unlock(&s->lock);
            if(!got)
                break;

            if(!s->run_open) {
                // The first program slice opens the run. The controller is at rest between runs:
                // its counters are the run's start point.
                s->run_open = true;
                s->first_queued = 0;
                memcpy(s->run_start, st.live_steps, sizeof(s->run_start));
                memset(s->run_sent, 0, sizeof(s->run_sent));
            }

            uint8_t buf[RNR_OUT_BYTES];
            rnr_build_slice(buf, RnrOp_GcodeSlice, 0, slice.steps, s->outputs);
            if(send_frame(s, buf) != RNR_IO_OK) {
                set_fault(s, "slice write failed", false);
                break;
            }
            for(int a = 0; a < RNR_AXES; a++)
                s->run_sent[a] += slice.steps[a];
            s->last_slice_seq = s->seq;
            s->slices_sent++;
            s->busy_until = (s->busy_until > now ? s->busy_until : now) + SLICE_US;
        }

        // Close the run once the controller has run out of slices
        rnr_mutex_lock(&s->lock);
        bool empty = s->qcount == 0;
        slice_outputs_t latest = s->latest;
        bool blocked = s->blocked;
        rnr_mutex_unlock(&s->lock);
        if(s->run_open && (empty || s->outputs_change) && now > s->busy_until + (uint64_t)REST_MS * 1000u &&
            (int16_t)(st.ack_seq - s->last_slice_seq) >= 0) {
            send_short(s, RnrOp_EndOfMotion, s->outputs);
            s->run_open = false;
            s->outputs_change = false;
            s->check_pending = true;
            s->check_seq = s->seq;
        }

        // Outputs at rest (M3 without motion, M5 at the end of a program): applied once the
        // controller has run every slice
        if(!s->run_open && empty && !s->estop_input &&
            (map_outputs(s, latest.flags) != s->outputs || latest.spindle_pwm != s->pwm)) {
            if(map_outputs(s, latest.flags) && !s->armed && (blocked || !arm(s, &st))) {
                if(!blocked)
                    set_fault(s, "cannot arm the controller", false);
            } else if(apply_outputs(s, latest) != RNR_IO_OK)
                set_fault(s, "output write failed", false);
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
        halted = s->halted && !s->stop_request;
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

static void rnr_set_outputs (slice_sink_t *sink, slice_outputs_t out)
{
    rnr_sink_t *s = sink->ctx;

    rnr_mutex_lock(&s->lock);
    s->latest = out;
    rnr_mutex_unlock(&s->lock);
}

static bool rnr_estop_active (slice_sink_t *sink)
{
    rnr_sink_t *s = sink->ctx;

    rnr_mutex_lock(&s->lock);
    bool active = s->estop_input || rnr_now_us() < s->estop_until;
    rnr_mutex_unlock(&s->lock);

    return active;
}

static void rnr_close (slice_sink_t *sink)
{
    rnr_sink_t *s = sink->ctx;
    uint64_t deadline = rnr_now_us() + 10000000u;   // let pending motion finish, 10 s at most

    while(rnr_now_us() < deadline) {
        rnr_mutex_lock(&s->lock);
        bool done = s->fault || s->estop_input || (s->qcount == 0 && !s->run_open_shared);
        rnr_mutex_unlock(&s->lock);
        if(done)
            break;
        rnr_sleep_ms(10);
    }
    rnr_sleep_ms(80);   // let the I/O thread run the last position check (status every ~50 ms)

    rnr_mutex_lock(&s->lock);
    s->quit = true;
    rnr_mutex_unlock(&s->lock);
    rnr_thread_join(s->thread);

    // Leave the controller disarmed, spindle and outputs off
    send_short(s, RnrOp_EStop, 0);
    outputs_off(s);
    fprintf(stderr, "[rnr] closed: %u slices sent, %u controller e-stops, position checks after runs: %u ok, %u mismatch\n",
             s->slices_sent, s->estops_seen, s->checks_ok, s->checks_bad);

    s->t->close(s->t);
    rnr_mutex_destroy(&s->lock);
    free(s);
    free(sink);
}

slice_sink_t *sink_rnr_open (rnr_transport_t *t, const rnr_options_t *opt)
{
    slice_sink_t *sink = calloc(1, sizeof(slice_sink_t));
    rnr_sink_t *s = calloc(1, sizeof(rnr_sink_t));
    uint8_t rx[RNR_IN_BYTES];
    static const int32_t zero[RNR_AXES] = {0};
    rnr_status_t st;
    bool got = false;

    if(!sink || !s) {
        free(s);
        free(sink);
        t->close(t);
        return NULL;
    }
    s->t = t;
    if(opt)
        s->opt = *opt;
    rnr_mutex_init(&s->lock);

    // Connect: configure, stay in e-stop until the first motion, outputs off, step counters = 0
    int r = send_configure(s);
    if(r == RNR_IO_OK)
        r = send_short(s, RnrOp_EStop, 0);
    if(r == RNR_IO_OK)
        r = send_pwm(s, 0);
    if(r == RNR_IO_OK)
        r = send_short(s, RnrOp_SetOutputs, 0);
    if(r == RNR_IO_OK)
        r = send_positions(s, zero);

    for(uint64_t end = rnr_now_us() + CONNECT_WAIT_MS * 1000u; r == RNR_IO_OK && !got && rnr_now_us() < end; )
        got = t->read(t, rx, 20) == RNR_IO_OK && rnr_parse_status(rx, RNR_IN_BYTES, &st);

    if(got) {
        s->status = st;
        s->estop_input = (st.fault_flags & RNR_FAULT_ESTOP) != 0;
    }

    if(!got || rnr_thread_start(&s->thread, io_thread, s) != 0) {
        fprintf(stderr, "[rnr] controller does not answer\n");
        t->close(t);
        rnr_mutex_destroy(&s->lock);
        free(s);
        free(sink);
        return NULL;
    }

    sink->name = t->name;
    sink->push = rnr_push;
    sink->get_position = rnr_get_position;
    sink->stop = rnr_stop;
    sink->resume = rnr_resume;
    sink->get_halted_position = rnr_get_halted_position;
    sink->close = rnr_close;
    sink->estop_active = rnr_estop_active;
    sink->set_outputs = rnr_set_outputs;
    sink->ctx = s;

    return sink;
}
