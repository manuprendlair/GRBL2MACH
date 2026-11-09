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

  The controller runs a slice in slightly more than its nominal period (about
  5.02 ms for 5 ms on the reference board), so pacing on the PC clock alone
  lets its buffer grow during long moves (seen: +7 ms of lag per second of
  jog). Each status report gives the controller's step counters: the bridge
  finds which slice of the run they correspond to and keeps the number of
  slices ahead of it at RNR_LEAD, whatever the controller's real pace.

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
    homing  : one axis per move: ArmHome(axis) + JogSlices; the controller stops by itself
              on the axis' home switch (state 4 -> 1) and ignores the slices still to come,
              which the bridge then drops. Pull-off: StartJog(axis) + JogSlices + EndOfMotion
    probing : ArmProbe + JogSlices; the controller stops by itself on contact and latches the
              four positions (rx[13 + 8a]), handed over to the core at the end of the cycle
    e-stop  : the controller stops by itself on its e-stop input (state 0,
              fault flag 0x20) and must be re-armed: reported as e-stop

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>

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
#define RECONNECT_MS     1000   // link lost: one reconnection attempt that often
#define SLICE_US         ((uint64_t)SLICE_MS * 1000u)
#define TRACK            1024   // slices of the open run remembered to locate the controller in it

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
    bool stop_jog_request;      // the jog in progress is being stopped (stop_jog)
    bool link_lost;             // no status any more: the machine position is unknown
    uint32_t generation;        // reconnections so far (link_ok)
    bool blocked;               // between stop() and resume(): pushed slices are dropped
    bool halted;                // the controller has taken the stop and reported its position since
    int32_t halted_pos[RNR_AXES];
    bool estop_input;           // the controller reports its e-stop input active
    bool inputs_known;          // status.inputs comes from the controller
    slice_outputs_t latest;     // outputs last set by the core
    // Homing / probing move that the controller cut short by itself (switch, contact): its
    // remaining slices are dropped and the switch is reported to the core until the next move
    bool cut_active;
    uint8_t cut_kind, cut_axis;
    bool resync_pending;        // the slicer's step count must be reset from the controller (take_resync)
    bool probe_contact;         // the last probing move ended on a contact
    bool idle_shared;           // nothing queued, no run open, every frame taken over, controller ready
    uint64_t estop_until;       // e-stop seen as a disarm only: reported until then
    char fault_msg[160];
    atomic_bool disarm_request, disarm_done;    // the program is ending (no lock: may come from a crash handler)

    // I/O thread only
    uint16_t seq;
    uint16_t arm_seq;           // sequence number of the Configure that ends the arming sequence
    bool arm_seen;              // the controller has taken it (sticky: the 16-bit sequence wraps)
    bool armed, run_open;
    uint64_t busy_until;        // PC time at which the controller will have run every slice sent
    // Where the controller is in the open run: cumulative steps after each slice sent (index 1..),
    // and the last estimate of the number of slices it has run, at the time of a status report
    int32_t track_cum[TRACK][RNR_AXES];
    uint32_t run_slices;        // slices sent in the open run
    double run_done;            // estimate: slices of the run the controller has run (integrated in time)
    uint64_t run_done_time;     // PC time of that estimate
    uint32_t track_fixes, track_ambiguous;  // status reports used / skipped to locate the controller
    uint64_t run_t0;            // PC time the run's first slice was sent
    bool run_done_valid;
    uint64_t first_queued;      // PC time at which slices were first seen queued with no run open (0: none)
    uint64_t last_status;
    uint64_t next_reconnect;    // link lost: PC time of the next reconnection attempt
    uint16_t last_slice_seq;
    uint8_t outputs;            // OUT1..OUT4 last sent
    uint16_t pwm;               // spindle PWM duty last sent
    bool outputs_change;        // the run must close: the next slice switches an output
    bool kind_change;           // the run must close: the next slice is for another kind of move
    uint8_t run_kind, run_axis; // slice_kind_t and axis of the open run
    uint16_t run_arm_seq;       // sequence number of the frame that opened the run
    bool run_armed;             // the controller has taken that frame (sticky: the 16-bit sequence wraps in a long run)
    bool run_cut;               // StopJog sent in this run: it dropped slices, steps sent no longer tell where the controller is
    uint32_t slices_sent, estops_seen, dropped_after_cut;
    bool awaiting_halt;         // stop handled, waiting for a status report that covers it
    uint16_t halt_seq;
    // Position check after each run: controller counters must equal start + steps sent
    int32_t run_start[RNR_AXES], run_sent[RNR_AXES];
    bool check_pending;
    uint16_t check_seq;
    int32_t check_expected[RNR_AXES];   // where the controller must be once the closed run is over
    uint8_t check_kind;
    uint32_t check_slices;
    uint32_t checks_ok, checks_bad;
    // Timing of the open run, to see how much margin the PC leaves (report_run)
    uint64_t last_loop;         // PC time of the previous pass of the I/O loop
    uint64_t worst_gap;         // longest time between two passes
    uint64_t queue_empty_since; // no slice waiting while the run needed one (0: not waiting)
    uint64_t worst_wait;        // longest such wait: the simulator threads were late
    double min_lead;            // fewest slices ahead of the controller, when sending a slice
    uint32_t underruns;         // times the controller had run out of slices and got more
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
    rnr_config_t cfg = { .pulse_khz = s->opt.pulse_khz ? s->opt.pulse_khz : 100, .slice_ms = SLICE_MS, .step_dir_invert = s->opt.step_dir_invert };
    if(s->opt.pins_raw)
        memcpy(cfg.pins, s->opt.pins, RNR_PIN_BYTES);
    else
        rnr_encode_pins(cfg.pins, &s->opt.inputs);

    rnr_build_configure(buf, 0, &cfg);

    return send_frame(s, buf);
}

static int send_positions (rnr_sink_t *s, const int32_t steps[RNR_AXES])
{
    uint8_t buf[RNR_OUT_BYTES];

    rnr_build_positions(buf, 0, steps, 0x0F);

    return send_frame(s, buf);
}

// One line per run of at least a second: the margin the PC left. The controller stops abruptly
// when it runs out of slices in the middle of a move.
static void report_run (rnr_sink_t *s)
{
    if(s->run_slices < 200 || s->min_lead > 1e8)
        return;

    fprintf(stderr, "[rnr] run: %u slices, lead min %.1f slices (%.0f ms), underruns %u, USB thread worst gap %.1f ms, longest wait for motion %.1f ms\n",
             s->run_slices, s->min_lead, s->min_lead * SLICE_MS, s->underruns, s->worst_gap / 1000.0, s->worst_wait / 1000.0);
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

    // values stored when the run was closed: a new run may have opened since
    for(int a = 0; a < RNR_AXES; a++)
        ok = ok && st->live_steps[a] == s->check_expected[a];

    if(ok)
        s->checks_ok++;
    else {
        s->checks_bad++;
        static const char *kinds[] = { "program", "homing seek", "homing pull-off", "probing", "jog" };
        fprintf(stderr, "[rnr] POSITION MISMATCH after a %s run of %u slices: controller X=%d Y=%d Z=%d A=%d, expected X=%d Y=%d Z=%d A=%d (state %u)\n",
                 s->check_kind < 5 ? kinds[s->check_kind] : "?", s->check_slices,
                 st->live_steps[0], st->live_steps[1], st->live_steps[2], st->live_steps[3],
                 s->check_expected[0], s->check_expected[1], s->check_expected[2], s->check_expected[3], st->state);
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
    s->arm_seen = false;

    return true;
}

// True when the input is used and triggered (polarity applied)
static bool input_on (const rnr_input_t *in, uint16_t raw)
{
    return in->pin && !!(raw & (1u << (in->pin - 1))) != in->active_low;
}

// Estimate of the slices of the open run the controller has run: one per slice period since the
// last estimate, never more than sent (the controller waits when it runs out of slices).
static double controller_done (rnr_sink_t *s, uint64_t now)
{
    if(s->run_done_valid) {
        s->run_done += (double)(now - s->run_done_time) / SLICE_US;
        s->run_done_time = now;
        if(s->run_done > s->run_slices)
            s->run_done = s->run_slices;
    }

    return s->run_done;
}

// Puts the estimate back where the controller's step counters say it is: the slice of the run whose
// cumulative steps match them. Only when that match is unambiguous: a toolpath can pass several
// times by the same point, the start of a move or a dwell repeat the same counters. Otherwise the
// estimate goes on with time (the controller drifts by a few ms per second only).
#define TRACK_BACK 400
static void track_controller (rnr_sink_t *s, const rnr_status_t *st, uint64_t now)
{
    if(!s->run_open || !s->run_slices || !s->run_armed || s->run_cut)
        return;

    if(!s->run_done_valid) {     // first report of the run: time since its first slice
        s->run_done = (double)(now - s->run_t0) / SLICE_US;
        s->run_done_time = now;
        s->run_done_valid = true;
    }
    double done = controller_done(s, now);

    int32_t rel[RNR_AXES];
    for(int a = 0; a < RNR_AXES; a++)
        rel[a] = st->live_steps[a] - s->run_start[a];

    uint32_t hi = s->run_slices;
    uint32_t lo = done > TRACK_BACK ? (uint32_t)done - TRACK_BACK : 0;
    if(hi - lo >= TRACK)
        lo = hi - TRACK + 1;

    int64_t d[TRACK_BACK + TRACK];
    uint32_t n = 0, best = lo;
    int64_t best_d = INT64_MAX;
    for(uint32_t k = lo; k <= hi && n < sizeof(d) / sizeof(d[0]); k++, n++) {
        d[n] = 0;
        for(int a = 0; a < RNR_AXES; a++) {
            int64_t c = k ? s->track_cum[k % TRACK][a] : 0;
            d[n] += c > rel[a] ? c - rel[a] : rel[a] - c;
        }
        if(d[n] < best_d) {
            best_d = d[n];
            best = k;
        }
    }

    // tolerance: the counters may be read in the middle of a slice
    int64_t tol = 2;
    if(best) {
        int64_t step = 0;
        for(int a = 0; a < RNR_AXES; a++) {
            int64_t c1 = s->track_cum[best % TRACK][a], c0 = best > 1 ? s->track_cum[(best - 1) % TRACK][a] : 0;
            step += c1 > c0 ? c1 - c0 : c0 - c1;
        }
        if(step > tol)
            tol = step;
    }
    if(best_d > tol) {          // nothing matches (should not happen): leave the estimate alone
        s->track_ambiguous++;
        return;
    }
    for(uint32_t i = 0; i < n; i++) {
        uint32_t k = lo + i;
        if(d[i] <= best_d + tol && (k + 3 < best || k > best + 3)) {
            s->track_ambiguous++;
            return;             // another part of the run matches as well
        }
    }

    s->run_done = best;
    s->run_done_time = now;
    s->track_fixes++;
    // the controller will have run every slice sent then
    s->busy_until = now + (uint64_t)(s->run_slices - best) * SLICE_US;
}

static void on_status (rnr_sink_t *s, const rnr_status_t *st, uint64_t now)
{
    // sequence numbers wrap every 65536 frames (5.5 min of slices): compare once, then remember
    if(s->run_open && !s->run_armed && (int16_t)(st->ack_seq - s->run_arm_seq) >= 0)
        s->run_armed = true;
    track_controller(s, st, now);
    bool covered = (int16_t)(st->ack_seq - s->seq) >= 0;    // every frame sent so far taken over
    // E-stop input, also read by the bridge itself in the raw inputs: the controller stops on its
    // own only when its configuration frame declares it; otherwise it just reports the input. As
    // Mach3, the bridge sends an EStop frame either way.
    bool input_pressed = input_on(&s->opt.inputs.estop, st->inputs);
    bool estop_input = (st->fault_flags & RNR_FAULT_ESTOP) != 0 || input_pressed;
    if(s->armed && !s->arm_seen && (int16_t)(st->ack_seq - s->arm_seq) >= 0)
        s->arm_seen = true;
    bool armed_seen = s->armed && s->arm_seen;

    // E-stop: stop the controller at once (it drops its buffer) and re-arm it before the next
    // motion. A press shorter than a status period may show only as the controller's disarm.
    if(estop_input || (armed_seen && st->state == RnrState_EStop)) {
        if(s->armed) {
            send_short(s, RnrOp_EStop, 0);
            outputs_off(s);
            s->estops_seen++;
            fprintf(stderr, "[rnr] controller e-stop%s at X=%d Y=%d Z=%d A=%d: e-stop sent\n",
                     estop_input ? " input" : " (seen as a disarm)",
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

    if(!s->inputs_known || st->inputs != s->status.inputs)
        fprintf(stderr, "[rnr] inputs IN1..IN12: %c%c%c%c %c%c%c%c %c%c%c%c (0x%04X)\n",
                 '0' + (st->inputs & 1), '0' + (st->inputs >> 1 & 1), '0' + (st->inputs >> 2 & 1), '0' + (st->inputs >> 3 & 1),
                 '0' + (st->inputs >> 4 & 1), '0' + (st->inputs >> 5 & 1), '0' + (st->inputs >> 6 & 1), '0' + (st->inputs >> 7 & 1),
                 '0' + (st->inputs >> 8 & 1), '0' + (st->inputs >> 9 & 1), '0' + (st->inputs >> 10 & 1), '0' + (st->inputs >> 11 & 1),
                 st->inputs);

    // Homing or probing move: the controller has stopped by itself on the switch / the contact
    // (back to ready while the run is open). The slices still to come for this move are dropped.
    if(s->run_open && (s->run_kind == Slice_HomeSeek || s->run_kind == Slice_Probe) &&
        s->run_armed && st->state == RnrState_Ready) {
        bool probe = s->run_kind == Slice_Probe;
        fprintf(stderr, "[rnr] %s%c: controller stopped at X=%d Y=%d Z=%d A=%d\n",
                 probe ? "probe contact" : "home switch ", probe ? ' ' : "XYZA"[s->run_axis & 3],
                 st->live_steps[0], st->live_steps[1], st->live_steps[2], st->live_steps[3]);
        s->run_open = false;
        s->kind_change = false;
        s->busy_until = 0;
        s->check_pending = false;
        rnr_mutex_lock(&s->lock);
        s->cut_active = true;
        s->cut_kind = s->run_kind;
        s->cut_axis = s->run_axis;
        // homing: the slicer resets its step count at rest (take_resync); probing: the end of the
        // probing cycle does it, together with the core's position (get_probe_result)
        if(probe)
            s->probe_contact = true;
        else
            s->resync_pending = true;
        rnr_mutex_unlock(&s->lock);
    }

    rnr_mutex_lock(&s->lock);
    s->inputs_known = true;
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

// Link lost: tries to reach the controller again (at most once per RECONNECT_MS). Once it answers,
// it gets the same frames as at start (configuration, disarmed, outputs off) and the bridge forgets
// everything about the motion in progress: the controller may have been powered off or have run
// part of it. Its counters are taken as they are (the next arming keeps them): the slicer resyncs
// on them, the core is told the machine position is lost.
static bool reconnect_link (rnr_sink_t *s, uint64_t now)
{
    uint8_t rx[RNR_IN_BYTES];
    rnr_status_t st;
    bool got = false;

    if(now < s->next_reconnect)
        return false;
    s->next_reconnect = now + (uint64_t)RECONNECT_MS * 1000u;

    if(!s->opt.reconnect(&s->t) || !s->t)
        return false;

    if(send_configure(s) != RNR_IO_OK || send_short(s, RnrOp_EStop, 0) != RNR_IO_OK ||
        send_pwm(s, 0) != RNR_IO_OK || send_short(s, RnrOp_SetOutputs, 0) != RNR_IO_OK)
        return false;
    for(uint64_t end = rnr_now_us() + CONNECT_WAIT_MS * 1000u; !got && rnr_now_us() < end; ) {
        int r = s->t->read(s->t, rx, 20);
        if(r == RNR_IO_DISCONNECTED)
            return false;
        got = r == RNR_IO_OK && rnr_parse_status(rx, RNR_IN_BYTES, &st);
    }
    if(!got)
        return false;

    s->armed = false;
    s->run_open = false;
    s->awaiting_halt = false;
    s->check_pending = false;
    s->busy_until = 0;
    s->first_queued = 0;
    s->outputs = 0;
    s->pwm = 0;
    s->outputs_change = false;
    s->kind_change = false;
    s->last_status = rnr_now_us();

    rnr_mutex_lock(&s->lock);
    s->fault = false;
    s->link_lost = false;
    s->fault_msg[0] = 0;
    s->qcount = 0;
    s->status = st;
    s->inputs_known = true;
    s->estop_input = (st.fault_flags & RNR_FAULT_ESTOP) != 0;
    s->cut_active = false;
    s->probe_contact = false;
    s->halted = false;
    s->idle_shared = false;
    s->resync_pending = false;  // the slicer takes the controller's counters itself (link_ok)
    s->generation++;
    rnr_mutex_unlock(&s->lock);

    fprintf(stderr, "[rnr] controller reconnected, at X=%d Y=%d Z=%d A=%d: machine position lost, home the machine\n",
             st.live_steps[0], st.live_steps[1], st.live_steps[2], st.live_steps[3]);

    return true;
}

static void io_thread (void *arg)
{
    rnr_sink_t *s = arg;
    uint8_t rx[RNR_IN_BYTES];
    rnr_status_t st = s->status;
    bool awake = false;

    s->last_status = rnr_now_us();

    while(true) {

        uint64_t loop_start = rnr_now_us();
        if(s->run_open && s->last_loop && loop_start - s->last_loop > s->worst_gap)
            s->worst_gap = loop_start - s->last_loop;
        s->last_loop = loop_start;

        // A job in progress must not be stopped by the computer going to sleep
        bool busy = s->run_open || s->outputs || s->pwm;
        if(busy != awake)
            rnr_keep_awake(awake = busy);

        if(atomic_load(&s->disarm_request)) {
            if(s->t) {
                send_short(s, RnrOp_EStop, 0);
                outputs_off(s);
            }
            s->armed = false;
            fprintf(stderr, "[rnr] program ending: controller disarmed, outputs off\n");
            atomic_store(&s->disarm_done, true);    // the process may end as soon as this is set
            break;
        }

        if(s->opt.reconnect) {
            rnr_mutex_lock(&s->lock);
            bool lost = s->link_lost, quit = s->quit;
            if(lost)
                s->stop_request = false;    // nothing to stop: the controller is out of reach
            rnr_mutex_unlock(&s->lock);
            if(lost && !s->armed) {         // (disarmed once: see the fault handling below)
                if(quit)
                    break;
                if(!reconnect_link(s, rnr_now_us()))
                    rnr_sleep_ms(20);
                continue;
            }
        }

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
        bool quit = s->quit, fault = s->fault, stop = s->stop_request, stop_jog = s->stop_jog_request;
        s->stop_request = false;
        s->stop_jog_request = false;
        if(stop_jog && !fault && !stop && s->run_open && s->run_kind == Slice_Jog)
            s->qcount = 0;          // what is queued is the jog at full speed: the ramp comes next
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
        } else if(stop_jog && s->armed && s->run_open && s->run_kind == Slice_Jog) {
            // As Mach3 when a jog key is released: the controller drops what it holds and runs the
            // deceleration slices sent after this. It has nothing left: the pacing restarts from there.
            send_short(s, RnrOp_StopJog, 0);
            s->run_cut = true;
            s->run_done = s->run_slices;
            s->run_done_time = now;
            s->run_done_valid = true;
            s->busy_until = now;
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
            if(s->run_open && s->run_done_valid) {
                // closed loop: slices ahead of where the controller is now (estimated)
                double done = controller_done(s, now);
                if((double)s->run_slices - done >= RNR_LEAD)
                    break;
            } else if(s->busy_until > now + RNR_LEAD * SLICE_US - SLICE_US / 2)
                break;  // far enough ahead of real time (start of a run: no status yet)
            if((int16_t)(s->seq - st.ack_seq) > RNR_MAX_UNACKED)
                break;  // safety limit on unacknowledged frames

            slice_t slice;
            bool dropped = false;
            rnr_mutex_lock(&s->lock);
            bool got = s->qcount > 0 && !s->estop_input;
            if(got) {
                slice = s->queue[s->qhead];
                if(s->cut_active && slice.kind == s->cut_kind && slice.axis == s->cut_axis) {
                    // the rest of a move the controller has cut short: it would ignore it anyway
                    s->qhead = (s->qhead + 1) % RNR_QUEUE;
                    s->qcount--;
                    dropped = true;
                } else
                    s->cut_active = false;  // next move: the inputs alone tell the switch state again
            }
            rnr_mutex_unlock(&s->lock);
            if(!got) {
                if(s->run_open && !s->queue_empty_since)
                    s->queue_empty_since = now;
                break;
            }
            if(s->queue_empty_since) {
                if(now - s->queue_empty_since > s->worst_wait)
                    s->worst_wait = now - s->queue_empty_since;
                s->queue_empty_since = 0;
            }
            if(dropped) {
                s->dropped_after_cut++;
                continue;
            }

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

            if(s->run_open && (slice.kind != s->run_kind || slice.axis != s->run_axis)) {
                // another kind of move (homing phase, probing, program): it needs its own run
                s->kind_change = true;
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
            if(!s->run_open && !(st.state == RnrState_Ready && (int16_t)(st.ack_seq - s->seq) >= 0))
                break;  // a new run only once the controller is ready (the previous run over), as Mach3
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
                s->run_kind = slice.kind;
                s->run_slices = 0;
                s->run_done = 0;
                s->run_done_valid = false;
                s->run_t0 = now;
                s->run_axis = slice.axis;
                s->worst_gap = 0;
                s->worst_wait = 0;
                s->queue_empty_since = 0;
                s->min_lead = 1e9;
                s->underruns = 0;
                rnr_opcode_t op = slice.kind == Slice_HomeSeek ? RnrOp_ArmHome :
                                  slice.kind == Slice_HomeJog || slice.kind == Slice_Jog ? RnrOp_StartJog :
                                  slice.kind == Slice_Probe ? RnrOp_ArmProbe : 0;
                if(op && send_short(s, op, slice.kind == Slice_Probe ? 0 : slice.axis) != RNR_IO_OK) {
                    set_fault(s, "write failed", false);
                    break;
                }
                s->run_arm_seq = s->seq;
                s->run_armed = false;
                s->run_cut = false;
            }

            uint8_t buf[RNR_OUT_BYTES];
            rnr_build_slice(buf, slice.kind == Slice_Program ? RnrOp_GcodeSlice : RnrOp_JogSlice, 0, slice.steps, s->outputs);
            if(send_frame(s, buf) != RNR_IO_OK) {
                set_fault(s, "slice write failed", false);
                break;
            }
            for(int a = 0; a < RNR_AXES; a++)
                s->run_sent[a] += slice.steps[a];
            if(s->run_done_valid && !s->run_cut) {
                double lead = (double)s->run_slices - controller_done(s, now);
                if(lead < s->min_lead)
                    s->min_lead = lead;
                if(lead <= 0.0)
                    s->underruns++;
            }
            s->run_slices++;
            memcpy(s->track_cum[s->run_slices % TRACK], s->run_sent, sizeof(s->run_sent));
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
        if(s->run_open && (empty || s->outputs_change || s->kind_change) && now > s->busy_until + (uint64_t)REST_MS * 1000u &&
            (int16_t)(st.ack_seq - s->last_slice_seq) >= 0) {
            send_short(s, RnrOp_EndOfMotion, s->outputs);
            report_run(s);
            s->run_open = false;
            s->kind_change = false;
            s->outputs_change = false;
            s->check_pending = !s->run_cut;     // (a jog stopped short: the slicer takes the controller's position)
            s->check_seq = s->seq;
            for(int a = 0; a < RNR_AXES; a++)
                s->check_expected[a] = s->run_start[a] + s->run_sent[a];
            s->check_kind = s->run_kind;
            s->check_slices = s->run_slices;
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

        bool covered = (int16_t)(st.ack_seq - s->seq) >= 0;
        rnr_mutex_lock(&s->lock);
        s->run_open_shared = s->run_open;
        s->idle_shared = s->qcount == 0 && !s->run_open && covered && st.state == RnrState_Ready;
        rnr_mutex_unlock(&s->lock);
    }

    rnr_keep_awake(false);
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
        // Not at rest any more, at once: the I/O thread only recomputes this flag once per loop,
        // and the core may queue a whole move then ask at once (homing pull-off: wait_idle, then
        // the switch is read; a stale "at rest" read the switch before the pull-off had run).
        s->idle_shared = false;
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
    s->cut_active = false;
    s->resync_pending = false;
    s->probe_contact = false;
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

static bool rnr_wait_idle (slice_sink_t *sink, uint32_t timeout_ms)
{
    rnr_sink_t *s = sink->ctx;
    uint64_t deadline = rnr_now_us() + (uint64_t)timeout_ms * 1000u;
    bool idle = false, give_up = false;

    while(!idle && !give_up && rnr_now_us() < deadline) {
        rnr_mutex_lock(&s->lock);
        idle = s->idle_shared && !s->stop_request;
        give_up = s->fault || s->link_lost || s->blocked;
        rnr_mutex_unlock(&s->lock);
        if(!idle && !give_up)
            rnr_sleep_ms(2);
    }

    return idle;
}

static bool rnr_take_resync (slice_sink_t *sink, int32_t position[SLICE_AXES])
{
    rnr_sink_t *s = sink->ctx;

    rnr_mutex_lock(&s->lock);
    bool ok = s->resync_pending && s->idle_shared && !s->stop_request;
    if(ok) {
        s->resync_pending = false;
        for(int a = 0; a < SLICE_AXES; a++)
            position[a] = a < RNR_AXES ? s->status.live_steps[a] : 0;
    }
    rnr_mutex_unlock(&s->lock);

    return ok;
}

static bool rnr_get_probe_result (slice_sink_t *sink, int32_t position[SLICE_AXES], int32_t latched[SLICE_AXES],
                                   bool *contact, uint32_t timeout_ms)
{
    rnr_sink_t *s = sink->ctx;
    uint64_t deadline = rnr_now_us() + (uint64_t)timeout_ms * 1000u;
    bool ok = false;

    while(true) {
        rnr_mutex_lock(&s->lock);
        ok = s->idle_shared && !s->stop_request && !s->fault;
        if(ok) {
            *contact = s->probe_contact;
            s->probe_contact = false;
            s->resync_pending = false;
            for(int a = 0; a < SLICE_AXES; a++) {
                position[a] = a < RNR_AXES ? s->status.live_steps[a] : 0;
                latched[a] = a < RNR_AXES ? s->status.latched_steps[a] : 0;
            }
        }
        bool give_up = s->fault || s->link_lost;
        rnr_mutex_unlock(&s->lock);
        if(ok || give_up || rnr_now_us() >= deadline)
            break;
        rnr_sleep_ms(2);
    }

    return ok;
}

static bool rnr_get_inputs (slice_sink_t *sink, sink_inputs_t *in)
{
    rnr_sink_t *s = sink->ctx;
    const rnr_inputs_t *cfg = &s->opt.inputs;

    rnr_mutex_lock(&s->lock);
    bool known = s->inputs_known;
    uint16_t raw = s->status.inputs;
    rnr_mutex_unlock(&s->lock);

    memset(in, 0, sizeof(*in));
    for(int a = 0; a < RNR_AXES; a++) {
        in->home |= (uint8_t)(input_on(&cfg->home[a], raw) << a);
        in->limit_pos |= (uint8_t)(input_on(&cfg->limit_pos[a], raw) << a);
        in->limit_neg |= (uint8_t)(input_on(&cfg->limit_neg[a], raw) << a);
    }
    in->probe = input_on(&cfg->probe, raw);
    rnr_mutex_lock(&s->lock);
    if(s->cut_active) {
        // the controller stopped on it: it is triggered, whatever the next status says
        if(s->cut_kind == Slice_HomeSeek)
            in->home |= (uint8_t)(1u << s->cut_axis);
        else if(s->cut_kind == Slice_Probe)
            in->probe = true;
    }
    rnr_mutex_unlock(&s->lock);

    return known;
}

static bool rnr_estop_active (slice_sink_t *sink)
{
    rnr_sink_t *s = sink->ctx;

    rnr_mutex_lock(&s->lock);
    bool active = s->estop_input || rnr_now_us() < s->estop_until;
    rnr_mutex_unlock(&s->lock);

    return active;
}

static bool rnr_link_ok (slice_sink_t *sink, uint32_t *generation)
{
    rnr_sink_t *s = sink->ctx;

    rnr_mutex_lock(&s->lock);
    bool ok = !s->link_lost;
    *generation = s->generation;
    rnr_mutex_unlock(&s->lock);

    return ok;
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
    if(s->t) {
        send_short(s, RnrOp_EStop, 0);
        outputs_off(s);
    }
    fprintf(stderr, "[rnr] closed: %u slices sent, %u controller e-stops, %u slices dropped after a switch or contact, position checks after runs: %u ok, %u mismatch; controller located %u times (%u reports ambiguous)\n",
             s->slices_sent, s->estops_seen, s->dropped_after_cut, s->checks_ok, s->checks_bad, s->track_fixes, s->track_ambiguous);

    if(s->t)
        s->t->close(s->t);
    rnr_mutex_destroy(&s->lock);
    free(s);
    free(sink);
}

static void rnr_stop_jog (slice_sink_t *sink)
{
    rnr_sink_t *s = sink->ctx;

    rnr_mutex_lock(&s->lock);
    s->stop_jog_request = true;
    rnr_mutex_unlock(&s->lock);
}

static void rnr_disarm (slice_sink_t *sink, uint32_t timeout_ms)
{
    rnr_sink_t *s = sink->ctx;
    uint64_t deadline = rnr_now_us() + (uint64_t)timeout_ms * 1000u;

    atomic_store(&s->disarm_request, true);
    while(!atomic_load(&s->disarm_done) && rnr_now_us() < deadline)
        rnr_sleep_ms(1);
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
    sink->get_inputs = rnr_get_inputs;
    sink->take_resync = rnr_take_resync;
    sink->wait_idle = rnr_wait_idle;
    sink->get_probe_result = rnr_get_probe_result;
    sink->link_ok = rnr_link_ok;
    sink->disarm = rnr_disarm;
    sink->stop_jog = rnr_stop_jog;
    sink->ctx = s;

    return sink;
}
