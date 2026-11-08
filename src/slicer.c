/*
  slicer.c - cuts the simulator's step pulses into SLICE_MS slices

  Steps are counted at the source (driver.c, on every pulse) and stored in the
  slice matching their simulated time: slice n = [n, n+1) * SLICE_MS. Steps are
  never derived from sys.position, which the core overwrites during homing.

  While the stepper timer runs, every slice is emitted, even an empty one, so
  that the controller keeps the same timeline as the simulator. When idle,
  nothing is emitted.

  Back-pressure: when the sink is full, the simulator thread waits, so
  simulated time cannot run ahead of the machine.

  Position resync: the simulator runs a little ahead of the machine (queued
  slices). When the core resets during motion (soft reset, alarm, e-stop), the
  sink stops the machine and reports where it actually stopped. sys.position
  is then set to that position plus the offset between sys.position and the
  counted steps, sampled at rest, before the core syncs its planner and parser
  to it. The offset is resampled after the core overwrites sys.position
  (homing).

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>

#include "slicer.h"
#include "irq_lock.h"
#include "simulator.h"
#include "mcu.h"
#include "driver.h"

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/system.h"
#include "grbl/gcode.h"
#include "grbl/planner.h"
#include "grbl/machine_limits.h"

#define SLICE_TICKS ((uint64_t)F_CPU / 1000 * SLICE_MS)
#define HALT_TIMEOUT_MS 200

static slice_sink_t *sink = NULL;
static slice_t cur;
static volatile bool open = false;  // a slice is in progress (also read by the core thread: homing_get_state)
static atomic_bool sink_failed;      // the sink reported an error: slices are dropped, motor fault signalled
static atomic_bool reset_pending;   // set by the core's on_reset (any thread), handled on the simulator thread
static on_reset_ptr prev_on_reset = NULL;
static driver_reset_ptr prev_driver_reset = NULL;
static int32_t counted[SLICE_AXES]; // steps counted since start: the machine position if every slice ran
static int32_t idle_offset[N_AXIS]; // sys.position - counted, sampled at rest (see advance())
static atomic_bool resync_busy;     // from a reset until its resync is done: idle_offset is frozen
static atomic_uint sim_heartbeat;   // incremented on every simulator tick
static atomic_uint out_state;       // current outputs: flags | spindle_pwm << 16 (set from any thread)
static atomic_uint phase;           // kind of motion (slice_kind_t) | axis << 8, set by the core's hooks
static on_homing_rate_set_ptr prev_on_homing_rate_set = NULL;
static on_homing_completed_ptr prev_on_homing_completed = NULL;
static home_get_state_ptr prev_homing_get_state = NULL;

// Puts sys.position back where the machine is: machine position + offset sampled at rest.
// The offset is not taken at the time of the reset: the core adds a step to sys.position one
// stepper interrupt before the pulse goes out, so a reset can leave sys.position one step ahead
// of the pulses that were actually emitted.
static void resync_position (const int32_t machine[SLICE_AXES])
{
    int32_t delta[SLICE_AXES] = {0};
    bool moved = false;

    for(int i = 0; i < SLICE_AXES && i < N_AXIS; i++) {
        int32_t target = machine[i] + idle_offset[i];
        delta[i] = target - sys.position[i];
        counted[i] = machine[i];
        if(delta[i]) {
            sys.position[i] = target;
            moved = true;
        }
    }

    if(moved)
        fprintf(stderr, "[slicer] position resynced from the machine: X%+d Y%+d Z%+d A%+d steps\n",
                 delta[0], delta[1], delta[2], delta[3]);
}

// Stops the machine: blocks the sink (pending slices must not run) and tells the simulator
// thread to drop its slice in progress. Safe from any thread, may be called twice per reset.
static void stop_machine (void)
{
    slicer_set_phase(Slice_Program, 0);
    atomic_store(&resync_busy, true);       // freeze idle_offset until the resync is done
    sink->stop(sink);
    atomic_store(&reset_pending, true);
}

// Core reset, early warning: grbl.on_reset is called at the end of mc_reset() (interrupt
// context). Stop the machine at once. In the simulator, mc_reset() runs on another thread than
// the core and can return early without calling on_reset (it sets EXEC_RESET, the core thread
// sees it and sets sys.reset_pending, which mc_reset() then reads): on_driver_reset() below is
// the reliable path, this one only saves a few milliseconds when it happens.
static void on_core_reset (void)
{
    if(sink)
        stop_machine();

    if(prev_on_reset)
        prev_on_reset();
}

// Core reset, reliable path: hal.driver_reset() is called on the core thread whenever it handles
// a reset (protocol_exec_rt_system), before its re-initialisation syncs the planner and parser to
// sys.position. Stops the steppers and the machine, waits until both are at rest, then puts
// sys.position back where the machine actually stopped.
static void on_driver_reset (void)
{
    if(sink) {
        int32_t machine[SLICE_AXES];
        uint32_t beat;

        stop_machine();

        // mc_reset() may not have stopped the simulated steppers (see on_core_reset): stop the
        // stepper timer like the core does from its thread. With the interrupt lock (irq_lock.h)
        // no stepper interrupt can be running while this core code runs. Without it, wait until
        // the simulator thread has gone round its loop twice, so that no stepper interrupt is
        // still updating sys.position. Nothing else runs on the core thread meanwhile.
        hal.stepper.go_idle(false);
        if(!irq_lock_enabled()) {
            beat = atomic_load(&sim_heartbeat);
            for(int i = 0; i < HALT_TIMEOUT_MS && (timer[STEPPER_TIMER].enable || atomic_load(&sim_heartbeat) - beat < 2); i++)
                platform_sleep(1000);
        }

        if(sink->get_halted_position(sink, machine, HALT_TIMEOUT_MS)) {
            resync_position(machine);
            sync_position();    // the core syncs again from sys.position in its reset sequence
        } else
            fprintf(stderr, "[slicer] machine position unknown after stop: position not resynced\n");

        atomic_store(&resync_busy, false);
    }

    if(prev_driver_reset)
        prev_driver_reset();
}

void slicer_set_phase (slice_kind_t kind, int axis)
{
    atomic_store(&phase, (unsigned int)kind | (unsigned int)(axis & 0xFF) << 8);
}

// The core reads the home switches. At the end of a pull-off it checks at once that the switch
// has released, as on a microcontroller where motion and switch are in sync. Here the machine still
// has up to RNR_LEAD_MS of that motion to run, plus a status period before its inputs are known:
// wait until it has run it all, else a switch with some hysteresis still reads engaged.
static home_signals_t homing_get_state (void)
{
    bool pulloff = (atomic_load(&phase) & 0xFF) == Slice_HomeJog, moving = timer[STEPPER_TIMER].enable;

    if(pulloff && !moving && sink && sink->wait_idle) {
        for(int i = 0; i < 100 && open; i++)   // the last slice of the move must reach the sink first
            platform_sleep(1000);
        if(!sink->wait_idle(sink, 2000))
            fprintf(stderr, "[slicer] machine not at rest after a homing pull-off: switch state may be stale\n");
    }

    home_signals_t state = prev_homing_get_state();
    if(pulloff && state.a.mask)
        fprintf(stderr, "[slicer] end of a homing pull-off: switch still engaged (mask 0x%02X%s)\n",
                 state.a.mask, moving ? ", read while the pull-off was still running" : "");

    return state;
}

// Homing phase starting: seek and locate move towards the switch, pull-off away from it. With one
// axis, the machine can stop by itself on its switch; with several, the core stops each axis when
// it sees its switch in the machine's inputs (less precise: status reports come about every 50 ms).
static void on_homing_rate_set (axes_signals_t axes, coord_data_t *feedrate, homing_mode_t mode)
{
    static bool warned = false;
    int n = 0, axis = 0;

    for(int i = 0; i < SLICE_AXES; i++)
        if(axes.mask & (1u << i)) {
            n++;
            axis = i;
        }

    if(hal.homing.get_state && hal.homing.get_state != homing_get_state) {
        prev_homing_get_state = hal.homing.get_state;
        hal.homing.get_state = homing_get_state;
    }

    // No motion runs between two phases (the core has stopped the steppers): a cycle-complete
    // still pending belongs to the previous phase. That happens when the machine stops on its
    // switch: the bridge drops the rest of the seek, the core's simulated seek then runs to its end
    // at once and may flag cycle complete just as the core leaves the seek loop. The pull-off loop
    // would take the stale flag for its own end and check the switch before any pull-off: ALARM:8.
    if(sys.rt_exec_state & EXEC_CYCLE_COMPLETE) {
        system_clear_exec_state_flag(EXEC_CYCLE_COMPLETE);
        fprintf(stderr, "[slicer] homing: stale cycle-complete from the previous phase cleared\n");
    }

    if(n == 1)
        slicer_set_phase(mode == HomingMode_Pulloff ? Slice_HomeJog : Slice_HomeSeek, axis);
    else {
        slicer_set_phase(Slice_Program, 0);
        if(!warned && mode == HomingMode_Seek) {
            warned = true;
            fprintf(stderr, "[slicer] homing several axes at once: switches read from status reports, "
                             "less precise than one axis per cycle ($44..$46)\n");
        }
    }

    if(prev_on_homing_rate_set)
        prev_on_homing_rate_set(axes, feedrate, mode);
}

static void on_homing_completed (axes_signals_t cycle, bool success)
{
    slicer_set_phase(Slice_Program, 0);

    if(prev_on_homing_completed)
        prev_on_homing_completed(cycle, success);
}

// End of a probing cycle (core thread, before the core syncs its planner to sys.position): the
// machine stopped by itself on contact, where the core has not seen it yet (status reports come
// about every 50 ms). Put sys.position where the machine is and the probe position where it touched.
void slicer_probe_end (void)
{
    int32_t machine[SLICE_AXES], latched[SLICE_AXES];
    bool contact = false;

    if((atomic_load(&phase) & 0xFF) != Slice_Probe)
        return;
    slicer_set_phase(Slice_Program, 0);

    if(!sink || !sink->get_probe_result)
        return;

    atomic_store(&resync_busy, true);
    if(sink->get_probe_result(sink, machine, latched, &contact, 1000)) {
        for(int i = 0; i < SLICE_AXES && i < N_AXIS; i++) {
            sys.position[i] = machine[i] + idle_offset[i];
            counted[i] = machine[i];
            if(contact)
                sys.probe_position[i] = latched[i] + idle_offset[i];
        }
        if(contact)
            fprintf(stderr, "[slicer] probe position from the machine: X%d Y%d Z%d steps\n",
                     sys.probe_position[0], sys.probe_position[1], sys.probe_position[2]);
    } else
        fprintf(stderr, "[slicer] machine position unknown after probing: not resynced\n");
    atomic_store(&resync_busy, false);
}

void slicer_attach_core (void)
{
    if(grbl.on_homing_rate_set != on_homing_rate_set) {
        prev_on_homing_rate_set = grbl.on_homing_rate_set;
        grbl.on_homing_rate_set = on_homing_rate_set;
    }
    if(grbl.on_homing_completed != on_homing_completed) {
        prev_on_homing_completed = grbl.on_homing_completed;
        grbl.on_homing_completed = on_homing_completed;
    }
    if(grbl.on_reset != on_core_reset) {
        prev_on_reset = grbl.on_reset;
        grbl.on_reset = on_core_reset;
    }
    if(hal.driver_reset != on_driver_reset) {
        prev_driver_reset = hal.driver_reset;
        hal.driver_reset = on_driver_reset;
    }
}

bool slicer_enabled (void)
{
    return sink != NULL;
}

bool slicer_motor_fault (void)
{
    return atomic_load(&sink_failed);
}

bool slicer_estop (void)
{
    slice_sink_t *s = sink;

    return s && s->estop_active && s->estop_active(s);
}

bool slicer_get_inputs (sink_inputs_t *in)
{
    slice_sink_t *s = sink;

    return s && s->get_inputs && s->get_inputs(s, in);
}

// The machine stopped on its own e-stop input: tell the core like a control input interrupt
// would (it resets and raises ALARM:10, which stays until the input is released and the user
// resets). Runs on the simulator thread, as an interrupt handler: exclusive with the core
// thread under the interrupt lock.
static void check_estop (void)
{
    static bool was_active = false;
    bool active = slicer_estop();

    if(active && !was_active) {
        fprintf(stderr, "[slicer] machine e-stop: signalled to the core\n");
        bool locked = irq_lock_isr_enter();
        hal.control.interrupt_callback(hal.control.get_state());
        irq_lock_isr_leave(locked);
    }
    was_active = active;
}

void slicer_init (slice_sink_t *s)
{
    sink = s;
    open = false;
}

static void emit (void)
{
    sink_status_t status;

    if(atomic_load(&sink_failed))
        return;

    unsigned int o = atomic_load(&out_state);
    cur.out.flags = (uint8_t)o;
    cur.out.spindle_pwm = (uint16_t)(o >> 16);

    while((status = sink->push(sink, &cur)) == Sink_Full) {
        platform_sleep(1000);   // back-pressure: simulated time waits for the machine
        if(atomic_load(&reset_pending))
            return;
    }

    if(status == Sink_Error) {
        // The machine can no longer follow: report a motor fault, like a driver fault input would.
        // mc_reset() reads it through hal.control.get_state() and raises ALARM:17.
        atomic_store(&sink_failed, true);
        fprintf(stderr, "[slicer] sink '%s' failed: motor fault, reset\n", sink->name);
        mc_reset();
    }
}

// Starts a slice: it belongs to the kind of motion in progress
static void open_slice (uint32_t index)
{
    unsigned int p = atomic_load(&phase);

    memset(&cur, 0, sizeof(cur));
    cur.index = index;
    cur.kind = (uint8_t)p;
    cur.axis = (uint8_t)(p >> 8);
    open = true;
}

static void advance (void)
{
    uint32_t idx = (uint32_t)(sim.masterclock / SLICE_TICKS);
    bool motion = timer[STEPPER_TIMER].enable;

    if(atomic_exchange(&reset_pending, false)) {
        open = false;           // drop the slice in progress (its steps are in the resync)
        // sink_failed stays latched: the sink has no way to recover yet (TODO: reconnect), so the
        // motor fault keeps grblHAL in ALARM:17 until the bridge is restarted.
        sink->resume(sink);     // from now on, slices belong to motion started after the reset
    }

    if(!open) {
        if(motion)
            open_slice(idx);
        else if(!atomic_load(&resync_busy)) {
            // A homing or probing move cut short by the machine (switch, contact): the steps it
            // dropped are in counted. The machine is at rest now: take its position.
            int32_t machine[SLICE_AXES];
            if(sink->take_resync && sink->take_resync(sink, machine)) {
                for(int i = 0; i < SLICE_AXES; i++)
                    counted[i] = machine[i];
            }
            // At rest every pulse has gone out: sys.position - counted is exact here. It changes
            // only when the core overwrites sys.position (homing), and is resampled after that.
            for(int i = 0; i < N_AXIS && i < SLICE_AXES; i++)
                idle_offset[i] = sys.position[i] - counted[i];
        }
        return;
    }

    // Close expired slices. While moving, intermediate slices are emitted
    // empty; when idle, stop after the last slice that was started.
    while(open && cur.index < idx) {
        emit();
        uint32_t next = cur.index + 1;
        open_slice(next);
        open = motion;
    }
}

void slicer_step (uint32_t step_bits, uint32_t dir_bits)
{
    if(!sink)
        return;

    advance();

    if(!open)       // pulse outside of a declared motion: open a slice anyway
        open_slice((uint32_t)(sim.masterclock / SLICE_TICKS));
    else {
        // The kind of motion changed within the slice (homing seek -> pull-off): the slice so far
        // belongs to the previous move
        unsigned int p = atomic_load(&phase);
        if(cur.kind != (uint8_t)p || cur.axis != (uint8_t)(p >> 8)) {
            emit();
            open_slice(cur.index);
        }
    }

    for(int i = 0; i < SLICE_AXES && i < N_AXIS; i++) {
        if(step_bits & (1u << i)) {
            int32_t step = (dir_bits & (1u << i)) ? -1 : 1;
            cur.steps[i] += step;
            counted[i] += step;
        }
    }
}

void slicer_poll (void)
{
    atomic_fetch_add(&sim_heartbeat, 1);

    if(sink) {
        check_estop();
        advance();
    }
}

void slicer_close (void)
{
    if(!sink)
        return;

    if(open)
        emit();
    open = false;

    sink->close(sink);
    sink = NULL;
}

// Output changes come from the core thread (spindle, coolant) or from the stepper interrupt
// (laser mode PWM): each one updates its part of the state and tells the sink.
static void update_outputs (uint8_t flags_mask, uint8_t flags, bool set_pwm, uint16_t pwm)
{
    unsigned int o = atomic_load(&out_state), n;

    do {
        n = (o & ~(unsigned int)flags_mask) | (flags & flags_mask);
        if(set_pwm)
            n = (n & 0xFFFFu) | ((unsigned int)pwm << 16);
    } while(!atomic_compare_exchange_weak(&out_state, &o, n));

    slice_sink_t *s = sink;
    if(n != o && s && s->set_outputs)
        s->set_outputs(s, (slice_outputs_t){ .flags = (uint8_t)n, .spindle_pwm = (uint16_t)(n >> 16) });
}

void slicer_set_spindle (bool on, bool ccw, uint16_t pwm)
{
    update_outputs(SLICE_OUT_SPINDLE | SLICE_OUT_SPINDLE_CCW,
                    (on ? SLICE_OUT_SPINDLE : 0) | (on && ccw ? SLICE_OUT_SPINDLE_CCW : 0), true, on ? pwm : 0);
}

void slicer_set_spindle_pwm (uint16_t pwm)
{
    update_outputs(0, 0, true, pwm);
}

void slicer_set_coolant (bool flood, bool mist)
{
    update_outputs(SLICE_OUT_FLOOD | SLICE_OUT_MIST, (flood ? SLICE_OUT_FLOOD : 0) | (mist ? SLICE_OUT_MIST : 0), false, 0);
}
