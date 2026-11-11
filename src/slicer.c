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
#include <stdlib.h>
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
#include "grbl/report.h"
#include "grbl/state_machine.h"

#define SLICE_TICKS ((uint64_t)F_CPU / 1000 * SLICE_MS)
#define HALT_TIMEOUT_MS 200
#define SAFE_STOP_TIMEOUT_MS 10000  // controlled stop: time allowed for the machine to come to rest

static atomic_bool safe_stop_wanted; // see slicer_safe_stop_poll()
static atomic_bool core_stalled;     // see on_lock_timeout()
static slice_sink_t *sink = NULL;
static slice_t cur;
static volatile bool open = false;  // a slice is in progress (also read by the core thread: homing_get_state)
static atomic_bool sink_failed;      // the sink reported an error: slices are dropped, motor fault signalled
static atomic_bool reconnected;      // the sink is back after a link loss: slicer_core_poll resyncs and clears the fault
static atomic_bool reset_pending;   // set by the core's on_reset (any thread), handled on the simulator thread
static on_reset_ptr prev_on_reset = NULL;
static driver_reset_ptr prev_driver_reset = NULL;
static on_state_change_ptr prev_on_state_change = NULL;
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
    // The stop command (0x19, gSender's Stop button) calls this too, but it is not a reset: the core
    // goes on and brings the motion to a stop on a ramp. Stopping the machine here would make that
    // an abrupt stop and leave the core waiting for steppers that no longer run.
    bool just_a_stop = sys.cancel && !sys.abort && !sys.reset_pending && !(sys.rt_exec_state & EXEC_RESET);

    if(sink && !just_a_stop) {
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

// Simulator thread, waiting for the interrupt lock: the core has held it far too long. At rest
// that is harmless (a slow disk while saving settings). During motion the core no longer follows
// what the machine does: stop the machine now, through the USB thread, which needs no lock.
// The alarm is raised by slicer_core_poll once the core runs again.
static void on_lock_timeout (void)
{
    if(!sink || !(timer[STEPPER_TIMER].enable || open))
        return;

    fprintf(stderr, "[slicer] the core stalled during motion: machine stopped\n");
    atomic_store(&sink_failed, true);   // no slice goes out any more
    stop_machine();
    atomic_store(&core_stalled, true);
}

// Fake controller only (tests): the core keeps the interrupt lock that long, once, during a move
// or, with GRBL2MACH_FAKE_CORE_STALL_REST, at rest.
static void test_core_stall (void)
{
    static int ms = -1;
    static bool at_rest;
    static uint32_t since = 0;

    if(ms < 0) {
        const char *e = getenv("GRBL2MACH_FAKE_CORE_STALL_MS");
        ms = e ? atoi(e) : 0;
        at_rest = getenv("GRBL2MACH_FAKE_CORE_STALL_REST") != NULL;
    }
    if(ms && state_get() == (at_rest ? STATE_IDLE : STATE_CYCLE)) {
        if(!since)
            since = hal.get_elapsed_ticks();
        else if(hal.get_elapsed_ticks() - since > 800) {
            platform_sleep((long)ms * 1000);
            ms = 0;
        }
    }
}

// A jog of one axis starts: the machine can run it as a jog, and stop it at once (slicer_jog_poll)
static void on_state_change (sys_state_t state)
{
    if(state == STATE_JOG) {
        plan_block_t *block = plan_get_current_block();
        int n = 0, axis = 0;

        for(int i = 0; block && i < SLICE_AXES && i < N_AXIS; i++)
            if(block->steps.value[i]) {
                n++;
                axis = i;
            }
        if(n == 1)
            slicer_set_phase(Slice_Jog, axis);
    } else if((atomic_load(&phase) & 0xFF) == Slice_Jog)
        slicer_set_phase(Slice_Program, 0);

    if(prev_on_state_change)
        prev_on_state_change(state);
}

// Core thread: waits until the last slice has reached the sink and the machine has run everything.
// The interrupt lock is given up while waiting: the slice can only come from the simulator thread,
// and keeping the lock that long would make the simulator give up on it for good (irq_lock.c).
static bool wait_machine_idle (uint32_t timeout_ms)
{
    for(int i = 0; i < 100 && open; i++) {
        platform_sleep(1000);
        irq_lock_core_yield(true);
    }

    for(uint32_t waited = 0; waited < timeout_ms && !atomic_load(&sink_failed); waited += 2) {
        if(sink->wait_idle(sink, 2))
            return true;
        irq_lock_core_yield(true);
    }

    return false;
}

// The core reads the home switches. At the end of a pull-off it checks at once that the switch
// has released, as on a microcontroller where motion and switch are in sync. Here the machine still
// has up to RNR_LEAD_MS of that motion to run, plus a status period before its inputs are known:
// wait until it has run it all, else a switch with some hysteresis still reads engaged.
static home_signals_t homing_get_state (void)
{
    bool pulloff = (atomic_load(&phase) & 0xFF) == Slice_HomeJog, moving = timer[STEPPER_TIMER].enable;

    if(pulloff && !moving && sink && sink->wait_idle) {
        if(!wait_machine_idle(2000))
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
    bool known = false;
    for(int waited = 0; waited < 1000 && !known; waited += 2) {     // (lock given up: see wait_machine_idle)
        known = sink->get_probe_result(sink, machine, latched, &contact, 2);
        if(!known)
            irq_lock_core_yield(true);
    }
    if(known) {
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
    if(grbl.on_state_change != on_state_change) {
        prev_on_state_change = grbl.on_state_change;
        grbl.on_state_change = on_state_change;
    }
    irq_lock_set_timeout_handler(on_lock_timeout);
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

bool slicer_link_ok (void)
{
    slice_sink_t *s = sink;
    uint32_t g;

    return !s || !s->link_ok || s->link_ok(s, &g);
}

uint32_t slicer_core_state (void)
{
    return (uint32_t)state_get();
}

uint32_t slicer_core_alarm (void)
{
    return (uint32_t)sys.alarm;
}

bool slicer_can_reconnect (void)
{
    slice_sink_t *s = sink;

    return s && s->reconnect;
}

bool slicer_reconnect (void)
{
    slice_sink_t *s = sink;

    return s && s->reconnect && s->reconnect(s);
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
        // sink_failed stays latched until the sink reports its link back (check_link): the motor
        // fault keeps grblHAL in ALARM:17 meanwhile.
        sink->resume(sink);     // from now on, slices belong to motion started after the reset
    }

    if(!open) {
        if(motion)
            open_slice(idx);
        else if(!atomic_load(&resync_busy) && !atomic_load(&sink_failed)) {
            // (Link lost: the motion was cut anywhere, sys.position - counted means nothing until
            // the position is taken from the machine again.)
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

// Link to the machine (sinks that report it): lost, the core gets a motor fault (ALARM:17) right
// away, even at rest; back again (reconnected: the machine may have reset), the fault clears and
// the core thread takes the machine's position (slicer_core_poll). The core stays in alarm: the
// user unlocks or homes.
static void check_link (void)
{
    static bool known = false;
    static uint32_t generation;
    uint32_t g;

    if(!sink->link_ok)
        return;

    bool ok = sink->link_ok(sink, &g);
    if(!known) {
        known = true;
        generation = g;
    }

    if(!ok) {
        if(!atomic_exchange(&sink_failed, true)) {
            fprintf(stderr, "[slicer] link to the machine lost: motor fault, reset\n");
            mc_reset();
        }
    } else if(g != generation) {
        generation = g;
        if(atomic_load(&sink_failed)) {
            fprintf(stderr, "[slicer] machine reachable again: motor fault cleared\n");
            atomic_store(&reconnected, true);   // sink_failed is cleared by slicer_core_poll
        }
    }
}

// Core thread (realtime hook). After a reconnection, once the core is at rest in alarm (the reset
// that followed the loss is over): sys.position is put where the machine's counters say, the
// motor fault cleared and the core reset again, which it needs to leave a critical alarm (that
// reset does not go through on_driver_reset). Once it is over, the homed state is dropped (the
// machine may have run part of the motion, or been powered off) and the user told.
void slicer_core_poll (void)
{
    static bool resetting = false;

    test_core_stall();

    if(atomic_exchange(&core_stalled, false)) {
        // Same alarm as a link loss, but nothing has to come back: the fault is over at once
        report_message("The bridge stalled during motion: machine stopped, position may be lost, home the machine", Message_Warning);
        mc_reset();     // reads the motor fault and raises ALARM:17
        atomic_store(&sink_failed, false);
        return;
    }

    // A reset under way: wait for its end. Except in the core's critical alarm loop, which only
    // ends on a reset once the motor fault is gone: a reset that came during it (the sender's, while
    // the machine was away) stays pending until this function has cleared the fault.
    bool reset_under_way = ((sys.rt_exec_state & EXEC_RESET) || sys.reset_pending) && !sys.blocking_event;

    if(!atomic_load(&reconnected) || reset_under_way ||
        state_get() != STATE_ALARM || timer[STEPPER_TIMER].enable)
        return;

    if(!resetting) {
        int32_t machine[SLICE_AXES];

        // Freeze the simulator thread's view (idle_offset, counted) while the position is
        // replaced. With the interrupt lock, it cannot run meanwhile; without it, let it go round
        // its loop twice (as on_driver_reset)
        atomic_store(&resync_busy, true);
        if(!irq_lock_enabled()) {
            uint32_t beat = atomic_load(&sim_heartbeat);
            for(int i = 0; i < HALT_TIMEOUT_MS && atomic_load(&sim_heartbeat) - beat < 2; i++)
                platform_sleep(1000);
        }
        sink->get_position(sink, machine);
        resync_position(machine);
        sync_position();
        atomic_store(&resync_busy, false);

        resetting = true;
        atomic_store(&sink_failed, false);
        mc_reset();
    } else {
        resetting = false;
        atomic_store(&reconnected, false);
        sys.homed.mask = 0;
        report_message("Controller reconnected: machine position lost, home the machine", Message_Warning);
    }
}

// Core thread (realtime hook). The core is decelerating a jog (cancelled, feed hold, stop): the
// machine is still running the slices sent ahead of it. As Mach3, drop them so that the
// deceleration starts now. The core still counts the dropped steps: once at rest, its position is
// taken from the machine's counters.
void slicer_jog_poll (void)
{
    static bool stopping = false;

    if(!sink || !sink->stop_jog)
        return;

    sys_state_t state = state_get();

    if(!stopping) {
        if(state == STATE_JOG && sys.step_control.execute_hold && (atomic_load(&phase) & 0xFF) == Slice_Jog) {
            stopping = true;
            atomic_store(&resync_busy, true);   // idle_offset keeps its value from before the jog
            sink->stop_jog(sink);
        }
    } else if(state != STATE_JOG && !timer[STEPPER_TIMER].enable &&
               !((sys.rt_exec_state & EXEC_RESET) || sys.reset_pending)) {
        stopping = false;
        if(state == STATE_IDLE && !atomic_load(&sink_failed) && wait_machine_idle(2000)) {
            int32_t machine[SLICE_AXES];
            sink->get_position(sink, machine);
            resync_position(machine);
            sync_position();
        }
        atomic_store(&resync_busy, false);
    }
}

void slicer_report_starved (void)
{
    static unsigned int count = 0;

    fprintf(stderr, "[slicer] STARVED: the core was late with the next step segments, motion paused until they came (%u so far)\n", ++count);
}

void slicer_safe_stop_request (void)
{
    if(sink)
        atomic_store(&safe_stop_wanted, true);
}

bool slicer_safe_stop_wait (uint32_t timeout_ms)
{
    for(uint32_t i = 0; i < timeout_ms && atomic_load(&safe_stop_wanted); i++)
        platform_sleep(1000);

    return !atomic_load(&safe_stop_wanted);
}

void slicer_disarm (void)
{
    slice_sink_t *s = sink;

    if(s && s->disarm)
        s->disarm(s, 200);
}

// Core thread (realtime hook). Same sequence as an operator would follow: feed hold, wait for the
// machine to be at rest, then reset, which switches the spindle and coolant off and flushes the
// program. The motion in progress is never resumed: that decision belongs to the sender.
// At rest with nothing running or queued there is nothing to do: no reset, the modal state stays.
void slicer_safe_stop_poll (void)
{
    static bool holding = false;
    static uint32_t deadline;

    if(!atomic_load(&safe_stop_wanted) || !sink || (sys.rt_exec_state & EXEC_RESET) || sys.reset_pending)
        return;

    sys_state_t state = state_get();
    bool reset = false;

    if(state & STATE_HOMING)
        return;             // cannot be held; ends by itself, at rest

    if(state == STATE_IDLE)
        reset = (atomic_load(&out_state) & 0xFF) || plan_get_current_block() ||
                 (hal.stream.get_rx_buffer_count && hal.stream.get_rx_buffer_count());
    else if(state & STATE_TOOL_CHANGE)
        reset = true;       // waiting for the operator: nothing moves
    else if(!(state & (STATE_ALARM|STATE_ESTOP|STATE_SLEEP|STATE_CHECK_MODE))) {
        // Cycle, jog, hold, safety door: the machine must come to rest first
        if(!holding) {
            holding = true;
            deadline = hal.get_elapsed_ticks() + SAFE_STOP_TIMEOUT_MS;
            if(state & (STATE_CYCLE|STATE_JOG)) {
                fprintf(stderr, "[slicer] controlled stop: feed hold\n");
                system_set_exec_state_flag(EXEC_FEED_HOLD);
            }
            return;
        }
        if((int32_t)(hal.get_elapsed_ticks() - deadline) >= 0) {
            fprintf(stderr, "[slicer] controlled stop: machine not at rest in time, reset\n");
            reset = true;
        } else if(!(state & (STATE_CYCLE|STATE_JOG)) && sys.holding_state == Hold_Complete && !timer[STEPPER_TIMER].enable) {
            // Let the end of the ramp run: a reset drops what the machine has not executed yet.
            // A hold that is over is not motion: without this flag the reset would raise ALARM:3.
            if(sink->wait_idle)
                wait_machine_idle(2000);
            sys.step_control.execute_hold = Off;
            reset = true;
        } else
            return;
    }

    holding = false;
    atomic_store(&safe_stop_wanted, false);
    if(reset) {
        fprintf(stderr, "[slicer] controlled stop: spindle and coolant off, program flushed\n");
        mc_reset();
    }
}

void slicer_poll (void)
{
    atomic_fetch_add(&sim_heartbeat, 1);

    if(sink) {
        check_estop();
        check_link();
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
