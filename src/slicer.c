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
  sink stops the machine and reports where it actually stopped. The steps
  counted here but never executed (dropped slice in progress, slices discarded
  by the stop) are then taken back out of sys.position, before the core syncs
  its planner and parser to it. Working with this difference keeps the resync
  right even if the core has overwritten sys.position (homing).

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>

#include "slicer.h"
#include "simulator.h"
#include "mcu.h"
#include "driver.h"

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/system.h"
#include "grbl/gcode.h"
#include "grbl/planner.h"
#include "grbl/task.h"

#define SLICE_TICKS ((uint64_t)F_CPU / 1000 * SLICE_MS)
#define HALT_TIMEOUT_MS 200

static slice_sink_t *sink = NULL;
static slice_t cur;
static bool open = false;           // a slice is in progress
static atomic_bool sink_failed;      // the sink reported an error: slices are dropped, motor fault signalled
static atomic_bool reset_pending;   // set by the core's on_reset (any thread), handled on the simulator thread
static on_reset_ptr prev_on_reset = NULL;
static int32_t counted[SLICE_AXES]; // steps counted since start: the machine position if every slice ran
static int32_t idle_offset[N_AXIS]; // sys.position - counted, sampled at rest (see advance())
static atomic_bool resync_busy;     // from a reset until its resync is done: idle_offset is frozen
static atomic_uint sim_heartbeat;   // incremented on every simulator tick

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

// Foreground task queued by on_core_reset(): runs on the core thread, from its main loop or its
// critical alarm loop. Waits for the machine to be at rest, takes the steps that never ran back
// out of sys.position and syncs the planner and parser to it. Whether it runs before or after the
// core's own reset sequence (which also syncs to sys.position), the result is the same.
static void resync_task (void *data)
{
    int32_t machine[SLICE_AXES];
    uint32_t beat = atomic_load(&sim_heartbeat);

    (void)data;

    // The simulator thread may still be finishing a stepper interrupt that updates sys.position:
    // wait until the stepper timer is off and the simulator thread has gone round its loop twice
    // before writing sys.position. No new motion can start meanwhile: this task runs on the core
    // thread before it reads any command issued after the reset.
    for(int i = 0; i < HALT_TIMEOUT_MS && (timer[STEPPER_TIMER].enable || atomic_load(&sim_heartbeat) - beat < 2); i++)
        platform_sleep(1000);

    if(sink->get_halted_position(sink, machine, HALT_TIMEOUT_MS)) {
        resync_position(machine);
        sync_position();
    } else
        fprintf(stderr, "[slicer] machine position unknown after stop: position not resynced\n");

    atomic_store(&resync_busy, false);
}

// Core reset (soft reset, alarm, e-stop), from mc_reset(): interrupt context, any thread, and the
// core's own thread may already be re-initialising. Kept short: stop the machine, queue the resync.
static void on_core_reset (void)
{
    if(sink) {
        atomic_store(&resync_busy, true);       // freeze idle_offset until the resync is done
        sink->stop(sink);                       // pending slices must not run
        atomic_store(&reset_pending, true);     // the simulator thread drops its slice in progress
        task_add_immediate(resync_task, NULL);
    }

    if(prev_on_reset)
        prev_on_reset();
}

void slicer_attach_core (void)
{
    if(grbl.on_reset != on_core_reset) {
        prev_on_reset = grbl.on_reset;
        grbl.on_reset = on_core_reset;
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
        if(motion) {
            memset(&cur, 0, sizeof(cur));
            cur.index = idx;
            open = true;
        } else if(!atomic_load(&resync_busy)) {
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
        memset(&cur, 0, sizeof(cur));
        cur.index = next;
        open = motion;
    }
}

void slicer_step (uint32_t step_bits, uint32_t dir_bits)
{
    if(!sink)
        return;

    advance();

    if(!open) {     // pulse outside of a declared motion: open a slice anyway
        memset(&cur, 0, sizeof(cur));
        cur.index = (uint32_t)(sim.masterclock / SLICE_TICKS);
        open = true;
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

    if(sink)
        advance();
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
