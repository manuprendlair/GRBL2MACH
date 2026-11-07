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

#define SLICE_TICKS ((uint64_t)F_CPU / 1000 * SLICE_MS)

static slice_sink_t *sink = NULL;
static slice_t cur;
static bool open = false;           // a slice is in progress
static bool sink_failed = false;    // the sink reported an error: slices are dropped
static atomic_bool reset_pending;   // set by the core's on_reset (any thread), handled on the simulator thread
static on_reset_ptr prev_on_reset = NULL;

// Core reset (soft reset, alarm, e-stop): pending slices must not run.
static void on_core_reset (void)
{
    atomic_store(&reset_pending, true);

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

void slicer_init (slice_sink_t *s)
{
    sink = s;
    open = false;
}

static void emit (void)
{
    sink_status_t status;

    if(sink_failed)
        return;

    while((status = sink->push(sink, &cur)) == Sink_Full) {
        platform_sleep(1000);   // back-pressure: simulated time waits for the machine
        if(atomic_load(&reset_pending))
            return;
    }

    if(status == Sink_Error) {
        // The machine can no longer follow: stop the core like a hardware fault would
        sink_failed = true;
        fprintf(stderr, "[slicer] sink '%s' failed: raising alarm\n", sink->name);
        mc_reset();
        system_set_exec_alarm(Alarm_MotorFault);
    }
}

static void advance (void)
{
    uint32_t idx = (uint32_t)(sim.masterclock / SLICE_TICKS);
    bool motion = timer[STEPPER_TIMER].enable;

    if(atomic_exchange(&reset_pending, false)) {
        open = false;           // drop the slice in progress
        sink->stop(sink);       // and whatever the sink still holds
        sink_failed = false;    // a sink that is still broken reports it again on the next push
    }

    if(!open) {
        if(motion) {
            memset(&cur, 0, sizeof(cur));
            cur.index = idx;
            open = true;
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
        if(step_bits & (1u << i))
            cur.steps[i] += (dir_bits & (1u << i)) ? -1 : 1;
    }
}

void slicer_poll (void)
{
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
