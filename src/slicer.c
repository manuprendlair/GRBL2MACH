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

#include <string.h>

#include "slicer.h"
#include "simulator.h"
#include "mcu.h"
#include "driver.h"

#include "grbl/hal.h"

#define SLICE_TICKS ((uint64_t)F_CPU / 1000 * SLICE_MS)

static slice_sink_t *sink = NULL;
static slice_t cur;
static bool open = false;   // a slice is in progress

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

    while((status = sink->push(sink, &cur)) == Sink_Full)
        platform_sleep(1000);

    // TODO (controller): on Sink_Error, trigger a motion stop on the grblHAL side.
}

static void advance (void)
{
    uint32_t idx = (uint32_t)(sim.masterclock / SLICE_TICKS);
    bool motion = timer[STEPPER_TIMER].enable;

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
