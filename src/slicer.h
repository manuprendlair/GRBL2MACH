/*
  slicer.h - cuts the simulator's step pulses into SLICE_MS slices

  All functions are called from the simulator thread (sim_loop / simulated
  interrupts), never from the grblHAL thread.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef slicer_h
#define slicer_h

#include "slice_sink.h"

void slicer_init (slice_sink_t *sink);            // sink == NULL: slicer disabled
bool slicer_enabled (void);
void slicer_step (uint32_t step_bits, uint32_t dir_bits); // one pulse (bit i = axis i; dir set = negative direction)
void slicer_poll (void);                          // on every simulated tick: closes expired slices
void slicer_close (void);                         // sends the current slice and closes the sink

#endif
