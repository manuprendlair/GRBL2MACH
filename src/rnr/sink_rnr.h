/*
  sink_rnr.h - slice sink driving an RNR Eco Motion 2.0 controller

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef sink_rnr_h
#define sink_rnr_h

#include "../slice_sink.h"
#include "rnr_transport.h"

typedef struct {
    // Configure frame byte 15: bits 0-3 = step pulse active low X..A, bits 4-7 = direction
    // inverted X..A (Mach3: "Step Low Active" / "Dir LowActive" in Ports & Pins > Motor Outputs)
    uint8_t step_dir_invert;
} rnr_options_t;

// Takes ownership of the transport (closed on failure or by sink->close).
// Returns NULL if the controller does not answer.
slice_sink_t *sink_rnr_open (rnr_transport_t *t, const rnr_options_t *opt);

#endif
