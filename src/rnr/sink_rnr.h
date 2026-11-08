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
    // Controller output (1..4 = OUT1..OUT4, 0 = none) driven by: spindle on, spindle on in
    // counter-clockwise (0: same output as spindle_out), flood coolant, mist coolant
    uint8_t spindle_out, spindle_ccw_out, flood_out, mist_out;
    // E-stop input watched by the bridge (1..16 = IN1..IN16, 0 = none), and its polarity. The
    // controller stops on it by itself only with input pins in its configuration frame.
    uint8_t estop_in;
    bool estop_in_active_low;
    // Configure frame bytes 5..14 (input pins), copied from Mach3's frame (--rnr-pins)
    uint8_t pins[10];
} rnr_options_t;

// Takes ownership of the transport (closed on failure or by sink->close).
// Returns NULL if the controller does not answer.
slice_sink_t *sink_rnr_open (rnr_transport_t *t, const rnr_options_t *opt);

#endif
