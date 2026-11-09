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
    // Step pulse frequency, kHz (Configure byte 16; Mach3: controller plugin "Frequency of Pulse Output"). 0 = 100
    uint8_t pulse_khz;
    // Inputs declared to the controller (--rnr-in, --rnr-estop-in). The e-stop input is also
    // watched by the bridge itself. The controller stops on it by itself once declared.
    rnr_inputs_t inputs;
    // Configure bytes 5..14 given as is (--rnr-pins, copied from a Mach3 capture): replace the
    // bytes encoded from inputs
    bool pins_raw;
    uint8_t pins[RNR_PIN_BYTES];
    // Link lost (unplugged, controller silent): called about once a second until it returns true,
    // with the transport in use. It may close it (then sets *t = NULL) and open a new one into *t.
    // NULL: no reconnection, the bridge must be restarted.
    bool (*reconnect)(rnr_transport_t **t);
} rnr_options_t;

// Takes ownership of the transport (closed on failure or by sink->close).
// Returns NULL if the controller does not answer.
slice_sink_t *sink_rnr_open (rnr_transport_t *t, const rnr_options_t *opt);

#endif
