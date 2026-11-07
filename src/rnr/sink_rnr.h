/*
  sink_rnr.h - slice sink driving an RNR Eco Motion 2.0 controller

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef sink_rnr_h
#define sink_rnr_h

#include "../slice_sink.h"
#include "rnr_transport.h"

// Takes ownership of the transport (closed on failure or by sink->close).
// Returns NULL if the controller does not answer.
slice_sink_t *sink_rnr_open (rnr_transport_t *t);

#endif
