/*
  rnr_transport.h - byte transport to an RNR controller: real USB HID or fake controller

  A transport is used by a single thread (the sink's I/O thread): implementations
  need not be thread-safe.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef rnr_transport_h
#define rnr_transport_h

#include <stddef.h>

#include "rnr_protocol.h"

#define RNR_IO_OK            0
#define RNR_IO_TIMEOUT       1
#define RNR_IO_FAILED       -1
#define RNR_IO_DISCONNECTED -2

typedef struct rnr_transport rnr_transport_t;

struct rnr_transport {
    const char *name;
    // Writes one output report. Returns RNR_IO_OK, RNR_IO_FAILED or RNR_IO_DISCONNECTED.
    int (*write)(rnr_transport_t *t, const uint8_t buf[RNR_OUT_BYTES]);
    // Waits up to timeout_ms for one input report. Returns RNR_IO_OK, RNR_IO_TIMEOUT or RNR_IO_DISCONNECTED.
    int (*read)(rnr_transport_t *t, uint8_t buf[RNR_IN_BYTES], uint32_t timeout_ms);
    void (*close)(rnr_transport_t *t);
    void *ctx;
};

// Real controller over Windows HID. Returns NULL if no controller is found.
// info receives a one-line description (product, serial, report sizes).
rnr_transport_t *rnr_hid_open (char *info, size_t info_len);

// Fake controller for tests: executes slices in real time and logs every frame to log_path.
// Environment variable GRBL2MACH_FAKE_SILENT_MS makes it stop reporting that long after the first slice.
rnr_transport_t *rnr_fake_open (const char *log_path);

#endif
