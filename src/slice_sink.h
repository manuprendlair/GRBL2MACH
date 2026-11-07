/*
  slice_sink.h - "slice sink" interface (GRBL2MACH design note, section 4 bis)

  Single contract between the simulator (producer of step slices) and whatever
  consumes them: RNR USB motion controller, test file, dummy sink.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef slice_sink_h
#define slice_sink_h

#include <stdint.h>
#include <stdbool.h>

#define SLICE_AXES 4    // X, Y, Z, A: the four axes of the RNR controller
#define SLICE_MS   25   // slice duration (the RNR controller executes one slice every 25 ms)

typedef struct {
    uint32_t index;             // slice number since start; slice starts at index * SLICE_MS (simulated time)
    int32_t  steps[SLICE_AXES]; // signed steps per axis to execute during the slice
    uint8_t  outputs;           // outputs OUT1..OUT4 (bit 0 = OUT1)
} slice_t;

typedef enum {
    Sink_Ok = 0,    // slice accepted
    Sink_Full,      // queue full: retry later (back-pressure)
    Sink_Error      // sink out of order: the producer must stop motion
} sink_status_t;

typedef struct slice_sink slice_sink_t;

struct slice_sink {
    const char *name;
    // Sends one slice. Never blocks: returns Sink_Full when the queue is full.
    sink_status_t (*push)(slice_sink_t *sink, const slice_t *slice);
    // Accumulated position, in steps, of what the sink has taken over
    // (file: slices written; controller: slices executed).
    void (*get_position)(slice_sink_t *sink, int32_t position[SLICE_AXES]);
    // Immediate stop: discards whatever has not been executed yet.
    void (*stop)(slice_sink_t *sink);
    // Finishes cleanly (drains the queue) and releases the sink.
    void (*close)(slice_sink_t *sink);
    void *ctx;
    // To come with the controller: input reading (limits, probe, e-stop),
    // acknowledgements, homing/probe arming, latched positions.
};

// File sink (CSV): dry run without hardware.
slice_sink_t *sink_file_open (const char *path);

#endif
