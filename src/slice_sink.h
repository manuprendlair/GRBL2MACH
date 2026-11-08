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
#define SLICE_MS   5    // slice duration (the RNR controller executes one slice every 5 ms, as Mach3 configures it)

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
    // Immediate stop: discards whatever has not been executed yet. Thread-safe. Slices pushed
    // after a stop are dropped until resume() is called: a slice computed before the stop can
    // never run after it.
    void (*stop)(slice_sink_t *sink);
    // Accepts slices again after a stop (called by the slicer once it has dropped its own state).
    void (*resume)(slice_sink_t *sink);
    // After stop(): waits up to timeout_ms until the machine is known to be at rest and returns
    // where it stopped, in steps. Returns false if that position cannot be known (link lost).
    // Thread-safe.
    bool (*get_halted_position)(slice_sink_t *sink, int32_t position[SLICE_AXES], uint32_t timeout_ms);
    // Finishes cleanly (drains the queue) and releases the sink.
    void (*close)(slice_sink_t *sink);
    // True while the machine reports an emergency stop of its own (hardware e-stop input).
    // Optional (NULL: no such input). Thread-safe.
    bool (*estop_active)(slice_sink_t *sink);
    void *ctx;
    // To come with the controller: limit and probe inputs, homing/probe arming, latched positions.
};

// File sink (CSV): dry run without hardware.
slice_sink_t *sink_file_open (const char *path);

#endif
