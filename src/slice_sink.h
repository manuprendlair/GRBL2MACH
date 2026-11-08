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

// Machine outputs as the core sets them; each sink maps them to its own outputs.
#define SLICE_OUT_SPINDLE     0x01  // spindle on (M3 or M4)
#define SLICE_OUT_SPINDLE_CCW 0x02  // spindle turning counter-clockwise (M4)
#define SLICE_OUT_FLOOD       0x04  // flood coolant (M8)
#define SLICE_OUT_MIST        0x08  // mist coolant (M7)
#define SLICE_PWM_PERIOD      1000  // spindle_pwm unit: duty cycle in 1/1000 (0 = off, 1000 = max speed)

typedef struct {
    uint8_t  flags;             // SLICE_OUT_xxx
    uint16_t spindle_pwm;       // spindle speed in 1/SLICE_PWM_PERIOD of the maximum speed ($30)
} slice_outputs_t;

typedef struct {
    uint32_t index;             // slice number since start; slice starts at index * SLICE_MS (simulated time)
    int32_t  steps[SLICE_AXES]; // signed steps per axis to execute during the slice
    slice_outputs_t out;        // outputs while the slice runs
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
    // The core changed the outputs (spindle, coolant). Slices pushed from now on carry the new
    // state; this call tells the sink about it when no motion follows (outputs at rest). The sink
    // applies it once every slice pushed before it has run. Optional. Thread-safe.
    void (*set_outputs)(slice_sink_t *sink, slice_outputs_t out);
    void *ctx;
    // To come with the controller: limit and probe inputs, homing/probe arming, latched positions.
};

// File sink (CSV): dry run without hardware.
slice_sink_t *sink_file_open (const char *path);

#endif
