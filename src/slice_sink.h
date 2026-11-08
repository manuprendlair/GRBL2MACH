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

// What the motion is for: the sink may drive the machine differently (a controller that stops
// by itself on a home switch or a probe contact). The core tells the slicer.
typedef enum {
    Slice_Program = 0,  // G0/G1/G2... and anything else
    Slice_HomeSeek,     // homing move towards the switch of one axis (seek or locate)
    Slice_HomeJog,      // homing pull-off (or move of a homing cycle with several axes)
    Slice_Probe         // G38.2 / G38.3: towards the probe
} slice_kind_t;

typedef struct {
    uint32_t index;             // slice number since start; slice starts at index * SLICE_MS (simulated time)
    int32_t  steps[SLICE_AXES]; // signed steps per axis to execute during the slice
    slice_outputs_t out;        // outputs while the slice runs
    uint8_t  kind;              // slice_kind_t
    uint8_t  axis;              // Slice_HomeSeek, Slice_HomeJog: axis 0..3
} slice_t;

// Machine inputs as the sink reads them, polarity applied: bit a (X, Y, Z, A) set = triggered
typedef struct {
    uint8_t home, limit_pos, limit_neg;
    bool probe;
} sink_inputs_t;

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
    // Current state of the machine's home, limit and probe inputs. Returns false while unknown.
    // Optional (NULL: the simulator's own inputs are used). Thread-safe.
    bool (*get_inputs)(slice_sink_t *sink, sink_inputs_t *in);
    // After a homing or probing move that the machine cut short by itself (switch, contact), the
    // slices it dropped make the slicer's step count run ahead. Once the machine is at rest, returns
    // true once with its position, in steps. Optional. Thread-safe.
    bool (*take_resync)(slice_sink_t *sink, int32_t position[SLICE_AXES]);
    // End of a probing move: waits up to timeout_ms until the machine is at rest and returns its
    // position and whether it touched (then *contact = true and latched = position at contact).
    // Returns false if unknown (no such feature, link lost, timeout). Optional. Thread-safe.
    bool (*get_probe_result)(slice_sink_t *sink, int32_t position[SLICE_AXES], int32_t latched[SLICE_AXES],
                              bool *contact, uint32_t timeout_ms);
    // Waits up to timeout_ms until every slice pushed so far has run and the machine is at rest,
    // with inputs read after that. Returns false on timeout or fault. Optional. Thread-safe.
    bool (*wait_idle)(slice_sink_t *sink, uint32_t timeout_ms);
    void *ctx;
};

// File sink (CSV): dry run without hardware.
slice_sink_t *sink_file_open (const char *path);

#endif
