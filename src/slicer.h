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
void slicer_attach_core (void);                   // hooks the core's reset event (call once the core is initialised)
bool slicer_motor_fault (void);                   // true while the sink has failed (reported as the motor fault signal)
bool slicer_estop (void);                         // true while the machine reports an e-stop (reported as the e-stop signal); any thread
bool slicer_get_inputs (sink_inputs_t *in);       // machine home/limit/probe inputs; false: use the simulated ones; any thread
void slicer_set_phase (slice_kind_t kind, int axis); // kind of motion the next slices belong to; any thread
void slicer_probe_end (void);                     // end of a probing cycle: position and probe position from the machine; core thread
void slicer_core_poll (void);                     // realtime hook; core thread
// Controlled stop when nobody is left to supervise the job (sender gone, program closing): feed
// hold (deceleration ramp, position kept), then spindle and coolant off and the program flushed.
// Never resumes by itself.
void slicer_safe_stop_request (void);             // any thread
bool slicer_safe_stop_wait (uint32_t timeout_ms); // true once it is over; any thread except the core's
void slicer_safe_stop_poll (void);                // realtime hook; core thread
void slicer_jog_poll (void);                      // stops a jog at once on the machine when it is cancelled; realtime hook, core thread
void slicer_disarm (void);                        // last resort before the program ends: controller disarmed, outputs off; any thread
// Outputs set by the core (any thread): stamped on every slice and passed to the sink
void slicer_set_spindle (bool on, bool ccw, uint16_t pwm); // pwm: 0..SLICE_PWM_PERIOD
void slicer_set_spindle_pwm (uint16_t pwm);           // speed change only (laser mode, S word)
void slicer_set_coolant (bool flood, bool mist);

#endif
