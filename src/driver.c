/*
  driver.c - driver code for simulator MCU

  Part of grblHAL

  Copyright (c) 2020-2026 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>
#include <stdatomic.h>

// On the host, the grbl thread and the simulator thread share one (or few) real
// CPU cores. grbl's main loop and driver_delay_ms's spin loop run at full host
// speed, so without cooperation they starve the simulator thread and burn a
// whole core (process CPU ~= 2x wall). Yielding the host CPU here does NOT change
// simulation timing: the simulated tick at which every event fires is driven by
// the simulator thread's clock/ISRs, not by how often the grbl thread polls.
// It only lets the scheduler hand cycles to the simulator thread when the grbl
// thread would otherwise busy-spin.
#ifndef WIN32
#include <sched.h>
#define sim_yield() sched_yield()
#else
// SwitchToThread() hands the rest of the timeslice to another ready thread
// (the simulator thread) and returns immediately when none is waiting - the
// Windows equivalent of sched_yield(). windows.h comes in via platform.h.
#define sim_yield() SwitchToThread()
#endif

#include "mcu.h"
#include "driver.h"
#include "serial.h"
#include "simulator.h"
#include "eeprom.h"
#include "grbl_eeprom_extensions.h"
#include "grbl_interface.h"
#include "platform.h"
#include "slicer.h"
#include "mach3.h"
#include "irq_lock.h"

#include "grbl/hal.h"
#include "grbl/planner.h"

#ifndef SQUARING_ENABLED
#define SQUARING_ENABLED 0
#endif

// NOTE: probe_invert, ticks and delay are shared between the grbl thread and the
// simulator thread's ISRs; they need volatile/_Atomic so stores are visible across threads.
static spindle_id_t spindle_id;
static volatile bool probe_invert;
static _Atomic uint32_t ticks = 0;
static volatile delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static on_execute_realtime_ptr on_execute_realtime;

void SysTick_Handler (void);
void Stepper_IRQHandler (void);
void Limits0_IRQHandler (void);
void Control_IRQHandler (void);

#if SQUARING_ENABLED
static axes_signals_t motors_0 = {AXES_BITMASK}, motors_1 = {AXES_BITMASK};
void Limits1_IRQHandler (void);
#endif

status_code_t mc_rigid_tapping (plan_line_data_t *pl_data, coord_data_t *target, coord_data_t *position, float pitch, float rpm_multiplier)
{
    return Status_GcodeUnsupportedCommand; // TBC
}

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        systick_timer.enable = 1;
        if(!(delay.callback = callback))
            while(delay.ms)
                irq_lock_core_yield(true); // GRBL2MACH: release the interrupt lock and the CPU; delay.ms is decremented by SysTick on the sim thread
    } else if(callback)
        callback();
}

#if SQUARING_ENABLED

inline static void set_step_outputs (axes_signals_t step_out_0)
{
    axes_signals_t step_out_1;

    step_out_1.bits = (step_out_0.bits & motors_1.bits) ^ settings.steppers.step_invert.bits;
    step_out_0.bits = (step_out_0.bits & motors_0.bits) ^ settings.steppers.step_invert.bits;

    mcu_gpio_set(&gpio[STEP_PORT0], step_out_0.bits, AXES_BITMASK);
    mcu_gpio_set(&gpio[STEP_PORT1], step_out_1.bits, AXES_BITMASK);
}

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
        ganged.x = On;
    } else {
        ganged.x = On;
    }

    return ganged;
}

#else

inline static void set_step_outputs (axes_signals_t step_out)
{
    step_out.bits = (step_out.bits) ^ settings.steppers.step_invert.bits;

    mcu_gpio_set(&gpio[STEP_PORT0], step_out.bits, AXES_BITMASK);
}

#endif

inline static void set_dir_outputs (axes_signals_t dir_out)
{
    mcu_gpio_set(&gpio[DIR_PORT], dir_out.value ^ settings.steppers.dir_invert.mask, AXES_BITMASK);
}

static void stepperEnable (axes_signals_t enable, bool hold)
{
    mcu_gpio_set(&gpio[STEPPER_ENABLE_PORT], enable.value ^ settings.steppers.enable_invert.mask, AXES_BITMASK);
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    timer[STEPPER_TIMER].load = 5000;
    timer[STEPPER_TIMER].value = 0;
    mcu_timer_enable(STEPPER_TIMER, true);   // enable + keep timer_enabled_mask in sync

//    hal.stepper_interrupt_callback();   // start the show
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].load = 0;
    mcu_timer_enable(STEPPER_TIMER, false);  // disable + keep timer_enabled_mask in sync

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout, limiting the slowest speed
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    timer[STEPPER_TIMER].load = cycles_per_tick;
    timer[STEPPER_TIMER].value = 0;
    mcu_timer_enable(STEPPER_TIMER, true);   // enable + keep timer_enabled_mask in sync
}

// "Normal" version: Sets stepper direction and pulse pins and starts a step pulse a few nanoseconds later.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }

    if(stepper->step_out.bits) {
        set_step_outputs(stepper->step_out);
        slicer_step(stepper->step_out.bits, stepper->dir_out.bits);
    }
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
// TODO: only delay after setting dir outputs?
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }

    if(stepper->step_out.bits) {
        slicer_step(stepper->step_out.bits, stepper->dir_out.bits);
//        next_step_out = stepper->step_out; // Store out_bits
//        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }
}

static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};
    sink_inputs_t in;

    // GRBL2MACH: the machine's inputs, polarity already applied by the sink (--rnr-in). Home
    // switches and limit switches of an axis are one signal for the core, as on most grbl boards.
    if(slicer_get_inputs(&in)) {
        signals.min.mask = in.home | in.limit_pos | in.limit_neg;
        return signals;
    }

    signals.min.value = gpio[LIMITS_PORT0].state.value;

    if (settings.limits.invert.mask)
        signals.min.mask ^= settings.limits.invert.mask;

    return signals;
}

#if SQUARING_ENABLED

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_0.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0);
    motors_1.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0);
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
static limit_signals_t limitsGetHomeState()
{
    limit_signals_t signals = {0};

    if(motors_0.mask) {

        signals.min.mask = gpio[LIMITS_PORT0].state.value;

        if (settings.limits.invert.mask)
            signals.min.mask ^= settings.limits.invert.mask;
    }

    if(motors_1.mask) {

       signals.max.mask = gpio[LIMITS_PORT1].state.value;

        if (settings.limits.invert.mask)
            signals.max.mask ^= settings.limits.invert.mask;
    }

    return signals;
}

#endif

static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    gpio[LIMITS_PORT0].irq_mask.mask = on ? AXES_BITMASK : 0;
    gpio[LIMITS_PORT0].irq_state.mask = 0;

  #if SQUARING_ENABLED
    gpio[LIMITS_PORT1].irq_mask.mask = on ? AXES_BITMASK : 0;
    gpio[LIMITS_PORT1].irq_state.mask = 0;

    hal.limits.get_state = homing_cycle.mask != 0 ? limitsGetHomeState : limitsGetState;
  #endif
}

static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.mask = gpio[CONTROL_PORT].state.value;
	signals.limits_override = settings.control_invert.limits_override;
    // GRBL2MACH: a failed slice sink (controller lost) is reported as a motor fault
    signals.motor_fault = slicer_motor_fault();

    if(settings.control_invert.mask)
        signals.mask ^= settings.control_invert.mask;

    // GRBL2MACH: the controller's own e-stop input (after inversion: it is already active high)
    if(slicer_estop())
        signals.e_stop = On;

    return signals;
}

static void probeConfigureInvertMask (bool is_probe_away, bool probing)
{
    // GRBL2MACH: probing towards the probe: the machine stops by itself on contact. At the end of
    // the cycle, take its position and the position where it touched.
    if(probing)
        slicer_set_phase(is_probe_away ? Slice_Program : Slice_Probe, 0);
    else
        slicer_probe_end();

  probe_invert = settings.probe.invert_probe_pin;

  if (is_probe_away)
      probe_invert ^= is_probe_away;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.value = mcu_gpio_get(&gpio[PROBE_PORT], PROBE_MASK);

    // GRBL2MACH: the machine's probe input, polarity applied by the sink (--rnr-in)
    sink_inputs_t in;
    if(slicer_get_inputs(&in)) {
        state.connected = On;
        state.triggered = in.probe;
    }

    state.triggered ^= probe_invert;

    return state;
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm);

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    mcu_gpio_set(&gpio[SPINDLE_PORT], state.value ^ settings.pwm_spindle.invert.mask, SPINDLE_MASK);
    slicer_set_spindle(state.on, state.ccw, (uint16_t)spindleGetPWM(spindle, rpm));   // GRBL2MACH
}

// Variable spindle control functions

// Sets spindle speed
static void spindle_set_speed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    slicer_set_spindle_pwm((uint16_t)pwm_value);   // GRBL2MACH
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    // GRBL2MACH: duty cycle in 1/SLICE_PWM_PERIOD of the maximum speed ($30), like Mach3 (S12000
    // with a 24000 rpm spindle = 500/1000)
    float max = settings.pwm_spindle.rpm_max;

    if(rpm <= 0.0f || max <= 0.0f)
        return 0;
    if(rpm < settings.pwm_spindle.rpm_min)
        rpm = settings.pwm_spindle.rpm_min;

    return rpm >= max ? SLICE_PWM_PERIOD : (uint_fast16_t)(rpm * SLICE_PWM_PERIOD / max);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    mcu_gpio_set(&gpio[SPINDLE_PORT], state.value ^ settings.pwm_spindle.invert.mask, SPINDLE_MASK);
    slicer_set_spindle(state.on, state.ccw, (uint16_t)spindleGetPWM(spindle, rpm));   // GRBL2MACH
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {0};

    state.value = gpio[SPINDLE_PORT].state.value ^ settings.pwm_spindle.invert.mask;

    return state;
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

	static spindle_pwm_t spindle_pwm;

    spindle->context.pwm = &spindle_pwm;
	spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, 1000000);

    // GRBL2MACH: copies the new speed range ($30, $31) to the enabled spindle, as hardware drivers
    // do: without it, $30 only took effect after a restart
    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

static void coolantSetState (coolant_state_t mode)
{
    mcu_gpio_set(&gpio[COOLANT_PORT], mode.value ^ settings.coolant.invert.mask, COOLANT_MASK);
    slicer_set_coolant(mode.flood, mode.mist);    // GRBL2MACH
}

static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.value = gpio[COOLANT_PORT].state.value ^ settings.coolant.invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)

static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __atomic_fetch_or(ptr, bits, __ATOMIC_SEQ_CST);
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    return __atomic_fetch_and(ptr, (uint_fast16_t)~bits, __ATOMIC_SEQ_CST);
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    return __atomic_exchange_n(ptr, value, __ATOMIC_SEQ_CST);
}

void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    mach3_settings_changed();   // GRBL2MACH: write changed settings back to the Mach3 profile

    if(changed.spindle) {
        spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
        if(spindle_id == spindle_get_default())
            spindle_select(spindle_id);
    }

#if SQUARING_ENABLED
    hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif
}

bool driver_setup (settings_t *settings)
{
    // GRBL2MACH: stop the slice sink on every core reset
    slicer_attach_core();
    mach3_attach_core();        // GRBL2MACH: settings from the machine's Mach3 profile, and back

    timer[STEPPER_TIMER].prescaler = 0;
    timer[STEPPER_TIMER].irq_enable = 1;
    mcu_register_irq_handler(Stepper_IRQHandler, Timer0_IRQ);

    gpio[STEPPER_ENABLE_PORT].dir.mask = AXES_BITMASK;
    gpio[STEP_PORT0].dir.mask = AXES_BITMASK;
    gpio[DIR_PORT].dir.mask = AXES_BITMASK;

    gpio[COOLANT_PORT].dir.mask = COOLANT_MASK;
    gpio[SPINDLE_PORT].dir.mask = SPINDLE_MASK;

    gpio[LIMITS_PORT0].dir.mask = AXES_BITMASK;
    gpio[LIMITS_PORT0].rising.mask = AXES_BITMASK;
    mcu_register_irq_handler(Limits0_IRQHandler, LIMITS_IRQ0);

#if SQUARING_ENABLED
    gpio[STEP_PORT1].dir.mask = AXES_BITMASK;

    gpio[LIMITS_PORT1].dir.mask = AXES_BITMASK;
    gpio[LIMITS_PORT1].rising.mask = AXES_BITMASK;
    mcu_register_irq_handler(Limits1_IRQHandler, LIMITS_IRQ1);
#endif

    gpio[CONTROL_PORT].dir.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].rising.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].irq_mask.mask = CONTROL_MASK;
    mcu_register_irq_handler(Control_IRQHandler, CONTROL_IRQ);

    mcu_gpio_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT, PROBE_CONNECTED_BIT); // default to connected

    grbl.on_settings_changed(settings, (settings_changed_flags_t){0});
    hal.stepper.go_idle(true);
    spindle_ptrs_t* spindle;

    if((spindle = spindle_get(0))) {
        spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
    }

    hal.coolant.set_state((coolant_state_t){0});

    return settings->version.id == 23;
}

// used to inject a sleep in grbl main loop,
// ensures hardware simulator gets some cycles in "parallel"
void sim_process_realtime (uint_fast16_t state)
{
    // heartbeat for the simulator thread: the boot sequence (which flushes the
    // input stream) is done and the main loop has run since the last beat.
    // The serial feed is paced against it so a burst of simulated time cannot
    // deliver input faster than the grbl thread consumes it - mirroring real
    // hardware, where the main loop runs many times between two serial bytes.
    sim.grbl_pulse++;

    grbl_print_recent_block(); // on this (grbl) thread, which owns the planner

    // GRBL2MACH: let pending interrupts run. With nothing to do (no input to parse, or a full planner
    // that cannot take it), sleep instead of spinning at full host speed: on a two-core PC, that
    // spinning takes the CPU from the simulator thread and the sender. Each pass fills the whole
    // stepper segment buffer, which holds far more than 1 ms of motion.
    if(plan_check_full_buffer() || (!hal.stream.get_rx_buffer_count() && !simulate_serial_pending()))
        irq_lock_core_sleep();
    else
        irq_lock_core_yield(true);
    slicer_core_poll(); // GRBL2MACH: slicer work on the core thread (reconnection)
    slicer_safe_stop_poll(); // GRBL2MACH: controlled stop (client gone, program closing)
    slicer_jog_poll(); // GRBL2MACH: jog cancelled: the machine stops at once
    if(sim.greet_client) { // GRBL2MACH: a new TCP client gets the welcome message, as after a controller reset
        sim.greet_client = false;
        grbl.report.init_message(hal.stream.write);
    }
    on_execute_realtime(state);
}

uint32_t millis (void)
{
    // GRBL2MACH: the core polls the clock in its waiting loops (task_execute(), stream TX
    // blocking, ...). If an interrupt is waiting for the lock, let it run here, otherwise
    // a loop waiting for an interrupt would never end. No-op on other threads.
    irq_lock_core_yield(false);

    return ticks;
}

bool driver_delay_pending (void)
{
    return delay.ms != 0;
}

__attribute__((weak)) void board_init (void)
{
    // NOOP
}

bool driver_init ()
{
    mcu_reset();

    mcu_register_irq_handler(SysTick_Handler, Systick_IRQ);

    systick_timer.load = F_CPU / 1000 - 1;
    systick_timer.irq_enable = 1;
    systick_timer.enable = 1;

    hal.info = "Simulator";
    hal.driver_version = "261007";
    hal.driver_setup = driver_setup;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.f_step_timer = F_CPU;
    hal.delay_ms = driver_delay_ms;
    grbl.on_settings_changed = settings_changed;

    on_execute_realtime = grbl.on_execute_realtime;
    grbl.on_execute_realtime = sim_process_realtime;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#if SQUARING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigureInvertMask;

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
        .cap.variable = On,
        .cap.laser = On,
        .cap.direction = On,
        .config = spindleConfig,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindle_set_speed,
        .set_state = spindleSetState,
        .get_state = spindleGetState
    };

    spindle_register(&spindle, "simulated PWM spindle");

    hal.control.get_state = systemGetState;
/*
    hal.show_message = showMessage;
*/

    memcpy(&hal.stream, serialInit(), sizeof(io_stream_t));
    hal.nvs.type = NVS_EEPROM;
    hal.nvs.get_byte = eeprom_get_char;
    hal.nvs.put_byte = eeprom_put_char;
    hal.nvs.memcpy_to_nvs = memcpy_to_eeprom;
    hal.nvs.memcpy_from_nvs = memcpy_from_eeprom;

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = millis;

    hal.coolant_cap.flood = On;
    hal.coolant_cap.mist = On;
    // hal.driver_cap.software_debounce = On;
    // This is required for the hal to initialize properly!
    hal.driver_cap.step_pulse_delay = On;

    hal.signals_cap.safety_door_ajar = On;
    hal.signals_cap.motor_fault = On;   // GRBL2MACH: controller lost
    hal.signals_cap.e_stop = slicer_enabled();  // GRBL2MACH: e-stop input of the controller
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
	
	board_init();

    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 10;
}

// Main stepper driver
void Stepper_IRQHandler (void)
{
    hal.stepper.interrupt_callback();
}

void Control_IRQHandler (void)
{
    gpio[CONTROL_PORT].irq_state.value = ~CONTROL_MASK;
    hal.control.interrupt_callback(hal.control.get_state());
}

void Limits0_IRQHandler (void)
{
    gpio[LIMITS_PORT0].irq_state.value = (uint8_t)~AXES_BITMASK;
    hal.limits.interrupt_callback(hal.limits.get_state());
}

#if SQUARING_ENABLED

void Limits1_IRQHandler (void)
{
    gpio[LIMITS_PORT1].irq_state.value = (uint8_t)~AXES_BITMASK;
    hal.limits.interrupt_callback(hal.limits.get_state());
}

#endif

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
    ticks++;

    if(delay.ms && --delay.ms == 0) {
//        systick_timer.enable = 0;
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
