/*
  rnr_protocol.h - RNR Eco Motion 2.0 USB HID protocol (frames and status report)

  Written from the interoperability specification published by BreakoutCNC
  (docs/protocol/PROTOCOL_SPEC.md), static analysis of the vendor Mach3 plugin
  and USB captures of Mach3 driving the controller. No vendor code is used.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef rnr_protocol_h
#define rnr_protocol_h

#include <stdint.h>
#include <stdbool.h>

#define RNR_VID 0xA720
#define RNR_PID 0xF803

// The HID report descriptor declares three reports: input ID 1 (48 bytes), output ID 2
// (8 bytes) and output ID 3 (26 bytes). Byte 0 of every frame is the report ID. Output frames
// are built as 27-byte buffers (the largest output report); Windows sends a short frame
// (ID 2) as a 9-byte transfer, as seen in the captures of Mach3.
#define RNR_OUT_BYTES 27    // PC -> controller, report ID included
#define RNR_IN_BYTES  49    // controller -> PC, report ID included
#define RNR_AXES      4
#define RNR_PWM_PERIOD 1000 // SpindlePwm period: Mach3 always sends 1000
#define RNR_PIN_BYTES  10   // Configure buf[5..14]: input pins

#define RNR_FRAME_SHORT    0x02 // report ID 2: 8 data bytes
#define RNR_FRAME_EXTENDED 0x03 // report ID 3: 26 data bytes

#define RNR_FLAG_LIMIT_OVERRIDE 0x40 // buf[2]; Mach3 never sets it

typedef enum {
    RnrOp_EStop          = 0x01, // short: immediate halt, drops the buffer, disarms (state 0)
    RnrOp_ClearEStop     = 0x02, // short: enable, sent after PreReset
    RnrOp_SetOutputs     = 0x03, // short: buf[5] = OUT1..OUT4 (Mach3 repeats it every ~31 ms at rest)
    RnrOp_GcodeSlice     = 0x04, // extended: program motion slice; opens a program run (state 6)
    RnrOp_JogSlice       = 0x05, // extended: motion slice of a jog (after StartJog), homing or probing run
    RnrOp_EndOfMotion    = 0x06, // short: end of a run, buf[5] = outputs (state 7 until the buffer is empty)
    RnrOp_PreReset       = 0x07, // short: sent before ClearEStop
    RnrOp_Cancel         = 0x08, // short: immediate halt, drops the buffer, stays armed (Mach3 "Stop")
    RnrOp_SetPositions   = 0x09, // extended: step counters buf[5 + 4a], axis mask buf[21]
    RnrOp_StartJog       = 0x0A, // short: start of a continuous jog, buf[5] = axis (state 2)
    RnrOp_StartStepJog   = 0x0B, // short: step / MPG jog (not seen in the captures)
    RnrOp_ArmProbe       = 0x0C, // short: probing run, stops on the probe input and latches (state 5)
    RnrOp_ArmHome        = 0x0D, // short: homing run of axis buf[5], stops on its home input (state 4)
    RnrOp_SpindlePwm     = 0x0E, // short: spindle PWM, buf[5..6] = duty, buf[7..8] = period (1000)
    RnrOp_Configure      = 0x0F, // extended: pins, inversions, pulse rate, slice period
    RnrOp_StopJog        = 0x10  // short: drops the buffered slices of a jog; the slices sent next run
} rnr_opcode_t;

// rx[1]: controller state
typedef enum {
    RnrState_EStop   = 0,   // e-stop (hardware input or EStop frame): re-arm with PreReset + ClearEStop
    RnrState_Ready   = 1,   // armed, at rest
    RnrState_Jog     = 2,   // continuous jog run (StartJog)
    RnrState_Homing  = 4,   // homing run (ArmHome)
    RnrState_Probing = 5,   // probing run (ArmProbe)
    RnrState_Program = 6,   // program run (GcodeSlice)
    RnrState_Ending  = 7    // after EndOfMotion, until the buffered slices have run
} rnr_state_t;

// rx[2]: 0x81 at rest on the reference board; 0x20 follows the e-stop input (IN1) live
#define RNR_FAULT_LIMIT 0x10    // from the BreakoutCNC specification, not seen yet
#define RNR_FAULT_ESTOP 0x20

typedef struct {
    uint8_t  report_id;                 // rx[0] = 1
    uint8_t  state;                     // rx[1]: rnr_state_t
    uint8_t  fault_flags;               // rx[2]: RNR_FAULT_*
    uint16_t ack_seq;                   // rx[3..4]: last sequence number taken over (buffered, not executed)
    uint32_t status_word;               // rx[5..8]
    int32_t  live_steps[RNR_AXES];      // rx[9 + 8a]
    int32_t  latched_steps[RNR_AXES];   // rx[13 + 8a]: position of every axis at the last probe contact
    uint8_t  outputs;                   // rx[41]: OUT1..OUT4 as applied
    uint16_t inputs;                    // rx[42..43]: IN1..IN12 = bits 0..11
    uint16_t tick;                      // rx[44..45]: ~1 kHz counter
    uint16_t mpg;                       // rx[46..47] >> 2
} rnr_status_t;

typedef struct {
    uint8_t pulse_khz;          // 1..100 (Mach3: <RnRMotion><PulseRates>)
    uint8_t slice_ms;           // 5, as Mach3 configures it
    uint8_t step_dir_invert;    // bits 0-3: step active low X..A, bits 4-7: dir active low X..A
    // buf[5..14], input pins as Mach3 encodes them: one (limit, home) byte pair per axis, then
    // probe; low nibble = input number, other bits to be decoded. Example (reference machine,
    // homes on IN3 active low, probe on IN2): 00 73 00 73 00 73 00 70 12 00. The controller stops
    // on its e-stop input (IN1) by itself only with pins declared here.
    uint8_t pins[RNR_PIN_BYTES];
} rnr_config_t;

void rnr_build_short (uint8_t buf[RNR_OUT_BYTES], rnr_opcode_t op, uint16_t seq, uint8_t arg5, uint8_t arg6);
void rnr_build_slice (uint8_t buf[RNR_OUT_BYTES], rnr_opcode_t op, uint16_t seq, const int32_t steps[RNR_AXES], uint8_t outputs);
void rnr_build_positions (uint8_t buf[RNR_OUT_BYTES], uint16_t seq, const int32_t steps[RNR_AXES], uint8_t axis_mask);
void rnr_build_configure (uint8_t buf[RNR_OUT_BYTES], uint16_t seq, const rnr_config_t *cfg);
void rnr_build_pwm (uint8_t buf[RNR_OUT_BYTES], uint16_t seq, uint16_t duty);  // SpindlePwm: duty in 1/RNR_PWM_PERIOD
bool rnr_parse_status (const uint8_t *rx, int len, rnr_status_t *status);

// Helpers shared with the fake controller
int32_t rnr_get_le32 (const uint8_t *p);
uint16_t rnr_get_le16 (const uint8_t *p);
void rnr_put_le32 (uint8_t *p, uint32_t v);
void rnr_put_le16 (uint8_t *p, uint16_t v);

#endif
