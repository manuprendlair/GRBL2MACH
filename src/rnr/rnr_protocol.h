/*
  rnr_protocol.h - RNR Eco Motion 2.0 USB HID protocol (frames and status report)

  Written from the interoperability specification published by BreakoutCNC
  (docs/protocol/PROTOCOL_SPEC.md) and checked against static analysis of the
  vendor Mach3 plugin. No vendor code is used.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef rnr_protocol_h
#define rnr_protocol_h

#include <stdint.h>
#include <stdbool.h>

#define RNR_VID 0xA720
#define RNR_PID 0xF803

#define RNR_OUT_BYTES 27    // PC -> controller, no report ID
#define RNR_IN_BYTES  49    // controller -> PC, no report ID
#define RNR_AXES      4

#define RNR_FRAME_SHORT    0x02
#define RNR_FRAME_EXTENDED 0x03

#define RNR_FLAG_LIMIT_OVERRIDE 0x40

typedef enum {
    RnrOp_EStop          = 0x01, // short: immediate halt
    RnrOp_ClearEStop     = 0x02, // short: enable, sent after PreReset
    RnrOp_SetOutputs     = 0x03, // short: buf[5] = OUT1..OUT4
    RnrOp_GcodeSlice     = 0x04, // extended: motion slice
    RnrOp_JogSlice       = 0x05, // extended: motion slice (used for program runs, as BreakoutCNC does)
    RnrOp_EndOfMotion    = 0x06, // short: end of a run, buf[5] = outputs
    RnrOp_PreReset       = 0x07, // short: sent before ClearEStop
    RnrOp_Cancel         = 0x08, // short: abort active sequence
    RnrOp_SetPositions   = 0x09, // extended: X/Y/Z/A step counters
    RnrOp_StartJog       = 0x0A, // short: start continuous jog / start of a run, buf[5] = axis
    RnrOp_StartStepJog   = 0x0B, // short: step / MPG jog
    RnrOp_ArmProbe       = 0x0C, // short: arm hardware probe latch (G31)
    RnrOp_ArmHome        = 0x0D, // short: arm hardware home switch stop
    RnrOp_SpindlePwm     = 0x0E, // short: spindle PWM
    RnrOp_Configure      = 0x0F, // extended: pinout, inversions, pulse rate, slice period
    RnrOp_StopJog        = 0x10  // short: jog deceleration
} rnr_opcode_t;

#define RNR_FAULT_LIMIT 0x10
#define RNR_FAULT_ESTOP 0x20

typedef struct {
    uint8_t  report_id;                 // rx[0]
    uint8_t  motion_state;              // rx[1]: 0 = idle, 1 = busy
    uint8_t  fault_flags;               // rx[2]: RNR_FAULT_*
    uint16_t ack_seq;                   // rx[3..4]: last sequence number taken over (buffered, not executed)
    uint32_t status_word;               // rx[5..8]
    int32_t  live_steps[RNR_AXES];      // rx[9 + 8a]
    int32_t  latched_steps[RNR_AXES];   // rx[13 + 8a]: probe / home latch
    uint8_t  aux_status;                // rx[41]
    uint16_t inputs;                    // rx[42..43]: IN1..IN12 = bits 0..11
    uint16_t tick;                      // rx[44..45]: 200 Hz counter
    uint16_t mpg;                       // rx[46..47] >> 2
} rnr_status_t;

typedef struct {
    uint8_t pulse_khz;          // 1..100
    uint8_t slice_ms;           // 25
    uint8_t step_dir_invert;    // low nibble: step active low X..A, high nibble: dir active low X..A
    // Pin assignments (limits, home, probe, e-stop) are left at 0 (disabled) for now.
} rnr_config_t;

void rnr_build_short (uint8_t buf[RNR_OUT_BYTES], rnr_opcode_t op, uint16_t seq, bool limit_override, uint8_t arg5, uint8_t arg6);
void rnr_build_slice (uint8_t buf[RNR_OUT_BYTES], rnr_opcode_t op, uint16_t seq, bool limit_override, const int32_t steps[RNR_AXES], uint8_t outputs);
void rnr_build_positions (uint8_t buf[RNR_OUT_BYTES], uint16_t seq, const int32_t steps[RNR_AXES]);
void rnr_build_configure (uint8_t buf[RNR_OUT_BYTES], uint16_t seq, const rnr_config_t *cfg);
bool rnr_parse_status (const uint8_t *rx, int len, rnr_status_t *status);

// Helpers shared with the fake controller
int32_t rnr_get_le32 (const uint8_t *p);
uint16_t rnr_get_le16 (const uint8_t *p);
void rnr_put_le32 (uint8_t *p, uint32_t v);
void rnr_put_le16 (uint8_t *p, uint16_t v);

#endif
