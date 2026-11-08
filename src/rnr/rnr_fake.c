/*
  rnr_fake.c - fake RNR controller for tests

  Behaves like the real controller as seen in USB captures of Mach3:
  - acknowledges a frame as soon as it is buffered (ack_seq), not when executed;
  - state machine reported in rx[1]: 0 e-stop, 1 ready, 2 jog, 6 program, 7 ending;
  - starts in e-stop; PreReset + ClearEStop arm it; EStop disarms it and drops the buffer;
  - a GcodeSlice opens a program run, StartJog a jog run; one buffered slice is executed
    every slice period (Configure byte 17, 5 ms); EndOfMotion lets the buffer run out,
    then back to ready;
  - Cancel stops at once and drops the buffer, the controller stays armed; StopJog drops
    the buffered slices of a jog, the slices sent after it run;
  - slices received while not armed are rejected (counted);
  - hardware e-stop input (IN1): stops at once, drops the buffer, state 0, fault flag 0x20
    while the input is active; re-arming is needed after release;
  - homing run (ArmHome, axis in byte 5, state 4) and probing run (ArmProbe, state 5) execute
    JogSlices and stop by themselves when the home switch of the axis / the probe triggers
    (position clamped to the switch, buffer dropped, back to ready); a probe contact latches
    the four positions in rx[13 + 8a]. JogSlices received while ready are ignored (counted);
  - a status report every 50 ms, like the real board.

  Every frame received is logged with a timestamp. The log ends with statistics
  that the tests check: maximum buffer depth (pacing), slices rejected,
  sequence errors, starvation inside a run, final positions.

  Environment (times in ms after the first slice):
    GRBL2MACH_FAKE_SILENT_MS=<n>         stop reporting (watchdog test)
    GRBL2MACH_FAKE_ESTOP_MS=<on>,<off>   press the e-stop input at <on>, release at <off>
    GRBL2MACH_FAKE_HOME=<x>,<y>,<z>,<a>  home switches: axis a triggered at live[a] >= <a> steps
                                         (homes at the positive end); wired to the inputs the
                                         configuration frame declares
    GRBL2MACH_FAKE_HOME_HYST=<n>         home switch hysteresis: once triggered, released only
                                         <n> steps below its trigger position (default 0)
    GRBL2MACH_FAKE_PROBE_Z=<z>           probe triggered at live Z <= <z> steps
  The e-stop input is IN1, active high. As the real board, the fake one stops by itself on it only
  when the configuration frame declares it (byte 13, high nibble = 1); otherwise it only reports it.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rnr_transport.h"
#include "rnr_os.h"

#define BUFFER_SLICES   400     // the real controller buffers at least 100 slices
#define REPORT_US       50000   // a status report every 50 ms (real board: 48 to 51 ms)

typedef struct {
    int32_t steps[RNR_AXES];
    uint8_t outputs;
} fake_slice_t;

typedef struct {
    FILE *log;
    uint64_t t0, next_report, next_exec;
    uint32_t slice_us;
    fake_slice_t buf[BUFFER_SLICES];
    int head, count;
    uint8_t state;
    bool estop_input;
    uint16_t ack_seq, last_seq;
    bool seq_started;
    int32_t live[RNR_AXES];
    uint8_t outputs;
    uint16_t pwm;
    // statistics
    int max_depth, rejected, seq_errors, overflows, starved, executed, frames, estops;
    int idle_in_run;    // empty periods since the last executed slice of the open run
    uint64_t first_slice_us;
    int64_t silent_ms, estop_on_ms, estop_off_ms;
    bool estop_self;    // e-stop input declared in Configure: stops by itself on it (else only reports it)
    // inputs declared in Configure (pin 0 = none) and switch positions (steps)
    uint8_t home_pin[RNR_AXES], probe_pin;
    bool home_low[RNR_AXES], probe_low;
    bool has_home[RNR_AXES], has_probe;
    int32_t home_pos[RNR_AXES], probe_z;
    int32_t home_hyst;              // a triggered switch releases only that far below its trigger point
    bool home_on[RNR_AXES];
    int home_axis;                  // axis of the homing run
    int32_t latched[RNR_AXES];      // positions at the last probe contact
    int ignored, homes, probes;
} fake_t;

static double ms (fake_t *f, uint64_t t)
{
    return (t - f->t0) / 1000.0;
}

// Output changes are logged when they take effect, with the position at that time
static void set_outputs (fake_t *f, uint64_t now, uint8_t outputs)
{
    if(outputs != f->outputs)
        fprintf(f->log, "%10.3f OUTPUTS 0x%02X -> 0x%02X pwm=%u pos=%d,%d,%d,%d buffered=%d\n", ms(f, now), f->outputs, outputs,
                 f->pwm, f->live[0], f->live[1], f->live[2], f->live[3], f->count);
    f->outputs = outputs;
}

static bool running (fake_t *f)
{
    return f->state == RnrState_Jog || f->state == RnrState_Program || f->state == RnrState_Ending ||
            f->state == RnrState_Homing || f->state == RnrState_Probing;
}

static bool home_switch (fake_t *f, int a)
{
    if(!f->has_home[a])
        return false;
    if(f->live[a] >= f->home_pos[a])
        f->home_on[a] = true;
    else if(f->live[a] < f->home_pos[a] - f->home_hyst)
        f->home_on[a] = false;

    return f->home_on[a];
}

static bool probe_switch (fake_t *f)
{
    return f->has_probe && f->live[2] <= f->probe_z;
}

// Raw inputs IN1..IN12 as the board reports them: at rest IN3 and IN4 high (active low switches)
static uint16_t raw_inputs (fake_t *f)
{
    uint16_t in = 0x000C | (f->estop_input ? 0x0001 : 0);

    for(int a = 0; a < RNR_AXES; a++)
        if(f->home_pin[a] && home_switch(f, a))
            in = (uint16_t)(f->home_low[a] ? in & ~(1u << (f->home_pin[a] - 1)) : in | (1u << (f->home_pin[a] - 1)));
    if(f->probe_pin && probe_switch(f))
        in = (uint16_t)(f->probe_low ? in & ~(1u << (f->probe_pin - 1)) : in | (1u << (f->probe_pin - 1)));

    return in;
}

// Homing / probing run: the board stops by itself on the switch, exactly where it triggers
static void stop_on_switch (fake_t *f, uint64_t t, const char *what)
{
    fprintf(f->log, "%10.3f %s: halted pos=%d,%d,%d,%d dropped=%d\n", ms(f, t), what,
             f->live[0], f->live[1], f->live[2], f->live[3], f->count);
    f->state = RnrState_Ready;
    f->count = 0;
}

// Executes the slices that are due at time now.
static void run_until (fake_t *f, uint64_t now)
{
    while(now >= f->next_exec) {
        if(running(f)) {
            if(f->count) {
                fake_slice_t *s = &f->buf[f->head];
                int ha = f->home_axis;
                if(f->state == RnrState_Homing && f->home_pin[ha] && home_switch(f, ha)) {
                    f->homes++;
                    stop_on_switch(f, f->next_exec, "HOME SWITCH");   // already on the switch
                    f->next_exec += f->slice_us;
                    continue;
                }
                for(int a = 0; a < RNR_AXES; a++)
                    f->live[a] += s->steps[a];
                if((f->state == RnrState_Homing && f->home_pin[ha] && home_switch(f, ha)) ||
                    (f->state == RnrState_Probing && f->probe_pin && probe_switch(f))) {
                    // stops on the step that closes the switch
                    bool home = f->state == RnrState_Homing;
                    if(home) {
                        f->live[ha] = f->home_pos[ha];
                        f->homes++;
                    } else {
                        f->live[2] = f->probe_z;
                        memcpy(f->latched, f->live, sizeof(f->latched));
                        f->probes++;
                    }
                    f->head = (f->head + 1) % BUFFER_SLICES;
                    f->count--;
                    f->executed++;
                    stop_on_switch(f, f->next_exec, home ? "HOME SWITCH" : "PROBE CONTACT");
                    f->next_exec += f->slice_us;
                    continue;
                }
                set_outputs(f, f->next_exec, s->outputs);
                f->head = (f->head + 1) % BUFFER_SLICES;
                f->count--;
                f->executed++;
                f->idle_in_run = 0;
            } else if(f->state == RnrState_Ending)
                f->state = RnrState_Ready;
            else
                f->idle_in_run++;   // run open but nothing to execute (normal at the end of a run)
        }
        f->next_exec += f->slice_us;
    }
}

// Hardware e-stop input, driven by GRBL2MACH_FAKE_ESTOP_MS
static void update_estop_input (fake_t *f, uint64_t now)
{
    if(f->estop_on_ms < 0 || !f->first_slice_us)
        return;

    int64_t t = (int64_t)(now - f->first_slice_us) / 1000;
    bool active = t >= f->estop_on_ms && t < f->estop_off_ms;

    if(active && !f->estop_input && f->estop_self) {
        fprintf(f->log, "%10.3f E-STOP INPUT pressed: halted pos=%d,%d,%d,%d dropped=%d\n",
                 ms(f, now), f->live[0], f->live[1], f->live[2], f->live[3], f->count);
        f->state = RnrState_EStop;
        f->count = 0;
        f->estops++;
    } else if(active && !f->estop_input) {
        fprintf(f->log, "%10.3f E-STOP INPUT pressed (reported only): pos=%d,%d,%d,%d buffered=%d\n",
                 ms(f, now), f->live[0], f->live[1], f->live[2], f->live[3], f->count);
        f->estops++;
    } else if(!active && f->estop_input)
        fprintf(f->log, "%10.3f E-STOP INPUT released\n", ms(f, now));
    f->estop_input = active;
}

static int fake_write (rnr_transport_t *t, const uint8_t b[RNR_OUT_BYTES])
{
    fake_t *f = t->ctx;
    uint64_t now = rnr_now_us();
    uint16_t seq = rnr_get_le16(&b[3]);
    uint8_t op = b[1];

    run_until(f, now);
    update_estop_input(f, now);
    f->frames++;

    if(f->seq_started && seq != (uint16_t)(f->last_seq + 1)) {
        f->seq_errors++;
        fprintf(f->log, "%10.3f SEQ ERROR: got %u after %u\n", ms(f, now), seq, f->last_seq);
    }
    f->seq_started = true;
    f->last_seq = seq;
    f->ack_seq = seq;   // acknowledged once buffered

    int new_outputs = -1;     // SetOutputs / EndOfMotion: applied after the frame is logged
    fprintf(f->log, "%10.3f op=0x%02X seq=%u flags=0x%02X", ms(f, now), op, seq, b[2]);

    switch(op) {

        case RnrOp_EStop:
            fprintf(f->log, " halted pos=%d,%d,%d,%d dropped=%d", f->live[0], f->live[1], f->live[2], f->live[3], f->count);
            f->state = RnrState_EStop;
            f->count = 0;
            break;

        case RnrOp_Cancel:
            fprintf(f->log, " halted pos=%d,%d,%d,%d dropped=%d", f->live[0], f->live[1], f->live[2], f->live[3], f->count);
            f->count = 0;
            if(f->state != RnrState_EStop)
                f->state = RnrState_Ready;
            break;

        case RnrOp_StopJog:
            fprintf(f->log, " dropped=%d", f->count);
            f->count = 0;
            break;

        case RnrOp_PreReset:
            break;

        case RnrOp_ClearEStop:
            if(f->estop_input && f->estop_self)
                fprintf(f->log, " IGNORED (e-stop input active)");
            else if(f->state == RnrState_EStop)
                f->state = RnrState_Ready;
            break;

        case RnrOp_SetOutputs:
        case RnrOp_EndOfMotion:
            fprintf(f->log, " outputs=0x%02X", b[5]);
            new_outputs = b[5];
            if(op == RnrOp_EndOfMotion && (f->state == RnrState_Jog || f->state == RnrState_Program ||
                                               f->state == RnrState_Homing || f->state == RnrState_Probing)) {
                if(f->count)
                    fprintf(f->log, " (%d slices still buffered)", f->count);
                f->state = f->count ? RnrState_Ending : RnrState_Ready;
            }
            break;

        case RnrOp_SpindlePwm:
            f->pwm = rnr_get_le16(&b[5]);
            fprintf(f->log, " pwm=%u/%u", f->pwm, rnr_get_le16(&b[7]));
            break;

        case RnrOp_StartJog:
            fprintf(f->log, " axis=%u", b[5]);
            if(f->state == RnrState_Ready) {
                f->state = RnrState_Jog;
                f->next_exec = now + f->slice_us;
                f->idle_in_run = 0;
            }
            break;

        case RnrOp_ArmHome:
        case RnrOp_ArmProbe:
            if(op == RnrOp_ArmHome)
                fprintf(f->log, " axis=%u", b[5]);
            if(f->state == RnrState_Ready) {
                f->state = op == RnrOp_ArmHome ? RnrState_Homing : RnrState_Probing;
                f->home_axis = b[5] & 3;
                f->next_exec = now + f->slice_us;
                f->idle_in_run = 0;
            } else
                fprintf(f->log, " IGNORED (state %u)", f->state);
            break;

        case RnrOp_SetPositions:
            for(int a = 0; a < RNR_AXES; a++)
                if(b[21] & (1 << a))
                    f->live[a] = rnr_get_le32(&b[5 + a * 4]);
            fprintf(f->log, " pos=%d,%d,%d,%d mask=0x%02X", f->live[0], f->live[1], f->live[2], f->live[3], b[21]);
            break;

        case RnrOp_Configure:
            fprintf(f->log, " khz=%u slice_ms=%u invert=0x%02X", b[16], b[17], b[15]);
            fprintf(f->log, " pins=");
            for(int i = 5; i <= 14; i++)
                fprintf(f->log, "%02X", b[i]);
            if(b[17])
                f->slice_us = b[17] * 1000u;
            f->estop_self = (b[13] >> 4) == 1 && !(b[14] & 0x20);    // e-stop declared on IN1, active high
            for(int a = 0; a < RNR_AXES; a++) {
                f->home_pin[a] = b[6 + 2 * a] & 0x0F;
                f->home_low[a] = (b[6 + 2 * a] & 0x40) != 0;
            }
            f->probe_pin = b[13] & 0x0F;
            f->probe_low = (b[14] & 0x10) != 0;
            break;

        case RnrOp_GcodeSlice:
        case RnrOp_JogSlice: {
            fake_slice_t s;
            for(int a = 0; a < RNR_AXES; a++)
                s.steps[a] = rnr_get_le32(&b[9 + a * 4]);
            s.outputs = b[25];
            fprintf(f->log, " d=%d,%d,%d,%d", s.steps[0], s.steps[1], s.steps[2], s.steps[3]);
            if(s.outputs)
                fprintf(f->log, " out=0x%02X", s.outputs);
            if(!f->first_slice_us)
                f->first_slice_us = now;
            if(op == RnrOp_GcodeSlice && (f->state == RnrState_Ready || f->state == RnrState_Ending)) {
                if(f->state == RnrState_Ready) {
                    f->next_exec = now + f->slice_us;
                    f->idle_in_run = 0;
                }
                f->state = RnrState_Program;
            }
            if(op == RnrOp_JogSlice && f->state == RnrState_Ready) {
                f->ignored++;       // as the real board after a home switch or probe contact
                fprintf(f->log, " IGNORED (ready)");
            } else if(!running(f)) {
                f->rejected++;
                fprintf(f->log, " REJECTED (state %u)", f->state);
            } else if(f->count == BUFFER_SLICES) {
                f->overflows++;
                fprintf(f->log, " REJECTED (buffer full)");
            } else {
                if(f->executed && f->idle_in_run) {
                    f->starved += f->idle_in_run;   // underrun: the run went dry, then got more slices
                    fprintf(f->log, " UNDERRUN(%d)", f->idle_in_run);
                    f->idle_in_run = 0;
                }
                f->buf[(f->head + f->count) % BUFFER_SLICES] = s;
                f->count++;
                if(f->count > f->max_depth)
                    f->max_depth = f->count;
                fprintf(f->log, " depth=%d", f->count);
            }
            break;
        }

        default:
            break;
    }
    fprintf(f->log, " state=%u\n", f->state);
    if(new_outputs >= 0)
        set_outputs(f, now, (uint8_t)new_outputs);

    return RNR_IO_OK;
}

static int fake_read (rnr_transport_t *t, uint8_t rx[RNR_IN_BYTES], uint32_t timeout_ms)
{
    fake_t *f = t->ctx;
    uint64_t now = rnr_now_us();
    uint64_t deadline = now + (uint64_t)timeout_ms * 1000u;

    run_until(f, now);
    update_estop_input(f, now);

    if(f->silent_ms >= 0 && f->first_slice_us && now >= f->first_slice_us + (uint64_t)f->silent_ms * 1000u) {
        rnr_sleep_ms(timeout_ms);
        return RNR_IO_TIMEOUT;
    }

    if(f->next_report > deadline) {
        rnr_sleep_ms(timeout_ms);
        return RNR_IO_TIMEOUT;
    }
    while((now = rnr_now_us()) < f->next_report)
        rnr_sleep_ms(1);
    f->next_report += REPORT_US;
    if(f->next_report < now)
        f->next_report = now + REPORT_US;

    run_until(f, now);
    update_estop_input(f, now);

    memset(rx, 0, RNR_IN_BYTES);
    rx[0] = 0x01;
    rx[1] = f->state;
    rx[2] = 0x81 | (f->estop_input && f->estop_self ? RNR_FAULT_ESTOP : 0);
    rnr_put_le16(&rx[3], f->ack_seq);
    for(int a = 0; a < RNR_AXES; a++)
        rnr_put_le32(&rx[9 + a * 8], (uint32_t)f->live[a]);
    rx[41] = f->outputs;
    for(int a = 0; a < RNR_AXES; a++)
        rnr_put_le32(&rx[13 + a * 8], (uint32_t)f->latched[a]);
    rnr_put_le16(&rx[42], raw_inputs(f));
    rnr_put_le16(&rx[44], (uint16_t)((now - f->t0) / 1000u));

    return RNR_IO_OK;
}

static void fake_close (rnr_transport_t *t)
{
    fake_t *f = t->ctx;

    run_until(f, rnr_now_us());
    fprintf(f->log, "# frames=%d executed=%d max_depth=%d starved=%d rejected=%d overflows=%d seq_errors=%d still_buffered=%d estops=%d ignored=%d homes=%d probes=%d\n",
             f->frames, f->executed, f->max_depth, f->starved, f->rejected, f->overflows, f->seq_errors, f->count, f->estops,
             f->ignored, f->homes, f->probes);
    fprintf(f->log, "# final position X=%d Y=%d Z=%d A=%d state=%u\n", f->live[0], f->live[1], f->live[2], f->live[3], f->state);
    fclose(f->log);
    free(f);
    free(t);
}

static void parse_estop_env (fake_t *f)
{
    const char *e = getenv("GRBL2MACH_FAKE_ESTOP_MS");
    long long on, off;
    f->estop_on_ms = f->estop_off_ms = -1;
    if(e && sscanf(e, "%lld,%lld", &on, &off) == 2 && on >= 0 && off > on) {
        f->estop_on_ms = on;
        f->estop_off_ms = off;
    }
}

rnr_transport_t *rnr_fake_open (const char *log_path)
{
    rnr_transport_t *t = calloc(1, sizeof(rnr_transport_t));
    fake_t *f = calloc(1, sizeof(fake_t));
    const char *silent = getenv("GRBL2MACH_FAKE_SILENT_MS");

    if(!t || !f || !(f->log = fopen(log_path, "w"))) {
        free(f);
        free(t);
        return NULL;
    }

    f->t0 = rnr_now_us();
    f->next_report = f->t0;
    f->next_exec = f->t0;
    f->slice_us = 5000;
    f->state = RnrState_EStop;  // like a controller that has just been powered
    f->silent_ms = silent ? atoll(silent) : -1;
    parse_estop_env(f);
    const char *h = getenv("GRBL2MACH_FAKE_HOME"), *pz = getenv("GRBL2MACH_FAKE_PROBE_Z");
    long x, y, z, aa;
    if(h && sscanf(h, "%ld,%ld,%ld,%ld", &x, &y, &z, &aa) == 4) {
        long v[RNR_AXES] = { x, y, z, aa };
        for(int a = 0; a < RNR_AXES; a++) {
            f->home_pos[a] = (int32_t)v[a];
            f->has_home[a] = true;
        }
    }
    const char *hy = getenv("GRBL2MACH_FAKE_HOME_HYST");
    if(hy)
        f->home_hyst = (int32_t)atol(hy);
    if(pz) {
        f->probe_z = (int32_t)atol(pz);
        f->has_probe = true;
    }
    fprintf(f->log, "# GRBL2MACH fake RNR controller log (time in ms)\n");

    t->name = "fake";
    t->write = fake_write;
    t->read = fake_read;
    t->close = fake_close;
    t->ctx = f;

    return t;
}
