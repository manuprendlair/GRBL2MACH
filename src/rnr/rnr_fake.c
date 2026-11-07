/*
  rnr_fake.c - fake RNR controller for tests

  Behaves like the real controller as far as it is known:
  - acknowledges a frame as soon as it is buffered (ack_seq), not when executed;
  - executes one buffered slice every slice period while a run is open
    (StartJog .. EndOfMotion) and the controller is not in E-stop;
  - starts in E-stop; PreReset + ClearEStop enable it, EStop disables it and
    drops the buffer;
  - sends a status report every 5 ms (200 Hz).

  Every frame received is logged with a timestamp. The log ends with statistics
  that the tests check: maximum buffer depth (pacing), slices rejected in
  E-stop, sequence errors, starvation inside a run, final positions.

  GRBL2MACH_FAKE_SILENT_MS=<n>: stop reporting n ms after the first slice
  (watchdog test).

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rnr_transport.h"
#include "rnr_os.h"

#define BUFFER_SLICES   400     // the real controller buffers several seconds
#define REPORT_US       5000    // 200 Hz status reports

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
    bool estop, run_open, armed_once;
    uint16_t ack_seq, last_seq;
    bool seq_started;
    int32_t live[RNR_AXES];
    uint8_t outputs;
    // statistics
    int max_depth, rejected_estop, seq_errors, overflows, starved, executed, frames;
    int idle_in_run;    // empty periods since the last executed slice of the open run
    uint64_t first_slice_us;
    int64_t silent_ms;
} fake_t;

static double ms (fake_t *f, uint64_t t)
{
    return (t - f->t0) / 1000.0;
}

// Executes the slices that are due at time now.
static void run_until (fake_t *f, uint64_t now)
{
    while(now >= f->next_exec) {
        if(f->run_open && !f->estop) {
            if(f->count) {
                fake_slice_t *s = &f->buf[f->head];
                for(int a = 0; a < RNR_AXES; a++)
                    f->live[a] += s->steps[a];
                f->outputs = s->outputs;
                f->head = (f->head + 1) % BUFFER_SLICES;
                f->count--;
                f->executed++;
                f->idle_in_run = 0;
            } else
                f->idle_in_run++;   // run open but nothing to execute (normal at the end of a run)
        }
        f->next_exec += f->slice_us;
    }
}

static int fake_write (rnr_transport_t *t, const uint8_t b[RNR_OUT_BYTES])
{
    fake_t *f = t->ctx;
    uint64_t now = rnr_now_us();
    uint16_t seq = rnr_get_le16(&b[3]);
    uint8_t op = b[1];

    run_until(f, now);
    f->frames++;

    if(f->seq_started && seq != (uint16_t)(f->last_seq + 1)) {
        f->seq_errors++;
        fprintf(f->log, "%10.3f SEQ ERROR: got %u after %u\n", ms(f, now), seq, f->last_seq);
    }
    f->seq_started = true;
    f->last_seq = seq;
    f->ack_seq = seq;   // acknowledged once buffered

    fprintf(f->log, "%10.3f op=0x%02X seq=%u flags=0x%02X", ms(f, now), op, seq, b[2]);

    switch(op) {

        case RnrOp_EStop:
            fprintf(f->log, " halted pos=%d,%d,%d,%d dropped=%d", f->live[0], f->live[1], f->live[2], f->live[3], f->count);
            f->estop = true;
            f->run_open = false;
            f->count = 0;
            break;

        case RnrOp_PreReset:
            break;

        case RnrOp_ClearEStop:
            f->estop = false;
            f->armed_once = true;
            break;

        case RnrOp_Cancel:
            f->run_open = false;
            f->count = 0;
            break;

        case RnrOp_SetOutputs:
        case RnrOp_EndOfMotion:
            fprintf(f->log, " outputs=0x%02X", b[5]);
            f->outputs = b[5];
            if(op == RnrOp_EndOfMotion) {
                if(f->count)
                    fprintf(f->log, " (%d slices still buffered)", f->count);
                f->run_open = false;
            }
            break;

        case RnrOp_StartJog:
            fprintf(f->log, " axis=%u", b[5]);
            f->run_open = true;
            f->next_exec = now + f->slice_us;
            f->idle_in_run = 0;
            break;

        case RnrOp_SetPositions:
            for(int a = 0; a < RNR_AXES; a++)
                f->live[a] = rnr_get_le32(&b[5 + a * 4]);
            fprintf(f->log, " pos=%d,%d,%d,%d", f->live[0], f->live[1], f->live[2], f->live[3]);
            break;

        case RnrOp_Configure:
            fprintf(f->log, " khz=%u slice_ms=%u invert=0x%02X", b[16], b[17], b[15]);
            if(b[17])
                f->slice_us = b[17] * 1000u;
            break;

        case RnrOp_GcodeSlice:
        case RnrOp_JogSlice: {
            fake_slice_t s;
            for(int a = 0; a < RNR_AXES; a++)
                s.steps[a] = rnr_get_le32(&b[9 + a * 4]);
            s.outputs = b[25];
            fprintf(f->log, " d=%d,%d,%d,%d out=0x%02X", s.steps[0], s.steps[1], s.steps[2], s.steps[3], s.outputs);
            if(!f->first_slice_us)
                f->first_slice_us = now;
            if(f->estop) {
                f->rejected_estop++;
                fprintf(f->log, " REJECTED (e-stop)");
            } else if(f->count == BUFFER_SLICES) {
                f->overflows++;
                fprintf(f->log, " REJECTED (buffer full)");
            } else {
                if(f->run_open && f->executed && f->idle_in_run) {
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
    fprintf(f->log, "\n");

    return RNR_IO_OK;
}

static int fake_read (rnr_transport_t *t, uint8_t rx[RNR_IN_BYTES], uint32_t timeout_ms)
{
    fake_t *f = t->ctx;
    uint64_t now = rnr_now_us();
    uint64_t deadline = now + (uint64_t)timeout_ms * 1000u;

    if(f->silent_ms >= 0 && f->first_slice_us && now >= f->first_slice_us + (uint64_t)f->silent_ms * 1000u) {
        rnr_sleep_ms(timeout_ms);
        return RNR_IO_TIMEOUT;
    }

    if(f->next_report > deadline) {
        rnr_sleep_ms(timeout_ms);
        run_until(f, rnr_now_us());
        return RNR_IO_TIMEOUT;
    }
    while((now = rnr_now_us()) < f->next_report)
        rnr_sleep_ms(1);
    f->next_report += REPORT_US;
    if(f->next_report < now)
        f->next_report = now + REPORT_US;

    run_until(f, now);

    memset(rx, 0, RNR_IN_BYTES);
    rx[1] = (f->run_open && f->count) ? 1 : 0;
    rx[2] = f->estop ? RNR_FAULT_ESTOP : 0;
    rnr_put_le16(&rx[3], f->ack_seq);
    for(int a = 0; a < RNR_AXES; a++)
        rnr_put_le32(&rx[9 + a * 8], (uint32_t)f->live[a]);
    rnr_put_le16(&rx[44], (uint16_t)((now - f->t0) / 5000u));

    return RNR_IO_OK;
}

static void fake_close (rnr_transport_t *t)
{
    fake_t *f = t->ctx;

    run_until(f, rnr_now_us());
    fprintf(f->log, "# frames=%d executed=%d max_depth=%d starved=%d rejected_estop=%d overflows=%d seq_errors=%d still_buffered=%d\n",
             f->frames, f->executed, f->max_depth, f->starved, f->rejected_estop, f->overflows, f->seq_errors, f->count);
    fprintf(f->log, "# final position X=%d Y=%d Z=%d A=%d estop=%d\n", f->live[0], f->live[1], f->live[2], f->live[3], f->estop);
    fclose(f->log);
    free(f);
    free(t);
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
    f->slice_us = 25000;
    f->estop = true;    // like a controller that has just been powered or reset
    f->silent_ms = silent ? atoll(silent) : -1;
    fprintf(f->log, "# GRBL2MACH fake RNR controller log (time in ms)\n");

    t->name = "fake";
    t->write = fake_write;
    t->read = fake_read;
    t->close = fake_close;
    t->ctx = f;

    return t;
}
