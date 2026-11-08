/*
  sink_file.c - "file" slice sink (CSV)

  One line per slice: index, start time (s), dX, dY, dZ, dA, output flags, spindle PWM.
  Always accepts (no back-pressure): used for dry runs without hardware
  and for regression tests.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>

#include "slice_sink.h"

typedef struct {
    FILE *f;
    int32_t position[SLICE_AXES];
    uint32_t count;
    uint32_t dropped;       // slices pushed between stop() and resume()
    atomic_bool blocked;
} sink_file_ctx_t;

static sink_status_t file_push (slice_sink_t *sink, const slice_t *s)
{
    sink_file_ctx_t *c = sink->ctx;

    if(atomic_load(&c->blocked)) {
        c->dropped++;
        return Sink_Ok;
    }

    if(fprintf(c->f, "%u,%.3f,%d,%d,%d,%d,%u,%u\n", s->index, s->index * (SLICE_MS / 1000.0),
                s->steps[0], s->steps[1], s->steps[2], s->steps[3], s->out.flags, s->out.spindle_pwm) < 0)
        return Sink_Error;

    for(int i = 0; i < SLICE_AXES; i++)
        c->position[i] += s->steps[i];
    c->count++;

    return Sink_Ok;
}

static void file_get_position (slice_sink_t *sink, int32_t position[SLICE_AXES])
{
    memcpy(position, ((sink_file_ctx_t *)sink->ctx)->position, sizeof(int32_t) * SLICE_AXES);
}

static void file_stop (slice_sink_t *sink)
{
    sink_file_ctx_t *c = sink->ctx;

    atomic_store(&c->blocked, true);
    fprintf(c->f, "# stop\n");
}

static void file_resume (slice_sink_t *sink)
{
    atomic_store(&((sink_file_ctx_t *)sink->ctx)->blocked, false);
}

// A file "executes" every slice it has written: the halted position is the written total.
static bool file_get_halted_position (slice_sink_t *sink, int32_t position[SLICE_AXES], uint32_t timeout_ms)
{
    (void)timeout_ms;
    file_get_position(sink, position);

    return true;
}

static void file_close (slice_sink_t *sink)
{
    sink_file_ctx_t *c = sink->ctx;

    fprintf(c->f, "# end: %u slices, total X=%d Y=%d Z=%d A=%d steps, %u dropped after a stop\n", c->count,
             c->position[0], c->position[1], c->position[2], c->position[3], c->dropped);
    fclose(c->f);
    free(c);
    free(sink);
}

slice_sink_t *sink_file_open (const char *path)
{
    slice_sink_t *sink = calloc(1, sizeof(slice_sink_t));
    sink_file_ctx_t *c = calloc(1, sizeof(sink_file_ctx_t));

    if(!sink || !c || !(c->f = fopen(path, "w"))) {
        free(c);
        free(sink);
        return NULL;
    }

    fprintf(c->f, "# GRBL2MACH %d ms step slices\n# index,t_s,dx,dy,dz,da,out,pwm\n", SLICE_MS);

    sink->name = "file";
    sink->push = file_push;
    sink->get_position = file_get_position;
    sink->stop = file_stop;
    sink->resume = file_resume;
    sink->get_halted_position = file_get_halted_position;
    sink->close = file_close;
    sink->ctx = c;

    return sink;
}
