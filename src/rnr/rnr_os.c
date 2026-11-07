/*
  rnr_os.c - minimal OS helpers for the RNR sink: monotonic clock, threads, mutex

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdlib.h>

#include "rnr_os.h"

typedef struct {
    void (*fn)(void *);
    void *arg;
} thread_start_t;

#ifdef _WIN32

uint64_t rnr_now_us (void)
{
    static LARGE_INTEGER freq = {0};
    LARGE_INTEGER now;

    if(freq.QuadPart == 0)
        QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);

    return (uint64_t)(now.QuadPart / freq.QuadPart) * 1000000u +
            (uint64_t)(now.QuadPart % freq.QuadPart) * 1000000u / (uint64_t)freq.QuadPart;
}

void rnr_sleep_ms (uint32_t ms)
{
    Sleep(ms);
}

static DWORD WINAPI thread_entry (LPVOID p)
{
    thread_start_t s = *(thread_start_t *)p;
    free(p);
    s.fn(s.arg);
    return 0;
}

int rnr_thread_start (rnr_thread_t *t, void (*fn)(void *), void *arg)
{
    thread_start_t *s = malloc(sizeof(thread_start_t));

    if(!s)
        return -1;
    s->fn = fn;
    s->arg = arg;
    if(!(*t = CreateThread(NULL, 0, thread_entry, s, 0, NULL))) {
        free(s);
        return -1;
    }
    // The I/O thread paces USB traffic: give it priority over the simulator threads
    SetThreadPriority(*t, THREAD_PRIORITY_HIGHEST);

    return 0;
}

void rnr_thread_join (rnr_thread_t t)
{
    WaitForSingleObject(t, INFINITE);
    CloseHandle(t);
}

void rnr_mutex_init (rnr_mutex_t *m)    { InitializeCriticalSection(m); }
void rnr_mutex_lock (rnr_mutex_t *m)    { EnterCriticalSection(m); }
void rnr_mutex_unlock (rnr_mutex_t *m)  { LeaveCriticalSection(m); }
void rnr_mutex_destroy (rnr_mutex_t *m) { DeleteCriticalSection(m); }

#else

#include <time.h>

uint64_t rnr_now_us (void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

void rnr_sleep_ms (uint32_t ms)
{
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L };

    nanosleep(&ts, NULL);
}

static void *thread_entry (void *p)
{
    thread_start_t s = *(thread_start_t *)p;
    free(p);
    s.fn(s.arg);
    return NULL;
}

int rnr_thread_start (rnr_thread_t *t, void (*fn)(void *), void *arg)
{
    thread_start_t *s = malloc(sizeof(thread_start_t));

    if(!s)
        return -1;
    s->fn = fn;
    s->arg = arg;
    if(pthread_create(t, NULL, thread_entry, s) != 0) {
        free(s);
        return -1;
    }

    return 0;
}

void rnr_thread_join (rnr_thread_t t)
{
    pthread_join(t, NULL);
}

void rnr_mutex_init (rnr_mutex_t *m)    { pthread_mutex_init(m, NULL); }
void rnr_mutex_lock (rnr_mutex_t *m)    { pthread_mutex_lock(m); }
void rnr_mutex_unlock (rnr_mutex_t *m)  { pthread_mutex_unlock(m); }
void rnr_mutex_destroy (rnr_mutex_t *m) { pthread_mutex_destroy(m); }

#endif
