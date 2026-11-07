/*
  rnr_os.h - minimal OS helpers for the RNR sink: monotonic clock, threads, mutex

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef rnr_os_h
#define rnr_os_h

#include <stdint.h>

#ifdef _WIN32
#include <windows.h>
typedef HANDLE rnr_thread_t;
typedef CRITICAL_SECTION rnr_mutex_t;
#else
#include <pthread.h>
typedef pthread_t rnr_thread_t;
typedef pthread_mutex_t rnr_mutex_t;
#endif

uint64_t rnr_now_us (void);             // monotonic, microseconds
void rnr_sleep_ms (uint32_t ms);

int rnr_thread_start (rnr_thread_t *t, void (*fn)(void *), void *arg);   // 0 = ok
void rnr_thread_join (rnr_thread_t t);

void rnr_mutex_init (rnr_mutex_t *m);
void rnr_mutex_lock (rnr_mutex_t *m);
void rnr_mutex_unlock (rnr_mutex_t *m);
void rnr_mutex_destroy (rnr_mutex_t *m);

#endif
