/*
  irq_lock.c - makes simulated interrupts exclusive with the core thread (GRBL2MACH)

  See irq_lock.h.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#include <stdio.h>
#include <stdatomic.h>

#include "irq_lock.h"
#include "rnr/rnr_os.h"

#ifdef _WIN32
#include <windows.h>
static CRITICAL_SECTION lock;                       // recursive
static DWORD core_tid;
#define lock_take()     EnterCriticalSection(&lock)
#define lock_try()      TryEnterCriticalSection(&lock)
#define lock_give()     LeaveCriticalSection(&lock)
#define is_core()       (GetCurrentThreadId() == core_tid)
#define cpu_yield()     SwitchToThread()
#else
#include <pthread.h>
#include <sched.h>
static pthread_mutex_t lock;                        // recursive
static pthread_t core_tid;
#define lock_take()     pthread_mutex_lock(&lock)
#define lock_try()      (pthread_mutex_trylock(&lock) == 0)
#define lock_give()     pthread_mutex_unlock(&lock)
#define is_core()       (core_started && pthread_equal(pthread_self(), core_tid))
#define cpu_yield()     sched_yield()
#endif

static atomic_bool enabled;
static atomic_int isr_waiting;      // handlers waiting for the lock
static bool core_started = false;
static int core_depth = 0;          // core thread's hold count (touched by the core thread only)

void irq_lock_init (bool on)
{
#ifdef _WIN32
    InitializeCriticalSection(&lock);
#else
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&lock, &attr);
    pthread_mutexattr_destroy(&attr);
#endif
    atomic_store(&enabled, on);
}

bool irq_lock_enabled (void)
{
    return atomic_load(&enabled);
}

void irq_lock_core_start (void)
{
#ifdef _WIN32
    core_tid = GetCurrentThreadId();
#else
    core_tid = pthread_self();
#endif
    core_started = true;
    if(atomic_load(&enabled)) {
        lock_take();
        core_depth = 1;
    }
}

void irq_lock_core_yield (bool always)
{
    // Only the core thread, and only from its own code (not from a handler it runs itself)
    if(!atomic_load(&enabled) || !core_started || !is_core() || core_depth != 1) {
        if(always)
            cpu_yield();
        return;
    }

    if(!always && atomic_load(&isr_waiting) == 0)
        return;     // nobody waiting: keep going, this is the hot path

    core_depth = 0;
    lock_give();
    // Hand the CPU to the waiting handlers until they have taken the lock (bounded)
    for(int i = 0; i < 1000 && atomic_load(&isr_waiting); i++)
        cpu_yield();
    if(always)
        cpu_yield();
    lock_take();
    core_depth = 1;
}

bool irq_lock_isr_enter (void)
{
    if(!atomic_load(&enabled))
        return false;

    if(core_started && is_core()) {     // handler called synchronously from core code
        lock_take();
        core_depth++;
        return true;
    }

    atomic_fetch_add(&isr_waiting, 1);
    uint64_t deadline = rnr_now_us() + (uint64_t)IRQ_LOCK_TIMEOUT_MS * 1000u;
    while(!lock_try()) {
        if(rnr_now_us() > deadline) {
            atomic_fetch_sub(&isr_waiting, 1);
            atomic_store(&enabled, false);
            fprintf(stderr, "[irq-lock] the core thread kept the lock for %d ms while an interrupt was waiting:\n"
                             "[irq-lock] lock disabled, interrupts now run concurrently with the core (previous behaviour)\n",
                     IRQ_LOCK_TIMEOUT_MS);
            return false;
        }
        cpu_yield();
    }
    atomic_fetch_sub(&isr_waiting, 1);

    return true;
}

void irq_lock_isr_leave (bool locked)
{
    if(!locked)
        return;
    if(core_started && is_core())
        core_depth--;
    lock_give();
}
