/*
  irq_lock.h - makes simulated interrupts exclusive with the core thread (GRBL2MACH)

  On a microcontroller the main program is suspended while an interrupt handler
  runs, and the grblHAL core relies on it. The simulator runs the handlers on
  its own thread, concurrently with the core thread. This lock restores the
  exclusion: the core thread holds it while it runs and releases it only at
  well-defined points (main loop pass, delays, waiting loops that poll the
  clock); the simulator thread takes it around every handler.

  If the core thread keeps the lock for IRQ_LOCK_TIMEOUT_MS while a handler
  waits, a message is printed and the timeout handler called, once per wait: the
  machine must not go on with a core that has stalled. The handler keeps waiting
  for the lock: the exclusion is never given up.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifndef irq_lock_h
#define irq_lock_h

#include <stdbool.h>

#define IRQ_LOCK_TIMEOUT_MS 250

void irq_lock_init (bool enabled);      // before the core thread starts
bool irq_lock_enabled (void);
void irq_lock_set_timeout_handler (void (*handler)(void));  // called on the thread of the waiting handler

// Core thread
void irq_lock_core_start (void);        // first thing on the core thread: takes the lock
void irq_lock_core_yield (bool always); // lets waiting handlers run; always: also yield the CPU
void irq_lock_core_sleep (void);        // nothing to do for now: sleeps 1 ms with the lock released

// Around every interrupt handler (any thread)
bool irq_lock_isr_enter (void);
void irq_lock_isr_leave (bool locked);

#endif
