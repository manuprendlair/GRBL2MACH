# Contributing to GRBL2MACH

GRBL2MACH is a PC bridge based on the grblHAL Simulator: the grblHAL core runs on the PC and drives an RNR Eco Motion 2.0
USB controller, so that a grbl sender such as gSender can use it. User documentation is in `README.md`, the controller
protocol in `docs/PROTOCOL.md`.

## Build

Windows needs MinGW GCC, Ninja and CMake on `PATH`. MSVC cannot build the core. `src/grbl` is a submodule on the project's
grblHAL-core fork, branch `grbl2mach`.

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc
cmake --build build
```

This produces `build\grbl2mach.exe` and `build\grbl2mach_validator.exe`. `packaging\build_installer.ps1` builds the Windows
installer (needs Inno Setup).

## Tests

Run all three before calling a change done. They need no hardware.

```
powershell -File tests\run_fake_board.ps1 -Scenario all
powershell -File tests\run_slices.ps1
node tests\mach3_editor_test.js
```

- `-Scenario <name>` runs one scenario of the fake controller (`src/rnr/rnr_fake.c`). See the `ValidateSet` at the top of
  the script for the names.
- On Windows the program's console input cannot be piped. Drive it over TCP with `-p <port>`.
- When a test is intermittent, repeat it 15 times or more and compare with a build of the previous commit before concluding.

## Timing model

- Three threads matter: the core, the simulation (simulated interrupts, TCP) and USB I/O. The first two share the interrupt
  lock in `src/irq_lock.c`.
- The target PC can have only 2 cores (Pentium G3240), so a core thread that spins starves the simulation thread and the
  machine hears it. Any core loop that waits must release the lock and sleep (`irq_lock_core_sleep`).
- `task_execute(wait)` calls `grbl.on_execute_delay` when `wait` is true and `grbl.on_execute_realtime` otherwise. A wait
  loop in the core may therefore never reach `sim_process_realtime`.
- The simulation runs only 5 % faster than the machine, so a run's initial lead (prefill) is most of its margin. Timing
  margins measured on a fast PC do not transfer to the target PC.
- Each run logs a `[rnr]` line (lead, underruns, USB worst gap). Use those numbers to judge a timing change.
- The status window (`src/gui_win.c`) runs at the lowest priority. In window mode stdout and stderr are a pipe drained
  by a reader thread that must never wait for the window: a full pipe would block the core on its next message.
  Messages are classified by the marks in `problem_marks` and `warning_marks`; keep them in step with new messages.
- Settings from `MACH3.xml` are reapplied at every launch (`put_setting` in `mach3.c`). A value sent with `$nnn=` from the
  sender does not survive the next launch.

## Project rules

- Code comments, program messages and test output are in English.
- Work on branch `grbl2mach`. `master` is only fast-forwarded to a validated step, so that a clone always gets a tested
  version.
- Never commit the vendor's `RnRMotion*.dll` or a user's `MACH3.xml` (it holds their file paths).
- On a release, update the version in `CMakeLists.txt` and the download link in `README.md`, then rebuild the installer into
  `release/`.
- Say plainly what was validated on the fake controller only and what ran on the real board.

## Testing on a real machine

- Mach3 must be closed: it uses the same controller.
- The bridge's position check compares the steps sent with the board's counters. It cannot detect steps lost at the motors.
