# GRBL2MACH

Drive Mach3-era Chinese USB motion controllers (the "RNR Eco Motion 2.0" family, including the red
4-axis Bitsensor boards) from **gSender** or any other GRBL-compatible sender.

GRBL2MACH is a bridge that runs on the PC. To the sender it looks like a GRBL board; on the machine
side it speaks the controller's USB HID protocol. Its core is the
[grblHAL Simulator](https://github.com/grblHAL/Simulator), which compiles the real grblHAL core
(G-code parser, look-ahead planner, homing, probing, `$` settings) as a desktop program.
GRBL2MACH replaces the simulated hardware with the USB controller.

> **Status: early development. Nothing has been tested on real hardware yet.**
> Do not connect this to a machine you care about. See [Safety](#safety).

## Goals

- Use gSender (or Candle, UGS, ...) with a controller that was sold for Mach3 only.
- **Never write to the controller's firmware**: Mach3 keeps working with the same board at any time.
- Probing (G38.x, workpiece and tool length) is a hard requirement.
- Import the machine configuration from an existing Mach3 XML profile (planned).

## Target hardware

| Item | Value |
|---|---|
| USB IDs | VID `0xA720`, PID `0xF803` ("RobotAndRobot.com", "RNR ECO MOTION 2.0") |
| Interface | USB HID, report IDs 2 (8 bytes) and 3 (26 bytes) out, 1 (48 bytes) in; status about every 50 ms |
| Axes | X, Y, Z, A, up to 100 kHz step rate |
| Principle | the PC plans, the controller executes: every 5 ms the PC sends a "slice" of signed steps per axis; outputs OUT1..OUT4 ride in the slices during motion and in separate frames at rest; spindle speed is a PWM frame |

Protocol information comes from static analysis of the vendor's Mach3 plugin, from the
specification published by [BreakoutCNC](https://github.com/rolanddd-hu/BreakoutCNC), and from USB
captures of Mach3 driving a real machine (start-up, homing, jogs, MDI moves, feed hold, stop,
e-stop, probing). The bridge drives the board the way Mach3 does. The vendor plugin itself is
proprietary and is **not** distributed here.

## How it works

```
 gSender / Candle / UGS --(TCP or virtual COM port)--> grblHAL core (running on the PC)
                                                          |  parser, planner, homing, probing,
                                                          |  status reports, $ settings
                                                          v
                                               slicer: counts step pulses per 5 ms slice
                                                          |
                                                          v
                                               slice sink: file, RNR USB controller or fake controller
```

- **Slicer** (`src/slicer.c`): step pulses are counted where the simulated driver emits them
  (`src/driver.c`) and stored in the 5 ms slice matching their simulated time. While the stepper
  runs, every slice is emitted (even empty ones) so the controller keeps the simulator's timeline.
- **Slice sink** (`src/slice_sink.h`): one interface for every consumer of slices. When a sink is
  full, the simulator waits (back-pressure), so simulated time never runs ahead of the machine.
  Implementations: **file** (`src/sink_file.c`, CSV) and **RNR controller** (`src/rnr/sink_rnr.c`), which
  talks either to the real board over USB HID or to a **fake controller** used by the tests.
- **RNR sink**: a dedicated I/O thread reads status reports continuously and sends slices paced by
  the PC clock, at most 75 ms ahead (Mach3 keeps about 0.5 s ahead). A run starts once 40 ms of
  motion is queued. The board acknowledges a slice as soon as it has buffered it, so pacing on
  acknowledgements alone would let the machine run far behind.
  - Program runs use the same frames as Mach3: arming (pre-reset, clear e-stop, set positions,
    configure), "G-code" slices, then end of motion. After each run, the board's step counters are
    checked against the steps sent.
  - Every grblHAL reset (Ctrl-X, alarm) sends a cancel at once, like Mach3's Stop button: the
    board stops and drops its buffer, but stays armed.
  - The board's own e-stop input (IN1) stops it by itself; the bridge reports it to grblHAL as an
    e-stop (ALARM:10) and re-arms the board before the next move, once the input is released and
    grblHAL reset.
  - A 500 ms status watchdog or USB loss is reported as a motor fault (ALARM:17) and disarms the
    board.
  - Spindle and coolant: grblHAL's spindle (M3/M4, S) and coolant (M7/M8) drive the board's PWM
    frame and outputs OUT1..OUT4 (`--rnr-out`). As grblHAL changes them between moves only, a run is
    closed before an output switches, and the switch goes out once the motion before it has run (not
    75 ms early). During a run, every slice carries the outputs, like Mach3. The PWM duty is
    `S / $30` in 1/1000, like Mach3 (S12000 with `$30=24000` = 500/1000).
- **Position resync**: the simulator runs a little ahead of the machine (queued slices). After a
  reset, grblHAL's position is set back to where the board actually stopped (its step counters plus
  the offset between grblHAL and the counted steps, sampled at rest), before any new command runs.
- **Interrupt lock** (`src/irq_lock.c`): the upstream simulator runs its simulated interrupt
  handlers on its own thread, concurrently with the core thread, while the core assumes (as on a
  microcontroller) that it is suspended during a handler. A recursive lock restores that: the core
  thread holds it and releases it only at its yield points (main loop pass, delays, clock polling
  while a handler waits); the simulator thread takes it around every handler. If the core thread
  ever keeps it 250 ms while a handler waits, the lock disables itself and says so.
  `--no-irq-lock` restores the upstream behaviour. See grblHAL/Simulator issue #28.

## Building

### Windows (MinGW-w64)

The simulator targets GCC; MSVC is not supported (the grblHAL core uses GCC extensions and POSIX
headers). Any MinGW-w64 GCC works, for example [WinLibs](https://winlibs.com/) (UCRT, POSIX
threads) or [MSYS2](https://www.msys2.org/), plus [CMake](https://cmake.org/) 3.10 or later.

```
git clone --recurse-submodules <this repository> GRBL2MACH
cd GRBL2MACH
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc
cmake --build build
```

(`-G "MinGW Makefiles"` works too.) A MinGW-only workaround for a `strset()` name clash between
the grblHAL core and MinGW's `<string.h>` is applied automatically, see `cmake/mingw_strset_fix.h`.

### Linux / macOS

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Usage

GRBL2MACH adds these options to the simulator (all other options are described in
[README.simulator.md](README.simulator.md)):

```
grblHAL_sim -o <slice file>    write 5 ms step slices (CSV) to a file
grblHAL_sim -u                 drive the RNR Eco Motion USB controller
grblHAL_sim -f <log file>      drive a fake RNR controller (tests), logging every frame
grblHAL_sim -l <address>       address to listen on with -p (default 127.0.0.1)
grblHAL_sim --no-irq-lock      let simulated interrupts run concurrently with the core (upstream)
grblHAL_sim --rnr-invert <hex> step/direction inversion sent to the controller (default 0)
grblHAL_sim --rnr-out spindle=1,ccw=0,flood=0,mist=0   controller outputs (default: spindle on OUT1)
```

`--rnr-invert` takes the same values as Mach3's *Ports & Pins > Motor Outputs* page: bits 0-3 make
the step pulse active low on X, Y, Z, A ("Step Low Active"), bits 4-7 invert the direction of X, Y,
Z, A ("Dir LowActive"). Example: step low active on all four axes, A direction inverted: `0x8F`.
**Use the same values as your Mach3 profile**: a wrong step polarity can lose steps.

`--rnr-out` assigns OUT1..OUT4 (0 = none) to: `spindle` (on with M3 or M4), `ccw` (M4, when the
spindle has a separate counter-clockwise output; 0 = same as `spindle`), `flood` (M8) and `mist`
(M7), as in Mach3's *Ports & Pins > Output Signals* and *Spindle Setup* pages. Set `$30` (maximum
spindle speed) to the speed reached at 100 % PWM.

`-o`, `-u` and `-f` are mutually exclusive. With `-u` or `-f`, the simulation runs at 1.05 x real
time unless `-t` is given; slice back-pressure then locks it to the machine.

Example: listen on TCP port 23023 and write slices to `slices.csv`:

```
grblHAL_sim -n -p 23023 -o slices.csv
```

Each CSV line is `index,t_s,dx,dy,dz,da,out,pwm` (slice number, start time in seconds, signed steps
per axis, outputs bitmask: 1 spindle, 2 counter-clockwise, 4 flood, 8 mist; spindle PWM in 1/1000). The last line gives the totals.

> `-u` has **not been tested on real hardware yet**. The controller's limit, home and probe inputs
> are not configured yet (all disabled in the configuration frame); its e-stop input (IN1) works
> without configuration.

On Windows the console input only reads the keyboard: type commands in the console window, or
connect over TCP (`-p`). Piping commands into stdin does not work.

## Tests

`tests/run_slices.ps1` (Windows PowerShell) starts the simulator, sends G-code over TCP, waits
for idle, shuts down cleanly and checks the slice file: total steps per axis must equal both the
expected value and the final position reported by grblHAL, with no missing slice during motion.

```
powershell -File tests\run_slices.ps1 -GCode "G21 G91 G1 X10 F600" -Expect "2500,0,0" -MaxStepsPerSlice 13
```

With the default settings (`$100=250` steps/mm), `G1 X10 F600` produces 2500 steps over 407 slices
(2.04 s, triangular velocity profile at 10 mm/s²).

`tests/run_fake_board.ps1` runs the RNR sink against the fake controller:

The fake controller behaves like the real board as seen in the Mach3 captures (state machine,
5 ms slices, status every 50 ms, cancel, e-stop input).

- `motion`: frame sequence (connect, arm, run, end of run, disarm), every step executed, real-time
  pacing (board buffer depth at most 20 slices, no underrun), no protocol error;
- `watchdog`: the fake controller goes silent during a move: grblHAL must raise ALARM:17 and the
  bridge must send an e-stop;
- `reset`: Ctrl-X during a long move: the board must get a cancel at once, stop and stay armed, and
  grblHAL's position must be resynced to where the board actually stopped (`-Immediate`: unlock and
  move right after the reset, the move must start from the resynced position);
- `estop`: the board's e-stop input is pressed during a move: grblHAL must raise ALARM:10, resync
  to where the board stopped, and re-arm it for the next move once released and reset.
- `outputs`: M3 S12000, a move, M8, a move, M5 M9: spindle PWM and outputs must switch before the
  first slice, between the two runs once the first move has run, and after the last one; every
  slice must carry the outputs.

```
powershell -File tests\run_fake_board.ps1 -Scenario all
```

## Known limitations

- With `-u` / `-f`, G4 dwells run 5% short (simulation at 1.05 x real time).
- Once the controller is lost (watchdog, USB unplugged), the bridge must be restarted: there is no
  reconnection yet, and ALARM:17 stays latched until then.
- The position resync after a reset has only been tested against the fake controller, and not yet
  with homing (which overwrites the core's position).
- Without the interrupt lock (`--no-irq-lock`), the upstream simulator often misses alarm 3
  (position lost) after a reset during motion: its `mc_reset()` can return early, without stopping
  the steppers or calling `grbl.on_reset`. The bridge stops the board and resyncs the position from
  `hal.driver_reset`, which the core always calls on its own thread, so the position is right
  either way. With the lock, alarm 3 is raised every time.
- The grblHAL core submodule (`src/grbl`) points to
  [manuprendlair/grblHAL-core](https://github.com/manuprendlair/grblHAL-core), branch `grbl2mach`:
  the upstream core plus one fix for a feed hold that could stay in `Hold:1` for seconds to minutes
  (float round-off at the end of the forced deceleration, in `stepper.c`). The fix is proposed
  upstream; the submodule will go back to grblHAL/core once it is merged.
- The frame sequence follows USB captures of Mach3; the bridge itself has not been captured on the
  real board yet (to compare frame by frame, including the 9-byte short reports).
- Spindle and outputs follow the Mach3 captures (PWM frame, outputs frame, outputs byte in every
  slice) but have only been tested against the fake controller. No spindle ramp or spin-down delay
  is added by the bridge (Mach3 waits 1 s before switching OUT1 off after M5); use grblHAL's own
  spindle delay settings if needed. Changing `$30` now takes effect at once (the upstream simulator
  only applied it after a restart).
- Feed hold: grblHAL decelerates in the slices it sends, like Mach3. The board's "stop jog" frame,
  which drops its buffer, is not used yet (it would make holds and jog cancels react faster).

## Roadmap

1. Validate the controller with BreakoutCNC's diagnostic tool (connection, reset, status).
2. ~~Build the simulator on Windows; slice sink interface with the file implementation.~~ Done.
3. RNR USB sink: written and tested against the fake controller. Next: first hardware tests (drivers
   off, then small moves without a tool, far from the limits).
4. ~~USB captures of Mach3 driving the same machine; bridge aligned on them.~~ Done.
5. Homing and probing using the controller's hardware latches.
6. ~~Spindle and outputs.~~ Done (against the fake controller). Mach3 XML profile importer.

## Safety

- This software is a **second layer** only. Use a hardwired emergency stop that cuts power to the
  stepper drivers.
- On USB link loss or 500 ms without a status report, the bridge sends an e-stop and grblHAL
  raises ALARM:17. Thanks to real-time pacing the board never holds more than about 100 ms of
  motion. The board has no watchdog of its own: if the PC freezes, it runs what it holds.
- The board's e-stop input (IN1) stops it by itself, without the PC.
- The TCP port listens on `127.0.0.1` only by default: GRBL has no authentication, so anyone
  reaching the port can move the machine. `-l <address>` opens it to the network and prints a
  warning. Never allow it on "Public" networks in your firewall.
- First tests with drivers powered off, then without a tool, with small moves in the middle of the
  travel.

## Credits and license

- [grblHAL](https://github.com/grblHAL) and the grblHAL Simulator: Terje Io and contributors;
  original Grbl simulator by Jens Geisler and Adam Shelly.
- [BreakoutCNC](https://github.com/rolanddd-hu/BreakoutCNC) for the published protocol
  specification and the Mach3 profile importer idea.

GRBL2MACH is free software released under the **GNU General Public License v3.0 or later**, like
the grblHAL Simulator it is based on (see [COPYING](COPYING)).

Mach3, RNR / RobotAndRobot, Bitsensor and gSender are names of their respective owners. This
project is not affiliated with or endorsed by any of them.
