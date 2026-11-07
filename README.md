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
| Interface | USB HID, 27-byte output reports, 49-byte input reports, no report ID |
| Axes | X, Y, Z, A, up to 100 kHz step rate |
| Principle | the PC plans, the controller executes: every 25 ms the PC sends a "slice" of signed steps per axis plus the OUT1..OUT4 states |

Protocol information comes from static analysis of the vendor's Mach3 plugin and from the
specification published by [BreakoutCNC](https://github.com/rolanddd-hu/BreakoutCNC), which agree.
The vendor plugin itself is proprietary and is **not** distributed here.

## How it works

```
 gSender / Candle / UGS --(TCP or virtual COM port)--> grblHAL core (running on the PC)
                                                          |  parser, planner, homing, probing,
                                                          |  status reports, $ settings
                                                          v
                                               slicer: counts step pulses per 25 ms slice
                                                          |
                                                          v
                                               slice sink: file, RNR USB controller or fake controller
```

- **Slicer** (`src/slicer.c`): step pulses are counted where the simulated driver emits them
  (`src/driver.c`) and stored in the 25 ms slice matching their simulated time. While the stepper
  runs, every slice is emitted (even empty ones) so the controller keeps the simulator's timeline.
- **Slice sink** (`src/slice_sink.h`): one interface for every consumer of slices. When a sink is
  full, the simulator waits (back-pressure), so simulated time never runs ahead of the machine.
  Implementations: **file** (`src/sink_file.c`, CSV) and **RNR controller** (`src/rnr/sink_rnr.c`), which
  talks either to the real board over USB HID or to a **fake controller** used by the tests.
- **RNR sink**: a dedicated I/O thread reads status reports continuously and sends slices paced by
  the PC clock, at most 3 slices (75 ms) ahead. The board acknowledges a slice as soon as it has
  buffered it, so pacing on acknowledgements alone would let the machine run seconds behind.
  A 500 ms status watchdog, USB loss or a board fault is reported to grblHAL as a motor fault
  (ALARM:17); every grblHAL reset (Ctrl-X, alarm) sends an e-stop to the board at once. After each
  run, the board's step counters are checked against the steps sent.
- **Position resync**: the simulator runs a little ahead of the machine (queued slices). After a
  reset, grblHAL's position is set back to where the board actually stopped (its step counters plus
  the offset between grblHAL and the counted steps, sampled at rest), before any new command runs.

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

GRBL2MACH adds one option to the simulator (all other options are described in
[README.simulator.md](README.simulator.md)):

```
grblHAL_sim -o <slice file>    write 25 ms step slices (CSV) to a file
grblHAL_sim -u                 drive the RNR Eco Motion USB controller
grblHAL_sim -f <log file>      drive a fake RNR controller (tests), logging every frame
grblHAL_sim -l <address>       address to listen on with -p (default 127.0.0.1)
```

`-o`, `-u` and `-f` are mutually exclusive. With `-u` or `-f`, the simulation runs at 1.05 x real
time unless `-t` is given; slice back-pressure then locks it to the machine.

Example: listen on TCP port 23023 and write slices to `slices.csv`:

```
grblHAL_sim -n -p 23023 -o slices.csv
```

Each CSV line is `index,t_s,dx,dy,dz,da,out` (slice number, start time in seconds, signed steps
per axis, outputs bitmask). The last line gives the totals.

> `-u` has **not been tested on real hardware yet**. The controller's limit, home, probe and e-stop
> inputs are not configured yet (all disabled in the configuration frame).

On Windows the console input only reads the keyboard: type commands in the console window, or
connect over TCP (`-p`). Piping commands into stdin does not work.

## Tests

`tests/run_slices.ps1` (Windows PowerShell) starts the simulator, sends G-code over TCP, waits
for idle, shuts down cleanly and checks the slice file: total steps per axis must equal both the
expected value and the final position reported by grblHAL, with no missing slice during motion.

```
powershell -File tests\run_slices.ps1 -GCode "G21 G91 G1 X10 F600" -Expect "2500,0,0" -MaxStepsPerSlice 63
```

With the default settings (`$100=250` steps/mm), `G1 X10 F600` produces 2500 steps over 83 slices
(2.08 s, triangular velocity profile at 10 mm/s²).

`tests/run_fake_board.ps1` runs the RNR sink against the fake controller:

- `motion`: frame sequence (connect, arm, run, end of run, disarm), every step executed, real-time
  pacing (board buffer depth at most 4 slices, no underrun), no protocol error;
- `watchdog`: the fake controller goes silent during a move: grblHAL must raise ALARM:17 and the
  bridge must send an e-stop;
- `reset`: Ctrl-X during a long move: the board must get an e-stop at once and stop, and grblHAL's
  position must be resynced to where the board actually stopped (`-Immediate`: unlock and move
  right after the reset, the move must start from the resynced position).

```
powershell -File tests\run_fake_board.ps1 -Scenario all
```

## Known limitations

- With `-u` / `-f`, G4 dwells run 5% short (simulation at 1.05 x real time).
- Once the controller is lost (watchdog, USB unplugged), the bridge must be restarted: there is no
  reconnection yet, and ALARM:17 stays latched until then.
- The position resync after a reset has only been tested against the fake controller, and not yet
  with homing (which overwrites the core's position).
- The upstream simulator sometimes misses alarm 3 (position lost) after a reset during motion:
  its `mc_reset()` runs on another thread than the core. With the resync the position is right
  anyway.
- The frame sequence (StartJog, jog slices, EndOfMotion) follows what BreakoutCNC bench-tested; it
  is still to be compared with USB captures of the vendor Mach3 plugin.

## Roadmap

1. Validate the controller with BreakoutCNC's diagnostic tool (connection, reset, status).
2. ~~Build the simulator on Windows; slice sink interface with the file implementation.~~ Done.
3. RNR USB sink: written and tested against the fake controller. Next: first hardware tests (drivers
   off, then small moves without a tool, far from the limits).
4. Compare with USB captures of Mach3 driving the same machine.
5. Homing and probing using the controller's hardware latches.
6. Mach3 XML profile importer, spindle and outputs.

## Safety

- This software is a **second layer** only. Use a hardwired emergency stop that cuts power to the
  stepper drivers.
- On USB link loss, a board fault or 500 ms without a status report, the bridge sends an e-stop and
  grblHAL raises ALARM:17. Thanks to real-time pacing the board never holds more than about 100 ms
  of motion.
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
