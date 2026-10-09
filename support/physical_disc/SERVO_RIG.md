# A dedicated servo rig, instead of a USB drive

Notes toward replacing the USB optical drive with a bare optical assembly driven
directly. Written after a session that pushed the USB path about as far as it goes,
so the limits below are measured rather than assumed.

## Why the USB path runs out

A USB drive will only move its sled by **completing a commanded seek**, and every
completion is a deceleration and a stop. Measured on a Mitsumi FX120T (1997, one of
the slowest drives to hand):

| primitive | sled velocity |
|---|---|
| `PLAY AUDIO` 1x tracking | 0.012 mm/s |
| `SCAN` (0xBA) | **rejected** — not implemented |
| `SET CD SPEED` 1x vs max | 524 vs 472 ms per 20 mm — 11% |
| one plain `SEEK(10)` | ~48 mm/s |
| **a Mega CD** | **16–21 mm/s** |

Two speeds, neither of them right, and nothing continuous in between. The only way
to reach Mega CD pace is to chop a traverse into short back-to-back seeks, because
the fixed per-command cost dominates a short move — a 7.3 mm leg measures 490 ms,
which is 15 mm/s. But each leg boundary is a stop and a start.

Those stop/start impulses are audible. Recorded with a phone sitting on the drive in
a silent room, they appear as events **93% below 200 Hz** with almost nothing above —
a mechanical thump through the chassis — while the sled's actual traverse is
broadband (24% of its energy at 2.5–6 kHz). So:

```
1 leg  -> 48 mm/s, 2 impulses    clean, but a modern seek
4 legs -> 20 mm/s, 8 impulses    right speed, more clatter
8 legs -> 15 mm/s, 16 impulses   slower, thumpier
```

**Halving the velocity doubles the impulses.** That trade is the wall, and it is a
property of the interface, not of the code.

## Three routes, and the cheap one first

The goal is both halves at once: read the real disc AND sound like the real console.
Ranked by effort.

### A. A period-correct drive. No electronics.

The drive in use is a **12x** unit, and its sled is fast because it was built to be.
A **1x or 2x CD-ROM drive from 1992-95** has a mechanism of the same generation and
design intent as a console's, and it reads discs through the code path that already
works. Nothing changes in software.

Measured verdict on the 12x Mitsumi, from `drivecheck.py`:

```
full-stroke seek   700 ms  ->  46 mm/s     "TOO FAST, will clatter"
4 mm legs          292 ms each -> 14 mm/s  (right speed, 8 legs = 16 impulses)
8 mm legs          202 ms each -> 40 mm/s  (wrong speed, 4 legs)
SCAN (0xBA)        rejected
SET CD SPEED       honoured, but only an 8% difference
```

Period 1x drives specced 400-600 ms *average* access, which usually implies well
over a second full-stroke -- in Mega CD range. If that holds, segmentation goes away
entirely and a seek becomes one continuous sweep at the right velocity.

Candidates: Mitsumi FX001D / LU005 (1x), Panasonic CR-562 (2x), Sony CDU-55/76 (2x),
Toshiba XM-3301 (1x SCSI). ATAPI via a USB-IDE bridge; SCSI needs more thought.

**Characterise any candidate with `drivecheck.py`** (kept at /media/fat on the test
box): it needs a reasonably full CD, measures full-stroke velocity and back-to-back
leg cost, checks SCAN and SET CD SPEED, and prints a verdict against the 16-21 mm/s
target.

### B. A real console CD block over its native protocol

The actual mechanism making its actual noises while reading the actual disc, with the
servo and decoder electronics on the console's own board doing all the reading.

More tractable than it looks for the Mega CD, because **the CDD protocol is
documented** -- see the MegaSD notes: ten 4-bit nibbles over HOCK/CDCK at 75 Hz, with
full command and status tables (0x03 READ/PLAY, 0x04 SEEK, status 0x02 SEEK, 0x0A
TRK_MOVE, and so on). The MiSTer core already emulates that CDD, so the work is
swapping an emulated device for a real one.

The cost is that it is per-console: Mega CD's CDD, the PS1's CXD-series block and
Saturn's are different interfaces and different integrations.

### C. Bare optical block plus your own servo and decoder

Everything below. Highest effort by a wide margin.

## Route C in detail: what a noise-only rig does not need

It does not need to read discs. The MiSTer core already has the data — from a CHD —
and in this design the drive is **only a noise source**. That separation is already
real: for CHD games the current implementation never asks the drive for a byte.

So everything hard about a bare optical assembly can be skipped: laser power control
and its APC loop, the RF amplifier, focus acquisition, tracking servo, the CLV
spindle PLL, EFM demodulation, CIRC error correction, the CD-ROM ECC layer. That
chain is an entire 1990s CD DSP chip's worth of function and the analogue front end
is precision work with no usable documentation. None of it makes a sound.

**Leave the laser unpowered, or remove it.**

## What the rig does need

A PS1 optical block (Sony KSM-440 family) gives four usable transducers:

* **Sled motor** — DC, worm gear. An H-bridge (DRV8871 or similar) plus PWM gives
  *arbitrary continuous velocity*. This is the whole point: 16–21 mm/s becomes a duty
  cycle, the traverse is genuinely continuous, and the segmentation problem
  disappears. Impulses happen only where they are wanted.
* **Spindle motor** — driven directly for the rise and fall, independent of the sled.
  A Mega CD at 1x CLV runs 917 rpm at the hub falling to 397 at the rim.
* **Focus and tracking coils** — voice coils of a few ohms. Driven at audio
  frequencies the lens assembly buzzes and ticks, which is a third independent noise
  channel, useful for lens-jump gestures that move no sled.
* **Inner limit switch** — homing.

Position: home on the switch, then open-loop timing is very likely enough, since
nothing depends on the head being anywhere in particular. A reflective encoder if
closed loop turns out to matter.

## How it attaches to this codebase

`acoustic_model.cpp` already emits exactly what such a device wants, and is
hardware-independent: a gesture carries kind, target LBA, **distance in mm**,
**duration in ms**, modelled **rpm** and CLV speed multiple. That is a velocity
command in all but name.

So the model is kept wholesale, and `physical_disc_acoustic.cpp`'s SCSI layer — the
part that spends its effort fighting the drive rather than describing the mechanism —
is replaced by a serial transport. Sketch:

```
SLED <mm_from_home> <ms>     move there, taking this long  (velocity implied)
SPIN <rpm> <ms>              ramp the spindle over this long
LENS <hz> <ms> <level>       buzz the focus/tracking coil
HOME                         seek the limit switch, reset the origin
STOP                         everything off
```

The gesture kinds map directly: `SLEW`/`STEP` to `SLED`, `SPINUP`/`SPINDOWN` to
`SPIN`, `JUMP`/`LOCK` to `LENS`, `SWEEP` to a `HOME` plus a full-stroke `SLED`.
`STREAM` becomes mostly silence with an occasional small `SLED` creep, which is what
a real mechanism does while tracking a read (the sled recentres when the lens runs
out of range, roughly every 10–15 s at 1x).

Everything that makes the current player complicated becomes unnecessary: the
closed-loop velocity controller, the leg-count/impulse trade, the cost model, the
`PLAY_MIN_BLOCKS` workaround, the gain-saturation clipping, the sub-channel polling.
All of it exists to coax velocity out of an interface that does not offer it.

## Numbers to build against

Derived during the session; see `HANDOFF.md` for how each was obtained.

| quantity | value | source |
|---|---|---|
| Mega CD sled velocity | 16–21 mm/s | derived from the below |
| full-stroke seek | 1.5–2 s | owner's ear on real hardware |
| Sonic CD data→CDDA transition | ~2 s | owner's ear; 0.703 of a stroke |
| cross-disc seek | ~2.7 s | same constant, `full_stroke_ms` 2400 |
| spindle, 1x CLV | 917 rpm hub → 397 rpm rim | CD geometry, `cd_geometry.cpp` |
| disc radius range | 25–58 mm | Red Book |
| track pitch | 1.6 µm | Red Book |
| CDD command loop | 75 Hz (13.3 ms), 15.8 ms when idle | MegaSD notes |
| lock-on | focus retries, ~100 ms units, no sled motion | CDD error 0x03 E-FOCUS |

Note that no published Mega CD seek time exists. Genesis Plus GX's author states its
CDD latency model is *"not accurate to how the real micro-controller and CD mechanism
worked"*, and the MegaSD reverse-engineering notes leave it as *"??seek time to be
defined"*. The figures above come from the owner's ear on real hardware, and two
independent observations of theirs are satisfied by the single constant 2400 ms,
which is the best corroboration available.

## MEASURED: the PS1 sled does a Mega CD sweep as one continuous move

Bench result, PS1 KSM-440 mechanism, sled driven from an RP2040 through the board's own
BA5977FP (IC722) at 25 kHz PWM. Time for a full stroke, outer stop to inner limit switch:

| duty | full-stroke time | repeat runs | note |
|------|------------------|-------------|------|
| 0.14 | did not reach the switch in 8 s | 1 | below the stiction floor |
| 0.16 | 5.04 s | 5.10, 4.98 | |
| 0.18 | 3.30 s | 3.29, 3.30 | |
| **0.20** | **2.37-2.43 s** | 2.373, 2.383, 2.431 | matches the ~2.4 s full stroke from the owner's ear |
| **0.22** | **1.84 s** | 1.82, 1.865 | |
| 0.25 | 1.37 s | 1.34, 1.41 | |
| 0.30 | 0.95 s | | |
| 0.35 | 0.78 s | | |
| 0.40 | 0.68 s | | |
| 0.50 | 0.51 s | | |
| 0.60 | 0.42 s | | |
| 0.75 | 0.33 s | | |
| 1.00 | 0.25 s | | |

**Repeatability is 1-5% run to run.** The USB drive's per-command cost swung 2-3x, which
is what made velocity impossible to hold. A Mega CD full stroke of 1.6-2.4 s sits at duty
0.20-0.25, so a seek is ONE continuous sweep with no segmentation and no start/stop
impulses. The curve is steeply non-linear below 0.20 (stiction), so it is used as a lookup
table with interpolation in log-time rather than a formula.

Method: wind or hop the sled to the outer stop, drive inward at the duty under test until
the limit switch closes, time it. The outward hop that finds the outer stop is a fixed
1.3 s at duty 0.30 from the switch, which lands on the stop without grinding.

### Channel map, from Sony's service manual schematic (not guessed)

IC722 pins 17/18 (ch3) drive the SLED, via CN701 pins 2/1. Pins 15/16 (ch4) drive the
SPINDLE, via CN701 pins 3/4. ch3 inputs are pin 23 (FIN) and pin 22 (RIN): a plain PWM
pair. ch4 is the one analogue channel (pin 24 through the board's own 4.7k + 0.22uF).
Pins 4-7 are ch1/ch2, the focus and tracking coils, which go to the laser flex.
Normal idle voltages: MUTE(20) 3.3 V, SW(3) 0 V, PowVcc 7.4 V, OUTVref(26) 1.7 V.

### Two bugs that cost most of a day, recorded so they are not repeated

1. `_limit.value()` with brackets. In CircuitPython `.value` is a property. The call raised
   "'bool' object is not callable" and silently killed code.py right after it created an
   empty results.csv. A stubbed test that never called the function could not see it.
2. "ch4 is the sled" was taken on trust. The schematic says ch4 is the spindle.

## The rig as built

### Wiring (RP2040 Pico, 3.3 V logic straight into the PS1's DIG3.5V-powered IC722)

| Pico | IC722 pin | function |
|------|-----------|----------|
| GP4  | 23 | ch3 FIN, sled PWM |
| GP5  | 22 | ch3 RIN, sled PWM |
| GP7  | 20 | MUTE, high = run |
| GP8  | 3  | SW, held low |
| GP2  | 24 (via the board's own 4.7k + 0.22uF) | ch4 IN, spindle, analogue-ish |
| GP6  | limit switch to GND | closed = sled at the hub |

The PS1 keeps its own PSU. `/dir.txt` on the board holds the sled polarity (0 = FIN carries
outward); `DIR 0|1` changes and saves it.

### Firmware and protocol (`rp2040/servo_fw.py`, deployed as `/code.py`)

`boot.py` enables a second USB serial port. On Windows the console is COM5 and the data
port COM6; on the MiSTer they are two `/dev/ttyACM*`, and `rig_connect()` finds the data one
by sending PING and looking for "OK servo". One ASCII line per command, one reply line each:

    PING | ST | HOME [duty] | MOVE <permille> <ms> | SPIN <rpm> <ms> | DRIVE <out|in> <duty> <ms>
    TEX <name> <value> | MUTE 0|1 | DIR 0|1 | STOP | RESET

`MOVE` and `HOME` reply at once and send `DONE ... <reason>` when they end (target, home,
limit, time). Positions are permille of the physical stroke from the limit switch. There is
no position sensor except the switch, so the firmware dead-reckons and re-zeroes at the
switch; targets under 8% snap to it, so every return to the data area re-homes. `MOVE` is
refused (`ERR notknown`) until the first HOME. The driver is muted whenever nothing is
moving, because the parked spindle input creeps.

### The textured drive: measured, and why the first model was wrong

Playback uses a rough low-frequency PWM (440 Hz carrier, 4.9 Hz swell, random grit: variant
P, picked by ear as the closest to a Mega CD) rather than the smooth 25 kHz drive the table
above was measured with. The carrier's full-voltage pulses break stiction, so the sled moves
at duties where the smooth drive stalls, and faster than the smooth table at the same mean
duty. Measured with `DRIVE` and a smooth `HOME 0.5` as a ruler (`driveprobe.ps1`), out and in
agreeing to a few percent:

| mean duty | 0.02 | 0.04 | 0.06 | 0.08 | 0.10 | 0.12 | 0.14 | 0.16 | 0.18 | 0.20 | 0.24 | 0.30 | 0.40 | 0.50 |
|-----------|------|------|------|------|------|------|------|------|------|------|------|------|------|------|
| strokes/s | 0.01 | 0.07 | 0.14 | 0.24 | 0.34 | 0.46 | 0.56 | 0.68 | 0.78 | 0.88 | 1.04 | 1.36 | 1.84 | 2.30 |

(A Mega CD full stroke of 2.4 s is 0.42 strokes/s, i.e. duty about 0.11.) This is
`TEX_CURVE` in the firmware; it is only valid for the default carrier/swell/amp/grit.

Why an earlier per-direction "efficiency" model was wrong: outward moves were running into
the outer stop (the ruler reads 0.93 of a 510 ms stroke because a full stroke at duty 0.5 is
really 472 ms), so the sled looked slower than it was and the return looked faster. Any
measurement that can saturate at an end stop must be checked against the ruler's plateau.
With the table, `MOVE 700 1971` covers 0.69-0.71 of the stroke in 1971 ms and the return
takes 1.85-1.92 s against 1.92 asked.

### Bench tools (`rp2040/`)

* `servo_sim.py` runs the real firmware against a model sled with CircuitPython stubbed;
  the model's true speeds are deliberately different from the firmware's belief.
* `deploy.ps1` copies a file onto the board through the REPL (base64, 100 chars a line,
  120 ms apart; 30 ms drops characters), CRC-checks it, resets and PINGs. boot.py leaves the
  drive read-only to the PC, so this is the only way to change files.
* `replay.ps1` plays the Sonic CD data->CDDA seek out and back, with the spindle glide.
* `chars.ps1`, `driveprobe.ps1` measure true travel and speed against the ruler.
* `picotool.py` does the same jobs from the MiSTer (python3, no pyserial there) when the Pico is
  plugged into it: `find` tells the console port from the data port, `send` and `steps` talk to the
  protocol, `deploy` uploads firmware. Leave the core, so the rig backend lets go of the port,
  before using it.
* `hosttest/run.sh` runs the MiSTer-side translator against a fake board on a pty in real
  time, using the real acoustic model.

### MiSTer side (`physical_disc_rig.cpp`)

`PHYSICAL_DISC_ACOUSTIC_RIG=1` (with `PHYSICAL_DISC_ACOUSTIC=1` or 2) in the core's ini
section makes the existing worker loop play gestures through the rig instead of the spare
disc. Gestures are played in real time: the translator sends the commands and waits out the
time the original drive would have taken.

| gesture | sent |
|---------|------|
| SLEW, STEP | `SPIN <dest rpm> <dur>` and `MOVE <permille of radius> <dur>` |
| SPINUP, SPINDOWN | `SPIN <rpm> <dur>` / `SPIN 0 <dur>` |
| SWEEP | `HOME`, then out to 950 and back in equal legs |
| PARK | `SPIN 0` and `MOVE 0` |
| STREAM | spindle follows the CLV rpm; a `MOVE` only once the creep reaches 4% of the stroke |
| JUMP, HOLD, LOCK | nothing (LOCK waits its time) |

If the board reports `ERR notknown` the translator homes and spends what is left of the time
on the move. After three unanswered commands it drops the port and probes again each second.

### The lens coils (planned wiring, route 2)

The hiss in the Mega CD clip is probably the pickup's own lens actuators (focus and tracking coils), which
the service manual shows driven with a noisy waveform the whole time. To play that, the Pico drives IC722's
channel 1 and channel 2 inputs, the way it already drives channel 3 (sled) and channel 4 (spindle).

| Pico | IC722 pin | function |
|------|-----------|----------|
| GP10 | 4 (CH1FIN) | focus coil, forward |
| GP11 | 5 (CH1RIN) | focus coil, reverse |
| GP12 | 6 (CH2FIN) | tracking coil, forward |
| GP13 | 7 (CH2RIN) | tracking coil, reverse |

Channel 1 = focus and channel 2 = tracking is read from how the output lines nest on the way to the pickup
connector CN702 (pins 13 FCS+, 14 TRK+, 15 TRK-, 16 FCS-); it is not yet confirmed. The board's own DSP still
drives pins 4-7, so they must be lifted from the board (or their traces cut) before the Pico is connected, as
was done for the sled. The pickup flex goes into CN702, but the laser diode is on CN702 pins 1 (LD) and 2 (VCC)
and the board can switch it on by itself, so those two flex conductors must be insulated, and ideally the laser
driver Q701 removed. The laser is infrared and invisible.

Firmware: `LENS <F|T|B> <N|D|R> <amp> <ms> [carrier]` (noise, steady level, ramp), `LENS OFF`. `amp` is a
fraction of `TEX lens_max` (duty, i.e. 5 V x duty across the coil per the datasheet), so no command can exceed
that cap, and a governor holds sustained drive to `lens_rms`.

### BA5977FP, from the Rohm datasheet (C:\t\BA5977FP.PDF, Japanese; read in full)

Channels 1-3 take a PWM pair (F, R). Inputs steer constant currents I1 = I2 = 25 uA (typ) into R1 = 100 kOhm
and C1 = 25 pF, a node of +-2.5 V about Vref and a time constant of 2.5 us (corner about 64 kHz), then
BTL buffers drive the load. So:

* Output across the load is linear in duty: **volts = 5 V x (duty_F - duty_R)**. The datasheet's maximum
  output amplitude is 5.0 V typ (4.4-5.6 V) into 8 Ohm at 8 V supply. That is the most a PS1 can ever put
  on a lens coil or sled motor, which is why asking for "7 V" (the Mega CD 2 manual's figure for a
  different driver on 9 V) cannot work here.
* The filter does not limit audio: 1 kHz and 7.8 kHz drives reach the load at full amplitude.
* Truth table: F=H R=L forward; F=L R=H reverse; L,L and H,H both zero output.
* Input levels: H >= 2.4 V, L <= 0.5 V. Mute (pin 20) at <= 0.5 V. Thermal shutdown about 180 C. Output
  offset within +-30 mV. Fig. 9 shows the duty-to-volts line was measured at 88.2 kHz.
* Application example (Fig. 6): pins 13/14 (channel 1) go to the TRACKING coil and pins 11/12 (channel 2)
  to the FOCUS coil, the opposite of what the line nesting on the PSone schematic suggested. Not yet
  settled for this board; the first test that moves the lens should say which axis each channel is.

### The lens orchestra: what is sourced, what is measured, what is a guess (2026-10-09/10)

The board can also play the pickup's lens coils (`LENS ... Z` broadband noise, `LENS ... G` a 1 kHz tone) and
kick the spindle (`SPIN rpm ms kick_ms`). `physical_disc_rig.cpp` has a per-console `lens_policy` table
(focus-search ramp length, the PlayStation's auto-gain tone, steady servo noise, seek noise, relock burst,
spindle kick). Sources: the Mega CD 2 service manual p.10 start-up order and focus search of about 2 s; the
Saturn manual's "approx. 3 seconds" of lens movement; the CXD2545Q datasheet (KICK, tracking servo open during a
sled move, AGCNTL's 1 kHz sine); a recording of a real PlayStation (tone at 1004 Hz for 0.4 s at 3.15 s). The
levels, lengths and which console does what are **best guesses**. The start-up lens sequence plays once per disc
and not under the generic profile (the core announces the mount several times).

**Microphone measurements** (`rp2040/mic/`: `rec.py` records the PC's default microphone through winmm with
ctypes, the analysis scripts run in WSL with numpy). A microphone beside the rig, in a room that is not silent:

* The pickup radiates the lens drive almost entirely at 4-14 kHz whatever the pulse rate of the noise (64 kHz
  down to 2 kHz, level 2): nothing measurable below 2 kHz. A lower rate only raises the 4-11 kHz level. This is
  the owner's "dog whistle" description.
* A real PlayStation reading is +20 to +26 dB over its idle floor at 1-3 kHz (the mechanics: spindle, sled,
  gears), +14.5 dB at 4-6 kHz, +12 dB at 6-11 kHz. The rig's steady noise (level 2, 8 kHz pulses) gives +9, +9 and
  +13 dB at 4-6, 6-8 and 8-11 kHz, so the lens part is the right order; **the 1-4 kHz body of the real sound has
  to come from the sled and spindle**, which were not working when this was measured.
* The 1 kHz tone: the recording's line is +12 to +17 dB over its surroundings with no harmonics. The rig's
  triangle at peak 0.5 gives +14 dB at 1 kHz but its third harmonic (3 kHz) reads +23 to +27 dB, because the
  pickup radiates 3 kHz far better than 1 kHz. A sine ("s", `BEEP_SHAPE` in the translator) brings 3 kHz down to
  about equal to 1 kHz. The triangle is the default because the owner asked for "more triangle than square".

### Open wires found by probing from the Pico (2026-10-10)

Sled and spindle did not move at all, with the old and the new firmware and with raw PWM from the REPL; the lens
channels worked at the same time (so the chip is powered and unmuted). Probing the Pico's own pins
(`rp2040/mic/pins_*.txt`, run with `picotool.py repl --file`):

* Held-high pin released: GP5 (sled RIN), GP7 (MUTE) and GP10-13 (lens) fall at once (a low-impedance path to
  ground, as the chip's inputs have). **GP4 (sled FIN, IC722 pin 23) and GP2 (spindle RC, pin 24) hold their level
  for over 3 ms: nothing is connected there.**
* Released-low pin with the pull-up on: GP2 reads high on the very first read; a connected 0.22 uF filter would
  take about 10 ms. So the RC is not on GP2's net either.
* Which fits everything seen: HOME works (it drives RIN, GP5), every outward move ends as `stuck` (it needs FIN,
  GP4), the spindle never turns, and the earlier "sled stuck at the hub for an hour" (probably the same loose
  joint, intermittent).
* To confirm: continuity from the Pico GP4 pad to IC722 pin 23, and GP2 to the RC and on to pin 24, with the
  power off. GP5, the neighbour of GP4, is fine, so a loose jumper in the Pico's header or a lifted joint at the
  chip are the likely places.

### Still open

* Spindle duty to rpm is set by ear, not measured: stepping 0.53-0.62 was smooth and 0.66 and
  0.70 were loud, like something hitting, so the hub is 0.61, the rim 0.57 and the firmware caps
  the duty at 0.62. The real rpm at those duties is unknown; the glide shape is right, the
  absolute speed may not be.
* `CD_COMM_TRACK_MOVE` in the Mega CD core pauses without moving its own position, so it is left
  silent: a seek noise for it would be followed by a seek back when the next read arrived.
* The ear verdict on the protocol-driven playback has not been taken yet, and the MiSTer
  end-to-end test needs the Pico moved onto a MiSTer USB port.
* The lens coils (ch1/ch2, IC722 pins 4-7) could add focus rattle; unused.
