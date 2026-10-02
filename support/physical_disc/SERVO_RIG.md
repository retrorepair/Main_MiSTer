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
