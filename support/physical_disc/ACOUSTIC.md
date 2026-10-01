# Acoustic mirroring

Drives a spare disc in the USB drive so it makes the noises the original
console's drive would have made for the disc activity the core is really
performing.

Enable with `PHYSICAL_DISC_ACOUSTIC=1` in `MiSTer.ini`, and put a disc you
don't care about in the drive. `PHYSICAL_DISC_ACOUSTIC_PROFILE` pins the
imitated mechanism (`0` follows whichever core is running, then `1` PSX,
`2` Mega CD, `3` Saturn, `4` PC Engine CD, `5` 3DO, `6` CD-i, `7` Neo Geo CD).

## How it is put together

    cores  ──event──▶  acoustic_model  ──gesture──▶  physical_disc_acoustic
                       (pure logic)                  (SCSI, thread)
                            │
                       cd_geometry
                       (disc physics)

**`cd_geometry`** is the disc itself. A CD is a constant-linear-density
spiral, so sector number maps to radius as a square root:

    r(lba) = sqrt(r_inner^2 + lba * pitch * sector_length / pi)

Everything else is derived from that — how many sectors are in one revolution
at a given radius, the CLV spindle speed, and how many spiral turns lie
between two LBAs. The model is checked against the drive measurements Dave
Shadoff took for `support/pcecd/seektime.cpp`; it reproduces his whole
sectors-per-revolution table to within 0.13 of a sector (`tests/acoustic`).

**`acoustic_model`** keeps a model of the original mechanism — where the sled
is, whether the spindle is running, what it is doing — and turns core activity
into physical *gestures*:

| gesture  | what the original drive is doing                        |
|----------|---------------------------------------------------------|
| JUMP     | objective lens hops a few turns; the sled never moves    |
| STEP     | one short sled move                                      |
| SLEW     | long coarse-then-fine sled travel, 2 or 3 stages         |
| STREAM   | sustained read, head creeping outward under CLV          |
| HOLD     | spindle turning, head parked on track                    |
| SPINUP / SPINDOWN / SWEEP / PARK | start-up, shutdown, boot calibration, tray |

The important distinction is JUMP vs STEP vs SLEW. A CD servo services a small
move by tilting the lens alone — the sled motor never runs and there is
**nothing to hear**. Only past roughly 32 spiral turns does the sled engage.
Treating every small move as a seek is what makes a naive mirror sound like a
hard disk rather than a console.

**`physical_disc_acoustic`** replays gestures on the real drive. Game LBAs are
mapped onto the mirror disc by *fraction of radial stroke*, not by sector
number, so sled travel is proportional and the mirror's own CLV spindle
reproduces the same pitch glide. It copes with a short CD-R or a DVD, which
have the same ~24–58 mm program area but very different sector counts.

## Where the activity comes from

Every CD core reports what its emulated drive is doing. This matters more than
anything else in here: previously the only signal was a single LBA per CHD
sector read, so `.cue`/`.bin` images produced **no sound at all**, and a CDDA
stream was indistinguishable from a random-access data read.

| core        | reported                                                     |
|-------------|--------------------------------------------------------------|
| Mega CD     | seek, per-sector data vs audio, scan, pause, resume, stop, tray open/close, TOC |
| Neo Geo CD  | as Mega CD (shares `cdd_t`), own slower profile               |
| Saturn      | read and seek commands, data sectors, CDDA, TOC, stop, pause  |
| PC Engine CD| READ6 seek + sectors, CDDA playback, SAPSP seek, pause        |
| 3DO         | read command seek, data sectors, pause/stop                   |
| CD-i        | read bursts with their real sector count                      |
| PSX         | every read with its burst count and data/audio flag, mount    |

`mister_chd_read_sector()` no longer hints. All six of its callers now report
at the right level, and hinting again from the storage layer would double-count
every sector and — because the second hint lands one sector *behind* the
modelled head — fabricate a seek that never happened.

## Tests

No MiSTer needed; these are pure logic.

```bash
g++ -O2 -o tgeom tests/acoustic/test_geometry.cpp support/physical_disc/cd_geometry.cpp -lm && ./tgeom
g++ -O2 -o tsim tests/acoustic/sim_timeline.cpp support/physical_disc/acoustic_model.cpp support/physical_disc/cd_geometry.cpp -lm && ./tsim
```

`test_geometry` checks the disc physics against Shadoff's measured table and
round-trips the LBA/radius conversion. `sim_timeline` runs a PSX boot-and-load
trace through the model and prints the gesture timeline, the radial mapping,
and the modelled seek times for every profile.

## The mirror disc matters more than anything else

Measured on hardware while playing Sonic CD: **684 of 694 gestures moved the
sled less than 0.05 mm.** That is faithful — a real Mega CD streaming CDDA
barely moves its sled either — but a 1991 deck at 1x is audibly working the
whole time, and a 2026 slot-load drive doing the same tiny moves is silent.

Two things follow.

**Use a disc that is genuinely full of readable data.** The engine can only
move the head across the recorded area. A lead-out address is not a promise: a
Mortal Kombat 3 PC CD whose TOC claimed 164214 sectors turned out to be
readable only to LBA 14064, leaving **2.3 mm of travel out of a possible 34**.
A full 700 MB CD-R gives roughly fifteen times the stroke. The usable extent is
discovered during play (reads that fail pull the span in) and logged, so check
the trace if it sounds cramped.

**`PHYSICAL_DISC_ACOUSTIC_GAIN`** multiplies each move while keeping its
direction, so a file-system hop that would be 17 µm becomes something you can
hear. Absolute radius is then no longer preserved, which costs some CLV spindle
pitch accuracy — that is the trade, and it is why the default is `1`
(unchanged, faithful). `8` was measured to give very close to 8x the travel.
Spin-up, the servo sweep and tray park re-anchor to the true position.

Reads use `READ(10)` with **Force Unit Access** so they come off the media
rather than out of the drive's cache. Without FUA the small repeated reads a
stream gesture issues are nearly all cache hits and the mechanism never moves
at all. `READ CD` (0xBE, raw 2352) is the fallback for an audio or mixed-mode
disc, where `READ(10)` cannot touch a CD-DA sector; seek-only is the last
resort.

## Tuning

The per-console numbers are all in one table at the top of
`acoustic_model.cpp` — speeds, the lens-jump and short-seek thresholds, settle
and stroke times, spin-up, spin-down, read-ahead. They are meant to be adjusted
by ear without touching any logic. The PC Engine row is Shadoff's measured
data; the others are scaled from each mechanism's rated access time and are a
starting point rather than gospel.

## The physical-disc interlock

There is one optical drive, so the mirror and a real physical-disc session can
never both use it. **A physical disc always wins.**

`physical_disc_open()` calls `physical_disc_acoustic_set_physical(1)` before it
touches the device, and that call blocks until the mirror confirms it has
closed its handle. `physical_disc_close()` clears it again. While the latch is
set the mirror drops every incoming event, refuses to open the device, and
aborts any gesture already in flight; `touch()` re-checks before *every* SCSI
command, and `START STOP UNIT` and the speed change additionally require
`own_device()`.

This matters more than it sounds. The drive is holding the user's actual game
disc, and `START STOP UNIT (stop)` spins it down — during a core load that
presents as a black screen, because the core simply never gets its data.

So in practice: **acoustic mirroring applies to image-backed play** (CHD or
`.cue`/`.bin`) with a disc in the drive to make the noise on. During real
physical-disc play the drive is already making entirely genuine noise and the
mirror stays completely out of the way. Note that whatever disc is in the drive
is what gets seeked — if that is your game disc rather than a scrap one, it
will spin up and chatter even though the core is reading from an image.

## Known limits

* In physical-disc mode the rhythm you hear is the HPS prefetcher's, not the
  console's. Shaping the real read pattern is possible but risks actual data
  delivery, so it is deliberately not done here.
* USB optical drives vary a lot in how quickly they service `READ(10)` and
  whether they honour `SEEK(10)` or a speed change at all. There is a
  seek-only fallback and a give-up path, both logged.
* The drive cannot be made to take a specific time over a seek; the engine
  issues the ops that move it the right distance and holds the remainder of
  each gesture's slot so the *rhythm* is right.

## PLAY AUDIO mode: how to get a console speed out of a modern drive

A data read on a modern USB drive cannot be slowed down. Measured on a
DE10-Nano's slot-load drive: `SET CD SPEED` is accepted and ignored, requesting
1x gives ~12x and anything above that gives ~19x. A Mega CD is 1x. No amount of
software makes a 19x spindle sound like a 1x one.

Audio playback is the way out, because it has to be real time. `PLAY AUDIO`
(0x45) was measured at **75.1 sectors/s, 1.002x** on the same drive -- a true 1x
CLV. It also gives three things for free:

* the spindle glides with radius exactly as CLV requires, because it really is
  CLV;
* the head advances itself, so streaming needs no commands at all;
* nothing can come from cache, because the drive is decoding as it goes.

Re-issuing `PLAY` at another address is a real sled seek -- 288 ms measured for
a 120000-sector jump -- so one mechanism covers both streaming and seeking.
`HOLD` maps onto audio pause, which is precisely "spindle on, head still".

**So the best mirror disc is a full audio CD.** The engine detects audio tracks
and selects this mode automatically; with a 74-minute music CD the usable
stroke is the full 24-56 mm, against 2.3 mm for a part-written data disc. The
read-based modes remain as fallbacks for a data-only disc.

With a full stroke and a genuine 1x spindle, `PHYSICAL_DISC_ACOUSTIC_GAIN` has
much less work to do; try 1 (faithful) first and only raise it if the
file-system hops are still too small to hear.

## Grime

`PHYSICAL_DISC_ACOUSTIC_GRIME`, 0..10, default 0.

The model deliberately describes a **healthy** drive reading a **clean** disc.
Real consoles are neither by now: the sled is dry, the lens is hazy, the disc is
scuffed, and the servo spends its life losing lock and recovering. That recovery
is most of what an old console actually sounds like — the stutter and the hunt,
not the smooth parts.

Grime adds that back, scaled by level:

| where | what a worn mechanism does |
|-------|-----------------------------|
| data stream | servo slips on a scuffed track and re-reads |
| end of a long seek | dry sled overshoots and corrects |
| spin-up | hazy lens takes several goes to focus |

The dial runs 0..11. Ten is a mechanism well past its best. **Eleven is one
louder**: frequency has nowhere left to go at ten, since it already fires on
everything, so eleven buys magnitude -- it wanders nearly twice as far and takes
several more attempts to find its way back. Measured consequence: a stream
gesture nominally lasting 240 ms takes 2 to 8 seconds, so the mirror stops
tracking the game in any timely way and simply thrashes. That is the point of
eleven, but 9-10 is the ceiling if you want it filthy AND still following the
game.

This is **the one part of the engine that invents activity the original would
not have had on a good day**, which is why it is opt-in and why it lives in the
player rather than in the model — the model stays an honest description of a
healthy mechanism. Everything grime adds is still real mechanism motion: a hunt
is an actual sled move, not a sample.

## Drive choice, measured

Three drives, same music CD, same 120000-sector seek:

| drive | PLAY AUDIO | seek | verdict |
|-------|-----------|------|---------|
| modern slot-load | 1.002x | 288 ms | right speed, far too quiet |
| Mitsumi FX120T (~1997, 12x) | 0.998x | 625 ms | period mechanism, audible |
| IDE-CD R/RW 8x4x32 (early 2000s) | 1.001x | 1385 ms | slowest sled of the three |

All three hit a true 1x under `PLAY AUDIO`, so the differences are purely
mechanical. The slot-loader is engineered to be silent and no software setting
changes that; an older tray drive is the single biggest improvement available.
The CD-RW's sled is slower than a real Mega CD's, which makes its seeks the most
prominent of the three.

## Pick the drive by sled speed

The single biggest factor, and the one no software setting can substitute for.
Measure it as `SEEK(10)` full-stroke latency: `SEEK` waits for the head to
arrive, and its latency scales smoothly with distance, so it reports actual
traverse time. `PLAY`'s latency does NOT -- it is mostly audio-servo
re-acquisition and only a little travel, which is a trap worth avoiding.

| drive | full-stroke `SEEK(10)` | vs a Mega CD (~800 ms) |
|-------|-----------------------|------------------------|
| modern slot-load | — (silent regardless) | — |
| IDE-CD R/RW 8x4x32 | 168 ms | ~5x too fast |
| **Mitsumi FX120T (12x, ~1997)** | **693 ms** | **within ~15%** |

On a 168 ms sled a traverse is over before you hear it, and no amount of
overshoot or grime makes it laboured -- it just makes it busy. On the Mitsumi
the mechanism is doing the work, so faithful settings are the right ones:
`GAIN=1` and a low `GRIME`. Grime levels tuned on a fast sled will be far too
aggressive here, since each overshoot pass is now a real 200-700 ms traverse.

## Where the Mega CD seek timing comes from

Not guessed. Genesis Plus GX's `core/cd_hw/cdd.c` is the reference
implementation for Mega CD CD emulation, and the MiSTer core's own latency model
agrees with it:

```c
cdd.latency  = 2 + 10*config.cd_latency;                        // base
cdd.latency += ((delta_lba) * 120 * config.cd_latency) / 270000; // distance
// "max. seek time = 1.5 s = 1.5 x 75 = 112.5 CDD interrupts
//  (rounded to 120) for 270000 sectors max on disc"
```

A fixed base of 2+10 interrupts (~160 ms at 75 Hz) plus a term proportional to
LBA distance, reaching **1.5 s across the whole disc**. No short-seek plateau.

That matters because a data-to-audio transition is two of those plus the
lock-on, which is where the three seconds people remember comes from: a Mega CD
game loading while music plays has to stop the audio, cross to the data track,
read, and cross back.

An earlier shape invented here had a short-seek plateau and a full-stroke figure
of 800 ms, making every Mega CD seek about half as long as the hardware takes --
the single most audible thing the drive does, played at double speed.

Distance is measured in sectors for the DURATION, because that is what both the
reference emulator and the core use and therefore what the games' timing was
built against. Spiral turns remain the right measure for deciding WHETHER the
sled moves at all, which is a question about the mechanism rather than about
emulated timing.

### Turning that into traverses

One traverse of this drive is 693 ms across the whole disc, measured, so a single
traverse is well short of a Mega CD seek. The player divides the modelled
duration by the drive's own measured traverse time -- `SEEK(10)` latency rises
close to linearly with distance (28 ms at 500 sectors, 188 at 20000, 392 at
150000, 693 at 280000) -- and crosses the distance that many times.

Deriving the count this way rather than from invented distance thresholds is what
makes correcting the model's timing actually change what comes out of the drive.
Measured afterwards: 3.6 s of continuous sled movement on a data-to-audio
transition.

## Making a gesture take the right amount of time

The model says how long the original drive would have spent. Getting the mirror
to agree took several wrong turns, and the measurements are worth keeping:

* **A fine staircase judders.** Every SCSI positioning command is a
  move-and-settle: the firmware runs the sled to the address and stops it dead.
  Back-to-back steps are not continuous motion, they are one settle after
  another -- a 24 Hz buzz at `SEEK`'s cost, a row of clunks at `PLAY`'s.
* **One traverse is too short.** The drive crosses the whole disc in 693 ms
  where a Mega CD seek runs to 1.66 s.
* **Overshoot ping-pong cannot fill a duration.** Each pass covers the whole
  distance plus an overshoot, so one pass is short and two are long. That is
  where 1.5-2x overruns came from, and an overrun is heard as a seek against the
  wrong thing on screen.

What works is solving for the step count from the drive's measured cost curve.
`SEEK(10)` here costs about **143 ms fixed plus 0.00196 ms per sector** (fitted to
261 ms at 60000 sectors and 693 ms at 280000). The fixed part dominates, so

    total = N*FIXED + PER_SECTOR*distance + TAIL   =>   N = (total - travel - TAIL)/FIXED

and `N` is the knob. `TAIL` is ~570 ms and matters: every grind ends with a
settle seek, an arrival seek and a `PLAY` to restore 1x, and all three land inside
the budget. Ignoring them is why a 1339 ms seek took 2564 ms -- the steps filled
the budget and then the tail ran past it.

Measured after: SPINDOWN 1.00x, cross-disc seek 1.09x, lock-on 1.06x of what the
model asked for, against 1.9-4.5x before.

### Two things deliberately NOT done

**No staleness test.** There was one, dropping moves older than 1.2 s on the
grounds that a late seek is worse than none. It threw away the audio lock-on
(which follows its own seek and is therefore late by construction) and two seeks
in five behind the boot sweep. It was also redundant: coalescing merges queued
seeks into one movement to the newest target, so a seek cannot be superseded by
the time it is played. Lateness is handled by going to the right place.

**Seeks are coalesced, not replayed.** A queue of seeks is not several movements,
it is one movement to wherever the last of them points, because the drive can
only be in one place. Their durations sum, so a transition still takes a long
time; it arrives as one continuous grind instead of four late ones.
