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
