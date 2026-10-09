# Handoff — physical acoustic mirroring

Session of 2026-10-01/02. Branch `acoustic-rework`.

## The root cause

`SEEK(10)` on this drive **does not wait for the head** when the drive is idle.
Measured per-command from inside the player: a bare `SEEK` returns in **1.0 ms for
every distance from 0.5 mm to 32 mm**. No sled crosses a disc in 1 ms — it is
queued and acknowledged at once.

So a staircase of seeks issued back to back was never several movements. Each
target overwrote the one before it and the drive performed **one** move, to the last
address. Every grind, hunt, stutter, overshoot and lock-on correction built on bare
`SEEK` was inaudible, which is why a night of reshaping them changed nothing that
could be heard.

An earlier test appeared to *disprove* this — six 13 mm moves showed 100% of the
asked travel — because it read the position back with `READ SUB-CHANNEL` after each
seek, and that read serialises behind the pending move. The sync was doing the
waiting, not the seek.

**But a seek issued while a move is already in flight DOES block**, because the
drive accepts one move at a time. That second fact is what the final design runs
on, and it took a 5x overrun to notice.

## What it sounds like, and two wrong turns

Feedback on the first fix: *"not grimy, just weak and shitty like the laser is
broken"*. Both halves were accurate.

**Wrong turn 1 — the oscillating cadence.** I had data reads nudge the sled 0.35 mm
each way at 4 Hz, on my own theory that a load is the CDC's buffer-fill rhythm. A
real sled advances **monotonically** through a read, and a small movement repeated
back and forth in one place is precisely what a drive does when it cannot hold the
track. It is the sound of a failing laser, not a worn one. Removed, along with the
matching nudge during CDDA — a deck tracking a track is quiet, and the noise belongs
in the traverses either side of it.

**Wrong turn 2 — synchronising every step.** A synchronised seek gives ~50 ms of
travel then 300–700 ms of waiting. In service the budget only ever bought one or
two, so a cross-disc seek came out as a short jerk, a long silence and a reversal.
That is the "weak", and it was structural rather than a tuning problem.

## Seeks are one continuous drag

`sled_drag()` re-aims repeatedly along the path with **bare** seeks and **no sleep**.
The drive takes one move at a time, so the next SEEK blocks until the current one
finishes and the sled never stops. The drive's own serialisation is the clock:
192 ms for a 0.7 mm step issued this way, against 300–700 ms through
`mirror_seek_sync`, whose sub-channel read turns out to be pure overhead.

Sleeping between the re-aims (the first attempt) added the budget *on top of* that
serialisation and overran 5x — a 1339 ms seek took 6881 ms. Removing the sleep
brought it to 1339 ms exactly.

Segment count comes from `n = (total − PER_MM·dist) / FIXED` with `FIXED` = 180 ms
(back-to-back bare seek) and `PER_MM` = 16 ms/mm, is re-checked against the
remaining budget before each segment because the drive varies run to run, and the
slot is padded if the drive beats it.

Short seeks do not wait at all: one bare SEEK moves the sled just as far and returns
at once, then the slot is held. That took a 286 ms gesture from 1154 ms to 287 ms.

The overshoot is dragged out and back, so settling is a reversal at the end of a
continuous traverse rather than two isolated jerks. `grime_hunt` wanders by dragging
for the same reason, and the spin-up only hunts if its budget can take it (an
unbounded one ran 4213 ms against 2200).

## Gain was off

`PHYSICAL_DISC_ACOUSTIC_GAIN=1` means **no amplification** — `amplified_mirror_lba()`
multiplies each head *delta* by the gain, so at 1 every move is only as big as the
game's own geometry, about a third of the stroke. That is a direct cause of "weak".

* **2** — a transition becomes a full 32.3 mm traverse. Current setting.
* **3** — saturates: the trace pinned at both clamps (LBA −13 and 309568) and the
  head parked at the rim, where further outward moves do nothing.

Original ini saved as `/media/fat/MiSTer.ini.preacoustic`. Grime is at 7, and it is
a real dial now — before this it was adjusting things that never moved.

## Measured, final

```
drag 32.3mm over 1125ms in 3 segs, whole 1125ms     SLEW  287ms / 286 modelled
drag 29.2mm over 1339ms in 4 segs, whole 1550ms     SLEW 1314ms / 1288
drag 32.3mm over 1288ms in 3 segs, whole 1314ms     LOCK 1534ms / 1491
                                                    SPINDOWN 2204 / 2200
```

3–4 contiguous segments across the whole stroke, filling the duration. Running
binary confirmed via `/proc/<pid>/exe` → `/media/fat/MiSTer_Physical-CD`.

## What a Mega CD drive actually does (researched, after "gravely")

Three findings, each of which contradicted something I had built.

**A long seek is ONE coarse move, not a march.** The sled motor runs the pickup to
the *estimated* position of the target track, partially open-loop for long jumps,
after which the servo closes the loop, reads the address and corrects. Equal
segments are a sound no drive makes: each one accelerates, decelerates and settles,
so 2-4 across a disc is a 2-3 Hz chug. That is exactly the reported gravel.

**Fine structure is not available over USB at all.** The CDD runs its command and
status loop once per sector period -- exactly 75 Hz. The real servo therefore
corrects about twenty times faster than anything reachable here, so imitating it at
whatever rate the bus allows does not give a coarser version of the real sound, it
gives a different one.

**The sled is a worm gear on two rails** (Model 2 repair documentation), which is a
continuous whirr rather than a ratchet -- and slow, which is why a real seek takes
1.5-2 s where this 1997 Mitsumi crosses the whole disc in about 670 ms.

### Consequence: two sweeps and one turnaround

One sweep leaves most of the budget silent, and subdividing it is the chug. The way
to fill the time is **distance, not subdivision**: overshoot the target and come
back. Two continuous sweeps, one direction change, sled moving nearly throughout --
and it is what a partially-open-loop seek does anyway, since it misses and gets
corrected. The overshoot is sized from the leftover budget rather than a fixed
fraction, so spare time becomes travel, and it grows for shorter seeks that would
otherwise finish early.

The lock settle and the wear hunt are single sweeps each now, for the same reason.

```
drag 29.2mm over 1339ms in 2 segs, whole 1339ms    data -> CDDA, 1.00x
settle -1.2mm from 56.3mm                          lock correction, inward
LOCK took 803ms / 746 modelled                     2142 ms total before the music
drag 32.3mm over 1288ms in 2 segs, whole 1288ms    CDDA -> data, 1.00x
SLEW 300ms / 286 modelled                          short seek: one move
```

### The CDD command set settles where the noise belongs

From the MegaSD reverse-engineering notes (gendev.spritesmind.net/page-megasd.html),
the most complete account of the CDD there is:

* **Command 0x03 READ/PLAY** is *"SEEK to start position THEN Play music / Read
  data"* -- one command, status going to PLAY straight away. There is no long lock
  phase between the seek and the music, so a quiet LOCK gesture sitting in that gap
  was a fiction of this model. It is now 1 revolution, 249 ms.
* **Error 0x03 E-FOCUS**: *"Focus down for more than 100msec will retry until ok"*.
  Locking on is FOCUS, a lens operation. It moves no carriage, so it makes no sled
  noise, and the two versions of a sled "settle" put here were both wrong.
* **Seek time is explicitly undefined** -- *"??seek time to be defined"* appears
  twice. Combined with the GPGX author's own disclaimer, there is no published
  figure anywhere, which is why the owner's ear is the authority.
* Interrupts are *"every 1/75s (13.3ms) while data transfer is on progress"*, every
  15.8 ms otherwise. The 75 Hz command loop is the hard ceiling on fine structure.

So the noise is all in the SEEK. `full_stroke_ms` is 2400, which satisfies both of
the owner's independent observations with one constant: the Sonic CD data-to-CDDA
transition (0.703 of a stroke) comes to 1972 ms, *"about two seconds"*, and a true
cross-disc seek to 2685 ms, *"three second sled drags"*.

### Filling a long budget without chunking it

A Mega CD fills 1.5-2 s with ONE continuous traverse because its worm-gear sled is
roughly three times slower than this drive, which crosses the whole disc in 670 ms.
That speed cannot be lowered. And once the target is at the rim -- where a CDDA
track is -- there is no more single-direction ground to cover.

So the budget is spent on the main traverse plus as many out-and-back pairs as it
affords, each leg at least 8 mm so it reads as a sweep and not as one of the equal
little steps that got called gravely. If there is no room for a proper leg it stops
and holds the slot, because a short jerk is worse than nothing.

```
drag 29.2mm over 1971ms in 3 segs, whole 1982ms    1.01x, ~59 mm of travel
LOCK took 249ms / 249 modelled                     brief and quiet
drag 32.3mm over 1923ms in 3 segs, whole 1963ms    1.02x
```

Note `sleep_ms` clamps at 500 ms (deliberately, to stay responsive to on/held), so
the remainder hold has to use `sleep_long_ms` -- it was silently truncating a 714 ms
pad and leaving the end of a seek silent.

### The jitter was never the seeks

Report: *"maybe one 1.5 second long travel but otherwise just little skittish
jitters and 0.25 second travels"*.

Measured over 260 s of play there were only **sixteen** head movements, nearly all
29-32 mm, with a 95 s gap between clusters. So the seeks could not have been the
skittishness. The clue was "0.25 second": a STREAM gesture is 250 ms, and the
data-read branch was issuing a `mirror_play` on **every** one of them -- four a
second, each re-acquiring the audio servo -- even though the modelled head movement
was 0.000 mm.

"Quiet" has to mean issuing nothing. It now re-aims only when the head is actually
somewhere else, which is the test the audio branch already used:

```
re-aims in ~150 s : 18      (was ~4 per second, about 600)
STREAM gestures   : 472     so 454 of them issue nothing at all
drags             : 1971ms -> 2026, 1923 -> 1959, 1971 -> 1987, 1865 -> 2041
```

Lesson worth keeping: a command that moves the head 0 mm is not silent. Every PLAY
re-establishes the audio servo and is audible. Count commands issued, not millimetres
modelled.

### Velocity, and why it was right only sometimes

The owner's key observation was that the correct sound DID happen, a few times, but
inconsistently. That rules out a hardware wall and points at variance, and measuring
per segment found it.

This drive has two sled speeds and a Mega CD's is in neither:

```
PLAY AUDIO tracking   0.012 mm/s
SCAN (0xBA)           rejected, not supported
SET CD SPEED 1x       524 ms per 20 mm against 472 at max -- 11%, no use
one plain seek        about 48 mm/s
a Mega CD             about 16-21 mm/s
```

Back-to-back seeks are the only thing in range, because the fixed per-command cost
dominates a short move. So segment length IS the velocity control:
`v = L / (FIXED + PER_MM*L)`, and shorter legs are SLOWER.

**The S-curve ramp was the bug.** Added to avoid a metronome, it made the last leg
tiny while still paying the full fixed cost. One drag measured:

```
2ms/6.5mm=3400   315ms/14.9mm=47   538ms/9.4mm=17   412ms/1.6mm=4   mm/s
```

Nothing, a zip, a correct grind, a crawl -- averaging to a respectable 17 mm/s that
was never played. One leg in four was at Mega CD speed, which is exactly "heard it a
few times".

**The drive's own cost varies 2-3x pass to pass**, 222 to 971 ms for an identical
8.1 mm leg, so a fixed plan gives 20 mm/s on one seek and 35 on the next. Equal legs
fixed the within-drag spread; the between-drag spread needed a feedback loop that
measures each leg, keeps a running estimate of the fixed cost, and re-solves for the
length that hits the target velocity.

Two details that matter: the first leg always returns in ~1 ms because the sled is
not yet moving, so only a leg that blocked carries information; and a leg's elapsed
time is really the PREVIOUS leg's travel, so the measurement lags by one and a high
gain rings (0.6 gave 27, 15, 23, 6 mm/s inside one drag -- 0.25 is stable).

Also removed: an unconditional final "land exactly" seek that travelled 0.0 mm and
blocked up to 1257 ms on the previous move, and any stub remainder leg, which still
pays the whole fixed cost and so comes out at 4-6 mm/s.

```
per-drag mean velocity : 14-20 mm/s   (was 11-18 with legs at 32-36)
duration               : 1.00-1.13x
trailing 0.0mm leg     : gone
stub legs              : gone
```

Still open: the within-drag spread is damped, not eliminated (legs still range
14-35 mm/s), because the drive's variance is large and the measurement lags. The
remaining lever with real headroom is mechanical -- a drive whose sled is genuinely
slower. Measure a candidate by full-stroke synced seek time: this Mitsumi does
670 ms and a Mega CD wants 1.5-2 s.

Direct servo control is not available: the servo is inside the drive firmware and no
SCSI path exposes it.

### Listening to the recording, which beat every other diagnostic

The owner sent a 99 s phone recording of a live session. It cannot be heard here,
but it can be measured: ffmpeg to raw PCM, 10 ms RMS envelope, threshold at
30% of the floor-to-peak range, runs joined across gaps under 80 ms.

```
86 events over 99.3 s
  a handful loud and long : 4.93s, 4.44s, 3.21s, 1.50s, 1.28s  (-34 to -44 dB)
  dozens quiet and short  : 0.08-0.72s                          (-55 dB)
  modulation inside the long events: 1-4 Hz at depth 0.01-0.10
```

Two conclusions, and the first one overturned the plan. **The segmentation was not
the problem** -- modulation depth inside the long drags is low, so the stepping is
not what was being heard. And **the gesture log accounted for about 6 head movements
in that span against 86 audible events**, so something was making noise fourteen
times more often than the model knew.

A per-command census found it, because the drive makes noise per COMMAND and the log
counted gestures:

```
subq=21 per 5s   4.2/s, the entire session
play=16-17 per 5s 3.4/s, throughout every CDDA track
```

The PLAY storm was a real bug. `tail = span_hi - target` clamped to a minimum of ONE
block, and with a gain applied the jump to a CDDA track saturates and pins the target
at `span_hi` -- so tail came out 1, every PLAY played a single sector and stopped,
the next gesture saw "not playing", and the drive spent whole music tracks
re-acquiring its audio servo four times a second.

Fixed by giving PLAY at least `PLAY_MIN_BLOCKS` and backing the start off when the
head is too near the rim for that. Which then reinstated the storm from the other
side, because the drift check still compared the drive's position against `target`
while playback legitimately started up to 4000 sectors earlier -- it has to compare
against the actual play start. And the sub-channel poll, which is only a drift check,
now runs at 1 Hz rather than once per gesture.

```
command rate, steady state : 7.6/s -> 1.4/s
subq                       : 4.2/s -> 1.0/s
play during CDDA           : 3.4/s -> 0.2-1.0/s
```

**Keep the recording loop.** `/c/t/analyse.py` takes an audio file and prints event
durations and their modulation. A static ffmpeg is at `/usr/local/bin` in WSL (apt is
broken on this machine -- systemd dpkg error -- so it came from
johnvansickle.com/ffmpeg). Asking for a phone recording and measuring it found in one
pass what several rounds of reasoning about the gesture log had missed.

### On the sources

Genesis Plus GX's author says its CDD latency model is *"not accurate to how the
real micro-controller and CD mechanism worked"* and that timings *"should be
measured on real hardware"*. The 1.5 s full stroke this profile was built on is an
emulator convenience, not a measurement -- so for an acoustic emulator the owner's
ear is the better authority, which is why lock_revs came down on their word.

There is no published Mega CD seek curve. The CDD protocol is documented (ten 4-bit
nibbles, 75 Hz, BCD MSF addresses) but its mechanical timings are not.

## Spindle: the drive honours speed changes

Settled, having been unknown all session. On the Mitsumi FX120T, raw-sector
throughput against requested speed: 1x→21.7 sectors/s, 2x→83.9, 4x→164.7, 8x→322.9,
max→333.2. Monotonic, capped near 4.4x, so the PSX's 1x/2x alternation is real
rather than cosmetic. **On Mega CD there is correctly nothing to ramp** — it is 1x
CLV throughout, and the log shows `spindle to 2x` then `spindle to 1x` at acquire.

## Instrumentation caveat

`headtrace.py` reports a drag as ONE hop of 24 mm between consecutive samples, which
would be impossible sled speed. With several seeks queued the sub-channel reports the
**commanded** position, not the actual one, so it cannot resolve a drag's interior.
It also only ever sees *net* displacement during a burst, because commands serialise
and the poller stalls behind each move. The elapsed time in `/tmp/acoustic.log` is
the reliable evidence: n blocking seeks taking 1288 ms means 1288 ms of motion.

## THE REASON IT KEPT SOUNDING "NO DIFFERENT"

The owner was listening to the **stock binary**. Proven by launching Sonic CD the
way they do -- a plain `MegaCD` core from a CHD -- and reading `/proc/<pid>/exe`:

```
BEFORE:  exe: /media/fat/MiSTer             core: MegaCD   <- stock, empty log
AFTER:   exe: /media/fat/MiSTer_Physical-CD core: MegaCD   <- 132 log lines
```

`main=MiSTer_Physical-CD` sits under `[A0CD-*]`, so it only applies to cores named
`A0CD-...`. A CHD launch produces the core name `MegaCD`, nothing matches, and the
global `main=ConsoleMode/MiSTer_ConsoleMode` **points at a directory that does not
exist on the card**, so MiSTer keeps the stock binary it booted with. Stock has no
acoustic mirroring at all.

Fixed by appending to `MiSTer.ini`:

```
[MegaCD]
main=MiSTer_Physical-CD
```

This very likely explains the whole run of "it makes the right noises... sometimes"
and "maybe 20% of the time" reports. Whether the feature ran at all depended on
launch order: my own deploy scripts leave the board running the build via a
two-stage launch, so a listen straight after a deploy heard it, and a listen after
launching the game manually heard stock. The same gate applies to PSX, Saturn,
PCECD, NeoGeoCD, 3DO and CD-i launched from images -- each needs its own section, or
the dangling global `main=` needs fixing.

**Check `/proc/<pid>/exe` before trusting any listening test.** I had documented
this trap and then only solved it for my own harness.

## Test harness — three traps that each silently run the STOCK binary

1. `/etc/inittab` uses `::sysinit:/media/fat/MiSTer &` — **sysinit, not respawn**.
   `killall` leaves the board dead until reboot. Use `/dev/MiSTer_cmd`.
2. `main=` is per-ini-section, so the **core name** decides the binary.
   `main=MiSTer_Physical-CD` sits under `[A0CD-*]`, and the global `main=` points at
   a missing `ConsoleMode/` directory. A test MGL without `<setname>A0CD-…</setname>`
   runs stock. **Always check `/proc/<pid>/exe`.**
3. You cannot overwrite the running main (ETXTBSY). Stage to a temp name and `mv`
   over it.

Launching Sonic CD from a CHD needs **two stages**, because an `A0CD-` core makes the
launcher mount the *physical disc* and ignore the MGL's CHD: load the `A0CD-` MGL
first to switch the running binary, then a plain-`MegaCD` MGL (the dangling global
`main=` means MiSTer keeps the binary it is already running).

MGL `<file path=…>` must be ABSOLUTE; a relative path fails silently. `pgrep` does
not exist on this board.

## 2026-10-09: the USB drive is replaced by a servo rig

The USB drive hit a physical wall (SEEK is non-blocking, the sled is 2-3x too fast, every
stop/start is an audible impulse), so the noise now comes from a PS1 optical block driven
through its own BA5977FP by an RP2040. Everything is in `support/physical_disc/SERVO_RIG.md`
under "The rig as built": wiring, the serial protocol, the measured textured-drive speed
table, the bench tools, and how gestures map to commands. Code: `rp2040/servo_fw.py`
(firmware), `physical_disc_rig.cpp` (MiSTer side, selected by `PHYSICAL_DISC_ACOUSTIC_RIG=1`).
The MiSTer's `[MegaCD]` section already has the two keys; the binary is deployed. What is
left is the end-to-end listen with the Pico on the MiSTer's USB.

## 2026-10-10: the lens orchestra, a microphone, and two open wires

* Firmware: `LENS ... Z` (broadband noise, PIO + DMA), `LENS ... G` (triangle or sine tone), `SPIN` kick, a 15 s
  host-silence watchdog (a core that exits no longer leaves the spindle or noise running), verified streaming
  deploy in `picotool.py` (the old REPL upload dropped characters and then ran out of RAM at 31 KB).
* Translator: per-console `lens_policy`; boot lens sequence once per disc; keepalive PING every 2 s while idle.
* All of it is in `SERVO_RIG.md` ("The lens orchestra", "Open wires"). Deployed: Pico firmware CRC 0xf746a0e7,
  MiSTer binary md5 4f90c214..., both on the MiSTer; core left at the menu.
* **The sled and spindle do not move because GP4 and GP2 are not connected** (probe results in SERVO_RIG.md). Fix
  the joints first, then re-run `rec.py` + `pattern.py` + `pattern_analysis.py`: with the spindle on versus off
  there must be a measurable difference, and the 1-4 kHz body of the real console's sound has to come from those
  two. Until then the PlayStation/Mega CD comparison is lens-only.
* Needs the owner: sine or triangle for the tone; whether the steady PlayStation noise (level 2 at 8 kHz, tuned
  by band level against the recording) sounds right by ear.

## Open

* **Needs ears, not measurement.** Whether 3–4 contiguous segments across the
  stroke reads as a sled *drag* or as a chug is not something the log can answer.
  The lever is segment size: fewer, longer segments are smoother but leave silence
  at the end of the budget; more, shorter ones fill it but risk judder. The count is
  now logged next to the duration that produced it, so a listen can be tied to a
  number.
* Boot `SWEEP` runs ~2.5x long deliberately: at grime ≥ 5 it adds a 2 s
  spin-down/up cycle on top, boot only. Not a defect.
* `SPINUP` now runs *short* (1.1–2.7 s against 1.5–2.2 s modelled) because the hunt
  is skipped when its budget will not fit. Under-running is less harmful than over,
  but it means the ramp is sometimes barely there.
* The PSX 1x↔2x ramp has never been **observed in game** — the drive is proven
  capable, but both test games played no CDDA unattended.
* Profiles: Mega CD is GPGX, PSX is DuckStation, Saturn is Mednafen `ss/cdb.cpp`
  (87 ms fixed / 620 ms stroke), PC Engine is Shadoff's measured curve. `auto`, 3DO,
  CD-i and Neo Geo CD remain derived.

## Diagnostics (push to /tmp; board has python3, no gcc)

`seeksync.py` — per-distance cost of bare SEEK vs SEEK+subq vs SEEK+read; this is
the one that found the root cause. `headtrace.py` — real head position from outside
the mirror (see caveat above). `discinfo.py` — drive/disc/TOC state.
`speedtest.py` — does the drive honour SET CD SPEED. `seektest.py`, `playtest.py`.
