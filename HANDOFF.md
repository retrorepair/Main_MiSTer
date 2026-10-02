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
