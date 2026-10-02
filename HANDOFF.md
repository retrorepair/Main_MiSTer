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
