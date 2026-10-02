# Handoff — physical acoustic mirroring

Session of 2026-10-01/02. Branch `acoustic-rework`.

## What was wrong, and it was one thing

`SEEK(10)` on this drive **does not wait for the head**. Measured per-command from
inside the player: a bare `SEEK` returns in **1.0 ms for every distance from
0.5 mm to 32 mm**. No sled crosses a disc in 1 ms — the drive queues the command
and acknowledges it at once.

So a staircase of seeks issued back to back was never several movements. Each
target overwrote the one before it and the drive performed **one** move, to the
last address. Every grind, hunt, stutter, overshoot and lock-on correction built
on bare `SEEK` was inaudible. That is why a night of reshaping them changed
nothing that could be heard.

An earlier test appeared to *disprove* this — six 13 mm moves showed 100% of the
asked travel — because it read the position back with `READ SUB-CHANNEL` after
each seek, and that read serialises behind the pending move. The sync was doing
the waiting, not the seek.

`mirror_seek_sync()` (SEEK then READ SUB-CHANNEL) is the fix, and everything meant
to register as a separate sled movement now goes through it.

### The second half of it

A data read was one `PLAY` followed by a sleep. An external head trace of a live
Sonic CD session found the sled moving **twice in forty seconds** — the game asks
for very few seeks, so seeks were never going to carry the sound, and the *load*,
the part anyone recognises, was silence. A Mega CD loading is a repeating cadence
(CDC takes a run of sectors, CDD pauses while the Sub-CPU drains the buffer, lens
re-locks, repeat). A data `STREAM` gesture now makes one real synchronised move,
alternating around the target — 4 Hz at the 250 ms gesture length.

## Correction: the cadence was wrong, and so was "weak"

Feedback on the first fix: *"not grimy, just weak and shitty like the laser is
broken"*. Both halves of that were accurate.

The data-read cadence oscillated the sled 0.35 mm each way at 4 Hz, on my own
theory that a load is the CDC's buffer-fill rhythm. A real sled advances
**monotonically** through a read, and a small movement repeated back and forth in
one place is exactly what a drive does when it cannot hold the track. Removed,
along with the matching nudge during CDDA.

"Weak" was structural. A synchronised seek gives ~50 ms of travel then 300-700 ms
of waiting, and in service the budget only bought one or two, so a cross-disc seek
was a short jerk, a long silence and a reversal.

### Seeks are now ONE CONTINUOUS DRAG

`sled_drag()` re-aims repeatedly along the path with **bare** seeks and no sleep.
The drive takes one move at a time, so the next SEEK blocks until the current
finishes and the sled never stops -- the drive's own serialisation is the clock.
192 ms for a 0.7 mm step issued this way, against 300-700 ms through
`mirror_seek_sync`, whose sub-channel read is pure overhead.

Sleeping between the re-aims (the first attempt) added the budget on top of that
serialisation and overran 5x -- a 1339 ms seek took 6881 ms. Removing the sleep
brought it to 1339 ms exactly.

Short seeks do not wait at all: one bare SEEK moves the sled just as far and
returns at once, then the slot is held. That took a 286 ms gesture from 1154 ms
back to 287 ms.

### Gain was off

`PHYSICAL_DISC_ACOUSTIC_GAIN=1` means **no amplification** -- every move is only as
big as the game's own geometry, about a third of the stroke. At 2 a transition
becomes a full 32.3 mm traverse. At 3 it saturates: the trace pinned at both clamps
(LBA -13 and 309568) and the head parked at the rim, where further outward moves do
nothing. Set to 2; original ini saved as `/media/fat/MiSTer.ini.preacoustic`.

### Measured, final

```
drag 32.3mm over 1125ms in 3 segs, whole 1125ms     SLEW  287ms / 286 modelled
drag 29.2mm over 1339ms in 4 segs, whole 1550ms     SLEW 1314ms / 1288
drag 32.3mm over 1288ms in 3 segs, whole 1314ms     LOCK 1534ms / 1491
```

3-4 contiguous segments across the whole stroke, filling the duration.

### Note on instrumentation

`headtrace.py` reports a drag as ONE hop of 24 mm between consecutive samples,
which would be impossible sled speed. With several seeks queued the sub-channel
reports the **commanded** position, not the actual one, so it cannot resolve a
drag's interior. The elapsed time in `/tmp/acoustic.log` is the reliable evidence:
n blocking seeks taking 1288 ms means the drive was in motion for 1288 ms.

## Verified end state

Measured externally with `headtrace.py`, same tool and game as the baseline:

| 40 s of Sonic CD | before | after |
|---|---|---|
| total sled travel | 41.34 mm | **82.70 mm** |
| discrete sled movements | **2** | **60** |
| head behaviour | parked at 0.01 mm/s for 38 of 40 s | active across 25.2–49.5 mm |

Running binary confirmed via `/proc/<pid>/exe` → `/media/fat/MiSTer_Physical-CD`.

Timing discipline over 12 long seeks: median **0.99x** of the modelled duration,
mean 1.10x, range 0.81–1.54x. LOCK 1.00–1.04x. The spread is the drive's own
run-to-run variance — the same 20.5 mm move has cost 471 ms on one pass and
1044 ms on the next — not a modelling error, and the step loop now checks the
remaining budget before each step so a slow pass is cut short rather than
overrunning.

## Calibration must be done IN SERVICE

A synchronised step costs 150 ms on an idle drive and **250–700 ms with the core
running**, because every command queues behind the read cadence and audio
playback. Budgeting against bench figures is what left long seeks 35–50% over.
`STEP_FIXED_MS` is the in-service 300 ms. `RESUME_PLAY_MS` was guessed at 250 ms
and measured at 25–28 ms.

Consequence: a 20 mm seek affords **one** staircase step plus the overshoot and
return — 3 real sled movements in ~1.3 s. That is this drive's ceiling at a
faithful duration, and the tail (the reversal) is the most expensive and most
acoustically useful part of it.

## Spindle: the drive honours speed changes

Settled, having been unknown all session. On the Mitsumi FX120T, raw-sector
throughput against requested speed: 1x→21.7 sectors/s, 2x→83.9, 4x→164.7,
8x→322.9, max→333.2. Monotonic, capped near 4.4x. So the PSX's 1x/2x alternation
is real rather than cosmetic. **On Mega CD there is correctly nothing to ramp** —
it is 1x CLV throughout, and the log shows `spindle to 2x` then `spindle to 1x` at
acquire, which is right.

## Test harness — three traps that each silently run the STOCK binary

1. `/etc/inittab` uses `::sysinit:/media/fat/MiSTer &` — **sysinit, not respawn**.
   `killall` leaves the board dead until reboot. Use `/dev/MiSTer_cmd`.
2. `main=` is per-ini-section, so the **core name** decides the binary.
   `main=MiSTer_Physical-CD` sits under `[A0CD-*]`, and the global `main=` points
   at a missing `ConsoleMode/` directory. A test MGL without
   `<setname>A0CD-…</setname>` runs stock. **Always check `/proc/<pid>/exe`.**
3. You cannot overwrite the running main (ETXTBSY). Stage to a temp name and
   `mv` over it.

Launching Sonic CD from a CHD needs **two stages**, because an `A0CD-` core makes
the launcher mount the *physical disc* and ignore the MGL's CHD: load the `A0CD-`
MGL first to switch the running binary, then a plain-`MegaCD` MGL (the dangling
global `main=` means MiSTer keeps the binary it is already running).

MGL `<file path=…>` must be ABSOLUTE; a relative path fails silently.

## Open

* **Needs the user's ears.** The cadence amplitude (`0.10 + 0.035 * grime`) and
  its 4 Hz rate are judgement calls. An earlier 0.71 mm at 3 Hz was 4.3 mm/s of
  continuous travel and likely reads as the judder previously complained about;
  the current default is a tick. Grime is now a real dial, because before this
  it was adjusting things that never moved.
* `SPINUP` runs 1.5x long (3327 ms against 2200 modelled) — the drive takes that
  long to come up; a hardware floor, not a budget error.
* Boot `SWEEP` runs 2.1x long, deliberately: at grime ≥ 5 it adds a 2 s
  spin-down/up cycle on top, boot only. Not a defect.
* The PSX 1x↔2x ramp has never been **observed in game** — the drive is proven
  capable, but both test games played no CDDA unattended.
* Saturn figures are now sourced (Mednafen `ss/cdb.cpp`). PC Engine is Shadoff's
  measured curve, Mega CD is GPGX, PSX is DuckStation. `auto`, 3DO, CD-i and
  Neo Geo CD remain derived.

## Diagnostics (push to /tmp; the board has python3, no gcc, no pgrep)

`tools/`-style scripts kept outside the repo this session: `seeksync.py` (the one
that found it — per-distance cost of bare SEEK vs SEEK+subq vs SEEK+read),
`headtrace.py` (real head position from outside the mirror; pass a lower hop
threshold to see the cadence), `discinfo.py` (drive/disc/TOC state),
`speedtest.py` (does the drive honour SET CD SPEED), `seektest.py`, `playtest.py`.

Note `headtrace.py` can only see *net* displacement during a burst: SCSI commands
serialise, so the poller stalls behind each in-flight move. It undercounts a
staircase; the per-step latencies in `/tmp/acoustic.log` are the finer instrument.
