#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "acoustic_model.h"
#include "cd_geometry.h"

// Gesture queue depth. Must match the array in acoustic_model.h.
#define ACU_OUT_MASK 31

// Drive characteristics per console.
//
// The seek figures are the published or measured behaviour of the drive
// mechanisms these consoles shipped with. The PC Engine row comes straight
// from the measurements in support/pcecd/seektime.cpp (Dave Shadoff ran those
// on real hardware); the others are scaled from their mechanism's rated access
// time and spin speed, so treat them as a good starting point rather than
// gospel. They are deliberately all in one table so they can be tuned by ear
// without touching any logic.
// lock_revs: a drive does not start playing the instant it reaches an audio
// track. It settles, acquires the subcode and spin-locks first, and the disc
// turns several times while that happens. On a Mega CD that is audible and
// unmistakable -- at least six revolutions of the disc before the music starts.
// Timed from the geometry, so it is longer at the rim where a revolution holds
// more sectors.
static const acu_drive_t drives[PD_ACU_PROFILE_COUNT] = {
	// `sstep` is the fixed base cost of any seek; `stroke` the extra for
	// crossing the whole disc. See acu_model_seek_ms().
	//
	// name       data audio jump short settle base stroke spinup sdown  ra sweep lock
	{ "auto",      2.0, 1.0,  32,  640,   30,   160,  1200,  1500, 8000,  8, 0,   4 },
	// PlayStation: Sony KSM-440, 2x data / 1x audio. Fast, chattery sled.
	// base 100 ms, stroke 900 ms and spin-up 1000 ms are DuckStation's figures
	// (src/core/cdrom.cpp): a medium seek costs 0.05-0.1 s, a sled seek
	// SLED_FIXED_COST 0.05 s plus SLED_VARIABLE_COST up to 0.9 s total, and
	// spin-up is one second. It also only engages the sled past 7200 sectors,
	// which is roughly 300 turns at mid-disc -- hence the lower break here.
	{ "PSX",       2.0, 1.0,  32,  300,   25,   100,   900,  1000, 0,     8, 1,   3 },
	// Mega CD / Sega CD: 1x only, slow sled, spins down when left idle.
	// base 160 ms and stroke 1500 ms are Genesis Plus GX's documented figures
	// (2 + 10 interrupts base; "max. seek time = 1.5 s" across 270000 sectors),
	// which the MiSTer core's own latency model matches. Measured, not guessed.
	{ "MegaCD",    1.0, 1.0,  24,  480,   55,   160,  1500,  2200, 6000,  4, 1,   6 },
	// Saturn: 2x. DERIVED, not sourced -- unlike the Mega CD and PSX rows there
	// is no citable emulator seek model for this drive. MAME's saturn_cdb.cpp
	// has no timing at all (its CD Block CPU is disabled), and Mednafen's CD
	// block does not publish a seek curve. So this is the PSX row, which is the
	// same class of 2x mechanism from the same period, with a slightly slower
	// and better damped sled. Treat it as the least trustworthy row here after
	// the auto one, and say so rather than implying it was measured.
	{ "Saturn",    2.0, 1.0,  32,  360,   30,   110,  1000,  1200, 0,     8, 1,   3 },
	// PC Engine CD: 1x, seek curve measured by Dave Shadoff.
	{ "PCECD",     1.0, 1.0,  24,  644,   50,   283,  2300,  2000, 7000,  4, 1,   5 },
	// 3DO: 2x on the FZ-10, slow to settle.
	{ "3DO",       2.0, 1.0,  32,  640,   40,   160,  1000,  2000, 0,     8, 1,   4 },
	// CD-i: 1x, a deliberately quiet consumer deck.
	{ "CDi",       1.0, 1.0,  24,  480,   60,   240,  1500,  2400, 9000,  4, 0,   5 },
	// Neo Geo CD: 1x top loader, famously slow. Same CDD family as the Mega CD,
	// so the same base and stroke, with a slower mechanism around it.
	{ "NeoGeoCD",  1.0, 1.0,  24,  480,   70,   200,  1700,  2500, 8000,  4, 1,   6 },
};

const acu_drive_t *acu_model_drive(pd_acoustic_profile_t profile)
{
	if (profile < 0 || profile >= PD_ACU_PROFILE_COUNT) profile = PD_ACU_PROFILE_AUTO;
	return &drives[profile];
}

const char *acu_gesture_name(gesture_kind_t k)
{
	switch (k) {
	case GEST_SLEW:      return "SLEW";
	case GEST_STEP:      return "STEP";
	case GEST_JUMP:      return "JUMP";
	case GEST_STREAM:    return "STREAM";
	case GEST_LOCK:      return "LOCK";
	case GEST_HOLD:      return "HOLD";
	case GEST_SPINUP:    return "SPINUP";
	case GEST_SPINDOWN:  return "SPINDOWN";
	case GEST_SWEEP:     return "SWEEP";
	case GEST_PARK:      return "PARK";
	default:             return "NONE";
	}
}

// Seek duration.
//
// The shape of this is taken from Genesis Plus GX's cdd.c, which is the
// reference implementation for Mega CD CD emulation and which the MiSTer core's
// own latency model agrees with:
//
//     cdd.latency  = 2 + 10*cd_latency;                         // base
//     cdd.latency += ((delta_lba) * 120 * cd_latency) / 270000;  // distance
//     // "max. seek time = 1.5 s = 1.5 x 75 = 112.5 CDD interrupts
//     //  (rounded to 120) for 270000 sectors max on disc"
//
// So: a fixed base plus a term proportional to the LBA distance, reaching the
// full-stroke figure across the whole disc. There is no short-seek plateau; an
// earlier shape invented here had one, and it combined with a full-stroke figure
// of 800 ms to make every Mega CD seek about half as long as the hardware takes.
// That is why a track change did not sound like one: the single most audible
// event the drive produces was being played at double speed.
//
// Distance is measured in sectors here, not spiral turns, because that is what
// both the reference emulator and the core use and therefore what the games'
// own timing was built against. Turns are still the right measure for deciding
// WHETHER the sled moves at all, which is a question about the mechanism.
#define CD_SECTORS_MAX 270000.0

double acu_model_seek_ms(const acu_drive_t *d, int from_lba, int to_lba)
{
	double turns = cd_geom_track_delta(from_lba, to_lba);

	// Rotational latency: half a revolution at the destination on average,
	// and a revolution takes longer out at the rim where there are more
	// sectors in it.
	double rot_ms = 1000.0 / 75.0 * cd_geom_sectors_per_rev(to_lba)
	                / d->data_speed * 0.5;

	if (turns <= d->lens_jump_turns) return rot_ms;

	double delta = (double)(to_lba > from_lba ? to_lba - from_lba : from_lba - to_lba);
	if (delta > CD_SECTORS_MAX) delta = CD_SECTORS_MAX;

	return d->short_seek_ms                                     // base cost
	     + d->full_stroke_ms * (delta / CD_SECTORS_MAX)         // travel
	     + rot_ms;
}

// "Where the head is now" -- only the newest of these carries any information,
// so they are the ones to throw away under pressure. Everything else is an
// event with its own sound and duration.
static int gesture_is_update(gesture_kind_t k)
{
	return k == GEST_STREAM || k == GEST_JUMP || k == GEST_HOLD;
}

static void emit(acu_model_t *m, const gesture_t *g)
{
	if (((m->out_tail + 1) & ACU_OUT_MASK) == m->out_head) {
		// Full. Drop the oldest DISCARDABLE gesture, not simply the oldest.
		//
		// Dropping the oldest outright cost the audio lock-on: while the player
		// spent two and a half seconds on a cross-disc traverse, the model
		// emitted the LOCK and then a run of stream updates behind it, and the
		// LOCK -- being oldest -- was the first thing thrown out. It never
		// reached the player at all, which is why it could not be heard and
		// could not be found in the player either.
		//
		// Third time this class of mistake has bitten: a burst of moves, then
		// the lock-on in the player, now the lock-on here. Anything that is not
		// positional is kept.
		int found = -1;
		for (int i = m->out_head; i != m->out_tail; i = (i + 1) & ACU_OUT_MASK) {
			if (gesture_is_update(m->out[i].kind)) { found = i; break; }
		}

		if (found < 0) {
			// Every queued gesture is a real event; nothing better to do than
			// lose the oldest.
			m->out_head = (m->out_head + 1) & ACU_OUT_MASK;
			m->dropped_events++;
		}
		else {
			int j = found, nxt = (found + 1) & ACU_OUT_MASK;
			while (nxt != m->out_tail) {
				m->out[j] = m->out[nxt];
				j   = nxt;
				nxt = (nxt + 1) & ACU_OUT_MASK;
			}
			m->out_tail = j;
		}
	}

	m->out[m->out_tail] = *g;
	m->out_tail = (m->out_tail + 1) & ACU_OUT_MASK;
}

int acu_model_poll(acu_model_t *m, gesture_t *out)
{
	if (m->out_head == m->out_tail) return 0;
	*out = m->out[m->out_head];
	m->out_head = (m->out_head + 1) & ACU_OUT_MASK;
	return 1;
}

void acu_model_init(acu_model_t *m, pd_acoustic_profile_t profile)
{
	memset(m, 0, sizeof(*m));
	m->profile     = profile;
	m->drive       = *acu_model_drive(profile);
	m->head_lba    = 0;
	m->stream_mult = m->drive.data_speed;
}

static void emit_move(acu_model_t *m, int from, int to)
{
	const acu_drive_t *d = &m->drive;
	gesture_t g;
	memset(&g, 0, sizeof(g));

	g.from_lba  = from;
	g.lba       = to;
	g.turns     = cd_geom_track_delta(from, to);
	g.radial_mm = cd_geom_radial_delta_mm(from, to);
	g.dur_ms    = acu_model_seek_ms(d, from, to);
	g.rpm       = cd_geom_rpm(to, m->stream_mult);

	if (g.turns <= d->lens_jump_turns) {
		// The objective lens covers this on its own: the sled motor never
		// runs, so there is no new noise. This is the commonest move during a
		// read, and treating it as a seek is exactly what makes a naive
		// mirror sound like a hard disk instead of a console.
		g.kind   = GEST_JUMP;
		g.stages = 0;
	}
	else if (g.turns <= d->short_seek_turns) {
		g.kind   = GEST_STEP;
		g.stages = 1;
	}
	else {
		// A long move is coarse-then-fine: the sled runs out to roughly the
		// right radius, the servo reads where it actually landed, and a short
		// correction follows. That two-part "chrrk-tk" is the signature sound
		// of a console loading.
		g.kind   = GEST_SLEW;
		g.stages = (g.turns > d->short_seek_turns * 8) ? 3 : 2;
	}

	emit(m, &g);
	m->head_lba   = to;
	m->holding    = 0;
	m->just_moved = 1;
}

static void emit_stream(acu_model_t *m, double now_ms, int lba, double mult, double rate, int audio)
{
	gesture_t g;
	memset(&g, 0, sizeof(g));
	g.kind           = GEST_STREAM;
	g.from_lba       = lba;
	g.lba            = lba;
	g.rpm            = cd_geom_rpm(lba, mult);
	g.rate_sectors_s = rate;
	g.audio          = audio;
	g.sectors        = (int)(rate * 0.25);   // re-evaluated four times a second
	if (g.sectors < 1) g.sectors = 1;
	g.dur_ms         = g.sectors * 1000.0 / (rate > 1.0 ? rate : 1.0);
	emit(m, &g);
	m->last_emit_ms = now_ms;
	m->holding      = 0;
}

static void simple(acu_model_t *m, gesture_kind_t k, int lba, double dur, double mult)
{
	gesture_t g;
	memset(&g, 0, sizeof(g));
	g.kind     = k;
	g.from_lba = m->head_lba;
	g.lba      = lba;
	g.dur_ms   = dur;
	g.rpm      = cd_geom_rpm(lba, mult);
	emit(m, &g);
}

static void ensure_spinning(acu_model_t *m)
{
	if (m->spinning) return;
	m->spinning = 1;
	m->holding  = 0;
	simple(m, GEST_SPINUP, m->head_lba, m->drive.spinup_ms, m->drive.data_speed);
}

void acu_model_event(acu_model_t *m, double now_ms, pd_acoustic_event_t ev, int lba, int count)
{
	const acu_drive_t *d = &m->drive;
	if (lba < 0) lba = 0;
	m->last_event_ms = now_ms;

	switch (ev) {

	case PD_ACU_SPINUP:
	case PD_ACU_TRAY_CLOSE:
		if (!m->spinning) {
			m->spinning = 1;
			simple(m, GEST_SPINUP, m->head_lba, d->spinup_ms, d->data_speed);
			// These decks drag the sled across the disc once to calibrate the
			// focus and tracking servos before they can read the TOC. It is
			// the loudest thing the drive ever does, and it is the sound
			// people recognise as "the console is booting".
			if (d->calib_sweep) {
				gesture_t g;
				memset(&g, 0, sizeof(g));
				g.kind      = GEST_SWEEP;
				g.from_lba  = 0;
				g.lba       = 0;
				g.dur_ms    = d->full_stroke_ms * 1.5;
				g.stages    = 2;
				g.radial_mm = CD_R_OUTER_MM - CD_R_INNER_MM;
				g.rpm       = cd_geom_rpm(0, d->data_speed);
				emit(m, &g);
				m->head_lba = 0;
			}
		}
		break;

	case PD_ACU_TRAY_OPEN:
		m->holding   = 0;
		m->spinning  = 0;
		m->streaming = 0;
		simple(m, GEST_PARK, 0, d->full_stroke_ms, 0.0);
		m->head_lba = 0;
		break;

	case PD_ACU_TOC:
		// The TOC lives in the lead-in just inside the program area, so the
		// drive always returns to the hub to read it.
		ensure_spinning(m);
		if (m->head_lba != 0) emit_move(m, m->head_lba, 0);
		simple(m, GEST_HOLD, 0, 300.0, d->data_speed);
		m->streaming = 0;
		break;

	case PD_ACU_STOP:
		if (m->spinning) {
			m->spinning  = 0;
			m->streaming = 0;
			m->holding   = 0;
			simple(m, GEST_SPINDOWN, m->head_lba, d->spinup_ms, 0.0);   // coasts down about as long as it ramps up
		}
		break;

	case PD_ACU_PAUSE:
		m->streaming = 0;
		if (!m->holding) {
			m->holding = 1;
			simple(m, GEST_HOLD, m->head_lba, 0.0, m->stream_mult);
		}
		break;

	case PD_ACU_SEEK:
	case PD_ACU_SCAN:
		// SCAN is fast-forward/rewind: the servo hops a block at a time, a
		// rapid regular ticking quite unlike either a seek or a read. It
		// arrives as a stream of these, one per hop.
		ensure_spinning(m);
		m->streaming = 0;
		emit_move(m, m->head_lba, lba);
		break;

	case PD_ACU_READ:
	case PD_ACU_PLAY: {
		ensure_spinning(m);
		if (count < 1) count = 1;

		double mult = (ev == PD_ACU_PLAY) ? d->audio_speed : d->data_speed;
		m->stream_mult = mult;

		int gap = lba - m->head_lba;

		// A few sectors either side of where the head already is costs the
		// mechanism nothing: forwards it is inside the drive's own read-ahead,
		// backwards the servo just waits for the sector to come round again.
		// Neither moves the sled, so neither is a seek. Being tolerant here
		// also keeps a core that re-reads a sector from producing a phantom
		// reposition.
		int contiguous = (gap >= -d->readahead_sectors && gap <= d->readahead_sectors);

		if (!contiguous) {
			// The core jumped somewhere else without issuing an explicit
			// seek. This is where most of a game's disc noise actually comes
			// from: a filesystem walk is dozens of these, and the old code
			// could not see any of them.
			emit_move(m, m->head_lba, lba);
			m->streaming         = 0;
			m->stream_anchor_lba = lba;
			m->stream_anchor_ms  = now_ms;
		}

		// Arriving at an audio track is not the same as playing it. The drive
		// settles, acquires the subcode and spin-locks first, and the disc turns
		// several times before any sound comes out -- six revolutions on a Mega
		// CD, well over a second out near the rim.
		//
		// Keyed on having just repositioned, NOT on the read being
		// discontiguous. A Mega CD issues an explicit seek before it plays, so
		// by the time the first audio sector arrives the model has already moved
		// the head and the read looks perfectly contiguous. Checking
		// contiguity alone meant the lock-on never fired on the one console
		// whose lock-on is most obvious, which is why a track change was
		// missing seconds of it.
		if (ev == PD_ACU_PLAY && m->just_moved && d->lock_revs > 0) {
			double rev_ms = cd_geom_sectors_per_rev(lba) / 75.0 * 1000.0 / mult;
			simple(m, GEST_LOCK, lba, d->lock_revs * rev_ms, mult);
		}
		m->just_moved = 0;

		// Never let a re-read drag the modelled head backwards.
		if (lba + count > m->head_lba || !contiguous) m->head_lba = lba + count;

		// Measure how fast the core is really consuming sectors, so that a 1x
		// CDDA track and a 2x data read do not sound the same.
		if (!m->streaming) {
			m->streaming         = 1;
			m->stream_anchor_lba = lba;
			m->stream_anchor_ms  = now_ms;
			m->stream_rate       = 75.0 * mult;
			emit_stream(m, now_ms, lba, mult, m->stream_rate, ev == PD_ACU_PLAY);
		}
		else {
			double dt = now_ms - m->stream_anchor_ms;
			if (dt >= 200.0) {
				double inst = (m->head_lba - m->stream_anchor_lba) * 1000.0 / dt;
				if (inst < 0.0) inst = 0.0;
				// Cap at the mechanism's real ceiling. The HPS can serve
				// sectors far faster than any of these drives could, and
				// following that would just sound like a modern optical
				// drive.
				double ceiling = 75.0 * mult * 1.25;
				if (inst > ceiling) inst = ceiling;
				m->stream_rate       = m->stream_rate * 0.6 + inst * 0.4;
				m->stream_anchor_lba = m->head_lba;
				m->stream_anchor_ms  = now_ms;
			}
			if (now_ms - m->last_emit_ms >= 150.0)
				emit_stream(m, now_ms, m->head_lba, mult, m->stream_rate, ev == PD_ACU_PLAY);
		}
		break;
	}

	default:
		break;
	}
}

void acu_model_tick(acu_model_t *m, double now_ms)
{
	const acu_drive_t *d = &m->drive;
	double idle = now_ms - m->last_event_ms;

	if (m->streaming && idle > 250.0) {
		// Sectors stopped arriving: the drive has caught up with the game and
		// is sitting on track rather than reading.
		m->streaming = 0;
		if (!m->holding) {
			m->holding = 1;
			simple(m, GEST_HOLD, m->head_lba, 0.0, m->stream_mult);
		}
	}

	if (m->spinning && d->spindown_idle_ms > 0 && idle > d->spindown_idle_ms) {
		m->spinning = 0;
		simple(m, GEST_SPINDOWN, m->head_lba, d->spinup_ms, 0.0);   // coasts down about as long as it ramps up
	}
}
