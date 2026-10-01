#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <climits>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/cdrom.h>
#include <scsi/sg.h>

#include "../../cfg.h"
#include "physical_disc.h"
#include "physical_disc_acoustic.h"
#include "acoustic_model.h"
#include "cd_geometry.h"

// Replays the gestures acoustic_model produces on a spare disc in the USB
// drive. The model says what the original mechanism would be doing; this file
// is only concerned with making a real drive do the same physical thing at the
// same time.

#define BURST_MAX                32
#define END_GUARD_SECTORS        32
#define READ_FAIL_LIMIT          6
#define REOPEN_FAIL_LIMIT        8
#define SEEK_FAIL_LIMIT          4
#define CMD_TIMEOUT_MS           4000
#define EVENT_RING               256

// PHYSICAL_DISC_ACOUSTIC=2 traces every gesture. Invaluable for tuning by ear:
// you can see what the model thought the drive should be doing at the moment
// you heard the wrong thing.
//
// The trace goes to its own file rather than stdout. MiSTer's console output
// is not reliably reachable -- it is line-buffered to whatever the launcher
// happened to hand it, and the physical-disc launcher forks children that
// reopen the standard streams -- so a trace printed to stdout can simply
// vanish. A file we open and flush ourselves always arrives.
#define ACU_TRACE_PATH "/tmp/acoustic.log"
#define verbose() (cfg.physical_disc_acoustic >= 2)

static FILE *acu_trace;

static void acu_log(const char *fmt, ...)
{
	if (!verbose()) return;
	if (!acu_trace) {
		acu_trace = fopen(ACU_TRACE_PATH, "w");
		if (!acu_trace) return;
		setvbuf(acu_trace, NULL, _IOLBF, 0);
	}

	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	fprintf(acu_trace, "[%8.3f] ", ts.tv_sec + ts.tv_nsec / 1e9);

	va_list ap;
	va_start(ap, fmt);
	vfprintf(acu_trace, fmt, ap);
	va_end(ap);
	fflush(acu_trace);
}

// Program area of the media in the drive. Both CDs and DVDs run from roughly
// 24 mm to 58 mm, so only the sector capacity differs.
#define MEDIA_R_IN               24.0
#define MEDIA_R_OUT              58.0
#define MEDIA_FULL_CD            333000.0
#define MEDIA_FULL_DVD           2298496.0
#define DVD_SPAN_THRESHOLD       400000

typedef struct {
	pd_acoustic_event_t ev;
	int lba;
	int count;
	double at_ms;
} acu_event_t;

typedef struct {
	volatile int on;
	volatile int held;
	volatile int alive;
	volatile int disabled_perm;
	volatile int no_read;
	volatile int raw_read;   // drive accepts READ CD (0xBE), so audio works too
	volatile int profile_req;

	// Set while physical_disc owns the drive. The mirror must be completely
	// silent then -- see physical_disc_acoustic_set_physical().
	volatile int phys_session;
	volatile int released;      // worker acknowledges it has let the device go

	// Single producer (whichever core thread is running) / single consumer
	// (the worker). Indices are free-running; only the difference matters.
	acu_event_t ring[EVENT_RING];
	volatile unsigned ring_head;
	volatile unsigned ring_tail;
	volatile unsigned dropped;

	int dev_fd;
	int disc_span;
	int span_lo, span_hi;
	double media_full;
	double r_lo, r_hi;

	pthread_t worker;
} mirror_state_t;

static mirror_state_t mir;
static acu_model_t model;
static uint8_t burst_buf[BURST_MAX * 2352];   // raw sectors: READ CD returns 2352

static double clock_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void sleep_ms(double ms)
{
	if (ms <= 0.0) return;
	if (ms > 500.0) ms = 500.0;   // stay responsive to on/held/alive
	struct timespec ts;
	ts.tv_sec  = (time_t)(ms / 1000.0);
	ts.tv_nsec = (long)((ms - ts.tv_sec * 1000.0) * 1e6);
	nanosleep(&ts, NULL);
}

// ---------------------------------------------------------------- SCSI ----

static int mirror_seek(int fd, int lba)
{
	uint8_t cdb[10] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	cdb[0] = 0x2B;                      // SEEK(10)
	cdb[2] = (lba >> 24) & 0xFF;
	cdb[3] = (lba >> 16) & 0xFF;
	cdb[4] = (lba >> 8) & 0xFF;
	cdb[5] = lba & 0xFF;

	memset(&io, 0, sizeof(io));
	io.interface_id    = 'S';
	io.cmd_len         = 10;
	io.cmdp            = cdb;
	io.dxfer_direction = SG_DXFER_NONE;
	io.sbp             = sense;
	io.mx_sb_len       = sizeof(sense);
	io.timeout         = CMD_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

// READ CD, raw 2352-byte sectors, any sector type.
//
// The scrap disc people leave in the drive is very often an audio or
// mixed-mode CD, and READ(10) cannot touch a CD-DA sector at all -- it only
// understands 2048-byte data blocks. Falling back to seek-only for that is a
// big loss, because a drive that is reading sounds quite different from one
// that is merely stepping. READ CD reads both kinds, so the whole disc stays
// usable as mirror surface and the full radial stroke stays available.
static int mirror_read_raw(int fd, int lba, int blocks)
{
	uint8_t cdb[12] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	if (blocks < 1) blocks = 1;
	if (blocks > BURST_MAX) blocks = BURST_MAX;

	cdb[0] = 0xBE;                      // READ CD
	cdb[1] = 0x00;                      // any sector type
	cdb[2] = (lba >> 24) & 0xFF;
	cdb[3] = (lba >> 16) & 0xFF;
	cdb[4] = (lba >> 8) & 0xFF;
	cdb[5] = lba & 0xFF;
	cdb[6] = (blocks >> 16) & 0xFF;
	cdb[7] = (blocks >> 8) & 0xFF;
	cdb[8] = blocks & 0xFF;
	cdb[9] = 0xF8;                      // sync + headers + user data + EDC

	memset(&io, 0, sizeof(io));
	io.interface_id    = 'S';
	io.cmd_len         = 12;
	io.cmdp            = cdb;
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.dxfer_len       = blocks * 2352;
	io.dxferp          = burst_buf;
	io.sbp             = sense;
	io.mx_sb_len       = sizeof(sense);
	io.timeout         = CMD_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

static int mirror_read(int fd, int lba, int blocks)
{
	uint8_t cdb[10] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	if (blocks < 1) blocks = 1;
	if (blocks > BURST_MAX) blocks = BURST_MAX;

	cdb[0] = 0x28;                      // READ(10)
	cdb[2] = (lba >> 24) & 0xFF;
	cdb[3] = (lba >> 16) & 0xFF;
	cdb[4] = (lba >> 8) & 0xFF;
	cdb[5] = lba & 0xFF;
	cdb[7] = (blocks >> 8) & 0xFF;
	cdb[8] = blocks & 0xFF;

	memset(&io, 0, sizeof(io));
	io.interface_id    = 'S';
	io.cmd_len         = 10;
	io.cmdp            = cdb;
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.dxfer_len       = blocks * 2048;
	io.dxferp          = burst_buf;
	io.sbp             = sense;
	io.mx_sb_len       = sizeof(sense);
	io.timeout         = CMD_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

static void mirror_spin(int fd, int start)
{
	uint8_t cdb[6] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	cdb[0] = 0x1B;                      // START STOP UNIT
	cdb[1] = 0x01;                      // immediate
	cdb[4] = start ? 0x01 : 0x00;

	memset(&io, 0, sizeof(io));
	io.interface_id    = 'S';
	io.cmd_len         = 6;
	io.cmdp            = cdb;
	io.dxfer_direction = SG_DXFER_NONE;
	io.sbp             = sense;
	io.mx_sb_len       = sizeof(sense);
	io.timeout         = CMD_TIMEOUT_MS;

	ioctl(fd, SG_IO, &io);
}

// ------------------------------------------------------------ geometry ----

// Radius of a given LBA on whatever disc is in the mirror drive. Unlike the
// game side this has to cope with DVDs, so it works from the media's full
// capacity rather than assuming Red Book.
static double media_radius_mm(int lba)
{
	double f = lba / mir.media_full;
	if (f < 0.0) f = 0.0;
	if (f > 1.0) f = 1.0;
	return sqrt(MEDIA_R_IN * MEDIA_R_IN
	            + f * (MEDIA_R_OUT * MEDIA_R_OUT - MEDIA_R_IN * MEDIA_R_IN));
}

static int media_lba_at_radius(double r)
{
	if (r < MEDIA_R_IN)  r = MEDIA_R_IN;
	if (r > MEDIA_R_OUT) r = MEDIA_R_OUT;
	double f = (r * r - MEDIA_R_IN * MEDIA_R_IN)
	           / (MEDIA_R_OUT * MEDIA_R_OUT - MEDIA_R_IN * MEDIA_R_IN);
	return (int)(f * mir.media_full + 0.5);
}

// Map a game LBA onto the mirror disc so that the two heads sit at the same
// FRACTION OF THE RADIAL STROKE. Mapping by sector number instead -- which is
// what the previous implementation did -- gets both the sled travel and the
// CLV spindle pitch wrong, because sector number goes as the square of radius.
static int map_to_mirror(int game_lba)
{
	double r = cd_geom_radius_mm(game_lba);
	double u = (r - CD_R_INNER_MM) / (CD_R_OUTER_MM - CD_R_INNER_MM);
	if (u < 0.0) u = 0.0;
	if (u > 1.0) u = 1.0;

	int lba = media_lba_at_radius(mir.r_lo + u * (mir.r_hi - mir.r_lo));
	if (lba < mir.span_lo) lba = mir.span_lo;
	if (lba > mir.span_hi) lba = mir.span_hi;
	return lba;
}

// --------------------------------------------------------- the device -----

static int mirror_acquire(void)
{
	if (mir.phys_session || physical_disc_drive_busy()) return -1;

	for (int i = 0; i < 8; i++) {
		char path[32];
		snprintf(path, sizeof(path), "/dev/sr%d", i);
		int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0) continue;

		if (ioctl(fd, CDROM_DRIVE_STATUS, CDSL_CURRENT) != CDS_DISC_OK) { close(fd); continue; }

		struct cdrom_tochdr hdr;
		struct cdrom_tocentry e;
		if (ioctl(fd, CDROMREADTOCHDR, &hdr) < 0) { close(fd); continue; }
		memset(&e, 0, sizeof(e));
		e.cdte_track  = CDROM_LEADOUT;
		e.cdte_format = CDROM_LBA;
		if (ioctl(fd, CDROMREADTOCENTRY, &e) < 0) { close(fd); continue; }

		mir.disc_span = e.cdte_addr.lba;
		mir.span_lo   = 0;
		mir.span_hi   = mir.disc_span - BURST_MAX - END_GUARD_SECTORS;
		if (mir.span_hi < mir.span_lo) mir.span_hi = mir.span_lo;

		mir.media_full = (mir.disc_span > DVD_SPAN_THRESHOLD) ? MEDIA_FULL_DVD : MEDIA_FULL_CD;
		mir.r_lo = media_radius_mm(mir.span_lo);
		mir.r_hi = media_radius_mm(mir.span_hi);

		mir.no_read  = 0;
		mir.raw_read = 1;   // try READ CD first; touch() downgrades if refused
		mir.dev_fd   = fd;

		// Lock the drive to the speed the console's own mechanism ran at. A
		// Mega CD is 1x; letting a modern drive sit at 4x or faster gives a
		// high steady whine that sounds nothing like the real thing, and it
		// finishes every gesture long before the gesture is supposed to end.
		// One change, at open: doing it mid-play is slow and often ignored.
		int want_x = (int)(model.drive.data_speed + 0.5);
		if (want_x < 1) want_x = 1;
		ioctl(fd, CDROM_SELECT_SPEED, want_x);

		acu_log("acquired %s: %d sectors, stroke %.1f-%.1f mm, %s media, "
		        "locked to %dx, profile %s\n",
		        path, mir.disc_span, mir.r_lo, mir.r_hi,
		        (mir.disc_span > DVD_SPAN_THRESHOLD) ? "DVD" : "CD",
		        want_x, model.drive.name);

		static int last_logged_span = -1;
		if (mir.disc_span != last_logged_span) {
			printf("physical_disc_acoustic: mirror disc on %s, %d sectors, "
			       "stroke %.1f-%.1f mm, locked to %dx\n",
			       path, mir.disc_span, mir.r_lo, mir.r_hi, want_x);
			last_logged_span = mir.disc_span;
		}
		return 0;
	}
	return -1;
}

static void mirror_release(void)
{
	if (mir.dev_fd >= 0) { close(mir.dev_fd); mir.dev_fd = -1; }
	mir.disc_span = 0;
}

// Returns 0 on success. Handles the fault bookkeeping that decides whether to
// fall back to seek-only or give up for the session.
static int touch(int lba, int blocks)
{
	static int read_faults = 0, seek_faults = 0, reopen_faults = 0;

	// Re-checked before every single command, not just once per gesture. A
	// physical disc session can claim the drive at any point, and one stray
	// SCSI command issued to the disc the user is actually playing is enough
	// to stall a load.
	if (mir.phys_session || physical_disc_drive_busy()) {
		mirror_release();
		return -1;
	}

	if (mir.dev_fd < 0) return -1;
	if (lba < mir.span_lo) lba = mir.span_lo;
	if (lba > mir.span_hi) lba = mir.span_hi;

	// Prefer raw reads: they work on data and audio alike, so the whole disc
	// is usable surface. Drop to READ(10) if the drive has no READ CD, and to
	// seek-only if it will not read at all.
	int r;
	if (mir.no_read)        r = mirror_seek(mir.dev_fd, lba);
	else if (mir.raw_read)  r = mirror_read_raw(mir.dev_fd, lba, blocks);
	else                    r = mirror_read(mir.dev_fd, lba, blocks);

	if (r == 0) { read_faults = seek_faults = reopen_faults = 0; return 0; }

	if (r == -1) {
		// The device went away underneath us.
		mirror_release();
		read_faults = seek_faults = 0;
		if (++reopen_faults >= REOPEN_FAIL_LIMIT) {
			printf("physical_disc_acoustic: drive keeps dropping, disabling for this session\n");
			mir.disabled_perm = 1;
		}
		return -1;
	}

	if (mir.no_read) {
		if (++seek_faults >= SEEK_FAIL_LIMIT) {
			printf("physical_disc_acoustic: drive does not accept SEEK, disabling for this session\n");
			mir.disabled_perm = 1;
			mirror_release();
		}
	}
	else if (mir.raw_read && ++read_faults >= READ_FAIL_LIMIT) {
		// READ CD is refused: try plain READ(10) before giving up on reading.
		acu_log("READ CD rejected at lba %d, falling back to READ(10)\n", lba);
		mir.raw_read = 0;
		read_faults  = 0;
	}
	else if (++read_faults >= READ_FAIL_LIMIT) {
		printf("physical_disc_acoustic: READ(10) rejected at lba %d, seek-only fallback "
		       "(an audio CD in the drive will do this -- CD-DA sectors are not "
		       "readable as 2048-byte blocks)\n", lba);
		mir.no_read = 1;
		read_faults = 0;
	}
	else {
		acu_log("read fail r=%d at lba %d (%d/%d)\n", r, lba, read_faults, READ_FAIL_LIMIT);
	}
	return -1;
}

// ------------------------------------------------------- gesture player ---

// True only when the mirror genuinely owns the device. START STOP UNIT and a
// speed change are the two commands that can wreck a real disc session
// outright -- spinning down the disc the user is playing looks exactly like a
// hung core -- so they are never issued without checking this first.
static int own_device(void)
{
	return mir.dev_fd >= 0 && !mir.phys_session && !physical_disc_drive_busy();
}

// A gesture carries how long the original mechanism would have taken. The USB
// drive takes whatever it takes; we issue the ops that make it move the right
// distance and then hold the remainder of the slot so the rhythm is right.
static void play_gesture(const gesture_t *g)
{
	double start = clock_ms();
	int target   = map_to_mirror(g->lba);
	int from     = map_to_mirror(g->from_lba);

	acu_log("%-8s game %7d->%-7d  mirror %7d->%-7d  %8.1f turns "
	        "%6.3fmm %5.0frpm %5.0fms x%d%s\n",
	        acu_gesture_name(g->kind), g->from_lba, g->lba, from, target,
	        g->turns, g->radial_mm, g->rpm, g->dur_ms, g->stages,
	        mir.no_read ? " [SEEK-ONLY]" : "");
	int ra       = model.drive.readahead_sectors;
	if (ra < 1) ra = 1;
	if (ra > BURST_MAX) ra = BURST_MAX;

	switch (g->kind) {

	case GEST_JUMP:
		// Lens jump on the original: the sled never moved, so do not make the
		// mirror's sled move either. Keep the drive loaded with a read where
		// it already is.
		touch(from, ra);
		break;

	case GEST_STEP:
		touch(target, ra);
		break;

	case GEST_SLEW: {
		// Coarse then fine. The intermediate points make the mirror's sled
		// perform the same multi-part move the original did, which is what
		// gives a console load its two-part "chrrk-tk" rather than one
		// featureless swish.
		static const double two[]   = { 0.85, 1.0 };
		static const double three[] = { 0.60, 0.92, 1.0 };
		const double *frac = (g->stages >= 3) ? three : two;
		int n = (g->stages >= 3) ? 3 : 2;

		double slot = g->dur_ms / n;
		for (int i = 0; i < n; i++) {
			if (!mir.on || mir.held || mir.phys_session || mir.dev_fd < 0) break;
			double rf = cd_geom_radius_mm(g->from_lba);
			double rt = cd_geom_radius_mm(g->lba);
			int stop_lba = cd_geom_lba_at_radius(rf + (rt - rf) * frac[i]);
			double t0 = clock_ms();
			touch(map_to_mirror(stop_lba), ra);
			sleep_ms(slot - (clock_ms() - t0));
		}
		break;
	}

	case GEST_STREAM: {
		// Walk the mirror head outward at the rate the core is really
		// consuming sectors, in radius-matched steps, so a 1x CDDA track
		// creeps and a 2x data read moves twice as fast.
		double rate = g->rate_sectors_s;
		if (rate < 1.0) rate = 1.0;
		int    done = 0;
		int    want = g->sectors;
		while (done < want && mir.on && !mir.held && !mir.phys_session && mir.dev_fd >= 0) {
			int chunk = ra;
			if (chunk > want - done) chunk = want - done;
			double t0 = clock_ms();
			touch(map_to_mirror(g->lba + done), chunk);
			done += chunk;
			sleep_ms(chunk * 1000.0 / rate - (clock_ms() - t0));
		}
		break;
	}

	case GEST_HOLD:
		// Spindle still turning, head parked on track. One small read keeps
		// the drive focused and spun up without moving the sled.
		touch(from, 1);
		sleep_ms(g->dur_ms);
		break;

	case GEST_SPINUP:
		if (own_device()) mirror_spin(mir.dev_fd, 1);
		touch(target, ra);
		sleep_ms(g->dur_ms - (clock_ms() - start));
		break;

	case GEST_SPINDOWN:
		if (own_device() && !mir.no_read) mirror_spin(mir.dev_fd, 0);
		break;

	case GEST_SWEEP: {
		// The boot calibration pass: hub, rim, back to the hub.
		int lo = mir.span_lo, hi = mir.span_hi;
		double slot = g->dur_ms / 3.0;
		int pts[3] = { lo, hi, lo };
		for (int i = 0; i < 3; i++) {
			if (!mir.on || mir.held || mir.phys_session || mir.dev_fd < 0) break;
			double t0 = clock_ms();
			touch(pts[i], ra);
			sleep_ms(slot - (clock_ms() - t0));
		}
		break;
	}

	case GEST_PARK:
		touch(mir.span_lo, ra);
		sleep_ms(g->dur_ms - (clock_ms() - start));
		if (own_device() && !mir.no_read) mirror_spin(mir.dev_fd, 0);
		break;

	default:
		break;
	}
}

// ------------------------------------------------------------- worker -----

static void *worker_main(void *arg)
{
	(void)arg;
	double open_attempt_at = 0;
	int applied_profile = -1;

	while (mir.alive) {

		if (!mir.on || mir.held || mir.disabled_perm || mir.phys_session) {
			mirror_release();
			// Throw away anything the cores queued while we were parked, so
			// we do not wake up and replay a minute of stale activity.
			mir.ring_head = mir.ring_tail;
			// Tell whoever is waiting that the device is theirs.
			mir.released = 1;
			sleep_ms(mir.phys_session ? 50 : 150);
			continue;
		}
		mir.released = 0;

		if (mir.profile_req != applied_profile) {
			applied_profile = mir.profile_req;
			acu_model_init(&model, (pd_acoustic_profile_t)applied_profile);
			printf("physical_disc_acoustic: imitating the %s drive\n", model.drive.name);
		}

		if (physical_disc_drive_busy()) {
			// The real disc is being read for data; that drive noise is
			// genuine and we must not fight it for the device.
			mirror_release();
			mir.ring_head = mir.ring_tail;
			sleep_ms(200);
			continue;
		}

		double now = clock_ms();

		// Drain whatever the cores reported into the model.
		int drained = 0;
		while (mir.ring_head != mir.ring_tail && drained < 64) {
			acu_event_t e = mir.ring[mir.ring_head & (EVENT_RING - 1)];
			mir.ring_head++;
			acu_model_event(&model, e.at_ms, e.ev, e.lba, e.count);
			drained++;
		}
		acu_model_tick(&model, now);

		gesture_t g;
		if (!acu_model_poll(&model, &g)) {
			sleep_ms(10);
			continue;
		}

		// Nothing to open the drive for unless there is real work.
		if (g.kind == GEST_NONE) continue;

		if (mir.dev_fd < 0) {
			if (now - open_attempt_at < 1000.0) { sleep_ms(100); continue; }
			open_attempt_at = now;
			if (mirror_acquire()) { sleep_ms(100); continue; }
		}

		play_gesture(&g);
	}

	mirror_release();
	return NULL;
}

// --------------------------------------------------------------- API ------

void physical_disc_acoustic_config(int enabled)
{
	// mir is static, so dev_fd starts at 0 -- a perfectly valid descriptor
	// (stdin). Nothing reaches it before the worker exists today, but close(0)
	// is not a mistake worth leaving one refactor away.
	static int inited = 0;
	if (!inited) {
		inited         = 1;
		mir.dev_fd     = -1;
		mir.media_full = MEDIA_FULL_CD;
	}

	acu_log("config(enabled=%d) cfg=%d alive=%d\n",
	        enabled, cfg.physical_disc_acoustic, mir.alive);

	mir.on = enabled ? 1 : 0;
	if (mir.on && !mir.alive) {
		mir.dev_fd      = -1;
		mir.held        = 0;
		mir.profile_req = PD_ACU_PROFILE_AUTO;
		mir.media_full  = MEDIA_FULL_CD;
		acu_model_init(&model, PD_ACU_PROFILE_AUTO);
		mir.alive = 1;
		if (pthread_create(&mir.worker, NULL, worker_main, NULL)) {
			mir.alive = 0;
			printf("physical_disc_acoustic: could not start thread\n");
			return;
		}
		printf("physical_disc_acoustic: enabled - put a spare disc in the drive\n");
	}
}

void physical_disc_acoustic_set_physical(int phys)
{
	if (!phys) {
		mir.phys_session = 0;
		return;
	}

	if (mir.phys_session) return;
	mir.phys_session = 1;

	// Drop anything the cores already queued: it describes the real disc the
	// drive is about to serve for itself, and replaying it later would be
	// nonsense.
	mir.ring_head = mir.ring_tail;

	if (!mir.alive || !mir.on) return;

	// Wait for the worker to confirm it has closed the device before the
	// caller opens it. The worker re-checks phys_session before every command,
	// so this is short; the bound is here only so a wedged SCSI command can
	// never hold up a disc mount.
	double start = clock_ms();
	while (!mir.released && clock_ms() - start < 1500.0) {
		struct timespec ts = { 0, 5 * 1000 * 1000 };
		nanosleep(&ts, NULL);
	}
	if (!mir.released)
		printf("physical_disc_acoustic: mirror did not release the drive in time\n");
}

void physical_disc_acoustic_set_profile(pd_acoustic_profile_t profile)
{
	// PHYSICAL_DISC_ACOUSTIC_PROFILE in MiSTer.ini pins the imitated drive;
	// left at 0 it follows whichever core has just mounted a disc.
	int forced = cfg.physical_disc_acoustic_profile;
	if (forced > PD_ACU_PROFILE_AUTO && forced < PD_ACU_PROFILE_COUNT) {
		mir.profile_req = forced;
		return;
	}

	if (profile <= PD_ACU_PROFILE_AUTO || profile >= PD_ACU_PROFILE_COUNT) return;
	mir.profile_req = (int)profile;
}

void physical_disc_acoustic_event(pd_acoustic_event_t ev, int lba, int count)
{
	// phys_session: the real drive is serving a real disc and is already
	// making the right noise by itself. Nothing to mirror, and nothing may
	// touch the device.
	if (!mir.on || mir.held || mir.disabled_perm || mir.phys_session) return;

	unsigned tail = mir.ring_tail;
	if (tail - mir.ring_head >= EVENT_RING - 1) {
		// The worker is behind. Dropping is correct: the model only needs to
		// know where the head ended up, and the newest event says that.
		mir.dropped++;
		return;
	}

	acu_event_t *e = &mir.ring[tail & (EVENT_RING - 1)];
	e->ev    = ev;
	e->lba   = lba;
	e->count = count;
	e->at_ms = clock_ms();

	// Publish only once the slot is fully written.
	__sync_synchronize();
	mir.ring_tail = tail + 1;
}

void physical_disc_acoustic_hint(int lba)
{
	physical_disc_acoustic_event(PD_ACU_READ, lba, 1);
}

void physical_disc_acoustic_pause(void)
{
	mir.held = 1;
}

void physical_disc_acoustic_resume(void)
{
	mir.held = 0;
}
