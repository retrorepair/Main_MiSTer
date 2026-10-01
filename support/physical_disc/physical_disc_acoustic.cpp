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
	volatile int play_mode;   // drive runs the mechanism via PLAY AUDIO at true 1x
	volatile int shrinks;     // reactive span pull-ins, reset on each acquire
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
static int mirror_read_raw(int fd, int lba, int blocks, int timeout_ms)
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
	io.timeout         = timeout_ms;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

static int mirror_read(int fd, int lba, int blocks, int timeout_ms)
{
	uint8_t cdb[10] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	if (blocks < 1) blocks = 1;
	if (blocks > BURST_MAX) blocks = BURST_MAX;

	cdb[0] = 0x28;                      // READ(10)
	// Force Unit Access: come off the media, not out of the drive's RAM cache.
	// Without this the mirror is near-silent. A modern drive has megabytes of
	// cache and reads ahead aggressively, so the small repeated reads a stream
	// gesture issues are nearly all cache hits and the mechanism never moves.
	// We are here for the mechanism, not the data.
	cdb[1] = 0x08;
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
	io.timeout         = timeout_ms;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

// ---- PLAY AUDIO: the only way to make a modern drive run at a console speed.
//
// Measured on the drive in this machine: SET CD SPEED is accepted and ignored,
// and a data read runs at 12-19x however it is asked. Audio playback has to be
// real time, so the drive has no choice -- measured 75.1 sectors/s, 1.002x,
// which is exactly what a Mega CD does. The spindle then also glides with
// radius the way CLV requires, the head advances itself with no commands at
// all, and nothing can be served from cache because it is decoding as it goes.
//
// Re-issuing PLAY at a new address is a real sled seek (288 ms measured for a
// 120000-sector jump), so one mechanism covers both streaming and seeking.
static int mirror_play(int fd, int lba, int blocks)
{
	uint8_t cdb[10] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	if (blocks < 1) blocks = 1;
	if (blocks > 0xFFFF) blocks = 0xFFFF;

	cdb[0] = 0x45;                      // PLAY AUDIO(10)
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
	io.dxfer_direction = SG_DXFER_NONE;
	io.sbp             = sense;
	io.mx_sb_len       = sizeof(sense);
	io.timeout         = CMD_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

static int mirror_pause(int fd, int resume)
{
	uint8_t cdb[10] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	cdb[0] = 0x4B;                      // PAUSE/RESUME
	cdb[8] = resume ? 0x01 : 0x00;

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

static int mirror_stop_play(int fd)
{
	uint8_t cdb[6] = { 0 };
	uint8_t sense[32];
	struct sg_io_hdr io;

	cdb[0] = 0x4E;                      // STOP PLAY/SCAN

	memset(&io, 0, sizeof(io));
	io.interface_id    = 'S';
	io.cmd_len         = 6;
	io.cmdp            = cdb;
	io.dxfer_direction = SG_DXFER_NONE;
	io.sbp             = sense;
	io.mx_sb_len       = sizeof(sense);
	io.timeout         = CMD_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;
	return 0;
}

// Where the head actually is, straight from the drive's sub-channel. Lets the
// mirror re-sync instead of assuming, and tells us whether it is still playing.
static int mirror_subq(int fd, int *lba_out, int *playing_out)
{
	uint8_t cdb[10] = { 0 };
	uint8_t sense[32];
	uint8_t data[16];
	struct sg_io_hdr io;

	cdb[0] = 0x42;                      // READ SUB-CHANNEL
	cdb[2] = 0x40;                      // SUBQ
	cdb[3] = 0x01;                      // current position
	cdb[8] = sizeof(data);

	memset(&io, 0, sizeof(io));
	io.interface_id    = 'S';
	io.cmd_len         = 10;
	io.cmdp            = cdb;
	io.dxfer_direction = SG_DXFER_FROM_DEV;
	io.dxfer_len       = sizeof(data);
	io.dxferp          = data;
	io.sbp             = sense;
	io.mx_sb_len       = sizeof(sense);
	io.timeout         = CMD_TIMEOUT_MS;

	if (ioctl(fd, SG_IO, &io) < 0) return -1;
	if (io.status || io.host_status || io.driver_status) return -2;

	if (playing_out) *playing_out = (data[1] == 0x11);
	if (lba_out)
		*lba_out = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
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

		// Is there an audio track to play? That decides everything about how
		// this disc gets used, because audio playback is the only way to make
		// the drive turn at a console's speed.
		int first_audio = -1;
		for (int t = hdr.cdth_trk0; t <= hdr.cdth_trk1; t++) {
			struct cdrom_tocentry te;
			memset(&te, 0, sizeof(te));
			te.cdte_track  = t;
			te.cdte_format = CDROM_LBA;
			if (ioctl(fd, CDROMREADTOCENTRY, &te) < 0) continue;
			if (!(te.cdte_ctrl & CDROM_DATA_TRACK)) { first_audio = te.cdte_addr.lba; break; }
		}

		mir.disc_span = e.cdte_addr.lba;
		mir.span_lo   = (first_audio > 0) ? first_audio : 0;
		mir.span_hi   = mir.disc_span - BURST_MAX - END_GUARD_SECTORS;
		if (mir.span_hi < mir.span_lo) mir.span_hi = mir.span_lo;

		mir.media_full = (mir.disc_span > DVD_SPAN_THRESHOLD) ? MEDIA_FULL_DVD : MEDIA_FULL_CD;
		mir.r_lo = media_radius_mm(mir.span_lo);
		mir.r_hi = media_radius_mm(mir.span_hi);

		mir.no_read   = 0;
		mir.shrinks   = 0;
		mir.play_mode = 0;
		mir.dev_fd   = fd;

		// Prefer PLAY AUDIO where the disc allows it: it is the only mode that
		// runs the mechanism at a console's speed instead of 12-19x.
		mir.play_mode = 0;
		if (first_audio >= 0) {
			int blocks = mir.span_hi - mir.span_lo;
			if (!mirror_play(fd, mir.span_lo, blocks > 0 ? blocks : 1)) {
				int pos = 0, playing = 0;
				if (!mirror_subq(fd, &pos, &playing) && playing) mir.play_mode = 1;
				mirror_stop_play(fd);
			}
		}

		if (!mir.play_mode) {
			// No audio to play: fall back to reads. FUA is what forces real
			// media access rather than a cache hit; READ CD also works on a
			// CD-DA sector, which READ(10) cannot touch at all.
			mir.raw_read = 0;
			if (mirror_read(fd, mir.span_lo, 2, CMD_TIMEOUT_MS)) {
				mir.raw_read = 1;
				if (mirror_read_raw(fd, mir.span_lo, 2, CMD_TIMEOUT_MS)) {
					mir.raw_read = 0;
					mir.no_read  = 1;
				}
			}
		}

		acu_log("drive mode: %s\n",
		        mir.play_mode ? "PLAY AUDIO (true 1x CLV)"
		        : mir.no_read ? "seek-only"
		        : mir.raw_read ? "READ CD raw 2352" : "READ(10)+FUA");

		// How far the disc is really readable is discovered during play, by
		// pulling the span in when a read fails (see touch()). Probing it up
		// front with a binary search was tried and measured at 18 seconds on a
		// part-written disc: an unreadable sector costs ~1.5 s because the
		// drive retries internally, and a short SG_IO timeout does not stop
		// it. A stall that long at every disc mount is far worse than
		// converging over the first few seconds of play.
		acu_log("stroke %.1f-%.1f mm from TOC; will pull in if reads fail\n",
		        mir.r_lo, mir.r_hi);

		// Lock the drive to the speed the console's own mechanism ran at. A
		// Mega CD is 1x; letting a modern drive sit at 4x or faster gives a
		// high steady whine that sounds nothing like the real thing, and it
		// finishes every gesture long before the gesture is supposed to end.
		// One change, at open: doing it mid-play is slow and often ignored.
		int want_x = (int)(model.drive.data_speed + 0.5);
		if (want_x < 1) want_x = 1;
		// We hold this device open for long stretches; the sr driver locks the
		// tray on open where the drive supports it. Leave the tray the user's.
		ioctl(fd, CDROM_LOCKDOOR, 0);
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
	else if (mir.raw_read)  r = mirror_read_raw(mir.dev_fd, lba, blocks, CMD_TIMEOUT_MS);
	else                    r = mirror_read(mir.dev_fd, lba, blocks, CMD_TIMEOUT_MS);

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

	// A read that fails out towards the rim usually means the disc simply is
	// not recorded that far, or its outer edge is unreadable -- not that the
	// drive cannot read. Pull the usable span in and carry on reading, rather
	// than condemning the whole session to seek-only and going quiet.
	// Before blaming the disc, check whether it is even the same disc. Swapping
	// the mirror disc leaves us holding a stale TOC, and every read then fails
	// for a reason that has nothing to do with how far the media is recorded.
	// Shrinking on that is actively wrong: it was measured collapsing a fresh
	// full disc's stroke to 1 mm because the span came from the previous one.
	if (r == -2 && ioctl(mir.dev_fd, CDROM_MEDIA_CHANGED, CDSL_CURRENT) > 0) {
		acu_log("mirror disc changed, re-reading its TOC\n");
		mirror_release();
		return -1;
	}

	if (!mir.no_read && r == -2 && lba > mir.span_lo + (mir.span_hi - mir.span_lo) / 8) {
		if (++mir.shrinks <= 10) {
			int keep = lba - (BURST_MAX + END_GUARD_SECTORS);
			if (keep < mir.span_lo) keep = mir.span_lo;
			mir.span_hi = keep;
			mir.r_hi    = media_radius_mm(mir.span_hi);
			acu_log("read failed at %d, shrinking usable span to %d (rim now %.1f mm)\n",
			        lba, mir.span_hi, mir.r_hi);
			return -1;
		}
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

// ------------------------------------------------------------- grime -------
//
// PHYSICAL_DISC_ACOUSTIC_GRIME, 0..10, 0 = a factory-fresh mechanism.
//
// The acoustic model deliberately describes a HEALTHY drive reading a CLEAN
// disc. Real consoles by now are neither: the sled is dry, the lens is hazy,
// the disc is scuffed, and the servo spends its life losing lock and
// recovering. That recovery is most of what an old console actually sounds
// like -- the stutter and hunt, not the smooth parts.
//
// This is the one place in here that invents activity the original would not
// have had on a good day, which is why it is opt-in and lives in the player
// rather than the model. Everything it adds is still real mechanism motion: a
// hunt is an actual sled move, not a sample.
static unsigned grime_rng(void)
{
	static unsigned s = 0x1234567u;
	s ^= s << 13; s ^= s >> 17; s ^= s << 5;
	return s;
}

// 0..11. Ten is a mechanism well past its best. Eleven is one louder.
static int grime_level(void)
{
	int g = cfg.physical_disc_acoustic_grime;
	if (g < 0)  g = 0;
	if (g > 11) g = 11;
	return g;
}

// Put the head at a radius, not at a sector. A hunt has to be specified in
// millimetres or it does not do anything: 1000 sectors is about 0.1 mm out at
// the rim, which the servo covers with the lens alone and you hear nothing.
// Moving the sled needs of the order of 6000 sectors near the hub and 12000
// near the rim, and only a radial figure gets that right at both ends.
static int lba_offset_mm(int lba, double delta_mm)
{
	int out = media_lba_at_radius(media_radius_mm(lba) + delta_mm);
	if (out < mir.span_lo) out = mir.span_lo;
	if (out > mir.span_hi) out = mir.span_hi;
	return out;
}

// Move the head and nothing else. SEEK(10) costs about 42 ms against PLAY's
// 200 ms, measured, because PLAY re-establishes the audio servo every time. For
// anything meant to sound quick -- a stutter, a hunt, a staircase step -- PLAY
// is simply too slow a primitive and turns it into a series of clunks.
static void grime_aim(int lba)
{
	if (mir.play_mode) mirror_seek(mir.dev_fd, lba);
	else               touch(lba, 2);
}

// SEEK leaves audio playback stopped, so put the spindle back to a true 1x.
static void grime_resume(int lba)
{
	if (!mir.play_mode || mir.dev_fd < 0 || mir.phys_session) return;
	int blk = mir.span_hi - lba;
	if (blk < 1) blk = 1;
	mirror_play(mir.dev_fd, lba, blk);
}

// A long, laboured traverse -- the thing an old console actually sounds like
// when it goes looking for something.
//
// One big PLAY to the destination is wrong for this: the drive services it with
// a single fast coordinated move, 1385 ms of smooth swoosh on the drive here,
// and then silence. A dry sled grinding across a disc is continuous, and to get
// that the sled motor must never be allowed to stop.
//
// So march the head across in steps that are each comfortably past the
// lens-jump range -- below about 0.04 mm the objective covers it and the sled
// never moves at all -- but small enough that the next step is issued before
// the previous one has settled. Back to back they run together into one
// continuous grind instead of a series of discrete clunks.
static void grime_grind(int from_lba, int to_lba, double total_ms)
{
	if (mir.dev_fd < 0 || mir.phys_session) return;

	double r0   = media_radius_mm(from_lba);
	double r1   = media_radius_mm(to_lba);
	double span = r1 - r0;
	double dist = span < 0 ? -span : span;
	if (dist < 0.30) return;               // too short to be worth grinding

	// Two competing constraints. Each step must clear the lens-jump range or
	// the sled does not move and the step is silent, which caps how finely a
	// given distance can be divided. But the steps must also come close enough
	// together that the motor never stops, or it is a row of separate clunks
	// rather than a grind -- a 1.6 mm seek stretched over 945 ms in six steps
	// is 157 ms of silence between each, which is what the first attempt did.
	//
	// So take the smaller of the two counts and let the traverse finish early
	// if the distance simply cannot fill the time. A short seek being short is
	// correct anyway.
	(void)total_ms;

	// A single long seek IS the grind, and trying to synthesise one out of
	// small steps was simply the wrong idea.
	//
	// Every SCSI positioning command is a move-and-settle: the firmware runs
	// the sled to the address and stops it. Issuing them back to back does not
	// produce continuous motion, it produces one settle after another -- at
	// SEEK's 42 ms that is a 24 Hz buzz, and at PLAY's 200 ms a row of clunks.
	// Two rounds of judder came from chopping up the one thing that actually
	// moves the sled smoothly: the drive's own coordinated traverse, measured
	// at 1385 ms unbroken for 20 mm on the drive here.
	//
	// So make the traverses FEWER and LONGER, not more and shorter. A worn
	// mechanism overshoots and has to come back, and each of those passes is a
	// full continuous grind in its own right.
	int passes = 1 + grime_level() / 4;             // 1..3 extra traverses
	int surge  = grime_level() >= 8 && !mir.no_read;

	for (int i = 0; i < passes; i++) {
		if (!mir.on || mir.held || mir.phys_session || mir.dev_fd < 0) break;

		// Overshoot past the target, alternating side and shrinking each time,
		// so it closes in rather than flailing.
		//
		// The floor matters more than the proportional part. Most in-game seeks
		// are short hops within the data track, and a short hop is 30 ms of
		// near-silence on this drive however faithfully it is reproduced. The
		// overshoot is what turns one into real audible travel, so it scales
		// with the grime level rather than only with the distance.
		double over = (dist * 0.35 + 1.5 + grime_level() * 0.9) / (i + 1);
		if (i & 1) over = -over;
		int at = lba_offset_mm(to_lba, span > 0 ? over : -over);

		mirror_seek(mir.dev_fd, at);

		// Spindle surge: a raw read spins the drive up hard, and dropping back
		// lets it fall. Failure is fine -- we want the spin-up, not the bytes.
		if (surge) mirror_read_raw(mir.dev_fd, at, 24, 700);

		sleep_ms(40 + (grime_rng() % 90));          // let it settle audibly
	}

	mirror_seek(mir.dev_fd, to_lba);                // finally arrives

	// SEEK stops audio playback outright (verified: PLAYING -> DONE), so hand
	// the spindle back to a true 1x before the mirror carries on.
	grime_resume(to_lba);
}

// A worn mechanism does not slip once and recover neatly. It slips, grabs,
// slips again, hunts past, comes back -- a burst of sled movement. `reach_mm`
// is how far it wanders and `reps` how many times before it gives up and
// settles, both scaled by the grime level.
static void grime_hunt(int lba, double reach_mm, int reps)
{
	int g = grime_level();
	if (!g || mir.dev_fd < 0 || mir.phys_session) return;

	// Chance of misbehaving at all: level 6 is most of the time, 10 is always.
	if ((int)(grime_rng() % 100) >= g * 10) return;

	reach_mm *= 0.4 + g / 5.0;
	reps     += g / 3;

	// Past ten there is nowhere left to go on frequency -- it already fires on
	// everything -- so eleven buys magnitude instead: it wanders nearly twice
	// as far and takes several more attempts to find its way back. Expect the
	// mirror to fall behind the game and resync, which is itself what a drive
	// this far gone does.
	if (g > 10) { reach_mm *= 1.8; reps += 3; }

	for (int i = 0; i < reps; i++) {
		if (!mir.on || mir.held || mir.phys_session || mir.dev_fd < 0) break;
		// Wander mostly inwards, sometimes past the target, never neatly.
		double away = reach_mm * (0.35 + (grime_rng() % 100) / 100.0);
		if (grime_rng() & 1) away = -away;
		grime_aim(lba_offset_mm(lba, away));
		sleep_ms(14 + (grime_rng() % 46));
	}
	grime_aim(lba);           // finally settles where it was supposed to be
	grime_resume(lba);
}

// A gesture carries how long the original mechanism would have taken. The USB
// drive takes whatever it takes; we issue the ops that make it move the right
// distance and then hold the remainder of the slot so the rhythm is right.
// Amplified head position, 0..1 of the mirror's stroke.
//
// Measured on real hardware: playing Sonic CD, 684 of 694 gestures moved the
// sled less than 0.05 mm. That is FAITHFUL -- a real Mega CD streaming CDDA
// also barely moves its sled -- but a 1991 deck at 1x is audibly working the
// whole time, while a 2026 slot-load drive doing the same tiny moves is
// silent. Radial fidelity is a means, not the end.
//
// GAIN multiplies each move while keeping its direction, so a file-system hop
// that would be 17 um becomes something you can hear. Absolute radius is then
// no longer preserved, which costs some of the CLV spindle pitch accuracy --
// that is the trade, and it is why the default is 1 (unchanged, faithful).
static double mir_u      = -1.0;   // current amplified position
static double mir_last_u = -1.0;   // last faithful position we saw

static double stroke_fraction(int game_lba)
{
	double r = cd_geom_radius_mm(game_lba);
	double u = (r - CD_R_INNER_MM) / (CD_R_OUTER_MM - CD_R_INNER_MM);
	if (u < 0.0) u = 0.0;
	if (u > 1.0) u = 1.0;
	return u;
}

static int amplified_mirror_lba(int game_lba, int resync)
{
	double gain = cfg.physical_disc_acoustic_gain;
	if (gain < 1.0) gain = 1.0;

	double u = stroke_fraction(game_lba);

	if (resync || mir_u < 0.0 || mir_last_u < 0.0) {
		mir_u = u;             // spin-up, sweep and park re-anchor to the truth
	}
	else {
		mir_u += (u - mir_last_u) * gain;
		if (mir_u < 0.0) mir_u = 0.0;
		if (mir_u > 1.0) mir_u = 1.0;
	}
	mir_last_u = u;

	int lba = media_lba_at_radius(mir.r_lo + mir_u * (mir.r_hi - mir.r_lo));
	if (lba < mir.span_lo) lba = mir.span_lo;
	if (lba > mir.span_hi) lba = mir.span_hi;
	return lba;
}

static int mir_prev_lba = -1;    // last place we actually sent the head

static void play_gesture(const gesture_t *g)
{
	double start = clock_ms();
	int resync   = (g->kind == GEST_SPINUP || g->kind == GEST_SWEEP ||
	                g->kind == GEST_PARK);
	int target   = amplified_mirror_lba(g->lba, resync);

	// `from` must be where the head actually IS, which once a gain is applied
	// is not the faithful mapping of the game's previous LBA.
	int from = (mir_prev_lba >= 0 && !resync) ? mir_prev_lba : target;
	mir_prev_lba = target;

	// Report BOTH travels: what the original mechanism would have done, and
	// what this drive is actually being asked to do. The second is the one
	// you can hear, and the ratio is the gain doing its job.
	double mirror_mm = media_radius_mm(target) - media_radius_mm(from);
	if (mirror_mm < 0) mirror_mm = -mirror_mm;
	acu_log("%-8s game %7d->%-7d  mirror %7d->%-7d  %8.1f turns "
	        "real %6.3fmm mirror %6.3fmm %5.0frpm %5.0fms x%d%s\n",
	        acu_gesture_name(g->kind), g->from_lba, g->lba, from, target,
	        g->turns, g->radial_mm, mirror_mm, g->rpm, g->dur_ms, g->stages,
	        mir.no_read ? " [SEEK-ONLY]" : "");
	int ra       = model.drive.readahead_sectors;
	if (ra < 1) ra = 1;
	if (ra > BURST_MAX) ra = BURST_MAX;

	// ---- PLAY AUDIO mode -------------------------------------------------
	// The drive advances the head itself at a true 1x, so most of the time the
	// right thing to do is nothing at all. We only intervene to put the head
	// somewhere else, which is exactly what a seek is.
	if (mir.play_mode) {
		int tail = mir.span_hi - target;
		if (tail < 1) tail = 1;

		switch (g->kind) {

		case GEST_JUMP:
			// Lens jump: the sled does not move and playback does not break.
			break;

		case GEST_STREAM: {
			if (g->audio) {
				// Red Book audio: the original deck tracked this smoothly and
				// quietly, and so does this one. Only step in if the drive has
				// drifted from where the model says the head should be.
				int pos = 0, playing = 0;
				if (mirror_subq(mir.dev_fd, &pos, &playing)) { playing = 0; pos = -1; }
				int drift = (pos < 0) ? INT_MAX
				          : (pos > target ? pos - target : target - pos);
				if (!playing || drift > 400) mirror_play(mir.dev_fd, target, tail);
				// A scratched disc makes a CD player hunt and skip mid-track.
				// Gentler than on a data read, because the drive is not also
				// fighting to get the sector right.
				if (grime_level() >= 9 && !(grime_rng() % 16)) grime_hunt(target, 1.2, 0);
				sleep_ms(g->dur_ms - (clock_ms() - start));
			}
			else {
				// A data read is not smooth tracking. The original mechanism
				// was working hard here -- seeking back over a sector it had to
				// retry, correcting, reacquiring -- and that busy servo is the
				// sound of a console LOADING, which is the sound anyone
				// actually recognises. Re-aiming the head a few times across
				// the gesture reproduces that; letting it glide does not.
				int n = 3;
				// One aim, not three. A PLAY costs about 173 ms on a period
				// drive, so three of them occupy ~520 ms to represent 240 ms
				// of game time: the player then runs at half the speed of the
				// thing it is mirroring, events back up, and a seek comes out
				// attached to whatever the game was doing a second earlier.
				// The command's own latency already fills the gesture.
				(void)n;
				mirror_play(mir.dev_fd, target, tail);
				if (grime_level() >= 9 && !(grime_rng() % 12)) grime_hunt(target, 1.5, 0);
				sleep_ms(g->dur_ms - (clock_ms() - start));
			}
			break;
		}

		case GEST_STEP: {
			// STEP was going straight out as one quiet PLAY with no grime at
			// all, and STEP is the commonest move there is during a load -- a
			// short hop between files on the data track. Those were all
			// bypassing everything.
			//
			// It also needs the grime more than a long seek does, not less. A
			// Mega CD takes about 305 ms over a short step; this drive does the
			// same move in 30 ms, so left alone it is a blip. The overshoot
			// passes are what give it any duration or travel.
			int gl = grime_level();
			if (gl) {
				grime_grind(from, target, g->dur_ms * (1.0 + gl * 0.30));
			}
			else {
				mirror_play(mir.dev_fd, target, tail);
				sleep_ms(g->dur_ms - (clock_ms() - start));
			}
			break;
		}

		case GEST_SLEW: {
			int gl = grime_level();
			if (gl) {
				// Grind the whole way across rather than jumping in two or
				// three stages. A tired sled is also slower than the mechanism
				// ever was when new, so the traverse is stretched well past
				// what the model says a healthy one would take.
				double drag = g->dur_ms * (1.0 + gl * 0.30);
				grime_grind(from, target, drag);
				// Then overshoot and come back, which is why an old console
				// takes two goes to settle before it starts reading.
				// the grind already overshoots and returns; no extra fidget
			}
			else {
				static const double two[]   = { 0.85, 1.0 };
				static const double three[] = { 0.60, 0.92, 1.0 };
				const double *frac = (g->stages >= 3) ? three : two;
				int n = (g->stages >= 3) ? 3 : 2;
				double slot = g->dur_ms / n;
				double rf = media_radius_mm(from), rt = media_radius_mm(target);
				for (int i = 0; i < n; i++) {
					if (!mir.on || mir.held || mir.phys_session || mir.dev_fd < 0) break;
					int stop_lba = media_lba_at_radius(rf + (rt - rf) * frac[i]);
					int blk = mir.span_hi - stop_lba; if (blk < 1) blk = 1;
					double t0 = clock_ms();
					mirror_play(mir.dev_fd, stop_lba, blk);
					sleep_ms(slot - (clock_ms() - t0));
				}
			}
			break;
		}

		case GEST_LOCK: {
			// Arrived at the track, spun up, not playing yet. The disc turns
			// several times while the servo settles and acquires the subcode,
			// and a Mega CD makes that unmistakably audible before the music
			// starts. So: spindle to a true 1x at the track start, then let it
			// turn for the modelled number of revolutions, with the occasional
			// correction a real servo makes while it is locking on.
			grime_resume(target);
			double until = start + g->dur_ms;
			while (clock_ms() < until) {
				if (!mir.on || mir.held || mir.phys_session || mir.dev_fd < 0) break;
				sleep_ms(90);
				if (grime_level() && !(grime_rng() % 4)) {
					mirror_seek(mir.dev_fd, lba_offset_mm(target, 0.12));
					mirror_seek(mir.dev_fd, target);
					grime_resume(target);
				}
			}
			break;
		}

		case GEST_HOLD:
			// Spindle on, head held: that is exactly audio pause. A tired
			// mechanism cannot hold still though -- it drifts off track and has
			// to pull itself back, which is the idle fidgeting you hear from a
			// console sitting on a menu.
			mirror_pause(mir.dev_fd, 0);
			if (grime_level() >= 7 && !(grime_rng() % 6)) grime_hunt(from, 2.0, 0);
			sleep_ms(g->dur_ms);
			break;

		case GEST_SPINUP:
			mirror_spin(mir.dev_fd, 1);
			mirror_play(mir.dev_fd, target, tail);
			// A hazy lens takes several goes to focus. Everyone who owned one
			// of these knows the sound of a console thinking about it.
			grime_hunt(target, 4.0, 3);
			sleep_ms(g->dur_ms - (clock_ms() - start));
			break;

		case GEST_SPINDOWN:
			mirror_stop_play(mir.dev_fd);
			if (own_device()) mirror_spin(mir.dev_fd, 0);
			break;

		case GEST_SWEEP: {
			// The boot calibration pass: hub to rim and back. This is the
			// longest travel the drive ever makes, so it is the one worth
			// grinding out in full -- it is the sound of the console waking up.
			int gl = grime_level();
			if (gl) {
				// Wind the spindle right down and back up before the sweep. A
				// console that has been sitting does not come straight up to
				// speed -- it drops, catches, and hauls itself back, and that
				// is the first thing you hear when one is switched on. Only
				// worth doing here: it costs seconds, which is fine once at
				// boot and intolerable anywhere else.
				if (gl >= 5) {
					mirror_stop_play(mir.dev_fd);
					if (own_device()) mirror_spin(mir.dev_fd, 0);
					sleep_ms(500);
					sleep_ms(500);               // spin-down is not instant
					if (own_device()) mirror_spin(mir.dev_fd, 1);
					sleep_ms(500);
					sleep_ms(500);
					grime_resume(mir.span_lo);
				}
				// Full stroke out and back: the longest continuous travel the
				// mechanism ever makes.
				double leg = g->dur_ms * (1.0 + gl * 0.30) / 2.0;
				grime_grind(mir.span_lo, mir.span_hi, leg);
				grime_grind(mir.span_hi, mir.span_lo, leg);
			}
			else {
				int pts[3] = { mir.span_lo, mir.span_hi, mir.span_lo };
				double slot = g->dur_ms / 3.0;
				for (int i = 0; i < 3; i++) {
					if (!mir.on || mir.held || mir.phys_session || mir.dev_fd < 0) break;
					int blk = mir.span_hi - pts[i]; if (blk < 1) blk = 1;
					double t0 = clock_ms();
					mirror_play(mir.dev_fd, pts[i], blk);
					sleep_ms(slot - (clock_ms() - t0));
				}
			}
			break;
		}

		case GEST_PARK:
			mirror_play(mir.dev_fd, mir.span_lo, 1000);
			sleep_ms(g->dur_ms - (clock_ms() - start));
			mirror_stop_play(mir.dev_fd);
			if (own_device()) mirror_spin(mir.dev_fd, 0);
			break;

		default:
			break;
		}
		return;
	}

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

		// Interpolate in MIRROR radius, between where the head is and where it
		// is going. Staging in game space would silently discard the gain and
		// collapse every stage onto almost the same place.
		double slot = g->dur_ms / n;
		double rf   = media_radius_mm(from);
		double rt   = media_radius_mm(target);
		for (int i = 0; i < n; i++) {
			if (!mir.on || mir.held || mir.phys_session || mir.dev_fd < 0) break;
			int stop_lba = media_lba_at_radius(rf + (rt - rf) * frac[i]);
			double t0 = clock_ms();
			touch(stop_lba, ra);
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

	case GEST_LOCK:
		// Read-mode equivalent: sit on the track while the disc turns, with
		// the odd correction, before any data starts flowing.
		touch(target, 1);
		sleep_ms(g->dur_ms);
		break;

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

		// Stay current rather than complete.
		//
		// A period drive's mechanism is slow -- that is the whole point of
		// using one -- so playing a gesture takes far longer than the slice of
		// game time it represents. Work through a backlog and the sled ends up
		// moving for something the game did a second ago, which is heard as the
		// right noises at the wrong moments.
		//
		// The drive can only be in one place, so a queue of gestures is not a
		// list of work, it is successively better information about where the
		// head should be. Take the newest. The exception is a real move: a
		// SLEW, STEP or SWEEP is the thing worth hearing, so it is never
		// discarded in favour of a stream or a hold that merely arrived later.
		{
			gesture_t next;
			int dropped_here = 0;
			while (acu_model_poll(&model, &next)) {
				int g_is_move    = (g.kind == GEST_SLEW || g.kind == GEST_STEP ||
				                    g.kind == GEST_SWEEP || g.kind == GEST_SPINUP);
				int next_is_move = (next.kind == GEST_SLEW || next.kind == GEST_STEP ||
				                    next.kind == GEST_SWEEP || next.kind == GEST_SPINUP);
				if (!g_is_move || next_is_move) g = next;
				dropped_here++;
			}
			if (dropped_here)
				acu_log("behind by %d gesture%s, skipping to %s\n", dropped_here,
				        dropped_here == 1 ? "" : "s", acu_gesture_name(g.kind));
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
		// A pinned profile from MiSTer.ini has to apply from the start, not
		// only once some core gets around to mounting a disc -- otherwise the
		// drive is set up for the wrong mechanism until then.
		int forced = cfg.physical_disc_acoustic_profile;
		mir.profile_req = (forced > PD_ACU_PROFILE_AUTO && forced < PD_ACU_PROFILE_COUNT)
		                ? forced : PD_ACU_PROFILE_AUTO;
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
