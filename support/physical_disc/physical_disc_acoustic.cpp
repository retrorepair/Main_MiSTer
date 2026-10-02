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
	volatile int spun;        // do we believe the spindle is actually turning
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
	double last_r;

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

// Some things genuinely take seconds -- a spindle ramping up or coasting down is
// the obvious one -- and the 500 ms clamp above silently truncated every one of
// them. A spin-up modelled at 2200 ms was waiting 500 ms and then being cut off
// by the next command, which is why neither end of the spin cycle sounded right.
// Waits in chunks so it still notices being told to stop.
static int mirror_aborted(void);

static void sleep_long_ms(double ms)
{
	double end = clock_ms() + ms;
	while (clock_ms() < end) {
		if (mirror_aborted()) return;
		sleep_ms(100);
	}
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

// Move the head and WAIT for it to arrive.
//
// SEEK(10) on this drive is fire-and-forget: measured at 1.0 ms for every
// distance from 0.5 mm to 32 mm, which no sled can do. The command is queued and
// returns at once, so a staircase of seeks issued back to back collapses -- each
// target overwrites the one before it and the drive performs a single move. That
// is why no amount of reshaping the staircase has ever been audible: there has
// only ever been one move.
//
// READ SUB-CHANNEL serialises behind the pending move, so SEEK followed by a
// sub-channel read costs the real mechanical time -- measured 150 ms fixed plus
// about 16 ms per millimetre (310 ms over 0.5 mm, 260 over 8, 373 over 16, 673
// over 32), and the head lands exactly on target. Anything that is meant to be
// heard as a separate sled movement has to go through here and pay that.
static int mirror_seek_sync(int fd, int lba)
{
	int r = mirror_seek(fd, lba);
	int pos = 0, playing = 0;
	mirror_subq(fd, &pos, &playing);
	return r;
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
	if (mir.phys_session) return -1;

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

		mir.spun      = 0;
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
	if (mir.phys_session) {
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
	if (mir.no_read)        r = mirror_seek_sync(mir.dev_fd, lba);
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
	return mir.dev_fd >= 0 && !mir.phys_session;
}

static int mirror_aborted(void)
{
	return !mir.on || mir.held || mir.phys_session || mir.dev_fd < 0;
}

// Follow the original drive's CLV speed, changing it as the gesture asks.
//
// The speed was being set once at open and never touched, which left the mirror
// at a constant 1x. For a Mega CD that is right -- it is a 1x drive and never
// anything else -- but a PlayStation reads data at 2x and plays audio at 1x, and
// the spindle ramping between the two is a large part of how one sounds. Nothing
// was reproducing that at all.
//
// Measured on the drive here, SET CD SPEED is genuinely honoured: raw-read
// throughput came out at 0.68x, 1.15x, 2.49x and 5.11x of real time for
// requested 1x, 2x, 4x and max. The absolute figures sit below the request
// because each read is a separate command, but it scales, and the ramp between
// settings is audible -- which is the entire point.
static void apply_speed(double mult)
{
	static int applied = -1;
	if (mir.dev_fd < 0 || mult < 0.5) return;

	int want = (int)(mult + 0.5);
	if (want < 1)  want = 1;
	if (want > 12) want = 12;
	if (want == applied) return;

	ioctl(mir.dev_fd, CDROM_SELECT_SPEED, want);
	applied = want;
	acu_log("  spindle to %dx\n", want);
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

// Move the head and nothing else. A bare SEEK(10) looked like the cheap
// primitive for this -- 42 ms against PLAY's 200 ms -- but that 42 ms was the
// drive queueing the command and returning, not moving: re-measured, a bare SEEK
// costs 1.0 ms over any distance from 0.5 mm to 32 mm. Everything built on it
// (the hunts, the stutters, the staircase) was therefore inaudible, and the one
// move that did happen was whatever the last target had been. Synchronised, this
// costs 150 ms + 16 ms/mm and the sled actually goes there.
static void grime_aim(int lba)
{
	if (mir.play_mode) mirror_seek_sync(mir.dev_fd, lba);
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

// Move the head to `to_lba`, taking about as long as the original mechanism
// would have taken, and making as much noise on the way as it really would.
//
// The history of this function is worth keeping, because three plausible ideas
// were wrong and the measurements say why:
//
//  - A fine staircase (many small steps) judders. Every SCSI positioning
//    command is a move-and-settle: the firmware runs the sled to the address
//    and stops it dead. Back-to-back steps are not continuous motion, they are
//    one settle after another -- a 24 Hz buzz at SEEK's cost, a row of clunks
//    at PLAY's.
//  - One single traverse is too short. The drive crosses the whole disc in
//    693 ms where a Mega CD seek is up to 1.66 s.
//  - Overshoot ping-pong cannot fill a duration predictably: each pass covers
//    the whole distance plus an overshoot, so one pass is too short and two are
//    too long, which produced the 1.5-2x overruns that were heard as a seek
//    against the wrong thing on screen.
//
// What works is to solve for the step count from the drive's measured cost
// curve. SEEK(10) here costs about a fixed 150 ms plus 0.0019 ms per sector --
// fitted to 28 ms at 500 sectors, 188 at 20000, 264 at 60000, 392 at 150000 and
// 693 at 280000. The fixed part dominates, so:
//
//     total = N*FIXED + PER_SECTOR*distance   =>   N = (total - travel)/FIXED
//
// N is few (one to eight), each step is a substantial monotonic traverse toward
// the target, and the duration comes out right.
// Fitted to the two measurements that matter for long moves: 261 ms at 60000
// sectors and 693 ms at 280000.
// Re-measured against a SYNCHRONISED seek (mirror_seek_sync), which is the only
// kind that moves the sled on its own. Per step: a fixed cost plus a distance
// term -- 150 ms + 16 ms/mm, fitted to 310 ms over 0.5 mm, 260 over 8, 373 over
// 16 and 673 over 32. The earlier constants were fitted to the same curve but
// spent against UNsynchronised seeks costing 1 ms each, so the budget was never
// the thing being consumed.
// ...then re-measured again IN SERVICE, which is the number that matters. On an
// idle drive a synchronised step costs the 150 ms above; with the core actually
// running -- audio playing, the read cadence issuing a move every 250 ms -- every
// command queues behind that, and the same step costs 250-700 ms. Timing the
// grind's own tail showed it plainly: the overshoot-and-return pair, two moves of
// 3.5 mm, took 830 and 1369 ms where the bench curve predicted 412. Budgeting
// against bench figures is what left a long seek 35-50% over its duration.
// 340, not 180: a seek issued while the sled is already moving costs MORE than an
// isolated one, because the drive has to decelerate, re-plan and accelerate again.
// Measured from the drags themselves -- 29.2 mm in 4 segments took 1958 ms, so
// 490 ms for a 7.3 mm segment against the 297 ms an isolated-seek curve predicts.
// The 180 ms figure was for 0.7 mm steps, where the fixed part dominates and the
// re-plan is cheap. Budgeting with it put a 1339 ms seek at 1958 ms.
#define STEP_FIXED_MS        340.0
#define STEP_PER_MM_MS        16.0
#define DRIVE_MIN_SEEK_MS    300.0

// Every grind ends with an overshoot seek, an arrival seek and a PLAY to put the
// spindle back to 1x. All three land inside the duration being budgeted, and
// ignoring them is why a 1339 ms seek took 2564 ms: the steps filled the budget
// and then the tail ran past it. The two seeks are now synchronised, so they
// cost real time and are real sled movement -- and a reversal at that, which is
// worth more acoustically than another step in the same direction.
// Measured at 25-28 ms, not the 250 ms guessed here before: the arrival seek has
// already put the head where the PLAY wants it, so the PLAY only re-establishes
// the audio servo rather than moving anything. The guess was spending a fifth of a
// short seek's whole budget on nothing.
#define RESUME_PLAY_MS        40.0

// Drag the sled continuously from one radius to another, taking `total_ms`.
//
// This is what the non-blocking SEEK(10) is actually good for. Waiting for each
// step (mirror_seek_sync) gives a discrete move: 50 ms of travel, then 300-700 ms
// of silence while the sync returns. A row of those is not a seek -- it is a drive
// twitching, and it reads as a failing laser rather than a working mechanism.
//
// A bare SEEK returns in 1 ms, so re-aiming every few tens of milliseconds along
// the path means the sled never stops: the next target arrives while it is still
// travelling and it simply keeps going. The sled is in motion for the WHOLE
// duration, which is the slow grinding traverse a real deck makes, and it costs
// exactly the budget rather than overrunning it.
//
// Monotonic on purpose. A real sled runs one way across a seek; oscillating back
// and forth in one place is what a drive does when it cannot track.
static int g_drag_segs;

// One sweep. A single bare SEEK, which the drive executes as one continuous
// worm-gear traverse -- the Mega CD's sled runs on a worm gear and two rails, and
// that is a continuous whirr, not a ratchet.
//
// Everything cleverer than this sounded wrong. Equal segments gave a 2-3 Hz chug
// ("gravely", "jittery") because each one accelerates, decelerates and settles. A
// coarse-then-fine pair was better but still two starts. The real servo corrects at
// up to 75 Hz -- the CDD runs its command loop once per sector period -- which is
// twenty times faster than anything reachable over USB, so fine structure is simply
// not available here. One clean sweep is.
static double g_seg_ms[8];
static double g_seg_mm[8];

static void sled_sweep(double r_to)
{
	if (mirror_aborted()) return;
	double from = (mir.last_r > 0.0) ? mir.last_r : r_to;
	double st   = clock_ms();
	mirror_seek(mir.dev_fd, media_lba_at_radius(r_to));
	if (g_drag_segs < 8) {
		g_seg_ms[g_drag_segs] = clock_ms() - st;
		g_seg_mm[g_drag_segs] = (r_to > from) ? r_to - from : from - r_to;
	}
	mir.last_r = r_to;
	g_drag_segs++;
}

static void grime_grind(int from_lba, int to_lba, double total_ms)
{
	if (mir.dev_fd < 0 || mir.phys_session) return;

	double grind_start = clock_ms();
	g_drag_segs = 0;

	double r0   = media_radius_mm(from_lba);
	double r1   = media_radius_mm(to_lba);
	double span = r1 - r0;
	double dist = span < 0 ? -span : span;

	// Below the lens-jump range the sled does not move at all, so there is
	// nothing to make noise with.
	if (dist < 0.05) { grime_resume(to_lba); return; }

	// Below about half a second there is only time for one move, and a short seek
	// played as a long one is what made a 0.6 mm hop sound identical to a cross-disc
	// transition. One move, and let the sync pay for it.
	if (total_ms < DRIVE_MIN_SEEK_MS * 2.0 || dist < 0.30) {
		mirror_seek(mir.dev_fd, to_lba);
		grime_resume(to_lba);
		sleep_ms(total_ms - (clock_ms() - grind_start));
		return;
	}

	// TWO SWEEPS AND ONE TURNAROUND.
	//
	// This drive crosses the whole disc in about 670 ms; a Mega CD takes 1.5-2 s,
	// because its sled is a slow worm gear. So a single sweep leaves most of the
	// budget as silence, and subdividing it into equal steps is the chug that got
	// called gravely. The way out is distance, not subdivision: overshoot the target
	// and come back. Two continuous sweeps, one direction change, and the sled is
	// moving almost the whole time.
	//
	// It is also what the mechanism does. A long jump is run partially open-loop to
	// the ESTIMATED position of the target track, so it misses and the servo corrects
	// -- the overshoot is the miss. Sizing it from the leftover budget rather than a
	// fixed fraction means the time gets spent on travel instead of waiting.
	// MATCH THE SLED VELOCITY, and go one way only.
	//
	// The drags read as modern quick seeks, and that is velocity: measured, this
	// drive has exactly two sled speeds and a Mega CD's is in neither of them.
	//
	//   PLAY AUDIO tracking    0.012 mm/s
	//   SCAN (0xBA)            rejected, not supported
	//   SET CD SPEED 1x        524 ms per 20 mm against 472 at max -- 11%, no use
	//   one plain seek         about 48 mm/s
	//   a Mega CD              about 16-21 mm/s
	//
	// Back-to-back seeks are the only thing that lands in range, because the fixed
	// per-command cost dominates a short move: a 7.3 mm segment measured 490 ms,
	// which is 15 mm/s. So segment size IS the velocity control, and solving
	// n*(FIXED + PER_MM*dist/n) = total for n gives the count that makes the
	// traverse take exactly as long as the mechanism would -- at the mechanism's
	// speed, rather than arriving early and waiting.
	//
	// Monotonic, and no overshoot. The out-and-back legs this replaces existed only
	// to burn a budget that a too-fast sweep left over; matching the velocity fills
	// the time with the real distance instead, so there is nothing to pad and no
	// reversal to hear. A seek goes one way.
	//
	// EQUAL distances, which is the only way the velocity comes out constant.
	//
	// An S-curve ramp sat here, on the reasoning that a real seek accelerates and
	// that equal steps are a metronome. Measuring per segment showed it was the
	// bug, not the polish: one drag came out
	//
	//   2ms/6.5mm=3400   315ms/14.9mm=47   538ms/9.4mm=17   412ms/1.6mm=4   mm/s
	//
	// -- nothing, a zip, a correct grind, then a crawl, averaging to a respectable
	// 17 mm/s that was never actually played. The tail segment is the giveaway: a
	// short leg still pays the whole fixed per-command cost, so making legs unequal
	// makes their velocities unequal, and the ear hears the pieces, not the mean.
	// That is the reported inconsistency, and why it came out right occasionally --
	// one segment in four was at Mega CD speed.
	//
	// Equal legs pay the same cost and cover the same ground: velocity is then
	// L/(FIXED + PER_MM*L) for every one of them.
	// ...and the leg LENGTH has to come from what the drive is actually doing,
	// because its per-command cost swings two to three times from pass to pass. The
	// same 8.1 mm leg has measured 222 ms and 750 ms, so a fixed plan gives 20 mm/s
	// on one seek and 35 mm/s on the next -- right sometimes and modern the rest of
	// the time, which is exactly how this has been reported all along.
	//
	// Velocity is L/(fixed + PER_MM*L), so it is controllable: a pass where the
	// drive is running quick needs SHORTER legs to stay slow, because the fixed cost
	// is what holds the speed down. Measure each leg, keep a running estimate of the
	// fixed cost, and re-solve for the length that hits the velocity the mechanism
	// would have had.
	double want_v = (total_ms > 1.0) ? dist / total_ms : 0.02;   // mm per ms
	double denom  = 1.0 - STEP_PER_MM_MS * want_v;
	double fixed  = STEP_FIXED_MS;
	double gone   = 0.0;
	int    segs   = 0;

	while (gone < dist - 0.10 && segs < 12 && !mirror_aborted()) {
		double L = (denom > 0.05) ? fixed * want_v / denom : dist;
		if (L < 0.8) L = 0.8;
		// Never leave a stub behind: a short final leg still pays the whole fixed
		// cost, so it comes out at 4-6 mm/s and is heard as the drag dying away.
		// If the remainder is less than another leg and a half, take it all now.
		if (dist - gone < L * 1.5) L = dist - gone;

		double st = clock_ms();
		gone += L;
		sled_sweep(r0 + (span > 0 ? gone : -gone));
		double el = clock_ms() - st;

		// Only a leg that actually blocked tells us anything: the first one returns
		// in about a millisecond because the sled is not yet moving.
		//
		// Gently. A leg's elapsed time is really the PREVIOUS leg's travel -- the
		// measurement is delayed by one -- so a high gain on it rings: 0.6 gave
		// 27, 15, 23, 6 mm/s within a single drag.
		if (el > 20.0) {
			double f = el - STEP_PER_MM_MS * L;
			if (f > 40.0 && f < 1500.0) fixed = fixed * 0.75 + f * 0.25;
		}

		segs++;
		if (clock_ms() - grind_start > total_ms) break;
	}

	// Land exactly, but only if the loop did not already arrive. An unconditional
	// final seek was travelling 0.0 mm and blocking for up to 1257 ms waiting on the
	// previous move, which showed up as a 1.3x overrun.
	if (!mirror_aborted() && gone < dist - 0.10) sled_sweep(r1);

	// SEEK leaves audio playback stopped, so hand the spindle back to a true 1x.
	grime_resume(to_lba);

	// Whatever is left after the legs: hold the slot so the rhythm is right.
	// sleep_long_ms, because sleep_ms clamps at 500 ms and was silently
	// truncating a 714 ms remainder.
	sleep_long_ms(total_ms - (clock_ms() - grind_start));

	// Report the per-segment VELOCITY, because that is what the ear judges and it
	// is not a constant: the same move has cost 263+288 ms on one pass and
	// 407+711 ms on another. At 700 ms a 7 mm segment is 16 mm/s, a Mega CD; at
	// 250 ms it is 45 mm/s, a modern drive. The same code therefore produces both
	// sounds, which is the reported inconsistency -- it was heard correctly "a few
	// times". This is the measurement that shows which pass was which.
	{
		char v[220];
		int  vn = 0;
		int  ns = (g_drag_segs < 8) ? g_drag_segs : 8;
		double whole = clock_ms() - grind_start;
		for (int i = 0; i < ns && vn < (int)sizeof(v) - 20; i++)
			vn += snprintf(v + vn, sizeof(v) - vn, " %.0fms/%.1fmm=%.0f",
			               g_seg_ms[i], g_seg_mm[i],
			               g_seg_ms[i] > 1.0 ? g_seg_mm[i] * 1000.0 / g_seg_ms[i] : 0.0);
		acu_log("  drag %.1fmm over %.0fms in %d segs, whole %.0fms, mean %.0fmm/s |%s\n",
		        dist, total_ms, g_drag_segs, whole,
		        whole > 1.0 ? dist * 1000.0 / whole : 0.0, v);
	}
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
		// Dragged, not aimed. A synchronised aim is a jerk followed by silence,
		// and a row of them in one place is what a drive does when the laser has
		// lost the track. Dragging keeps the wander a continuous movement.
		double here = media_radius_mm(lba);
		sled_sweep(here + away);
		sled_sweep(here);
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

// Wall-clock cost of the last gesture, so the trace can show what the drive
// really did against what the model asked for. Without this a gesture that
// returns early is indistinguishable from one that was never emitted.
static double gesture_actual_ms;

static int mir_prev_lba = -1;    // last place we actually sent the head

static void play_gesture(const gesture_t *g)
{
	double start = clock_ms();
	int resync   = (g->kind == GEST_SPINUP || g->kind == GEST_SWEEP ||
	                g->kind == GEST_PARK);
	int target   = amplified_mirror_lba(g->lba, resync);

	// Follow the modelled spindle speed; a change here IS the audible ramp.
	if (g->speed > 0.0) apply_speed(g->speed);

	// No staleness test any more. There was one, dropping a move older than
	// 1.2 s on the grounds that a late seek is worse than no seek, and it threw
	// away the audio lock-on and then two seeks out of five behind the boot
	// sweep. It was also redundant: coalescing already merges queued seeks into
	// one movement to the NEWEST target, so a seek cannot be superseded by the
	// time it is played. Lateness is handled by going to the right place, not by
	// refusing to go.

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

				// No nudging during playback. A tick out and a pull back, in
				// one place, is a drive hunting -- it reads as a broken laser,
				// not a worn one. A deck tracking a track is quiet, and the
				// noise belongs in the traverses either side of it.
				// A scratched disc makes a CD player hunt and skip mid-track.
				// Gentler than on a data read, because the drive is not also
				// fighting to get the sector right.
				if (grime_level() >= 9 && !(grime_rng() % 16)) grime_hunt(target, 1.2, 0);
				sleep_ms(g->dur_ms - (clock_ms() - start));
			}
			else {
				// THE SOUND OF A LOAD IS HERE, and this branch used to sleep
				// through it. A head trace of a live Sonic CD session found the
				// sled moving exactly twice in forty seconds: the game asks for
				// very few seeks, so shaping seeks -- which is what every change
				// before this one did -- cannot be heard. Everything between
				// them is a data read, and a data read was one PLAY followed by
				// a sleep. Silence, for the whole of the only stretch anyone
				// recognises.
				//
				// A Mega CD loading is not quiet and it is not one long sweep.
				// The CDC takes a short run of sectors, the CDD pauses while the
				// Sub-CPU drains the buffer, the lens re-locks, and it goes
				// again -- a few times a second, for as long as the load lasts.
				// That repeating cadence is the noise.
				//
				// So: one real sled move per gesture, alternating back and
				// forward around the target, which is the resync the mechanism
				// actually performed. Alternating rather than a pair per gesture
				// is what makes it affordable -- a SEEK costs about the same as
				// the PLAY it replaces (~200 ms against ~173 ms), so the player
				// keeps pace with the core instead of falling a second behind,
				// and the net head position still lands where the model says.
				// One aim, and no oscillation. This branch used to seek back
				// and forth around the target at 4 Hz, on the theory that a
				// load is the CDC's buffer-fill cadence. That theory was wrong
				// twice over: a real sled advances monotonically through a read,
				// and 0.35 mm of travel each way four times a second is a drive
				// hunting in place -- audibly a laser that cannot track. The
				// sound of a load is the TRAVERSES, which sled_sweep now makes
				// continuous; between them the mechanism is entitled to be quiet.
				//
				// And "quiet" has to mean ISSUING NOTHING. This was still firing
				// a PLAY every gesture -- four a second, every one of them
				// re-acquiring the audio servo -- which is heard as a constant
				// skittishness and as "0.25 second travels", 250 ms being
				// exactly the gesture period. Measured over 260 s of play there
				// were only sixteen real head movements, nearly all of them
				// 29-32 mm, with a 95 s gap between clusters: the seeks were
				// never the noise. This was.
				//
				// So re-aim only when the head is actually somewhere else, the
				// same test the audio branch already used.
				int pos = 0, playing = 0;
				if (mirror_subq(mir.dev_fd, &pos, &playing)) { playing = 0; pos = -1; }
				int drift = (pos < 0) ? INT_MAX
				          : (pos > target ? pos - target : target - pos);
				if (!playing || drift > 400) {
					mirror_play(mir.dev_fd, target, tail);
					acu_log("  re-aim: playing=%d drift=%d\n", playing, drift);
				}
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
				grime_grind(from, target, g->dur_ms);
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
				// Grind across for exactly as long as the ORIGINAL drive would
				// have taken -- no more. Wear changes the character of a seek,
				// not its duration: the emulated drive reports a seek time and
				// the game's own timing is built on it, so stretching it by the
				// grime level (3.1x at level 7) left the mirror grinding long
				// after the core had resumed playback. Heard as a long seek
				// during music, which no real drive does.
				grime_grind(from, target, g->dur_ms);
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
			// Arrived at the track, spun up, not playing yet -- and on a Mega CD
			// this is the SECOND HALF of the noise you hear before a CDDA track,
			// not a pause. It used to move the sled 0.000 mm: a trace of a track
			// change showed 1550 ms of drag, then 1534 ms of complete silence, then
			// the music. That is why the long seek sounded like it fired at the
			// wrong moment -- it fired at the right one and then stopped a second
			// and a half early.
			//
			// So settle onto the track audibly: step off it, then ease back on over
			// most of the lock, so the sled is still working when the music starts.
			// One reversal and a slow monotonic approach, which is a servo pulling
			// in -- deliberately NOT a hunt, because small movements repeated back
			// and forth in one place is the sound of a drive that cannot track.
			grime_resume(target);

			// No sled movement here. Locking on is FOCUS, which is the lens: the
			// CDD's own error table has 0x03 E-FOCUS retrying "until ok" when focus
			// is down for more than 100 ms, and none of that moves the carriage. And
			// command 0x03 READ/PLAY is "SEEK to start position THEN Play music", one
			// command whose status goes to PLAY immediately, so there is no long lock
			// phase between the seek and the music in the first place.
			//
			// Two earlier versions put the sled to work here -- first a drag out and
			// back, then a single correction -- to cover a 1.5 s silence between the
			// sweep and the music. The silence was the real bug: the seek was too
			// short. It is in the seek now (full_stroke 2400), and this is brief and
			// quiet, as the hardware is.
			sleep_ms(g->dur_ms - (clock_ms() - start));
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
			// The ramp IS the sound, so it has to be allowed to happen before
			// anything else is asked of the drive. START STOP UNIT is an
			// immediate command and returns at once, so following it straight
			// away with PLAY hauled the disc to 1x and the ramp was never
			// heard. Only wait when the spindle really was stopped, or every
			// resume picks up a spin-up it does not deserve.
			mirror_spin(mir.dev_fd, 1);
			if (!mir.spun) {
				sleep_long_ms(g->dur_ms * 0.7);
				mir.spun = 1;
			}
			mirror_play(mir.dev_fd, target, tail);
			// A hazy lens takes several goes to focus. Everyone who owned one
			// of these knows the sound of a console thinking about it. Only if
			// the budget can take it: each wander is now a drag rather than a
			// pair of aims, which is longer, and an unbounded hunt here ran the
			// spin-up to 4213 ms against 2200.
			if (g->dur_ms - (clock_ms() - start) > 900)
				grime_hunt(target, 4.0, 2);
			sleep_long_ms(g->dur_ms * 0.3 - (clock_ms() - start));
			break;

		case GEST_SPINDOWN:
			// Coasting down takes seconds and had no wait at all, so the next
			// gesture's PLAY spun the disc back up before any of it was
			// audible. Playback has to be stopped first, or the drive keeps the
			// spindle running to service it and the stop is ignored.
			mirror_stop_play(mir.dev_fd);
			if (own_device()) mirror_spin(mir.dev_fd, 0);
			mir.spun = 0;
			sleep_long_ms(g->dur_ms);
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
				double leg = g->dur_ms / 2.0;
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
		mir.spun = 0;
		sleep_long_ms(g->dur_ms);
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

			// A profile change means a core has just mounted a disc, so this is
			// the moment that deck calibrates. It also has to be re-issued here
			// because acu_model_init() clears the gesture queue: the boot
			// sequence queued at enable time was being wiped by the very profile
			// change that told us which drive to imitate, so the sweep was
			// emitted and then discarded before it could ever be played.
			acu_model_event(&model, clock_ms(), PD_ACU_TRAY_CLOSE, 0, 0);
			acu_model_event(&model, clock_ms(), PD_ACU_TOC, 0, 0);
		}

		// Deliberately NOT gated on physical_disc_drive_busy() any more. That
		// reports only "physical_disc has the device open", which is not the
		// same thing as "the game is being served off the disc". The PSX
		// launcher opens the drive speculatively to watch for a disc swap and
		// never closes it, so gating on it silenced the mirror for the whole
		// session even when the game was plainly running from a CHD.
		//
		// phys_session is the authoritative answer: set when physical_disc
		// takes the device, and corrected by whichever core mounts a disc,
		// which is the only place that knows whether the game is coming off
		// the disc or off an image.

		double now = clock_ms();

		// Drain whatever the cores reported into the model.
		int drained = 0;
		while (mir.ring_head != mir.ring_tail && drained < 64) {
			acu_event_t e = mir.ring[mir.ring_head & (EVENT_RING - 1)];
			mir.ring_head++;
			// Log what the cores actually report, not what they were assumed
			// to. Chasing a missing lock-on through the model was guesswork
			// without this.
			static const char *evname[] = { "SEEK", "READ", "PLAY", "SCAN",
			                                "PAUSE", "STOP", "TOC", "SPINUP",
			                                "TRAY_OPEN", "TRAY_CLOSE" };
			if (e.ev != PD_ACU_READ && e.ev != PD_ACU_PLAY)
				acu_log("  ev %-10s lba %7d cnt %d\n",
				        (e.ev <= PD_ACU_TRAY_CLOSE) ? evname[e.ev] : "?", e.lba, e.count);
			else if (!(drained % 64))
				acu_log("  ev %-10s lba %7d cnt %d (sampled)\n",
				        (e.ev <= PD_ACU_TRAY_CLOSE) ? evname[e.ev] : "?", e.lba, e.count);
			acu_model_event(&model, e.at_ms, e.ev, e.lba, e.count);
			drained++;
		}
		acu_model_tick(&model, now);

		// Anything lost here is a seek the user never hears, which is exactly
		// how "it works, sometimes" happens. Both counters should stay at zero;
		// report them when they move so it is never a guess again.
		{
			static unsigned seen_ring = 0, seen_model = 0;
			if (mir.dropped != seen_ring || model.dropped_events != seen_model) {
				acu_log("LOST: %u event%s at the ring, %u gesture%s at the model\n",
				        mir.dropped, mir.dropped == 1 ? "" : "s",
				        model.dropped_events, model.dropped_events == 1 ? "" : "s");
				seen_ring  = mir.dropped;
				seen_model = model.dropped_events;
			}
		}

		// One-slot pushback. Coalescing seeks has to read ahead past them, and
		// when it meets a real event it must be able to put it back: the model's
		// queue only pops, and dropping an event there is how the lock-on went
		// missing twice already.
		static gesture_t stash;
		static int       has_stash = 0;

		gesture_t g;
		if (has_stash) {
			g         = stash;
			has_stash = 0;
		}
		else if (!acu_model_poll(&model, &g)) {
			sleep_ms(10);
			continue;
		}

		// The spindle follows the model's CURRENT speed, every iteration, not
		// just when a gesture happens to be played. Stream gestures are mostly
		// collapsed away, so hanging the speed off them meant one ramp in
		// seventy seconds on a PlayStation, which alternates 1x audio and 2x
		// data constantly. apply_speed() is a no-op when nothing has changed.
		apply_speed(model.stream_mult);

		// Collapse position updates, but never collapse movement.
		//
		// A period drive is slow -- that is why we want one -- so playing a
		// gesture takes longer than the slice of game time it represents, and
		// working through a backlog puts the sled in motion for something the
		// game did a second ago. But the first version of this kept only ONE
		// gesture when behind, and that quietly destroyed the best sound the
		// feature has. A scene transition is a BURST of seeks -- the filesystem
		// walk, then the asset -- and the sequence is what makes it three
		// seconds of audible sled. Collapsing it to the newest left a single
		// 700 ms traverse where there should have been four.
		//
		// The distinction: a STREAM, JUMP or HOLD is merely "where the head is
		// now", so only the newest is worth anything. A SLEW, STEP or SWEEP is
		// an event with its own sound, and every one of them gets played in
		// order. Moves are rare next to streams -- single figures against
		// hundreds -- so keeping all of them costs almost nothing.
		{
			// Listing what is SAFE TO DISCARD, not what is worth keeping.
			//
			// The other way round has now cost two separate bugs: first the
			// move burst of a scene transition, then the audio lock-on, both
			// silently eaten because they were not in a list of exceptions. A
			// gesture that is merely "where the head is now" is discardable;
			// everything else is an event with its own sound and duration.
			// Written this way a gesture added later defaults to being kept,
			// which is the direction that fails safely.
			#define IS_UPDATE(k) ((k) == GEST_STREAM || (k) == GEST_JUMP || \
			                      (k) == GEST_HOLD)

			#define IS_SEEK(k) ((k) == GEST_SLEW || (k) == GEST_STEP)

			gesture_t next;
			int collapsed = 0;

			// COALESCE consecutive seeks into one movement to the final place.
			//
			// This is the fix for a problem no amount of policy tweaking solved:
			// 76 moves in 70 seconds were being discarded as stale, because a
			// gesture costs one to two seconds of real drive time and the model
			// emits them faster than that. Replaying a history cannot keep up
			// with a mechanism that needs 200 ms per move, so the mirror ran
			// permanently over a second late and a seek was heard against the
			// wrong thing on screen.
			//
			// The drive can only be in one place, so a queue of seeks is not
			// several movements -- it is one movement, to wherever the last of
			// them points. Summing their durations keeps what matters about a
			// burst, which is that a transition takes a long time; it just
			// arrives as one continuous grind rather than four late ones. That
			// is also closer to what the mechanism does than four discrete
			// traverses ever was.
			if (IS_SEEK(g.kind)) {
				double total = g.dur_ms;
				int    n     = 0;
				while (acu_model_poll(&model, &next)) {
					if (IS_SEEK(next.kind)) {
						total += next.dur_ms;
						g      = next;          // newest target wins
						n++;
						continue;
					}
					if (IS_UPDATE(next.kind)) { n++; continue; }
					// A real event: must not be lost, so stash it for the next
					// iteration rather than dropping it on the floor.
					stash     = next;
					has_stash = 1;
					break;
				}
				if (n) {
					g.dur_ms = total > 6000.0 ? 6000.0 : total;
					acu_log("coalesced %d into one %s of %0.0fms\n",
					        n, acu_gesture_name(g.kind), g.dur_ms);
				}
			}
			else if (IS_UPDATE(g.kind)) {
				// Scan forward for a real event. If one is queued it takes
				// priority over any number of stale position updates; if not,
				// the newest update is the only one that means anything.
				while (acu_model_poll(&model, &next)) {
					collapsed++;
					g = next;
					if (!IS_UPDATE(g.kind)) break;
				}
			}
			if (collapsed)
				acu_log("merged %d position update%s, now %s\n", collapsed,
				        collapsed == 1 ? "" : "s", acu_gesture_name(g.kind));
		}

		// Nothing to open the drive for unless there is real work.
		if (g.kind == GEST_NONE) continue;

		if (mir.dev_fd < 0) {
			if (now - open_attempt_at < 1000.0) { sleep_ms(100); continue; }
			open_attempt_at = now;
			if (mirror_acquire()) { sleep_ms(100); continue; }
		}

		double g_t0 = clock_ms();
		play_gesture(&g);
		gesture_actual_ms = clock_ms() - g_t0;
		if (g.kind != GEST_STREAM && g.kind != GEST_JUMP)
			acu_log("  %-8s took %5.0fms, model wanted %5.0fms\n",
			        acu_gesture_name(g.kind), gesture_actual_ms, g.dur_ms);
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

		// Synthesise the power-on sequence here, because the real one is always
		// missed. A core mounts its disc inside user_io_init(), and this engine
		// is not enabled until physical_disc_launch_startup() a few lines later
		// in main() -- so the mount's tray-close and TOC events are emitted
		// while the worker is still parked, and the parked worker flushes the
		// ring. The boot sweep, which is the longest travel the mechanism ever
		// makes and the most recognisable part of a console starting up, was
		// therefore never played once.
		//
		// Enabling the mirror is itself the moment a console comes on, so the
		// sequence belongs here regardless.
		physical_disc_acoustic_event(PD_ACU_TRAY_CLOSE, 0, 0);
		physical_disc_acoustic_event(PD_ACU_TOC, 0, 0);
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

	// Reject nonsense and repetition at the door.
	//
	// A CDD command handler runs on every command the core issues, and the BIOS
	// re-issues PLAY and SEEK constantly while it polls -- often with MSF values
	// below the 150-sector pregap, which come out as negative LBAs. Observed on
	// a Mega CD: a steady stream of seeks to -5, -2 and -1 among the real ones.
	//
	// Passed through, the model clamps them to zero and believes the head is
	// repeatedly slamming to the hub. That wrecks head tracking, swamps the
	// gesture queue so real movement is dropped, and clears the just-moved state
	// that the audio lock-on depends on. None of them are head movements.
	if (lba < 0) {
		if (ev == PD_ACU_SEEK || ev == PD_ACU_SCAN) return;
		lba = 0;
	}

	// The same seek re-issued every frame is one seek.
	static int last_seek_lba = INT_MIN;
	if (ev == PD_ACU_SEEK) {
		if (lba == last_seek_lba) return;
		last_seek_lba = lba;
	}

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
