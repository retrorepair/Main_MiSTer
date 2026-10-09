#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <time.h>

#include "physical_disc_rig.h"
#include "cd_geometry.h"

// Sled positions are permille of the PS1 mechanism's physical stroke, 0 at the inner limit switch.
// The disc radius the model reports (25-58 mm) is mapped straight onto it, so the sled covers the
// same FRACTION of its travel as the original drive's did of its own. Targets near the hub snap to
// the switch inside the firmware, which is also what keeps its dead-reckoned position honest.
#define SLED_MAX_PM       950
#define PORT_COUNT        8
#define PROBE_MS          500
#define REPLY_MS          400
#define HOME_MS           9000
#define RX_MAX            512
#define MIN_RAMP_MS       30
#define SPIN_RETUNE       0.04     // only re-send SPIN when the speed has moved this fraction
#define TIMEOUTS_TO_DROP  3

static struct {
	int    fd;
	char   rx[RX_MAX];
	int    rxn;

	char   reply[96];          // the last non-DONE line
	int    reply_ready;
	unsigned done_seq;         // counts DONE lines

	double spin_rpm;           // what the board was last told
	int    pos_pm;             // where it was last told to put the sled
	int    timeouts;

	int    want_smooth;       // drive style the current profile wants
	int    sent_smooth;       // the style the board was last told (-1 = not yet)

	const struct lens_policy *pol;   // what this console's lens coils do (set by rig_set_profile)
	int    noise;             // continuous coil noise level the board is running (0 = none)
	int    booted;            // the boot lens sequence has been played for this disc already
	double last_tx_ms;        // when we last sent the board anything
} rig = { -1, {0}, 0, {0}, 0, 0, 0, 0, 0, 0, -1, NULL, 0, 0, 0.0 };

static void (*g_log)(const char *line);

void rig_set_log(void (*fn)(const char *line)) { g_log = fn; }

static void rlog(const char *fmt, ...)
{
	if (!g_log) return;
	char line[200];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	g_log(line);
}

static double now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void nap_ms(double ms)
{
	if (ms <= 0.0) return;
	struct timespec ts;
	ts.tv_sec  = (time_t)(ms / 1000.0);
	ts.tv_nsec = (long)((ms - ts.tv_sec * 1000.0) * 1e6);
	nanosleep(&ts, NULL);
}

// ------------------------------------------------------- lens policy -----
//
// The board can drive the pickup's two lens coils (focus and tracking) as well as the sled and spindle,
// and the real consoles' pickups make noise with them: the PlayStation's one-kilohertz auto-gain tone,
// focus-search ramps, tracking pulling in after a seek, the broadband hiss of the servos working.
//
// WHAT IS SOURCED AND WHAT IS NOT. The existence and order of the boot steps come from the documents:
// the Mega CD 2 service manual p.10 (sled home, laser on, focus search, focus on, spindle, tracking,
// disc servo, TOC), the CXD2545Q datasheet (spindle KICK, AGCNTL's 1 kHz sine, tracking servo off during a
// sled move) and a recording of a real PlayStation starting up (the 1 kHz tone at about 3.2 s, lasting
// 0.4 s; steady broadband noise while reading). The LEVELS, LENGTHS and WHICH CONSOLE DOES WHICH are
// best guesses: nothing here was measured on any console but that one PlayStation recording, and the
// only other console recording (a Mega CD) has no lens-level detail. They are the numbers to tune when
// the rig is recorded and compared with the real thing.
//
// Levels are the firmware's LENS Z levels: 1 = about 0.3 V RMS across the coil, 2 = 0.9 V, 3 = 2.5 V
// (burst only, 1.5 s at most); the BA5977FP channels give 5 V x duty at most (datasheet).
struct lens_policy {
	int focus_ms;      // boot: focus-search ramp after the sled homes (0 = none)
	int beep;          // boot: the CXD2545Q auto-gain tone after focus is found
	int read_noise;    // continuous noise level while the disc is spinning and tracking
	int seek_noise;    // noise level while the sled travels (0 = lens quiet: tracking servo is open)
	int relock_ms;     // burst when tracking pulls back in at the end of a seek or a lock
	int kick_ms;       // spindle spin-up: full-drive KICK before the speed servo takes over
};

static const struct lens_policy policies[PD_ACU_PROFILE_COUNT] = {
	// focus beep read seek relock kick     (profile order is the pd_acoustic_profile_t order)
	{     0,  0,   0,   2,   100,   0   },   // auto: no console claimed yet, so nothing to imitate: just home
	{   300,  1,   1,   0,   150,   500 },   // PlayStation: the one with a recording behind it
	{  2000,  0,   0,   2,   100,   0   },   // Mega CD: focus search about 2 s (service manual flowchart)
	{  3000,  0,   0,   2,   100,   0   },   // Saturn: "approx. 3 seconds" of lens up and down (manual p.8)
	{  2000,  0,   0,   2,   100,   0   },   // PC Engine CD: same Sony family as the Mega CD 2
	{  1000,  0,   0,   2,   100,   0   },   // 3DO: nothing documented, generic
	{  1000,  0,   0,   2,   100,   0   },   // CD-i: nothing documented, generic
	{  2000,  0,   0,   2,   100,   0   },   // Neo Geo CD: same Sony family as the Mega CD 2
};

// ------------------------------------------------------------ serial -----

static void drop(const char *why)
{
	rlog("rig: lost the board (%s)\n", why);
	if (rig.fd >= 0) close(rig.fd);
	rig.fd = -1;
	rig.rxn = 0;
}

static void got_line(char *line)
{
	if (!strncmp(line, "DONE ", 5)) {
		rig.done_seq++;
		rlog("rig:   %s\n", line);
		if (strstr(line, " stuck")) {
			// The board gave up on an outward move because the carriage never left the hub switch.
			// Once it sat jammed for an hour while every move reported success; say so loudly.
			static double last_msg = -1e9;
			if (now_ms() - last_msg > 30000.0) {
				last_msg = now_ms();
				printf("physical_disc_acoustic: the rig sled did not leave the hub (jammed?)\n");
			}
		}
		return;
	}
	strncpy(rig.reply, line, sizeof(rig.reply) - 1);
	rig.reply[sizeof(rig.reply) - 1] = 0;
	rig.reply_ready = 1;
}

// Read whatever has arrived, waiting up to timeout_ms for the first byte.
static void pump(int timeout_ms)
{
	if (rig.fd < 0) return;

	struct pollfd p = { rig.fd, POLLIN, 0 };
	int r = poll(&p, 1, timeout_ms);
	if (r <= 0) return;
	if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) { drop("poll"); return; }

	char buf[128];
	ssize_t n = read(rig.fd, buf, sizeof(buf));
	if (n < 0) {
		if (errno != EAGAIN && errno != EINTR) drop("read");
		return;
	}
	if (n == 0) { drop("eof"); return; }

	for (ssize_t i = 0; i < n; i++) {
		char c = buf[i];
		if (c == '\r') continue;
		if (c == '\n') {
			rig.rx[rig.rxn] = 0;
			if (rig.rxn) got_line(rig.rx);
			rig.rxn = 0;
		}
		else if (rig.rxn < RX_MAX - 1) rig.rx[rig.rxn++] = c;
	}
}

static int send_line(const char *line)
{
	if (rig.fd < 0) return -1;
	char buf[96];
	rig.last_tx_ms = now_ms();
	int n = snprintf(buf, sizeof(buf), "%s\n", line);
	for (int tries = 0; tries < 50; tries++) {
		ssize_t w = write(rig.fd, buf, n);
		if (w == n) return 0;
		if (w < 0 && errno != EAGAIN && errno != EINTR) { drop("write"); return -1; }
		nap_ms(2);
	}
	drop("write stuck");
	return -1;
}

// Send a command and wait for its one-line reply (async DONE lines are absorbed on the way).
static const char *cmd(const char *line, int timeout_ms)
{
	if (rig.fd < 0) return NULL;
	rig.reply_ready = 0;
	if (send_line(line)) return NULL;

	double end = now_ms() + timeout_ms;
	while (!rig.reply_ready && rig.fd >= 0) {
		double left = end - now_ms();
		if (left <= 0.0) break;
		pump((int)left > 20 ? 20 : (int)left);
	}
	if (!rig.reply_ready) {
		if (rig.fd >= 0 && ++rig.timeouts >= TIMEOUTS_TO_DROP) drop("no replies");
		return NULL;
	}
	rig.timeouts = 0;
	return rig.reply;
}

int rig_connected(void) { return rig.fd >= 0; }

// The board switches everything off if the host goes quiet (a core that exits leaves nothing to stop it),
// so while we are alive say something at least every couple of seconds.
#define KEEPALIVE_MS  2000
void rig_keepalive(void)
{
	if (rig.fd < 0) return;
	if (now_ms() - rig.last_tx_ms < KEEPALIVE_MS) return;
	cmd("PING", REPLY_MS);
}

static int vendor_ok(int n)
{
	// CircuitPython boards show up as Adafruit (239a) or Raspberry Pi (2e8a). Skip anything else
	// so we never send PING down some unrelated serial device. If sysfs will not say, try it.
	char path[96], v[16] = "";
	snprintf(path, sizeof(path), "/sys/class/tty/ttyACM%d/device/../idVendor", n);
	FILE *f = fopen(path, "r");
	if (!f) return 1;
	if (!fgets(v, sizeof(v), f)) v[0] = 0;
	fclose(f);
	return !strncmp(v, "239a", 4) || !strncmp(v, "2e8a", 4) || !v[0];
}

int rig_connect(void)
{
	if (rig.fd >= 0) return 0;

	// PD_RIG_PORT pins one device path; it exists so the translator can be tested against a pty
	// on a development machine, and is not something a MiSTer has any reason to set.
	const char *forced = getenv("PD_RIG_PORT");

	for (int n = 0; n < (forced ? 1 : PORT_COUNT); n++) {
		if (!forced && !vendor_ok(n)) continue;

		char path[64];
		if (forced) snprintf(path, sizeof(path), "%s", forced);
		else        snprintf(path, sizeof(path), "/dev/ttyACM%d", n);
		int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
		if (fd < 0) continue;

		struct termios t;
		if (tcgetattr(fd, &t) == 0) {
			cfmakeraw(&t);
			cfsetispeed(&t, B115200);
			cfsetospeed(&t, B115200);
			t.c_cflag |= CLOCAL | CREAD;
			tcsetattr(fd, TCSANOW, &t);
		}
		tcflush(fd, TCIOFLUSH);

		rig.fd = fd;
		rig.rxn = 0;
		rig.timeouts = 0;

		const char *r = cmd("PING", PROBE_MS);
		if (r && !strncmp(r, "OK servo", 8)) {
			char who[sizeof(rig.reply)];
			snprintf(who, sizeof(who), "%s", r);

			// Whatever it was doing when we found it, start from a known quiet state.
			cmd("STOP", REPLY_MS);
			rig.noise = 0;
			rig.booted = 0;
			rig.spin_rpm = 0.0;
			rig.pos_pm   = 0;
			rig.sent_smooth = -1;
			rlog("rig: %s answered \"%s\"\n", path, who);
			printf("physical_disc_acoustic: noise rig found on %s (%s)\n", path, who);
			return 0;
		}

		if (rig.fd >= 0) close(rig.fd);
		rig.fd = -1;
	}
	return -1;
}

void rig_disconnect(int park)
{
	if (rig.fd < 0) return;
	if (park) cmd("STOP", REPLY_MS);
	rig.noise = 0;
	rig.booted = 0;
	if (rig.fd >= 0) close(rig.fd);
	rig.fd = -1;
	rig.rxn = 0;
}

// ---------------------------------------------------------- gestures -----

static int stroke_pm(int lba)
{
	double r = cd_geom_radius_mm(lba);
	double u = (r - CD_R_INNER_MM) / (CD_R_OUTER_MM - CD_R_INNER_MM);
	int pm = (int)(u * 1000.0 + 0.5);
	if (pm < 0) pm = 0;
	if (pm > SLED_MAX_PM) pm = SLED_MAX_PM;
	return pm;
}

// Wait until `end` (a now_ms() value), absorbing the board's chatter. Returns 1 if aborted.
static int wait_until(double end, int (*aborted)(void))
{
	while (rig.fd >= 0) {
		if (aborted && aborted()) return 1;
		double left = end - now_ms();
		if (left <= 0.0) return 0;
		pump((int)left > 20 ? 20 : (int)left);
	}
	return 0;
}

// Continuous broadband noise on both lens coils at `level`, or none at 0. Costs a command only on a change.
static void lens_noise(int level)
{
	if (level == rig.noise) return;
	char line[32];
	if (level > 0) snprintf(line, sizeof(line), "LENS B Z %d 0", level);
	else           snprintf(line, sizeof(line), "LENS OFF");
	if (cmd(line, REPLY_MS)) rig.noise = level;
}

// A burst of noise on both coils for `ms`. The board ends it by itself and is silent afterwards, so
// whatever continuous noise was running is gone and the caller puts it back when the burst is over.
static void lens_burst(int level, double ms)
{
	if (level <= 0 || ms < 20.0) return;
	char line[32];
	snprintf(line, sizeof(line), "LENS B Z %d %d", level, (int)ms);
	cmd(line, REPLY_MS);
	rig.noise = 0;
}

// The lens part of a boot, after the sled has found the hub: the focus-search ramp and, on a
// PlayStation, the auto-gain tone. Returns 1 if aborted.
static int lens_boot(int (*aborted)(void))
{
	const struct lens_policy *p = rig.pol;
	char line[48];
	rig.noise = 0;
	// A drive starts up once per disc. The core announces the mount more than once (before and after it
	// names its console, and again later), and each announcement is a TRAY_CLOSE, so without this the
	// focus search and the tone play two or three times in a row. Only a console's own policy counts:
	// the generic one has nothing to play and must not use up the boot.
	if (rig.booted || (p->focus_ms <= 0 && !p->beep)) return 0;
	rig.booted = 1;
	if (p->focus_ms > 0) {
		snprintf(line, sizeof(line), "LENS F S 1 %d", p->focus_ms);
		cmd(line, REPLY_MS);
		if (wait_until(now_ms() + p->focus_ms, aborted)) return 1;
	}
	if (p->beep) {
		// CXD2545Q AGCNTL: a 1 kHz sine into the loop. The recording has it at 1004 Hz for 0.4 s. A square
		// pulse train sounded far too sharp on the rig, so it is a triangle: peak 0.5 of the 5 V swing,
		// which has the same fundamental as the pulse train it replaced.
		cmd("LENS F G 0.5 400 1000", REPLY_MS);
		if (wait_until(now_ms() + 400.0, aborted)) return 1;
	}
	return 0;
}

// Drive the sled to the inner switch and wait for the board to say it got there.
// 0 = homed, 1 = aborted, -1 = no answer.
static int do_home(int (*aborted)(void))
{
	unsigned seq = rig.done_seq;
	if (!cmd("HOME", REPLY_MS)) return -1;

	double end = now_ms() + HOME_MS;
	while (rig.fd >= 0 && rig.done_seq == seq && now_ms() < end) {
		if (aborted && aborted()) return 1;
		pump(20);
	}
	return rig.done_seq == seq ? -1 : 0;
}

// Ask for the spindle to ramp to `rpm` over `ramp_ms`. Skipped when it is already there.
// From a standstill with kick_ms > 0 the first part of the time is the CXD2545Q's KICK (full forward
// drive until the speed is near) and the rest the ramp to the target.
static void do_spin(double rpm, double ramp_ms, int force, int kick_ms)
{
	if (!force && rpm > 0.0 && fabs(rpm - rig.spin_rpm) < SPIN_RETUNE * rpm) return;
	if (!force && rpm <= 0.0 && rig.spin_rpm <= 0.0) return;

	char line[64];
	if (ramp_ms < MIN_RAMP_MS) ramp_ms = MIN_RAMP_MS;
	if (kick_ms > 0 && rpm > 0.0 && rig.spin_rpm <= 0.0) {
		if (kick_ms > ramp_ms * 0.6) kick_ms = (int)(ramp_ms * 0.6);
		snprintf(line, sizeof(line), "SPIN %d %d %d", (int)(rpm + 0.5), (int)(ramp_ms - kick_ms), kick_ms);
	}
	else
		snprintf(line, sizeof(line), "SPIN %d %d", (int)(rpm + 0.5), (int)ramp_ms);
	if (cmd(line, REPLY_MS)) rig.spin_rpm = rpm;
}

// Start a sled move; if the board has lost track of where the sled is, home it first and spend what
// is left of the time on the move. Returns 0 when a move (or a deliberate skip) was started.
static int do_move(int pm, double dur_ms, double t0, int (*aborted)(void), int allow_home)
{
	char line[48];
	snprintf(line, sizeof(line), "MOVE %d %d", pm, (int)dur_ms);
	const char *r = cmd(line, REPLY_MS);
	if (!r) return -1;

	if (!strcmp(r, "ERR notknown")) {
		if (!allow_home || do_home(aborted) != 0) return -1;
		double left = dur_ms - (now_ms() - t0);
		if (left < 120.0) return 0;      // the homing run used up the time
		snprintf(line, sizeof(line), "MOVE %d %d", pm, (int)left);
		r = cmd(line, REPLY_MS);
		if (!r) return -1;
	}
	rig.pos_pm = pm;
	return strncmp(r, "OK", 2) ? -1 : 0;
}

void rig_set_profile(int profile)
{
	// The PlayStation's sled is driven by a plain, high-frequency PWM at a fixed level (CXD2545Q p.78)
	// and sounds like a clean whirr. The Mega CD's rough drag is the Mega CD's own.
	rig.want_smooth = (profile == PD_ACU_PROFILE_PSX) ? 1 : 0;
	if (profile < 0 || profile >= PD_ACU_PROFILE_COUNT) profile = PD_ACU_PROFILE_AUTO;
	rig.pol = &policies[profile];
}

// Make the board's drive style match the profile; costs one command, and only when it has changed.
static void sync_style(void)
{
	if (rig.sent_smooth == rig.want_smooth) return;
	char line[32];
	snprintf(line, sizeof(line), "TEX smooth %d", rig.want_smooth);
	if (cmd(line, REPLY_MS)) rig.sent_smooth = rig.want_smooth;
}

void rig_play(const gesture_t *g, int (*aborted)(void))
{
	if (rig.fd < 0) return;
	if (!rig.pol) rig.pol = &policies[PD_ACU_PROFILE_AUTO];
	const struct lens_policy *pol = rig.pol;
	sync_style();

	double t0  = now_ms();
	double dur = g->dur_ms;
	if (dur < 0.0) dur = 0.0;
	if (dur > 8000.0) dur = 8000.0;
	double end = t0 + dur;

	switch (g->kind) {

	case GEST_JUMP:
		// The lens covers it; the sled does not move.
		break;

	case GEST_HOLD:
		// Spindle turning, head parked, servos still tracking.
		lens_noise(pol->read_noise);
		break;

	case GEST_STREAM: {
		// Reading: the spindle follows the CLV glide, and the sled creeps outward. Both are
		// small. The creep is only worth a command once it has added up to something.
		lens_noise(pol->read_noise);
		do_spin(g->rpm, 250.0, 0, 0);
		int pm = stroke_pm(g->lba);
		if (abs(pm - rig.pos_pm) >= 40 && pm > 0)
			do_move(pm, 700.0, t0, aborted, 0);   // never worth a homing run
		break;
	}

	case GEST_STEP:
	case GEST_SLEW: {
		// The spindle glides to the speed of the destination while the sled travels, which is the
		// pitch swoop a CLV drive makes crossing the disc.
		do_spin(g->rpm, dur, 1, 0);
		// The tracking servo is open while the sled travels, so on a PlayStation the lens is quiet; the
		// others hiss. Tracking pulling back in at the end is a burst.
		double relock = pol->relock_ms;
		if (relock > dur * 0.3) relock = dur * 0.3;
		if (pol->seek_noise) lens_burst(pol->seek_noise, dur - relock);
		else                 lens_noise(0);
		if (do_move(stroke_pm(g->lba), dur, t0, aborted, 1)) break;
		if (wait_until(end - relock, aborted)) break;
		lens_burst(3, relock);
		if (wait_until(end, aborted)) break;
		lens_noise(pol->read_noise);
		break;
	}

	case GEST_LOCK: {
		// Focus and spin-lock: a lens operation, no sled. Tracking pulls in with a burst, then the
		// servo settles into its steady noise.
		double relock = pol->relock_ms;
		if (relock > dur) relock = dur;
		lens_burst(3, relock);
		if (wait_until(t0 + relock, aborted)) break;
		lens_noise(pol->read_noise);
		wait_until(end, aborted);
		break;
	}

	case GEST_SPINUP:
		do_spin(g->rpm, dur, 1, pol->kick_ms);
		wait_until(end, aborted);
		break;

	case GEST_SPINDOWN:
		lens_noise(0);
		do_spin(0.0, dur, 1, 0);
		wait_until(end, aborted);
		break;

	case GEST_SWEEP: {
		// The deck calibrates: in to the hub, out to the rim, back. Two equal legs after the homing.
		lens_noise(0);
		if (do_home(aborted) != 0) break;
		if (lens_boot(aborted)) break;
		if (g->home_only) {   // a PlayStation only finds its innermost track
			wait_until(end, aborted);
			break;
		}
		double left = dur - (now_ms() - t0);
		if (left < 400.0) break;
		double leg = left / 2.0;
		double t1  = now_ms();
		if (!do_move(SLED_MAX_PM, leg, t1, aborted, 1)) wait_until(t1 + leg, aborted);
		t1 = now_ms();
		if (!do_move(0, leg, t1, aborted, 1)) wait_until(t1 + leg, aborted);
		break;
	}

	case GEST_PARK:
		rig.booted = 0;            // the tray is opening: the next disc starts up afresh
		lens_noise(0);
		do_spin(0.0, dur, 1, 0);
		if (!do_move(0, dur, t0, aborted, 1)) wait_until(end, aborted);
		break;

	default:
		break;
	}
}
