// Drives the acoustic model with a realistic PSX boot-and-load trace and
// prints the gesture timeline, plus the radial mapping the player would use.
// Nothing here touches a drive; it exists so the behaviour can be inspected
// without a MiSTer in front of you.
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../../support/physical_disc/acoustic_model.h"
#include "../../support/physical_disc/cd_geometry.h"

// Same mapping physical_disc_acoustic.cpp uses, reproduced so the test can
// show where the mirror head would actually go on a 333000-sector CD-R.
#define MEDIA_R_IN  24.0
#define MEDIA_R_OUT 58.0
static double media_full = 333000.0;
static int    span_lo = 0, span_hi = 333000 - 64;

static double media_radius(int lba) {
	double f = lba / media_full;
	if (f < 0) f = 0;
	if (f > 1) f = 1;
	return sqrt(MEDIA_R_IN*MEDIA_R_IN + f*(MEDIA_R_OUT*MEDIA_R_OUT - MEDIA_R_IN*MEDIA_R_IN));
}
static int media_lba_at_radius(double r) {
	if (r < MEDIA_R_IN) r = MEDIA_R_IN;
	if (r > MEDIA_R_OUT) r = MEDIA_R_OUT;
	double f = (r*r - MEDIA_R_IN*MEDIA_R_IN) / (MEDIA_R_OUT*MEDIA_R_OUT - MEDIA_R_IN*MEDIA_R_IN);
	return (int)(f * media_full + 0.5);
}
static int map_new(int game_lba) {
	double r = cd_geom_radius_mm(game_lba);
	double u = (r - CD_R_INNER_MM) / (CD_R_OUTER_MM - CD_R_INNER_MM);
	if (u < 0) u = 0;
	if (u > 1) u = 1;
	double rlo = media_radius(span_lo), rhi = media_radius(span_hi);
	int lba = media_lba_at_radius(rlo + u * (rhi - rlo));
	if (lba < span_lo) lba = span_lo;
	if (lba > span_hi) lba = span_hi;
	return lba;
}
// What the previous implementation did: a flat sector-space scaling against a
// hardcoded 360000-sector reference.
static int map_old(int game_lba) {
	if (game_lba < 0) game_lba = 0;
	if (game_lba > 360000) game_lba = 360000;
	return span_lo + (int)((long long)game_lba * (span_hi - span_lo) / 360000);
}

static acu_model_t m;
static double t = 0;

static void drain(void) {
	gesture_t g;
	while (acu_model_poll(&m, &g)) {
		if (g.kind == GEST_JUMP) {
			printf("%8.0f  %-8s                       %7.1f turns %6.3f mm   (sled still)\n",
			       t, acu_gesture_name(g.kind), g.turns, g.radial_mm);
			continue;
		}
		if (g.kind == GEST_STREAM) {
			printf("%8.0f  %-8s lba %6d          %6.0f sec/s  %5.0f rpm  %5.0f ms\n",
			       t, acu_gesture_name(g.kind), g.lba, g.rate_sectors_s, g.rpm, g.dur_ms);
			continue;
		}
		printf("%8.0f  %-8s %6d -> %6d  %8.1f turns %6.3f mm  %5.0f rpm  %5.0f ms  x%d\n",
		       t, acu_gesture_name(g.kind), g.from_lba, g.lba,
		       g.turns, g.radial_mm, g.rpm, g.dur_ms, g.stages);
	}
}

static void ev(pd_acoustic_event_t e, int lba, int count) {
	acu_model_event(&m, t, e, lba, count);
	drain();
}
static void adv(double ms) { t += ms; acu_model_tick(&m, t); drain(); }

// A sequential read of `n` sectors from `lba`, delivered the way the core
// really delivers them: a burst at a time, as fast as the HPS can manage.
static void seq_read(int lba, int n, int burst, double per_burst_ms) {
	for (int i = 0; i < n; i += burst) {
		int c = (n - i < burst) ? n - i : burst;
		acu_model_event(&m, t, PD_ACU_READ, lba + i, c);
		drain();
		t += per_burst_ms;
	}
	drain();
}

int main()
{
	printf("=== PSX boot and load, gesture timeline ===\n");
	printf("   time   gesture   game lba move        spiral turns  sled   spindle  dur\n");
	acu_model_init(&m, PD_ACU_PROFILE_PSX);

	ev(PD_ACU_TRAY_CLOSE, 0, 0);     adv(1600);
	ev(PD_ACU_TOC, 0, 0);            adv(400);

	// BIOS reads the licence string at the very start of the disc.
	seq_read(150, 16, 8, 12); adv(120);

	// ISO9660 primary volume descriptor, then the root directory.
	ev(PD_ACU_SEEK, 166, 0);         adv(60);
	seq_read(166, 4, 4, 12);         adv(80);
	seq_read(172, 24, 8, 12);        adv(150);

	// SYSTEM.CNF, then the boot executable out in the middle of the disc.
	ev(PD_ACU_SEEK, 198, 0);         adv(40);
	seq_read(198, 2, 2, 12);         adv(100);
	printf("--- boot exe: the long one, hub out to mid-disc ---\n");
	ev(PD_ACU_SEEK, 124000, 0);      adv(60);
	seq_read(124000, 600, 8, 10);    adv(200);

	printf("--- in-game: a filesystem walk, lots of small hops ---\n");
	int spots[] = { 124300, 124180, 131400, 131405, 131960, 118020, 118024, 162700 };
	for (unsigned i = 0; i < sizeof(spots)/sizeof(spots[0]); i++) {
		seq_read(spots[i], 24, 8, 10);
		adv(45);
	}

	printf("--- in-game: a CDDA track streaming at 1x ---\n");
	int a = 288000;
	for (int i = 0; i < 12; i++) { acu_model_event(&m, t, PD_ACU_PLAY, a, 1); a++; t += 13.3; }
	drain();
	adv(300);

	printf("--- player pauses, then the disc goes idle ---\n");
	ev(PD_ACU_PAUSE, a, 0);
	adv(2000);
	ev(PD_ACU_STOP, a, 0);
	adv(200);

	printf("\n=== radial mapping: new vs old, on a full CD-R mirror ===\n");
	printf("  game lba   r_mm    new mirror lba   old mirror lba   sled error\n");
	int pts[] = { 150, 1000, 20000, 60000, 124000, 200000, 288000, 330000 };
	for (unsigned i = 0; i < sizeof(pts)/sizeof(pts[0]); i++) {
		int nl = map_new(pts[i]), ol = map_old(pts[i]);
		double rn = media_radius(nl), ro = media_radius(ol);
		printf("  %8d  %5.2f   %12d   %14d   %+6.2f mm\n",
		       pts[i], cd_geom_radius_mm(pts[i]), nl, ol, ro - rn);
	}

	printf("\n=== seek times, modelled drives ===\n");
	printf("  move                         ");
	for (int p = 1; p < PD_ACU_PROFILE_COUNT; p++)
		printf("%9s", acu_model_drive((pd_acoustic_profile_t)p)->name);
	printf("\n");
	struct { int a, b; const char *n; } mv[] = {
		{ 1000, 1050,     "50 sectors (lens jump)" },
		{ 1000, 1500,     "500 sectors" },
		{ 20000, 40000,   "20k sectors" },
		{ 150, 300000,    "hub to rim (full stroke)" },
	};
	for (unsigned i = 0; i < sizeof(mv)/sizeof(mv[0]); i++) {
		printf("  %-28s", mv[i].n);
		for (int p = 1; p < PD_ACU_PROFILE_COUNT; p++)
			printf("%7.0fms", acu_model_seek_ms(acu_model_drive((pd_acoustic_profile_t)p), mv[i].a, mv[i].b));
		printf("\n");
	}
	return 0;
}
