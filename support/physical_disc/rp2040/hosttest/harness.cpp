// Host test of the rig translator: feed a Mega CD style sequence through the real model and play the
// gestures it emits through rig_play(), timing each one.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "acoustic_model.h"
#include "physical_disc_rig.h"

static double now_ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1000.0 + t.tv_nsec / 1e6; }
static int never(void) { return 0; }
static void logsink(const char *l) { fputs(l, stdout); }

static acu_model_t m;

static void drain(const char *what)
{
	gesture_t g;
	while (acu_model_poll(&m, &g)) {
		if (g.kind == GEST_NONE) continue;
		double t0 = now_ms();
		printf(">> %-8s %-6s lba %6d  model %6.0f ms  rpm %5.1f\n", what, acu_gesture_name(g.kind), g.lba, g.dur_ms, g.rpm);
		rig_play(&g, never);
		printf("   played in %6.0f ms\n", now_ms() - t0);
	}
}

int main(int argc, char **argv)
{
	rig_set_log(logsink);
	if (rig_connect()) { printf("no board\n"); return 1; }
	// ./rigtest psx plays the PlayStation profile; the default is the Mega CD
	pd_acoustic_profile_t prof = (argc > 1 && !strcmp(argv[1], "psx")) ? PD_ACU_PROFILE_PSX : PD_ACU_PROFILE_MEGACD;
	rig_set_profile(prof);
	acu_model_init(&m, prof);

	double t = 1000.0;
	// power-on: tray closes, TOC read
	acu_model_event(&m, t, PD_ACU_TRAY_CLOSE, 0, 0);
	acu_model_event(&m, t, PD_ACU_TOC, 0, 0);
	drain("boot");

	// a data read near the start, then the Sonic CD style jump to a CDDA track out at the rim
	t += 500; acu_model_event(&m, t, PD_ACU_READ, 4000, 16);  drain("read");
	t += 500; acu_model_event(&m, t, PD_ACU_SEEK, 190000, 0); drain("seek");
	t += 100; acu_model_event(&m, t, PD_ACU_PLAY, 190000, 8); drain("play");
	for (int i = 0; i < 6; i++) { t += 200; acu_model_event(&m, t, PD_ACU_PLAY, 190000 + 16 * (i + 1), 16); acu_model_tick(&m, t); drain("stream"); }
	// back to the data area
	t += 300; acu_model_event(&m, t, PD_ACU_SEEK, 4000, 0);  drain("seek-back");
	t += 100; acu_model_event(&m, t, PD_ACU_READ, 4000, 16); drain("read");
	rig_disconnect(1);
	return 0;
}
