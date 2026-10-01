#ifndef ACOUSTIC_MODEL_INCLUDED
#define ACOUSTIC_MODEL_INCLUDED

#include "physical_disc_acoustic.h"

// The model of the original console's drive. Pure logic -- no I/O, no threads,
// no Linux headers -- so it can be exercised on a host. It is fed the disc
// activity the core is performing and emits the sequence of physical gestures
// the original drive would have made. physical_disc_acoustic.cpp replays those
// gestures on the real USB drive.

typedef enum {
	GEST_NONE = 0,
	GEST_SLEW,      // long sled travel: coarse move, several audible stages
	GEST_STEP,      // short sled move: one chirp
	GEST_JUMP,      // lens-only jump: the sled does not move, no new noise
	GEST_STREAM,    // sustained read, head creeping outward under CLV
	GEST_HOLD,      // spindle turning, head parked on track, no reads
	GEST_SPINUP,
	GEST_SPINDOWN,
	GEST_SWEEP,     // full-stroke calibration pass on disc insert
	GEST_PARK       // sled returned to the hub, tray about to move
} gesture_kind_t;

typedef struct {
	gesture_kind_t kind;
	int    from_lba;
	int    lba;             // target, in the GAME's LBA space
	double dur_ms;          // how long the original drive would take
	double rpm;             // modelled spindle speed at the target
	double turns;           // spiral turns crossed
	double radial_mm;       // sled travel
	int    stages;          // sled sub-moves for a slew (coarse then fine)
	double rate_sectors_s;  // for STREAM: how fast the head creeps
	int    sectors;         // for STREAM: how far it runs before re-evaluating
} gesture_t;

typedef struct {
	const char *name;
	double data_speed;        // CLV multiple for data reads
	double audio_speed;       // CLV multiple for Red Book audio (always 1)
	int    lens_jump_turns;   // at or below this the sled stays put: silent
	int    short_seek_turns;  // above this it becomes a staged coarse slew
	int    settle_ms;         // servo re-lock after the sled stops
	int    short_seek_ms;     // a one-chirp step
	int    full_stroke_ms;    // hub to rim
	int    spinup_ms;
	int    spindown_idle_ms;  // idle time before the spindle gives up
	int    readahead_sectors; // drive buffer: how much it grabs per burst
	int    calib_sweep;       // sweeps the sled when a disc is loaded
} acu_drive_t;

typedef struct {
	acu_drive_t drive;
	pd_acoustic_profile_t profile;

	int    head_lba;          // where the modelled sled is
	int    spinning;
	int    streaming;
	double stream_rate;       // measured sectors/s
	double stream_mult;       // CLV multiple currently in use
	int    stream_anchor_lba;
	double stream_anchor_ms;
	double last_event_ms;
	double last_emit_ms;
	int    pending_stream;
	int    holding;

	gesture_t out[8];
	int    out_head, out_tail;
} acu_model_t;

const acu_drive_t *acu_model_drive(pd_acoustic_profile_t profile);

void acu_model_init(acu_model_t *m, pd_acoustic_profile_t profile);
void acu_model_event(acu_model_t *m, double now_ms, pd_acoustic_event_t ev, int lba, int count);

// Lets the model emit time-driven gestures (spin-down, stream refresh) when no
// events are arriving. Call regularly.
void acu_model_tick(acu_model_t *m, double now_ms);

// Returns 1 and fills `out` when a gesture is queued.
int  acu_model_poll(acu_model_t *m, gesture_t *out);

const char *acu_gesture_name(gesture_kind_t k);

// Seek time of the modelled drive between two LBAs.
double acu_model_seek_ms(const acu_drive_t *d, int from_lba, int to_lba);

#endif
