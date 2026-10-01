#ifndef PHYSICAL_DISC_ACOUSTIC_INCLUDED
#define PHYSICAL_DISC_ACOUSTIC_INCLUDED

// Acoustic mirroring: drive a spare disc in the USB drive so that it makes the
// noises the original console's drive would have made for the disc activity
// the core is actually performing.
//
// The cores report what their emulated drive is doing through
// physical_disc_acoustic_event(). The engine keeps a model of the original
// drive (sled position, spindle, servo state) and replays that model on the
// real drive, mapping by RADIUS rather than by sector so that sled travel and
// the CLV spindle pitch glide both come out right.

typedef enum {
	PD_ACU_PROFILE_AUTO = 0,
	PD_ACU_PROFILE_PSX,
	PD_ACU_PROFILE_MEGACD,
	PD_ACU_PROFILE_SATURN,
	PD_ACU_PROFILE_PCECD,
	PD_ACU_PROFILE_3DO,
	PD_ACU_PROFILE_CDI,
	PD_ACU_PROFILE_NEOGEO,
	PD_ACU_PROFILE_COUNT
} pd_acoustic_profile_t;

typedef enum {
	PD_ACU_SEEK = 0,  // the core commanded a reposition to `lba`
	PD_ACU_READ,      // `count` data sectors were delivered starting at `lba`
	PD_ACU_PLAY,      // CDDA / sequential playback is passing `lba`
	PD_ACU_SCAN,      // fast-forward or rewind scanning through `lba`
	PD_ACU_PAUSE,     // held in place, spindle still turning
	PD_ACU_STOP,      // spindle stopped, sled parked
	PD_ACU_TOC,       // lead-in is being read
	PD_ACU_SPINUP,    // disc spun up (boot, lid close, resume)
	PD_ACU_TRAY_OPEN,
	PD_ACU_TRAY_CLOSE
} pd_acoustic_event_t;

void physical_disc_acoustic_config(int enabled);

// Hard interlock. While a physical disc session owns the drive the mirror must
// not touch it at all: that drive is holding the user's actual game disc, and
// its noise is already genuine. Called with 1 when physical_disc takes the
// device and 0 when it gives it back. Setting it to 1 blocks briefly until the
// mirror has confirmed it let go, so the caller can then open the drive safely.
void physical_disc_acoustic_set_physical(int phys);

// Pick the drive being imitated. Called when a core mounts a disc; AUTO leaves
// whatever the last core set.
void physical_disc_acoustic_set_profile(pd_acoustic_profile_t profile);

// The one call the cores make. `lba` is the disc LBA the emulated drive is at,
// `count` the number of sectors for READ/PLAY (0 where it does not apply).
void physical_disc_acoustic_event(pd_acoustic_event_t ev, int lba, int count);

// Pre-existing single-LBA hint, kept so untouched call sites still work.
void physical_disc_acoustic_hint(int lba);

void physical_disc_acoustic_pause(void);
void physical_disc_acoustic_resume(void);

#endif
