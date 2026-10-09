#ifndef PHYSICAL_DISC_RIG_INCLUDED
#define PHYSICAL_DISC_RIG_INCLUDED

#include "acoustic_model.h"

// The noise rig: a PlayStation optical block with no laser reading anything, its sled and spindle
// motors driven through the PS1's own BA5977FP driver chip by an RP2040 running
// support/physical_disc/rp2040/servo_fw.py. It replaces the USB drive as the thing that makes the
// noise, and takes the same gestures from the same acoustic model -- but because it is a purpose
// built mechanism there is no need to translate them into SCSI commands that only approximate
// what is wanted. The sled is told where to be and how long it has to get there.
//
// The board speaks ASCII lines over the second USB serial port CircuitPython exposes
// (/dev/ttyACM* on the MiSTer). See rp2040/servo_fw.py for the protocol.

// Find the board (any /dev/ttyACM* that answers PING with "OK servo") and open it. 0 on success.
int  rig_connect(void);
int  rig_connected(void);

// Let go of the board. `park` sends STOP first so nothing is left spinning or driven.
void rig_disconnect(int park);

// Play one gesture in real time: send the commands, then wait out the time the original
// mechanism would have taken, so the next gesture starts when it should. `aborted` is polled while
// waiting and ends the wait early when it returns non-zero.
void rig_play(const gesture_t *g, int (*aborted)(void));

// Tell the rig which console's drive is being imitated (a pd_acoustic_profile_t). The PlayStation's
// sled is driven smoothly, with a kick and a brake, where the Mega CD's is a rough drag, so the board's
// drive style follows the profile.
void rig_set_profile(int profile);

// Optional trace sink; the player points this at its own log.
void rig_set_log(void (*fn)(const char *line));

#endif
