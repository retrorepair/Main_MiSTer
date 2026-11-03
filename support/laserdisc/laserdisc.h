#ifndef LASERDISC_H
#define LASERDISC_H

// Laserdisc_MiSTer: feeds a DSCV stream (picture + sound per frame) to the core through DDR3.
//
// The core's player chooses the frame; this side keeps the 16 slots at 0x30100000 filled with the frames it
// will want, read from the status block the core writes at every frame boundary. The layout is the one in the
// core's rtl/ld_stream.sv header (Laserdisc_MiSTer docs/streaming_design.md).

void laserdisc_set_image(const char *path);   // "" or NULL: remove the disc
void laserdisc_poll();                        // call often from the main loop

#endif
