#ifndef CD_GEOMETRY_INCLUDED
#define CD_GEOMETRY_INCLUDED

// Physical geometry of a Red Book disc, and the drive behaviour that follows
// from it. Everything the acoustic mirror does is derived from here: where the
// sled physically is, how far it has to travel, how fast the spindle turns at
// that radius, and whether a given move is a silent lens jump or an audible
// sled slew.
//
// The spiral is a constant-linear-density track, so sector number maps to
// radius as a square root, not linearly. Track length from r0 out to r is
// pi*(r^2 - r0^2)/pitch, and each sector occupies v/75 of that length, giving
//
//     r(lba) = sqrt(r0^2 + lba * pitch * (v/75) / pi)
//
// With the Red Book nominals below this puts LBA 333000 at r = 57.8 mm, i.e.
// the outer edge of the program area, and reproduces the measured
// sectors-per-revolution curve in pcecd/seektime.cpp to within a count.

#define CD_R_INNER_MM      25.0   // program area starts at 25 mm (Red Book)
#define CD_R_OUTER_MM      58.0   // and may run out to 58 mm
#define CD_TRACK_PITCH_MM  0.0016 // 1.6 um between spiral turns
#define CD_SECTOR_LEN_MM   16.0   // 1.2 m/s / 75 Hz = 16 mm of track per sector

// Radius in mm at which a given LBA sits.
double cd_geom_radius_mm(int lba);

// Inverse: the LBA that lands at a given radius.
int cd_geom_lba_at_radius(double r_mm);

// Sectors per revolution at that LBA (the circumference divided by the track
// length of one sector). Matches Dave Shadoff's measured table: 10 at the
// hub, 23 at the rim.
double cd_geom_sectors_per_rev(int lba);

// Spindle speed in RPM for a CLV drive running at `speed_mult` times Red Book
// at that LBA. This is the pitch glide you hear when a drive seeks: a CLV
// spindle has to spin more than twice as fast at the hub as at the rim.
double cd_geom_rpm(int lba, double speed_mult);

// How many spiral turns lie between two LBAs. This, not the sector delta, is
// what decides how a drive services a move.
double cd_geom_track_delta(int from_lba, int to_lba);

// Radial distance the sled must cover, in mm.
double cd_geom_radial_delta_mm(int from_lba, int to_lba);

#endif
