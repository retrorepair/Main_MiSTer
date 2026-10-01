#include <math.h>
#include "cd_geometry.h"

// pitch * sector_length / pi, the constant in the spiral equation.
#define SPIRAL_K (CD_TRACK_PITCH_MM * CD_SECTOR_LEN_MM / M_PI)

double cd_geom_radius_mm(int lba)
{
	if (lba < 0) lba = 0;
	double r = sqrt(CD_R_INNER_MM * CD_R_INNER_MM + (double)lba * SPIRAL_K);
	if (r > CD_R_OUTER_MM) r = CD_R_OUTER_MM;
	return r;
}

int cd_geom_lba_at_radius(double r_mm)
{
	if (r_mm < CD_R_INNER_MM) r_mm = CD_R_INNER_MM;
	if (r_mm > CD_R_OUTER_MM) r_mm = CD_R_OUTER_MM;
	double lba = (r_mm * r_mm - CD_R_INNER_MM * CD_R_INNER_MM) / SPIRAL_K;
	if (lba < 0) lba = 0;
	return (int)(lba + 0.5);
}

double cd_geom_sectors_per_rev(int lba)
{
	return 2.0 * M_PI * cd_geom_radius_mm(lba) / CD_SECTOR_LEN_MM;
}

double cd_geom_rpm(int lba, double speed_mult)
{
	// CLV: linear velocity is fixed, so angular velocity falls as 1/r.
	// 75 sectors/s at 1x, each sector one sector-length of track.
	double v_mm_s = 75.0 * CD_SECTOR_LEN_MM * speed_mult;
	double r = cd_geom_radius_mm(lba);
	return v_mm_s / (2.0 * M_PI * r) * 60.0;
}

double cd_geom_track_delta(int from_lba, int to_lba)
{
	// Turns between two radii: (r2^2 - r1^2) * pi / (pitch * ...) reduces to
	// the integral of 1/sectors_per_rev, which for a spiral is exactly the
	// radial delta over the pitch.
	double d = cd_geom_radial_delta_mm(from_lba, to_lba);
	return d / CD_TRACK_PITCH_MM;
}

double cd_geom_radial_delta_mm(int from_lba, int to_lba)
{
	double a = cd_geom_radius_mm(from_lba);
	double b = cd_geom_radius_mm(to_lba);
	return a > b ? a - b : b - a;
}
