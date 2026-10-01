// Validates cd_geometry against the measured sectors-per-revolution table in
// support/pcecd/seektime.cpp (Dave Shadoff, 2018) -- real figures taken off a
// real PC Engine CD drive.
#include <stdio.h>
#include <math.h>
#include "../../support/physical_disc/cd_geometry.h"

struct { int spr; int lo; int hi; } measured[] = {
	{ 10, 0, 12572 },      { 11, 12573, 30244 },  { 12, 30245, 49523 },
	{ 13, 49524, 70408 },  { 14, 70409, 92900 },  { 15, 92901, 116998 },
	{ 16, 116999, 142703 },{ 17, 142704, 170014 },{ 18, 170015, 198932 },
	{ 19, 198933, 229456 },{ 20, 229457, 261587 },{ 21, 261588, 295324 },
	{ 22, 295325, 330668 },{ 23, 330669, 333012 },
};

int main()
{
	int fails = 0;
	printf("band  measured   model(mid)   r_mm    err\n");
	for (unsigned i = 0; i < sizeof(measured)/sizeof(measured[0]); i++) {
		int mid = (measured[i].lo + measured[i].hi) / 2;
		double spr = cd_geom_sectors_per_rev(mid);
		double r = cd_geom_radius_mm(mid);
		double err = spr - measured[i].spr;
		printf("%2d    %8d   %9.2f   %6.2f  %+5.2f%s\n",
		       i, measured[i].spr, spr, r, err,
		       fabs(err) <= 1.0 ? "" : "   <-- OUT");
		if (fabs(err) > 1.0) fails++;
	}

	printf("\nprogram area: LBA 0 -> r=%.2fmm, LBA 333000 -> r=%.2fmm\n",
	       cd_geom_radius_mm(0), cd_geom_radius_mm(333000));

	printf("\nCLV spindle glide (2x, PSX data):\n");
	int pts[] = { 0, 50000, 150000, 250000, 333000 };
	for (unsigned i = 0; i < sizeof(pts)/sizeof(pts[0]); i++)
		printf("  LBA %6d  r=%5.2fmm  %6.0f rpm\n",
		       pts[i], cd_geom_radius_mm(pts[i]), cd_geom_rpm(pts[i], 2.0));

	printf("\nmove classification (what the current code gets wrong):\n");
	struct { int a, b; const char *what; } mv[] = {
		{ 1000, 1090,  "the old REPOSITION_JUMP threshold, inner" },
		{ 300000, 300090, "the same 90 sectors, outer" },
		{ 20000, 21000, "1000 sectors" },
		{ 150, 250000, "boot: lead-in to outer game data" },
	};
	for (unsigned i = 0; i < sizeof(mv)/sizeof(mv[0]); i++)
		printf("  %-42s %7.1f turns  %6.3f mm\n", mv[i].what,
		       cd_geom_track_delta(mv[i].a, mv[i].b),
		       cd_geom_radial_delta_mm(mv[i].a, mv[i].b));

	// Round-trip
	for (int lba = 0; lba <= 330000; lba += 30000) {
		int back = cd_geom_lba_at_radius(cd_geom_radius_mm(lba));
		if (abs(back - lba) > 1) { printf("round-trip FAIL %d -> %d\n", lba, back); fails++; }
	}

	printf("\n%s\n", fails ? "FAILED" : "all bands within 1 sector/rev of measured, round-trip exact");
	return fails ? 1 : 0;
}
