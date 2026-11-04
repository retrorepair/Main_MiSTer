// Laserdisc_MiSTer: the DSCV reader. See laserdisc.h.
//
// The C form of the project's tools/mister/ld_feed.py, which ran the core on hardware first. The reader thread
// and its DDR3 handling follow support/megacd/megald_video.cpp (MiSTer-LaserActive): uncached /dev/mem
// mapping, copies of whole 8-aligned frames followed by a barrier, posix_fadvise so the page cache does not
// fill memory at 14.6 MB/s, and the thread pinned to core 0.
//
// Mailbox (core/rtl/ld_stream.sv):
//   0x30000000  header block, written here: "LDH1", version 1, disc generation, frame count, words per frame,
//               field1_row, crop_x, crop_y, audio offset, VBI offset, PPS, slot table (16 x u32, all ones = empty)
//   0x30000400  status block, written by the core at every frame boundary: "LDS1", boundary count, wanted
//               frame, player state, generation, frame being decoded, frame on screen, counters
//   0x30100000  16 slots of 1 MB: the frame's picture and audio as stored in the stream, then its 16-byte VBI
//               table entry (lines 16, 17, 329, 330), which the core's player reads like the disc's VBI
//
// Two-side play: a file whose name (without .dscv) ends "side1" or "side2" is one side of a disc when the file
// with the other digit exists beside it (user, 2026-11-04). The header tells the core (w2 bit 49: two sides, bit 50:
// this is side B). When the player turns (status w1 bits 40-41 change to the side wanted), the other file is opened.
// The core holds the last picture through the change, so the slot it plays from is kept until side B is up.

#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../../file_io.h"
#include "../../shmem.h"
#include "laserdisc.h"

#define LD_BASE    0x30000000u
#define HDR        0x000u
#define STAT       0x400u
#define TABLE      (HDR + 20 * 8)
#define RING       0x100000u
#define SLOT_SIZE  0x100000u
#define SLOTS      16
#define MAP_SIZE   (RING + SLOTS * SLOT_SIZE)
#define EMPTY      0xFFFFFFFFu
#define AHEAD      8                 // frames kept ahead of the wanted one, in the direction of travel
#define HDR_MAGIC  0x3148444Cu       // "LDH1"
#define STAT_MAGIC 0x3153444Cu       // "LDS1"

struct dscv
{
	int fd;
	uint32_t count, frame_bytes, stride, load_bytes, audio_off, vbi_slot_off;   // the stream in use
	uint64_t dsc_frames_off, cvb_frames_off;
	uint32_t dsc_frame_bytes, dsc_stride, dsc_audio_off, cvb_frame_bytes, cvb_stride, cvb_audio_off;
	int has_cvbs, cvbs;              // a composite region exists (tools/convert/dscv/add_cvbs.py); it is in use
	uint8_t *vbi;                    // the stream's VBI table, 16 bytes per frame (NULL: none)
	uint64_t frames_off;
	uint16_t width, height, field1_row, crop_x, crop_y;
	uint8_t pps[128];
	int side, two_sided;             // 1 A, 2 B (0: not a side file); the other side's file exists
	int ntsc;                        // a 525-line capture (DSCV src_height)
	char other[1024];                // that file
};

static uint8_t *mem = NULL;          // LD_BASE .. LD_BASE + MAP_SIZE, uncached
static uint8_t *stage = NULL;
static dscv disc = {};
static int have_disc = 0;
static uint16_t gen = 0;
static uint32_t slot_f[SLOTS];
static pthread_t reader;
static volatile int reader_stop = 0;
static int reader_running = 0;
static int inited = 0;
static uint32_t loads = 0, read_errors = 0;
static int last_ws = -1;             // the side the core last asked for (status w1 bits 40-41); -1 not seen yet
static volatile int reserved = -1;   // a slot the core plays a held picture from, over a side change
static uint16_t load_id = 0;         // counts Load disc (header w2 bits 51-63); a side change or stream switch keeps it

static uint32_t rd32(uint32_t off) { return *(volatile uint32_t *)(mem + off); }
static void wr32(uint32_t off, uint32_t v) { *(volatile uint32_t *)(mem + off) = v; }
static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint64_t le64(const uint8_t *p) { return le32(p) | ((uint64_t)le32(p + 4) << 32); }

static int map()
{
	if (!mem) mem = (uint8_t *)shmem_map(LD_BASE, MAP_SIZE);
	if (!stage) stage = (uint8_t *)aligned_alloc(64, SLOT_SIZE);
	return mem && stage;
}

static void remove_disc()
{
	wr32(HDR, 0);                    // the core sees no disc at its next frame boundary
	__sync_synchronize();
	have_disc = 0;
}

// ------------------------------------------------------------------ reader thread

static int start_on_core0(pthread_t *t, void *(*fn)(void *))
{
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(0, &set);
	pthread_attr_setaffinity_np(&attr, sizeof(set), &set);
	int r = pthread_create(t, &attr, fn, NULL);
	pthread_attr_destroy(&attr);
	if (r) r = pthread_create(t, NULL, fn, NULL);   // affinity refused: run anywhere
	return r;
}

// Load frame f into slot k. next (or -1) is the frame expected after it: the SD starts reading it while this one
// is copied, so reading and copying overlap. In series they take 33 + 8 ms per composite frame (measured: SD
// 21.7 MB/s, uncached DDR3 copy 86 MB/s), more than the 40 ms a frame lasts.
static void load(int k, uint32_t f, int64_t next)
{
	wr32(TABLE + 4 * k, EMPTY);      // the slot is not this frame while its data changes
	__sync_synchronize();
	slot_f[k] = EMPTY;
	uint64_t off = disc.frames_off + (uint64_t)f * disc.stride;
	uint32_t n = (disc.load_bytes + 7) & ~7u;
	if (pread(disc.fd, stage, disc.load_bytes, (off_t)off) != (ssize_t)disc.load_bytes)
	{
		read_errors++;
		return;
	}
	posix_fadvise(disc.fd, (off_t)off, (off_t)disc.load_bytes, POSIX_FADV_DONTNEED);
	if (next >= 0 && next < disc.count)
		posix_fadvise(disc.fd, (off_t)(disc.frames_off + (uint64_t)next * disc.stride), (off_t)disc.load_bytes,
		              POSIX_FADV_WILLNEED);
	memcpy(mem + RING + (size_t)k * SLOT_SIZE, stage, n);
	if (disc.vbi) memcpy(mem + RING + (size_t)k * SLOT_SIZE + disc.vbi_slot_off, disc.vbi + (size_t)f * 16, 16);
	__sync_synchronize();
	wr32(TABLE + 4 * k, f);
	__sync_synchronize();
	slot_f[k] = f;
	loads++;
}

// Make one of the stream's two regions the one in use: the DSC picture or the composite samples.
static void select_stream(int cvbs)
{
	disc.cvbs = cvbs;
	disc.frames_off = cvbs ? disc.cvb_frames_off : disc.dsc_frames_off;
	disc.frame_bytes = cvbs ? disc.cvb_frame_bytes : disc.dsc_frame_bytes;
	disc.stride = cvbs ? disc.cvb_stride : disc.dsc_stride;
	disc.audio_off = cvbs ? disc.cvb_audio_off : disc.dsc_audio_off;
	disc.load_bytes = disc.audio_off ? disc.audio_off + 8192 : disc.frame_bytes;
	disc.vbi_slot_off = (disc.load_bytes + 7) & ~7u;
}

// Header block for the stream in use, with a new disc generation and an empty slot table.
static void announce()
{
	uint32_t words = (disc.frame_bytes + 7) / 8;
	gen = (uint16_t)(gen + 1) ? (uint16_t)(gen + 1) : 1;
	uint32_t hb[56] = {};                // header words 0-27 as u32 pairs
	hb[2] = disc.count;
	hb[3] = words;
	hb[4] = disc.field1_row | ((uint32_t)disc.crop_x << 16);
	hb[5] = disc.crop_y | (disc.cvbs ? 1u << 16 : 0)    // w2 bit 48: the composite stream
	      | (disc.two_sided ? 1u << 17 : 0)               // bit 49: the other side is available
	      | (disc.side == 2 ? 1u << 18 : 0)               // bit 50: side B
	      | ((uint32_t)(load_id & 0x1FFF) << 19);         // bits 51-63: the load number
	hb[6] = disc.audio_off / 8;
	hb[7] = (disc.vbi ? disc.vbi_slot_off / 8 : 0) | (disc.ntsc ? 1u << 31 : 0);   // w3 bit 63: NTSC
	memcpy(&hb[8], disc.pps, 128);
	for (int k = 0; k < SLOTS; k++) hb[40 + k] = EMPTY;   // words 20-27: slot table
	wr32(HDR, 0);
	__sync_synchronize();
	for (int i = 2; i < 56; i++) wr32(HDR + 4 * i, hb[i]);
	__sync_synchronize();
	wr32(HDR + 4, 1 | ((uint32_t)gen << 16));
	__sync_synchronize();
	wr32(HDR, HDR_MAGIC);
	__sync_synchronize();
	for (int k = 0; k < SLOTS; k++) slot_f[k] = EMPTY;
}

static void *reader_thread(void *)
{
	pthread_setname_np(pthread_self(), "ld-frames");
	uint32_t last_seq = EMPTY, last_want = EMPTY;
	int step = 1;
	while (!reader_stop)
	{
		uint32_t magic = rd32(STAT), seq = rd32(STAT + 4), w1 = rd32(STAT + 8), w1h = rd32(STAT + 12);
		uint32_t decoding = rd32(STAT + 16), shown = rd32(STAT + 20);
		if (magic != STAT_MAGIC || (w1h >> 16) != gen)
		{
			usleep(10000);               // the core has not seen this disc yet
			continue;
		}
		// the core's menu chooses RGB (DSC) or composite: switch regions when it changes
		int want_cvbs = (w1h >> 6) & 1;
		if (want_cvbs != disc.cvbs && (disc.has_cvbs || !want_cvbs))
		{
			select_stream(want_cvbs);
			announce();
			last_seq = last_want = EMPTY;
			printf("Laserdisc: %s stream, generation %u\n", want_cvbs ? "composite" : "DSC", gen);
			continue;
		}
		// the held slot is free once the core has left the turn (player state 11) for two frame boundaries
		static uint32_t rel_seq = EMPTY;
		if (reserved >= 0)
		{
			if ((w1h & 0xF) == 11) rel_seq = EMPTY;
			else if (rel_seq == EMPTY) rel_seq = seq;
			else if (seq - rel_seq >= 2) { reserved = -1; rel_seq = EMPTY; }
		}
		uint32_t want = w1 & 0xFFFF;
		if (seq != last_seq)
		{
			if (last_want != EMPTY && want != last_want) step = (int)want - (int)last_want;
			last_seq = seq;
			last_want = want;
		}
		int d = step ? step : 1;

		// the first wanted frame not in a slot
		int64_t todo = -1;
		for (int i = 0; i <= AHEAD && todo < 0; i++)
		{
			int64_t f = (int64_t)want + (int64_t)i * d;
			if (f < 0 || f >= disc.count) break;
			int have = 0;
			for (int k = 0; k < SLOTS; k++) if (slot_f[k] == (uint32_t)f) have = 1;
			if (!have) todo = f;
		}
		if (todo < 0)
		{
			usleep(3000);
			continue;
		}

		// a free slot, else one holding a frame not wanted, never the wanted or decoding frame
		int victim = -1;
		for (int k = 0; k < SLOTS && victim < 0; k++) if (slot_f[k] == EMPTY && k != reserved) victim = k;
		for (int k = 0; k < SLOTS && victim < 0; k++)
		{
			if (k == reserved) continue;
			uint32_t f = slot_f[k];
			if (f == want || f == decoding || f == shown) continue;   // composite plays the shown frame from its slot
			int wanted = 0;
			for (int i = 0; i <= AHEAD; i++) if ((int64_t)f == (int64_t)want + (int64_t)i * d) wanted = 1;
			if (!wanted) victim = k;
		}
		if (victim < 0)
		{
			usleep(3000);
			continue;
		}
		load(victim, (uint32_t)todo, todo + d);
	}
	return NULL;
}

static void reader_halt()
{
	if (reader_running)
	{
		reader_stop = 1;
		pthread_join(reader, NULL);
		reader_running = 0;
	}
	reader_stop = 0;
}

// ------------------------------------------------------------------ public

// The side this file is, from its name, and the other side's file if it exists.
static void find_sides(const char *path)
{
	disc.side = disc.two_sided = 0;
	size_t n = strlen(path);
	if (n >= sizeof(disc.other)) return;
	const char *dot = strrchr(path, '.');
	const char *slash = strrchr(path, '/');
	size_t stem = (dot && (!slash || dot > slash)) ? (size_t)(dot - path) : n;
	if (stem < 5 || strncasecmp(path + stem - 5, "side", 4) || (path[stem - 1] != '1' && path[stem - 1] != '2')) return;
	disc.side = path[stem - 1] - '0';
	strcpy(disc.other, path);
	disc.other[stem - 1] = disc.side == 1 ? '2' : '1';
	disc.two_sided = access(disc.other, R_OK) == 0;
}

static int open_disc(const char *path)
{
	int fd = open(path, O_RDONLY | O_CLOEXEC);
	uint8_t h[512];
	if (fd < 0 || pread(fd, h, sizeof(h), 0) != (ssize_t)sizeof(h) || memcmp(h, "DSCV", 4))
	{
		printf("Laserdisc: %s is not a DSCV stream\n", path);
		if (fd >= 0) close(fd);
		return 0;
	}
	disc = {};
	disc.fd = fd;
	disc.ntsc = le16(h + 10) == 525;
	disc.crop_x = le16(h + 12);
	disc.crop_y = le16(h + 14);
	disc.width = le16(h + 16);
	disc.height = le16(h + 18);
	disc.field1_row = le16(h + 24);
	disc.count = le32(h + 28);
	disc.dsc_frame_bytes = le32(h + 40);
	disc.dsc_stride = le32(h + 44);
	disc.dsc_frames_off = le64(h + 48);
	disc.dsc_audio_off = le32(h + 72);   // Laserdisc_MiSTer extension (tools/convert/dscv/tbc2dscv.c)
	if (!memcmp(h + 256, "CVB1", 4))    // composite region (tools/convert/dscv/add_cvbs.py)
	{
		disc.has_cvbs = 1;
		disc.cvb_frames_off = le64(h + 264);
		disc.cvb_stride = le32(h + 272);
		disc.cvb_frame_bytes = le32(h + 276);
		disc.cvb_audio_off = le32(h + 288);
	}
	memcpy(disc.pps, h + 128, 128);
	find_sides(path);
	int bad = 0;
	if (disc.has_cvbs)
	{
		select_stream(1);
		bad = disc.vbi_slot_off + 16 > SLOT_SIZE || disc.load_bytes > disc.stride;
	}
	select_stream(0);
	uint64_t vbi_file_off = le64(h + 56);
	if (vbi_file_off && le32(h + 64) == 16)
	{
		size_t vbytes = (size_t)disc.count * 16;
		disc.vbi = (uint8_t *)malloc(vbytes);
		if (disc.vbi && pread(fd, disc.vbi, vbytes, (off_t)vbi_file_off) != (ssize_t)vbytes)
		{
			free(disc.vbi);
			disc.vbi = NULL;
		}
	}
	if (bad || disc.load_bytes > SLOT_SIZE || disc.load_bytes > disc.stride || disc.vbi_slot_off + 16 > SLOT_SIZE)
	{
		printf("Laserdisc: frames of %u bytes do not fit a slot\n", disc.load_bytes);
		free(disc.vbi);
		close(fd);
		return 0;
	}

	// announce the stream the core's menu already asks for (status w1 bit 38), so a mount is one generation and the
	// core never switches streams under a decode just started
	if (disc.has_cvbs && rd32(STAT) == STAT_MAGIC && ((rd32(STAT + 12) >> 6) & 1)) select_stream(1);
	announce();
	have_disc = 1;
	loads = read_errors = 0;
	printf("Laserdisc: %s, %u frames %ux%u, audio %s, composite %s, side %s, generation %u\n", path, disc.count,
	       disc.width, disc.height, disc.audio_off ? "yes" : "none", disc.has_cvbs ? "yes" : "none",
	       !disc.side ? "-" : disc.side == 1 ? (disc.two_sided ? "A of 2" : "A") : (disc.two_sided ? "B of 2" : "B"),
	       gen);
	if (start_on_core0(&reader, reader_thread) == 0) reader_running = 1;
	return 1;
}

void laserdisc_set_image(const char *path)
{
	reader_halt();
	if (!map())
	{
		printf("Laserdisc: cannot map DDR3\n");
		return;
	}
	if (have_disc)
	{
		close(disc.fd);
		free(disc.vbi);
	}
	remove_disc();
	reserved = -1;
	last_ws = -1;
	if (!path || !path[0]) return;
	// the menu gives a path relative to the storage root (games/Laserdisc/...), an MGL an absolute one
	char full[1024];
	snprintf(full, sizeof(full), "%s", getFullPath(path));
	load_id = (load_id + 1) & 0x1FFF ? (load_id + 1) & 0x1FFF : 1;   // never 0
	open_disc(full);
}

// The player turned the disc: open the other side's file. The slot of the picture on screen is kept, since the core
// holds that picture until side B is up (composite plays it from the slot).
static void switch_side()
{
	char path[1024];
	strcpy(path, disc.other);
	uint32_t shown = rd32(STAT + 20);
	int keep = -1;
	for (int k = 0; k < SLOTS; k++) if (shown != EMPTY && slot_f[k] == shown) keep = k;
	reader_halt();
	close(disc.fd);
	free(disc.vbi);
	disc.vbi = NULL;
	have_disc = 0;
	reserved = keep;
	printf("Laserdisc: turning to %s (held slot %d)\n", path, keep);
	open_disc(path);
}

void laserdisc_poll()
{
	// a turn: the core asks for the other side (status w1 bits 40-41 change to it)
	if (have_disc && rd32(STAT) == STAT_MAGIC && (rd32(STAT + 12) >> 16) == gen)
	{
		int ws = (rd32(STAT + 12) >> 8) & 3;
		if (last_ws < 0) last_ws = ws;
		else if (ws != last_ws)
		{
			last_ws = ws;
			if (ws && ws != disc.side && disc.two_sided) switch_side();
		}
	}
	if (!inited)
	{
		// a new session of the core: no disc until one is chosen (DDR3 keeps the last session's header)
		inited = 1;
		if (map())
		{
			uint32_t old = rd32(HDR + 4);
			gen = (uint16_t)(old >> 16);
			remove_disc();
		}
	}
}
