// Laserdisc_MiSTer: the DSCV reader. See laserdisc.h.
//
// The C form of the project's tools/mister/ld_feed.py, which ran the core on hardware first. The reader thread
// and its DDR3 handling follow support/megacd/megald_video.cpp (MiSTer-LaserActive): uncached /dev/mem
// mapping, copies of whole 8-aligned frames followed by a barrier, posix_fadvise so the page cache does not
// fill memory at 14.6 MB/s, and the thread pinned to core 0.
//
// Mailbox (core/rtl/ld_stream.sv):
//   0x30000000  header block, written here: "LDH1", version 1, disc generation, frame count, words per frame,
//               field1_row, crop_x, crop_y, audio offset, PPS, slot table (16 x u32, all ones = empty)
//   0x30000400  status block, written by the core at every frame boundary: "LDS1", boundary count, wanted
//               frame, player state, generation, frame being decoded, frame on screen, counters
//   0x30100000  16 slots of 1 MB

#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

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
	uint32_t count, frame_bytes, stride, load_bytes, audio_off;
	uint64_t frames_off;
	uint16_t width, height, field1_row, crop_x, crop_y;
	uint8_t pps[128];
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

static void load(int k, uint32_t f)
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
	memcpy(mem + RING + (size_t)k * SLOT_SIZE, stage, n);
	__sync_synchronize();
	wr32(TABLE + 4 * k, f);
	__sync_synchronize();
	slot_f[k] = f;
	loads++;
}

static void *reader_thread(void *)
{
	pthread_setname_np(pthread_self(), "ld-frames");
	uint32_t last_seq = EMPTY, last_want = EMPTY;
	int step = 1;
	while (!reader_stop)
	{
		uint32_t magic = rd32(STAT), seq = rd32(STAT + 4), w1 = rd32(STAT + 8), w1h = rd32(STAT + 12);
		uint32_t decoding = rd32(STAT + 16);
		if (magic != STAT_MAGIC || (w1h >> 16) != gen)
		{
			usleep(10000);               // the core has not seen this disc yet
			continue;
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
		for (int k = 0; k < SLOTS && victim < 0; k++) if (slot_f[k] == EMPTY) victim = k;
		for (int k = 0; k < SLOTS && victim < 0; k++)
		{
			uint32_t f = slot_f[k];
			if (f == want || f == decoding) continue;
			int wanted = 0;
			for (int i = 0; i <= AHEAD; i++) if ((int64_t)f == (int64_t)want + (int64_t)i * d) wanted = 1;
			if (!wanted) victim = k;
		}
		if (victim < 0)
		{
			usleep(3000);
			continue;
		}
		load(victim, (uint32_t)todo);
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

void laserdisc_set_image(const char *path)
{
	reader_halt();
	if (!map())
	{
		printf("Laserdisc: cannot map DDR3\n");
		return;
	}
	if (have_disc) close(disc.fd);
	remove_disc();
	if (!path || !path[0]) return;

	int fd = open(path, O_RDONLY | O_CLOEXEC);
	uint8_t h[512];
	if (fd < 0 || pread(fd, h, sizeof(h), 0) != (ssize_t)sizeof(h) || memcmp(h, "DSCV", 4))
	{
		printf("Laserdisc: %s is not a DSCV stream\n", path);
		if (fd >= 0) close(fd);
		return;
	}
	disc = {};
	disc.fd = fd;
	disc.crop_x = le16(h + 12);
	disc.crop_y = le16(h + 14);
	disc.width = le16(h + 16);
	disc.height = le16(h + 18);
	disc.field1_row = le16(h + 24);
	disc.count = le32(h + 28);
	disc.frame_bytes = le32(h + 40);
	disc.stride = le32(h + 44);
	disc.frames_off = le64(h + 48);
	disc.audio_off = le32(h + 72);   // Laserdisc_MiSTer extension (tools/convert/dscv/tbc2dscv.c)
	memcpy(disc.pps, h + 128, 128);
	disc.load_bytes = disc.audio_off ? disc.audio_off + 8192 : disc.frame_bytes;
	if (disc.load_bytes > SLOT_SIZE || disc.load_bytes > disc.stride)
	{
		printf("Laserdisc: frames of %u bytes do not fit a slot\n", disc.load_bytes);
		close(fd);
		return;
	}

	// header: fill, then validate with a new generation
	uint32_t words = (disc.frame_bytes + 7) / 8;
	gen = (uint16_t)(gen + 1) ? (uint16_t)(gen + 1) : 1;
	uint32_t hb[56] = {};                // header words 0-27 as u32 pairs
	hb[2] = disc.count;
	hb[3] = words;
	hb[4] = disc.field1_row | ((uint32_t)disc.crop_x << 16);
	hb[5] = disc.crop_y;
	hb[6] = disc.audio_off / 8;
	memcpy(&hb[8], disc.pps, 128);
	for (int k = 0; k < SLOTS; k++) hb[40 + k] = EMPTY;   // words 20-27: slot table
	for (int i = 2; i < 56; i++) wr32(HDR + 4 * i, hb[i]);
	__sync_synchronize();
	wr32(HDR + 4, 1 | ((uint32_t)gen << 16));
	__sync_synchronize();
	wr32(HDR, HDR_MAGIC);
	__sync_synchronize();
	for (int k = 0; k < SLOTS; k++) slot_f[k] = EMPTY;
	have_disc = 1;
	loads = read_errors = 0;
	printf("Laserdisc: %s, %u frames %ux%u, audio %s, generation %u\n", path, disc.count, disc.width, disc.height,
	       disc.audio_off ? "yes" : "none", gen);
	if (start_on_core0(&reader, reader_thread) == 0) reader_running = 1;
}

void laserdisc_poll()
{
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
