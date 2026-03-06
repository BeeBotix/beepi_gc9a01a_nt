/*!
 * @file videoview.cpp
 * BeeBotix GC9A01A — Animated GIF Player
 *
 * Plays assets/ocean.bin (converted from ocean.gif by gif_to_frames.py).
 * The bin file is mmap'd at startup — no heap allocation for frame data.
 *
 * MY|MX orientation: rows are flipped via a per-row memcpy into fb[]
 * before each pushFrame call.
 *
 * BeeBotix Autonomous Systems
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <csignal>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "beepi_gc9a01a_nt.h"

// ---------------------------------------------------------------------------
// Binary file format
// ---------------------------------------------------------------------------

struct __attribute__((packed)) BpgfHeader {
    char     magic[4];       // "BPGF"
    uint16_t width;          // 240
    uint16_t height;         // 240
    uint32_t frame_count;    // number of frames
    uint16_t fps;            // average fps
    // immediately followed by frame_count * uint16_t duration_ms[]
    // then zero-padding to 512-byte boundary
    // then frame_count * 240*240*2 bytes of pixel data (native LE RGB565)
};
static_assert(sizeof(BpgfHeader) == 14, "BpgfHeader must be 14 bytes packed");

// ---------------------------------------------------------------------------
// Hardware config
// ---------------------------------------------------------------------------

static BeePiHALConfig make_config()
{
    BeePiHALConfig cfg = {};
    cfg.spi_device   = "/dev/spidev0.0";
    cfg.spi_speed_hz = 20000000u;
    cfg.gpio_dc      = 25;
    cfg.gpio_rst     = 24;
    cfg.gpio_bl      = 18;
    cfg.gpio_chip    = "/dev/gpiochip0";
    return cfg;
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

static uint64_t now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)(ts.tv_nsec / 1000);
}

static void sleep_us(uint64_t us)
{
    if (us == 0) return;
    struct timespec ts;
    ts.tv_sec  = (time_t)(us / 1000000ull);
    ts.tv_nsec = (long)((us % 1000000ull) * 1000ull);
    nanosleep(&ts, nullptr);
}

// ---------------------------------------------------------------------------
// Signal handler
// ---------------------------------------------------------------------------

static BeePi_GC9A01A *g_disp  = nullptr;
static void         *g_mmap  = nullptr;
static size_t        g_msize = 0;
static int           g_fd    = -1;

static void sig_handler(int)
{
    if (g_disp) {
        g_disp->fill(BEEPI_BLACK);
        g_disp->setBacklight(false);
        g_disp->end();
    }
    if (g_mmap && g_mmap != MAP_FAILED) munmap(g_mmap, g_msize);
    if (g_fd >= 0) close(g_fd);
    _exit(0);
}

// ---------------------------------------------------------------------------
// Framebuffer + orientation correction
// ---------------------------------------------------------------------------

static uint16_t fb[240 * 240];

// MY|MX: fb[r*240+c] displays at screen (239-c, 239-r).
// Flip rows so fb row 0 = source row 239 etc.
static void push_frame_corrected(BeePi_GC9A01A &d, const uint16_t *src)
{
    // Match imgview's logo_get() which only flips rows:
    // fb[r*240 + c] = src[(239-r)*240 + c]
    for (int r = 0; r < 240; r++)
        memcpy(fb + r * 240, src + (239 - r) * 240, 240 * sizeof(uint16_t));
    d.pushFrame(fb);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    const char *bin_path = (argc > 1) ? argv[1] : OCEAN_BIN_PATH;

    // Open + mmap the bin file
    g_fd = open(bin_path, O_RDONLY);
    if (g_fd < 0) {
        fprintf(stderr, "ERROR: cannot open %s\n", bin_path);
        fprintf(stderr, "       Run gif_to_frames.py first:\n");
        fprintf(stderr, "       python3 assets/gif_to_frames.py assets/ocean.gif assets/ocean.bin\n");
        return 1;
    }

    struct stat st;
    fstat(g_fd, &st);
    g_msize = (size_t)st.st_size;
    g_mmap  = mmap(nullptr, g_msize, PROT_READ, MAP_PRIVATE, g_fd, 0);
    if (g_mmap == MAP_FAILED) {
        fprintf(stderr, "ERROR: mmap failed\n");
        close(g_fd);
        return 1;
    }

    // Parse header
    const auto  *hdr         = reinterpret_cast<const BpgfHeader *>(g_mmap);
    const uint8_t *base      = reinterpret_cast<const uint8_t *>(g_mmap);

    if (memcmp(hdr->magic, "BPGF", 4) != 0) {
        fprintf(stderr, "ERROR: %s is not a BPGF binary (wrong magic)\n", bin_path);
        return 1;
    }

    uint32_t frame_count = hdr->frame_count;
    uint16_t fps         = hdr->fps;

    // Per-frame durations immediately after the fixed header
    const uint16_t *frame_ms = reinterpret_cast<const uint16_t *>(base + sizeof(BpgfHeader));

    // Pixel data starts at first 512-byte boundary after header + duration table
    size_t hdr_end   = 14u + (size_t)frame_count * 2u;  // 14 = sizeof(BpgfHeader) packed
    size_t data_off  = (hdr_end + 511) & ~(size_t)511;
    const uint8_t *pixel_base = base + data_off;
    size_t bytes_per_frame    = 240u * 240u * 2u;

    printf("BeeBotix GC9A01A — videoview\n");
    printf("  Header: magic=%.4s w=%u h=%u frames=%u fps=%u\n",
           hdr->magic, hdr->width, hdr->height, frame_count, fps);
    printf("  data_off=%zu  file_size=%zu\n", data_off, g_msize);
    printf("  File  : %s  (%u frames, ~%u fps, %.1f MB)\n",
           bin_path, frame_count, fps,
           (double)g_msize / (1024.0 * 1024.0));
    printf("  SPI   : %s @ %u MHz\n", make_config().spi_device,
           make_config().spi_speed_hz / 1000000);

    BeePiHALConfig cfg = make_config();
    BeePi_GC9A01A display(cfg);
    g_disp = &display;

    if (!display.begin()) {
        fprintf(stderr, "ERROR: display.begin() failed\n");
        return 1;
    }
    printf("Display OK — playing (Ctrl+C to stop)\n\n");

    // Advise kernel we'll stream sequentially
    madvise(g_mmap, g_msize, MADV_SEQUENTIAL);

    uint32_t frame_idx  = 0;
    uint64_t frame_due  = now_us();
    uint32_t loop_count = 0;

    for (;;) {
        const uint16_t *src = reinterpret_cast<const uint16_t *>(
            pixel_base + (size_t)frame_idx * bytes_per_frame);

        push_frame_corrected(display, src);

        uint32_t dur_us = (uint32_t)frame_ms[frame_idx] * 1000u;
        frame_due += dur_us;
        uint64_t now = now_us();
        if (frame_due > now)
            sleep_us(frame_due - now);
        else
            frame_due = now;

        frame_idx++;
        if (frame_idx >= frame_count) {
            frame_idx = 0;
            loop_count++;
            printf("  loop %u complete\n", loop_count);
        }
    }

    display.fill(BEEPI_BLACK);
    display.end();
    munmap(g_mmap, g_msize);
    close(g_fd);
    return 0;
}