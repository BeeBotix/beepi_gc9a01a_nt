/*!
 * @file videoview_osd.cpp
 * BeeBotix GC9A01A — Animated GIF Player with FPS OSD
 *
 * Overlays a real-time FPS counter (actual displayed fps) in a solid
 * black bar at the bottom of the screen.
 *
 * Orientation: MY|MX active — rows are pre-flipped in copy_frame_to_fb(),
 * columns are pre-mirrored when writing OSD pixels so text reads correctly.
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
    char     magic[4];
    uint16_t width;
    uint16_t height;
    uint32_t frame_count;
    uint16_t fps;
};
static_assert(sizeof(BpgfHeader) == 14, "BpgfHeader must be 14 bytes packed");

// ---------------------------------------------------------------------------
// Hardware config
// ---------------------------------------------------------------------------

static BeePiHALConfig make_config()
{
    BeePiHALConfig cfg = {};
    cfg.spi_device   = "/dev/spidev0.0";
    cfg.spi_speed_hz = 62500000u;   // 62.5 MHz — ~68 fps theoretical max
    cfg.gpio_dc      = 25;
    cfg.gpio_rst     = 24;
    cfg.gpio_bl      = 18;
    cfg.gpio_chip    = "/dev/gpiochip0";
    return cfg;
}

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

static void  *g_mmap  = MAP_FAILED;
static size_t g_msize = 0;
static int    g_fd    = -1;

static uint16_t fb[240 * 240];

static volatile sig_atomic_t g_quit = 0;
static void on_signal(int) { g_quit = 1; }

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

static uint64_t now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000);
}

static void sleep_us(uint64_t us)
{
    struct timespec ts;
    ts.tv_sec  = (time_t)(us / 1000000ULL);
    ts.tv_nsec = (long)((us % 1000000ULL) * 1000ULL);
    nanosleep(&ts, nullptr);
}

// ---------------------------------------------------------------------------
// Frame copy with row-flip (MY|MX orientation correction)
// ---------------------------------------------------------------------------

static void copy_frame_to_fb(const uint16_t *src)
{
    for (int r = 0; r < 240; r++)
        memcpy(fb + r * 240, src + (239 - r) * 240, 240 * sizeof(uint16_t));
}

// ---------------------------------------------------------------------------
// OSD rendering
//
// MY|MX MADCTL means:
//   - Rows:    fb row 0 -> screen bottom  (compensated by copy_frame_to_fb row-flip)
//   - Columns: fb col 0 -> screen right   (MX column-flip)
//
// So OSD pixels must be written mirrored horizontally:
//   fb[fy * 240 + (239 - fx)]  instead of  fb[fy * 240 + fx]
//
// OSD bar: solid black strip at the bottom 20 rows of the screen.
// In fb[] space (already row-flipped) that is fb rows 0..19.
// Text is centred in that bar.
//
// Colour: BEEPI_WHITE = 0xFFFF (big-endian RGB565, HAL bswaps on send)
// ---------------------------------------------------------------------------

// 5x8 bitmap font — columns stored as bytes, bit0 = top row.
// Covers: space, F, P, S, colon, digits 0-9
static const uint8_t FONT5x8[][5] = {
    /* [0]  ' ' */ {0x00,0x00,0x00,0x00,0x00},
    /* [1]  'F' */ {0x7F,0x09,0x09,0x01,0x01},
    /* [2]  'P' */ {0x7F,0x09,0x09,0x09,0x06},
    /* [3]  'S' */ {0x46,0x49,0x49,0x49,0x31},
    /* [4]  ':' */ {0x00,0x36,0x36,0x00,0x00},
    /* [5]  '0' */ {0x3E,0x51,0x49,0x45,0x3E},
    /* [6]  '1' */ {0x00,0x42,0x7F,0x40,0x00},
    /* [7]  '2' */ {0x42,0x61,0x51,0x49,0x46},
    /* [8]  '3' */ {0x21,0x41,0x45,0x4B,0x31},
    /* [9]  '4' */ {0x18,0x14,0x12,0x7F,0x10},
    /* [10] '5' */ {0x27,0x45,0x45,0x45,0x39},
    /* [11] '6' */ {0x3C,0x4A,0x49,0x49,0x30},
    /* [12] '7' */ {0x01,0x71,0x09,0x05,0x03},
    /* [13] '8' */ {0x36,0x49,0x49,0x49,0x36},
    /* [14] '9' */ {0x06,0x49,0x49,0x29,0x1E},
};

static int font_idx(char c)
{
    switch (c) {
        case ' ': return 0;
        case 'F': return 1;  case 'P': return 2;  case 'S': return 3;
        case ':': return 4;
        case '0': return 5;  case '1': return 6;  case '2': return 7;
        case '3': return 8;  case '4': return 9;  case '5': return 10;
        case '6': return 11; case '7': return 12; case '8': return 13;
        case '9': return 14;
        default:  return 0;
    }
}

// Write one pixel into fb[]. Sequential writes fill left-to-right regardless
// of MX flag (MX reverses GRAM scan direction, not the sequential fill order).
static inline void fb_put(int fx, int fy, uint16_t colour)
{
    if (fx < 0 || fx >= 240 || fy < 0 || fy >= 240) return;
    fb[fy * 240 + fx] = colour;
}

// OSD bar at screen TOP.
// MY row-flip means fb row 239 = screen top row, fb row 0 = screen bottom.
// So bar at screen top occupies fb rows (240-BAR_H)..(239).
// Font is also rendered bottom-to-top in fb (row bit 0 = topmost on screen
// = highest fb row index), so we draw font rows from 7 down to 0.
static const int BAR_H   = 28;                   // bar height px, more padding
static const int SCALE   = 2;                    // font scale: 10x16 px per glyph
static const int GLYPH_W = (5 + 1) * SCALE;     // 12 px per char incl. gap
static const int FB_BAR_Y = 240 - BAR_H;         // fb row where bar starts

static void draw_fps_osd(uint32_t fps_val)
{
    // Fill solid black bar at fb rows FB_BAR_Y..239 (= screen top)
    for (int fy = FB_BAR_Y; fy < 240; fy++)
        for (int fx = 0; fx < 240; fx++)
            fb[fy * 240 + fx] = 0x0000u;

    // Build string and centre it
    char buf[16];
    snprintf(buf, sizeof(buf), "FPS:%u", fps_val);
    const int len = (int)strlen(buf);
    const int tw  = len * GLYPH_W;
    const int x0  = (240 - tw) / 2;
    // Vertically centre text in bar; anchor to fb row for top of glyph.
    // Glyph height = 8*SCALE. Bar centre in fb = FB_BAR_Y + BAR_H/2.
    // Font row 0 (screen-top of glyph) = highest fb row in glyph.
    const int glyph_h = 8 * SCALE;
    const int y_top   = FB_BAR_Y + (BAR_H - glyph_h) / 2 + glyph_h - 1; // fb row of glyph top

    for (int i = 0; i < len; i++) {
        const uint8_t *bits = FONT5x8[font_idx(buf[i])];
        int cx = x0 + i * GLYPH_W;
        for (int col = 0; col < 5; col++) {
            uint8_t colbits = bits[col];
            for (int row = 0; row < 8; row++) {
                if (!(colbits & (1 << row))) continue;
                // row 0 = top of glyph on screen = highest fb row index
                // so fb_y = y_top - row*SCALE
                for (int sy = 0; sy < SCALE; sy++)
                    for (int sx = 0; sx < SCALE; sx++)
                        fb_put(cx + col * SCALE + sx,
                               y_top - row * SCALE - sy,
                               0xFFFFu);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    const char *bin_path = (argc > 1) ? argv[1] : "../assets/ocean.bin";

    g_fd = open(bin_path, O_RDONLY);
    if (g_fd < 0) { perror(bin_path); return 1; }

    struct stat st;
    fstat(g_fd, &st);
    g_msize = (size_t)st.st_size;
    g_mmap  = mmap(nullptr, g_msize, PROT_READ, MAP_PRIVATE, g_fd, 0);
    if (g_mmap == MAP_FAILED) { perror("mmap"); close(g_fd); return 1; }
    madvise(g_mmap, g_msize, MADV_SEQUENTIAL);

    const uint8_t    *base  = static_cast<const uint8_t *>(g_mmap);
    const BpgfHeader *hdr   = reinterpret_cast<const BpgfHeader *>(base);

    uint32_t        frame_count = hdr->frame_count;
    uint16_t        gif_fps     = hdr->fps;
    const uint16_t *frame_ms    = reinterpret_cast<const uint16_t *>(base + sizeof(BpgfHeader));
    size_t          hdr_end     = 14u + (size_t)frame_count * 2u;
    size_t          data_off    = (hdr_end + 511) & ~(size_t)511;
    const uint8_t  *pixel_base  = base + data_off;
    const size_t    bpf         = 240u * 240u * 2u;

    printf("BeeBotix GC9A01A — videoview_osd\n");
    printf("  File  : %s  (%u frames, target %u fps, %.1f MB)\n",
           bin_path, frame_count, gif_fps, (double)g_msize / 1048576.0);
    printf("  SPI   : %.0f MHz  (theoretical max ~%u fps)\n",
           62500000.0 / 1e6,
           (unsigned)(62500000u / (240u * 240u * 2u * 8u)));

    BeePiHALConfig cfg = make_config();
    BeePi_GC9A01A  display(cfg);
    if (!display.begin()) {
        fprintf(stderr, "ERROR: display.begin() failed\n");
        munmap(g_mmap, g_msize); close(g_fd); return 1;
    }
    printf("  Display OK — playing (Ctrl+C to stop)\n\n");

    uint32_t frame_idx       = 0;
    uint64_t frame_due       = now_us();
    uint32_t fps_display     = 0;
    uint32_t frames_this_sec = 0;
    uint64_t fps_win_start   = now_us();

    for (;;) {
        if (g_quit) break;

        copy_frame_to_fb(reinterpret_cast<const uint16_t *>(
            pixel_base + (size_t)frame_idx * bpf));

        draw_fps_osd(fps_display);

        display.pushFrame(fb);
        frames_this_sec++;

        uint64_t now = now_us();
        if (now - fps_win_start >= 1000000ULL) {
            fps_display     = frames_this_sec;
            frames_this_sec = 0;
            fps_win_start   = now;
            printf("  Display FPS: %u  (target: %u)\r", fps_display, gif_fps);
            fflush(stdout);
        }

        // Frame timing: respect per-frame duration from GIF metadata
        uint32_t dur_us = (uint32_t)frame_ms[frame_idx] * 1000u;
        frame_due += dur_us;
        if (frame_due > now)
            sleep_us(frame_due - now);
        else
            frame_due = now;

        if (++frame_idx >= frame_count)
            frame_idx = 0;
    }

    printf("\nStopped.\n");
    display.fill(BEEPI_BLACK);
    display.end();
    munmap(g_mmap, g_msize);
    close(g_fd);
    return 0;
}