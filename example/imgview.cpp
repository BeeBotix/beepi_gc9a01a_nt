/*!
 * @file imgview.cpp
 * BeeBotix GC9A01A — Image Viewer
 *
 * logo_rgb565[] holds native LE RGB565 pixels (as written by png_to_rgb565.py).
 * fb[] also holds native LE. HAL byte-swaps on send. No manual swapping needed.
 *
 * MY|MX MADCTL: fb[r*240+c] displays at screen position (239-c, 239-r).
 * To show logo pixel (row,col) at screen (row,col) we read from
 * logo_rgb565[(239-row)*240 + col] — only rows are flipped, cols are correct.
 *
 * BeeBotix Autonomous Systems
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <ctime>
#include <csignal>

#include "beepi_gc9a01a_nt.h"
#include "../assets/logo_rgb565.h"

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

static void delay_ms(uint32_t ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, nullptr);
}

// ---------------------------------------------------------------------------
// Signal handler
// ---------------------------------------------------------------------------

static BeePi_GC9A01A *g_disp = nullptr;
static void sig_handler(int)
{
    if (g_disp) {
        g_disp->fill(BEEPI_BLACK);
        g_disp->setBacklight(false);
        g_disp->end();
    }
    _exit(0);
}

// ---------------------------------------------------------------------------
// Framebuffer
// ---------------------------------------------------------------------------

static uint16_t fb[240 * 240];

// ---------------------------------------------------------------------------
// Pixel helpers
// ---------------------------------------------------------------------------

// Get logo pixel for fb position (row, col).
// Only flip row to compensate for MY scan direction. Col is correct as-is.
static inline uint16_t logo_get(int row, int col)
{
    return logo_rgb565[(239 - row) * 240 + col];
}

// Unpack a logo pixel (stored big-endian in header) to r,g,b bytes.
// Swap bytes first so bits are in the correct LE positions before extracting.
static inline void unpack_be(uint16_t px, uint8_t &r, uint8_t &g, uint8_t &b)
{
    uint16_t v = (uint16_t)(((px & 0xFF) << 8) | (px >> 8));
    r = (uint8_t)(( v >> 11)        << 3);
    g = (uint8_t)(((v >>  5) & 0x3F) << 2);
    b = (uint8_t)(( v        & 0x1F) << 3);
}

// Pack r,g,b to big-endian RGB565 for fb[].
// fb[] holds big-endian values (same as logo header) — HAL bswaps on send.
static inline uint16_t pack_be(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t v = (uint16_t)(((uint16_t)(r & 0xF8) << 8) |
                             ((uint16_t)(g & 0xFC) << 3)  |
                              (uint16_t)(b >> 3));
    return (uint16_t)(((v & 0xFF) << 8) | (v >> 8));
}

// ---------------------------------------------------------------------------
// Effects
// ---------------------------------------------------------------------------

// Build framebuffer with logo dimmed by factor 0..255
static void build_logo_dimmed(uint8_t factor)
{
    for (int row = 0; row < 240; row++) {
        for (int col = 0; col < 240; col++) {
            uint8_t r, g, b;
            unpack_be(logo_get(row, col), r, g, b);
            r = (uint8_t)((uint16_t)r * factor / 255);
            g = (uint8_t)((uint16_t)g * factor / 255);
            b = (uint8_t)((uint16_t)b * factor / 255);
            fb[row * 240 + col] = pack_be(r, g, b);
        }
    }
}

static void effect_fade_in(BeePi_GC9A01A &d, int steps)
{
    for (int i = 0; i <= steps; i++) {
        build_logo_dimmed((uint8_t)(i * 255 / steps));
        d.pushFrame(fb);
    }
}

static void effect_fade_out(BeePi_GC9A01A &d, int steps)
{
    for (int i = steps; i >= 0; i--) {
        build_logo_dimmed((uint8_t)(i * 255 / steps));
        d.pushFrame(fb);
    }
}

static void effect_wipe_in(BeePi_GC9A01A &d)
{
    memset(fb, 0, sizeof(fb));
    for (int col = 0; col <= 239; col++) {
        for (int row = 0; row < 240; row++)
            fb[row * 240 + col] = logo_get(row, col);

        if (col + 1 < 240)
            for (int row = 0; row < 240; row++)
                fb[row * 240 + col + 1] = BEEPI_WHITE;

        d.pushFrame(fb);

        if (col + 1 < 240)
            for (int row = 0; row < 240; row++)
                fb[row * 240 + col + 1] = logo_get(row, col + 1);
    }
}

static void effect_radial_reveal(BeePi_GC9A01A &d, int steps)
{
    const float PI2         = (float)(2.0 * M_PI);
    const float start_angle = -(float)(M_PI / 2.0f);

    memset(fb, 0, sizeof(fb));
    for (int step = 1; step <= steps; step++) {
        float sweep = start_angle + PI2 * step / steps;

        for (int y = 0; y < 240; y++) {
            for (int x = 0; x < 240; x++) {
                int dx = x - 120, dy = y - 120;
                if (dx*dx + dy*dy > 120*120) continue;

                float angle = atan2f((float)dy, (float)dx);
                float rel = angle - start_angle;
                while (rel < 0)    rel += PI2;
                while (rel >= PI2) rel -= PI2;
                float lim = sweep - start_angle;
                while (lim < 0)    lim += PI2;
                while (lim >= PI2) lim -= PI2;

                fb[y * 240 + x] = (rel <= lim) ? logo_get(y, x) : (uint16_t)0;
            }
        }
        d.pushFrame(fb);
    }
}

static void effect_hue_sweep_out(BeePi_GC9A01A &d, int steps)
{
    const float PI2 = (float)(2.0 * M_PI);

    for (int step = 0; step < steps; step++) {
        float t         = (float)step / steps;
        float angle     = t * PI2;
        uint8_t dim_fac = (uint8_t)(255 - t * 255);

        for (int y = 0; y < 240; y++) {
            for (int x = 0; x < 240; x++) {
                int dx = x - 120, dy = y - 120;
                if (dx*dx + dy*dy > 120*120) { fb[y*240+x] = 0; continue; }

                uint8_t r, g, b;
                unpack_be(logo_get(y, x), r, g, b);

                float pangle = atan2f((float)dy, (float)dx);
                float rel = pangle - angle;
                while (rel < 0)    rel += PI2;
                while (rel >= PI2) rel -= PI2;
                float tint = (rel < 0.4f) ? (1.0f - rel / 0.4f) * (1.0f - t) : 0.0f;

                uint8_t tr = (uint8_t)(r + (255 - r) * tint);
                uint8_t tg = (uint8_t)(g + (255 - g) * tint);
                uint8_t tb = (uint8_t)(b + (255 - b) * tint);
                tr = (uint8_t)((uint16_t)tr * dim_fac / 255);
                tg = (uint8_t)((uint16_t)tg * dim_fac / 255);
                tb = (uint8_t)((uint16_t)tb * dim_fac / 255);

                fb[y * 240 + x] = pack_be(tr, tg, tb);
            }
        }
        d.pushFrame(fb);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

static void effect_spin_disk(BeePi_GC9A01A &d, int steps)
{
    // Rotate the image like a spinning disk over `steps` frames (1 full rotation).
    // For each output pixel (x,y), reverse-map through the rotation angle to find
    // which source pixel to sample. Uses nearest-neighbour — fast, no blurring.
    const float PI2 = (float)(2.0 * M_PI);

    for (int step = 0; step < steps; step++) {
        float angle = -PI2 * step / steps;   // negative = clockwise
        float cs = cosf(angle);
        float sn = sinf(angle);

        for (int y = 0; y < 240; y++) {
            for (int x = 0; x < 240; x++) {
                int dx = x - 120, dy = y - 120;
                if (dx*dx + dy*dy > 120*120) { fb[y*240+x] = 0; continue; }

                // Rotate (dx,dy) back by -angle to find source pixel
                int sx = 120 + (int)(dx * cs - dy * sn);
                int sy = 120 + (int)(dx * sn + dy * cs);

                if ((unsigned)sx < 240u && (unsigned)sy < 240u)
                    fb[y * 240 + x] = logo_get(sy, sx);
                else
                    fb[y * 240 + x] = 0;
            }
        }
        d.pushFrame(fb);
    }
}

int main()
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    if (LOGO_RGB565_WIDTH != 240 || LOGO_RGB565_HEIGHT != 240) {
        fprintf(stderr, "ERROR: logo.png must be 240x240 (got %ux%u)\n",
                LOGO_RGB565_WIDTH, LOGO_RGB565_HEIGHT);
        return 1;
    }

    BeePiHALConfig cfg = make_config();
    BeePi_GC9A01A display(cfg);
    g_disp = &display;

    printf("BeeBotix GC9A01A — imgview\n");
    printf("  SPI : %s @ %u MHz\n", cfg.spi_device, cfg.spi_speed_hz / 1000000);
    printf("  GPIO: DC=%d RST=%d BL=%d\n\n", cfg.gpio_dc, cfg.gpio_rst, cfg.gpio_bl);

    if (!display.begin()) {
        fprintf(stderr, "ERROR: display.begin() failed\n");
        return 1;
    }
    printf("Display OK\n\n");

    int cycle = 0;
    for (;;) {
        cycle++;
        printf("Cycle %d\n", cycle);

        printf("  radial reveal...\n");
        effect_radial_reveal(display, 120);
        delay_ms(2000);

        printf("  fade out...\n");
        effect_fade_out(display, 40);
        delay_ms(200);

        printf("  wipe in...\n");
        effect_wipe_in(display);
        delay_ms(2000);

        printf("  hue sweep out...\n");
        effect_hue_sweep_out(display, 80);
        delay_ms(200);

        printf("  fade in...\n");
        effect_fade_in(display, 40);
        delay_ms(1000);

        printf("  spin disk...\n");
        effect_spin_disk(display, 60);
        delay_ms(500);

        printf("  --- cycle complete ---\n\n");
    }

    display.fill(BEEPI_BLACK);
    display.end();
    return 0;
}