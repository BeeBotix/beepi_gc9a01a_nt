/*!
 * @file imgview.cpp
 *
 * BeeBotix GC9A01A — Image Viewer
 *
 * Displays assets/logo.png on the 240x240 round display.
 *
 * The PNG is converted to a C header (assets/logo_rgb565.h) at build time
 * by CMake running png_to_rgb565.py. No runtime image loading needed —
 * the pixel data is baked directly into the binary.
 *
 * What it does:
 *   1. Fade in    — logo appears from black over 40 frames
 *   2. Hold       — logo shown solid for 3 seconds
 *   3. Spin out   — logo rotates away (hue-shifted overlay sweep)
 *   4. Repeat
 *
 * Build:
 *   cd build && cmake .. && make -j$(nproc)
 *   sudo ./imgview
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

// Generated at build time by CMake running png_to_rgb565.py
// Defines:  logo_rgb565[240*240]  LOGO_RGB565_WIDTH  LOGO_RGB565_HEIGHT
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

static inline void fb_px(int x, int y, uint16_t c)
{
    if ((unsigned)x < 240u && (unsigned)y < 240u)
        fb[y * 240 + x] = c;
}

// ---------------------------------------------------------------------------
// Colour helpers
// ---------------------------------------------------------------------------

// Unpack big-endian RGB565 (as stored in the header) back to r,g,b
static void unpack565be(uint16_t be, uint8_t &r, uint8_t &g, uint8_t &b)
{
    // The header stores pixels byte-swapped for the display wire format.
    // Swap back to native to manipulate.
    uint16_t v = (uint16_t)(((be & 0xFF) << 8) | (be >> 8));
    r = (uint8_t)(((v >> 11) & 0x1F) << 3);
    g = (uint8_t)(((v >>  5) & 0x3F) << 2);
    b = (uint8_t)(( v        & 0x1F) << 3);
}

// Pack r,g,b back to big-endian RGB565 for the display
static uint16_t pack565be(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t v = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
    return (uint16_t)(((v & 0xFF) << 8) | (v >> 8));
}

// ---------------------------------------------------------------------------
// Effects
// ---------------------------------------------------------------------------

// Build the framebuffer with the logo dimmed by factor (0=black, 255=full)
static void build_logo_dimmed(uint8_t factor)
{
    const uint16_t *src = logo_rgb565;
    for (int i = 0; i < 240 * 240; i++) {
        uint8_t r, g, b;
        unpack565be(src[i], r, g, b);
        r = (uint8_t)((uint16_t)r * factor / 255);
        g = (uint8_t)((uint16_t)g * factor / 255);
        b = (uint8_t)((uint16_t)b * factor / 255);
        fb[i] = pack565be(r, g, b);
    }
}

// Fade in: 0 → 255 over `steps` frames
static void effect_fade_in(BeePi_GC9A01A &d, int steps)
{
    for (int i = 0; i <= steps; i++) {
        uint8_t brightness = (uint8_t)(i * 255 / steps);
        build_logo_dimmed(brightness);
        d.pushFrame(fb);
    }
}

// Fade out: 255 → 0
static void effect_fade_out(BeePi_GC9A01A &d, int steps)
{
    for (int i = steps; i >= 0; i--) {
        uint8_t brightness = (uint8_t)(i * 255 / steps);
        build_logo_dimmed(brightness);
        d.pushFrame(fb);
    }
}

// Wipe reveal: a vertical wipe line sweeps left-to-right, revealing logo
static void effect_wipe_in(BeePi_GC9A01A &d)
{
    // Start from all-black fb
    memset(fb, 0, sizeof(fb));

    for (int col = 0; col <= 239; col++) {
        // Reveal this column from the logo
        for (int row = 0; row < 240; row++) {
            fb[row * 240 + col] = logo_rgb565[row * 240 + col];
        }
        // Draw a bright leading edge line
        for (int row = 0; row < 240; row++) {
            if ((unsigned)(col+1) < 240u)
                fb[row * 240 + col + 1] = BEEPI_WHITE;
        }
        d.pushFrame(fb);
        // Erase the leading edge (will be covered by logo on next frame)
        for (int row = 0; row < 240; row++) {
            if ((unsigned)(col+1) < 240u)
                fb[row * 240 + col + 1] = logo_rgb565[row * 240 + col + 1];
        }
    }
}

// Radial reveal: sweeping arc uncovers the logo like a clock hand
static void effect_radial_reveal(BeePi_GC9A01A &d, int steps)
{
    const float PI2 = (float)(2.0 * M_PI);
    const float start_angle = -(float)(M_PI / 2.0f);   // 12 o'clock

    // Begin from all-black
    memset(fb, 0, sizeof(fb));

    for (int step = 1; step <= steps; step++) {
        float sweep = start_angle + PI2 * step / steps;

        // For every pixel inside the circle, check if its angle is within
        // the swept region (from start_angle going clockwise to sweep)
        for (int y = 0; y < 240; y++) {
            for (int x = 0; x < 240; x++) {
                int dx = x - 120, dy = y - 120;
                if (dx*dx + dy*dy > 120*120) continue;

                float angle = atan2f((float)dy, (float)dx);
                // Normalise angle to start_angle..start_angle+2PI
                float rel = angle - start_angle;
                while (rel < 0)      rel += PI2;
                while (rel >= PI2)   rel -= PI2;
                float lim = sweep - start_angle;
                while (lim < 0)      lim += PI2;
                while (lim >= PI2)   lim -= PI2;

                if (rel <= lim)
                    fb[y * 240 + x] = logo_rgb565[y * 240 + x];
                else
                    fb[y * 240 + x] = BEEPI_BLACK;
            }
        }
        d.pushFrame(fb);
    }
}

// Hue-rotate overlay: blends a rotating hue tint over the logo, then fades out
static void effect_hue_sweep_out(BeePi_GC9A01A &d, int steps)
{
    const float PI2 = (float)(2.0 * M_PI);

    for (int step = 0; step < steps; step++) {
        float t      = (float)step / steps;
        float angle  = t * PI2;
        uint8_t dim_factor = (uint8_t)(255 - t * 255);

        for (int y = 0; y < 240; y++) {
            for (int x = 0; x < 240; x++) {
                int dx = x - 120, dy = y - 120;
                if (dx*dx + dy*dy > 120*120) { fb[y*240+x] = BEEPI_BLACK; continue; }

                uint8_t r, g, b;
                unpack565be(logo_rgb565[y * 240 + x], r, g, b);

                // Compute a rotating hue tint based on pixel angle
                float pangle = atan2f((float)dy, (float)dx);
                float rel = pangle - angle;
                while (rel < 0)    rel += PI2;
                while (rel >= PI2) rel -= PI2;

                // Tint amount increases as we approach the sweep front
                float tint = (rel < 0.4f) ? (1.0f - rel / 0.4f) : 0.0f;
                tint *= (1.0f - t);

                // Tint colour: white-cyan flash
                uint8_t tr = (uint8_t)(r + (255-r) * tint);
                uint8_t tg = (uint8_t)(g + (255-g) * tint);
                uint8_t tb = (uint8_t)(b + (255-b) * tint);

                // Dim overall
                tr = (uint8_t)((uint16_t)tr * dim_factor / 255);
                tg = (uint8_t)((uint16_t)tg * dim_factor / 255);
                tb = (uint8_t)((uint16_t)tb * dim_factor / 255);

                fb[y * 240 + x] = pack565be(tr, tg, tb);
            }
        }
        d.pushFrame(fb);
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // Sanity check image dimensions
    if (LOGO_RGB565_WIDTH != 240 || LOGO_RGB565_HEIGHT != 240) {
        fprintf(stderr,
            "ERROR: logo.png must be 240x240 pixels.\n"
            "       Got %ux%u — resize and rebuild.\n",
            LOGO_RGB565_WIDTH, LOGO_RGB565_HEIGHT);
        return 1;
    }

    BeePiHALConfig cfg = make_config();
    BeePi_GC9A01A display(cfg);
    g_disp = &display;

    printf("BeeBotix GC9A01A — imgview\n");
    printf("===========================\n");
    printf("  Image : assets/logo.png  (%ux%u)\n",
           LOGO_RGB565_WIDTH, LOGO_RGB565_HEIGHT);
    printf("  SPI   : %s  @ %u MHz\n",
           cfg.spi_device, cfg.spi_speed_hz / 1000000);
    printf("  GPIO  : %s  DC=%d RST=%d BL=%d\n\n",
           cfg.gpio_chip, cfg.gpio_dc, cfg.gpio_rst, cfg.gpio_bl);

    if (!display.begin()) {
        fprintf(stderr, "ERROR: display.begin() failed\n");
        return 1;
    }
    printf("Display OK (%dx%d)\n\n", display.width(), display.height());

    int cycle = 0;
    for (;;) {
        cycle++;
        printf("Cycle %d\n", cycle);

        // --- Effect 1: Radial clock-sweep reveal ---
        printf("  radial reveal...\n");
        effect_radial_reveal(display, 120);
        delay_ms(2000);

        // --- Effect 2: Fade out then wipe back in ---
        printf("  fade out...\n");
        effect_fade_out(display, 40);
        delay_ms(200);
        printf("  wipe in...\n");
        effect_wipe_in(display);
        delay_ms(2000);

        // --- Effect 3: Hue sweep out ---
        printf("  hue sweep out...\n");
        effect_hue_sweep_out(display, 80);
        delay_ms(200);

        // --- Effect 4: Fade back in cleanly ---
        printf("  fade in...\n");
        effect_fade_in(display, 40);
        delay_ms(3000);

        printf("  --- cycle complete ---\n\n");
    }

    display.fill(BEEPI_BLACK);
    display.end();
    return 0;
}