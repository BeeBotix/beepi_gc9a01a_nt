/*!
 * @file testpattern.cpp
 *
 * BeeBotix GC9A01A — Test Pattern Suite (Linux / Raspberry Pi)
 *
 * Patterns:
 *   1.  Splash screen         — centred BeeBotix logo text + version
 *   2.  Solid colour fills    — R G B C M Y W K with centred label
 *   3.  Gradient sweep        — smooth RGB hue bar across full width
 *   4.  Grid + circles        — alignment grid, concentric rings, centre dot
 *   5.  Rounded rectangles    — nested colour-interpolated frames
 *   6.  Triangle fan          — filled + outlined fan with gradient
 *   7.  Text alignment test   — every scale, all anchors, centred strings
 *   8.  Reticle styles        — all 4 styles in quadrants with centred labels
 *   9.  Full HUD overlay      — camera-feed simulation with all OSD elements
 *  10.  Radar sweep           — animated rotating line + fading trail
 *  11.  Pulse rings           — expanding concentric rings animation
 *  12.  Colour wheel          — hue sweep radial gradient
 *
 * Build:
 *   g++ -std=c++17 -O2 -Wall \
 *       -I../lib \
 *       testpattern.cpp \
 *       ../lib/beepi_gc9a01a_nt.cpp \
 *       ../lib/hal/beepi_hal_linux.cpp \
 *       -llgpio -lm \
 *       -o testpattern
 *
 * Run:
 *   sudo ./testpattern
 *
 * Wiring (BCM / physical):
 *   DC   BCM 25  Pin 22
 *   RST  BCM 24  Pin 18   (-1 if not wired)
 *   BL   BCM 18  Pin 12   (-1 if hardwired ON)
 *   MOSI BCM 10  Pin 19
 *   SCLK BCM 11  Pin 23
 *   CS   BCM 8   Pin 24
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

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

static BeePiHALConfig make_config()
{
    BeePiHALConfig cfg = {};
    cfg.spi_device   = "/dev/spidev0.0";
    cfg.spi_speed_hz = 20000000u;          // 20 MHz — reliable on Pi 3B+ jumper wires
    cfg.gpio_dc      = 25;                 // GPIO 25 — Pin 22
    cfg.gpio_rst     = 24;                 // GPIO 24 — Pin 18  (-1 if not wired)
    cfg.gpio_bl      = 18;                 // GPIO 18 — Pin 12  (-1 if always on)
    cfg.gpio_chip    = "/dev/gpiochip0";   // Pi 3/4/Zero2W: gpiochip0
                                           // Pi 5:          gpiochip4
    return cfg;
}

// ---------------------------------------------------------------------------
// Time / delay helpers
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
    ts.tv_sec  =  ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, nullptr);
}

static void report(const char *name, uint64_t start_us)
{
    uint64_t elapsed = now_us() - start_us;
    printf("  %-32s  %6llu us\n", name, (unsigned long long)elapsed);
    fflush(stdout);
    delay_ms(1600);
}

// ---------------------------------------------------------------------------
// Text centering helpers
// Uses BEEPI_FONT_ADVANCE (6px per char * scale) for width calculation.
// ---------------------------------------------------------------------------

// Font geometry constants — matches beepi_font5x8.h
// 5px glyph + 1px gap = 6px advance per character, 8px tall
static const int16_t FONT_ADVANCE = 6;
static const int16_t FONT_H       = 8;

static void draw_centred(BeePi_GC9A01A &d, const char *text,
                         int16_t cx, int16_t y,
                         uint16_t color, uint8_t scale = 1,
                         uint16_t bg = BEEPI_BLACK)
{
    int16_t tw = (int16_t)(strlen(text) * FONT_ADVANCE * scale);
    d.drawLabel(text, cx - tw / 2, y, color, scale, bg);
}

static void draw_centred_xy(BeePi_GC9A01A &d, const char *text,
                             int16_t cx, int16_t cy,
                             uint16_t color, uint8_t scale = 1,
                             uint16_t bg = BEEPI_BLACK)
{
    int16_t tw = (int16_t)(strlen(text) * FONT_ADVANCE * scale);
    int16_t th = (int16_t)(FONT_H * scale);
    d.drawLabel(text, cx - tw / 2, cy - th / 2, color, scale, bg);
}

// ---------------------------------------------------------------------------
// Signal handler
// ---------------------------------------------------------------------------

static BeePi_GC9A01A *g_display = nullptr;

static void sig_handler(int)
{
    printf("\nInterrupted — cleaning up\n");
    if (g_display) {
        g_display->fill(BEEPI_BLACK);
        g_display->setBacklight(false);
        g_display->end();
    }
    _exit(0);
}

// ---------------------------------------------------------------------------
// Pattern 1 — Splash screen
// ---------------------------------------------------------------------------

static void pattern_splash(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);

    d.drawCircle(120, 120, 115, BEEPI_DARKGREY);
    d.drawCircle(120, 120, 113, BeePi_GC9A01A::dimColor(BEEPI_CYAN, 80));

    draw_centred(d, "BeeBotix",      120,  70, BEEPI_CYAN,    3, BEEPI_BLACK);
    draw_centred(d, "Autonomous",    120, 102, BEEPI_WHITE,   2, BEEPI_BLACK);
    draw_centred(d, "Systems",       120, 120, BEEPI_WHITE,   2, BEEPI_BLACK);
    draw_centred(d, "GC9A01A  v2.0", 120, 152, BEEPI_YELLOW,  1, BEEPI_BLACK);
    draw_centred(d, "240x240  Round",120, 164, BEEPI_DARKGREY,1, BEEPI_BLACK);

    d.fillRect(40, 186, 160, 3, BEEPI_CYAN);

    report("Splash screen", t);
}

// ---------------------------------------------------------------------------
// Pattern 2 — Solid fills
// ---------------------------------------------------------------------------

static void pattern_solid_fills(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    struct { uint16_t col; const char *name; uint16_t label; } colours[] = {
        { BEEPI_RED,     "RED",     BEEPI_WHITE },
        { BEEPI_GREEN,   "GREEN",   BEEPI_BLACK },
        { BEEPI_BLUE,    "BLUE",    BEEPI_WHITE },
        { BEEPI_CYAN,    "CYAN",    BEEPI_BLACK },
        { BEEPI_MAGENTA, "MAGENTA", BEEPI_WHITE },
        { BEEPI_YELLOW,  "YELLOW",  BEEPI_BLACK },
        { BEEPI_WHITE,   "WHITE",   BEEPI_BLACK },
        { BEEPI_BLACK,   "BLACK",   BEEPI_WHITE },
    };
    for (auto &c : colours) {
        d.fill(c.col);
        draw_centred_xy(d, c.name, 120, 120, c.label, 3, c.col);
        delay_ms(300);
    }
    report("Solid fills", t);
}

// ---------------------------------------------------------------------------
// Pattern 3 — Gradient sweep
// ---------------------------------------------------------------------------

static void pattern_gradient(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    for (int16_t x = 0; x < 240; x++) {
        uint16_t col;
        if      (x < 80)  col = BeePi_GC9A01A::lerpColor(BEEPI_RED,   BEEPI_GREEN, (uint8_t)(x * 255 / 80));
        else if (x < 160) col = BeePi_GC9A01A::lerpColor(BEEPI_GREEN, BEEPI_BLUE,  (uint8_t)((x-80)  * 255 / 80));
        else               col = BeePi_GC9A01A::lerpColor(BEEPI_BLUE,  BEEPI_RED,   (uint8_t)((x-160) * 255 / 80));
        d.drawVLine(x, 60, 120, col);
    }
    d.fillRect(0, 114, 240, 12, BEEPI_BLACK);
    draw_centred(d, "RGB GRADIENT", 120, 115, BEEPI_WHITE, 1, BEEPI_BLACK);
    report("Gradient sweep", t);
}

// ---------------------------------------------------------------------------
// Pattern 4 — Grid + concentric circles
// ---------------------------------------------------------------------------

static void pattern_grid(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);

    for (int16_t y = 0; y < 240; y += 20) d.drawHLine(0, y, 240, BEEPI_DARKGREY);
    for (int16_t x = 0; x < 240; x += 20) d.drawVLine(x, 0, 240, BEEPI_DARKGREY);

    for (int16_t r = 20; r <= 120; r += 20)
        d.drawCircle(120, 120, r,
                     BeePi_GC9A01A::lerpColor(BEEPI_DARKGREEN, BEEPI_CYAN,
                                               (uint8_t)(r * 2)));

    d.drawHLine(0,   120, 240, BeePi_GC9A01A::dimColor(BEEPI_WHITE, 100));
    d.drawVLine(120,   0, 240, BeePi_GC9A01A::dimColor(BEEPI_WHITE, 100));
    d.fillCircle(120, 120, 4, BEEPI_RED);

    draw_centred(d, "GRID ALIGN", 120, 4, BEEPI_YELLOW, 1, BEEPI_BLACK);

    report("Grid + circles", t);
}

// ---------------------------------------------------------------------------
// Pattern 5 — Rounded rectangles
// ---------------------------------------------------------------------------

static void pattern_rects(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    for (int16_t s = 10; s <= 110; s += 10) {
        uint16_t col = BeePi_GC9A01A::lerpColor(BEEPI_BLUE, BEEPI_YELLOW, (uint8_t)(s * 2));
        d.drawRoundRect(120-s, 120-s, s*2, s*2, s/5 + 2, col);
    }
    draw_centred_xy(d, "RECTS", 120, 120, BEEPI_WHITE, 2);
    report("Rounded rects", t);
}

// ---------------------------------------------------------------------------
// Pattern 6 — Triangle fan
// ---------------------------------------------------------------------------

static void pattern_triangles(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    for (int16_t i = 0; i < 110; i += 8) {
        d.fillTriangle(120, 120-i, 120-i, 120+i, 120+i, 120+i,
                       BeePi_GC9A01A::rgb(0, (uint8_t)(i*2), (uint8_t)i));
        d.drawTriangle(120, 120-i, 120-i, 120+i, 120+i, 120+i,
                       BeePi_GC9A01A::rgb(0, (uint8_t)(i*2), 160));
    }
    draw_centred_xy(d, "TRIANGLES", 120, 120, BEEPI_WHITE, 1);
    report("Triangle fan", t);
}

// ---------------------------------------------------------------------------
// Pattern 7 — Text alignment test
// ---------------------------------------------------------------------------

static void pattern_text(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);

    // Dim centre guides
    d.drawHLine(0, 120, 240, BeePi_GC9A01A::dimColor(BEEPI_WHITE, 35));
    d.drawVLine(120, 0, 240, BeePi_GC9A01A::dimColor(BEEPI_WHITE, 35));

    // Scale 1 rows — centred on X=120
    draw_centred(d, "scale-1  centred", 120, 10, BEEPI_WHITE,   1, BEEPI_BLACK);
    draw_centred(d, "ABCDEFGHIJKLMNO",  120, 20, BEEPI_CYAN,    1, BEEPI_BLACK);
    draw_centred(d, "0123456789  +-./", 120, 30, BEEPI_YELLOW,  1, BEEPI_BLACK);

    // Scale 2
    draw_centred(d, "Scale-2", 120, 48, BEEPI_GREEN, 2, BEEPI_BLACK);

    // Scale 3
    draw_centred(d, "Scl3",   120, 74, BEEPI_ORANGE, 3, BEEPI_BLACK);

    // Four corner anchors
    d.drawLabelAnchored("TL", BEEPI_ANCHOR_TOP_LEFT,     4, BEEPI_PINK, 1, BEEPI_BLACK);
    d.drawLabelAnchored("TR", BEEPI_ANCHOR_TOP_RIGHT,    4, BEEPI_PINK, 1, BEEPI_BLACK);
    d.drawLabelAnchored("BL", BEEPI_ANCHOR_BOTTOM_LEFT,  4, BEEPI_PINK, 1, BEEPI_BLACK);
    d.drawLabelAnchored("BR", BEEPI_ANCHOR_BOTTOM_RIGHT, 4, BEEPI_PINK, 1, BEEPI_BLACK);

    // Numeric centred
    char buf[32];
    snprintf(buf, sizeof(buf), "Int: %d", 3300);
    draw_centred(d, buf, 120, 116, BEEPI_ORANGE, 1, BEEPI_BLACK);

    snprintf(buf, sizeof(buf), "Flt: %.3f", 3.14159f);
    draw_centred(d, buf, 120, 128, BEEPI_MAGENTA, 1, BEEPI_BLACK);

    draw_centred(d, "TEXT  ALIGN  TEST", 120, 220, BEEPI_DARKGREY, 1, BEEPI_BLACK);

    report("Text alignment", t);
}

// ---------------------------------------------------------------------------
// Pattern 8 — Reticle styles
// ---------------------------------------------------------------------------

static void pattern_reticles(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);

    d.drawHLine(0, 120, 240, BeePi_GC9A01A::dimColor(BEEPI_WHITE, 50));
    d.drawVLine(120, 0, 240, BeePi_GC9A01A::dimColor(BEEPI_WHITE, 50));

    // Top-left — CROSSHAIR
    d.drawReticle(60, 60, 40, 0, BEEPI_RETICLE_CROSSHAIR, BEEPI_RED, 1);
    draw_centred(d, "CROSSHAIR", 60, 104, BEEPI_RED, 1, BEEPI_BLACK);

    // Top-right — CROSS_GAP
    d.drawReticle(180, 60, 40, 12, BEEPI_RETICLE_CROSS_GAP, BEEPI_GREEN, 1);
    draw_centred(d, "CROSS GAP", 180, 104, BEEPI_GREEN, 1, BEEPI_BLACK);

    // Bottom-left — CIRCLE_CROSS
    d.drawReticle(60, 180, 40, 16, BEEPI_RETICLE_CIRCLE_CROSS, BEEPI_CYAN, 1);
    draw_centred(d, "CIRCLE", 60, 128, BEEPI_CYAN, 1, BEEPI_BLACK);

    // Bottom-right — DOT (thick)
    d.drawReticle(180, 180, 0, 0, BEEPI_RETICLE_DOT, BEEPI_YELLOW, 5);
    draw_centred(d, "DOT", 180, 128, BEEPI_YELLOW, 1, BEEPI_BLACK);

    report("Reticle styles", t);
}

// ---------------------------------------------------------------------------
// Pattern 9 — Full HUD overlay
// ---------------------------------------------------------------------------

static void pattern_hud(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_RGB(6, 16, 10));
    d.drawCircle(120, 120, 118, BEEPI_DARKGREY);

    d.drawBearingArc(45.0f, 60.0f, 108, BEEPI_DARKGREY, BEEPI_WHITE);
    d.drawReticle(120, 120, 30, 10, BEEPI_RETICLE_CROSS_GAP, BEEPI_GREEN, 2);

    d.drawBadge("RNG", 847,  "m",   8,   8, BEEPI_GREEN,  BEEPI_RGB(0, 18, 0));
    d.drawBadge("AZI", 273,  "deg", 170,  8, BEEPI_CYAN,   BEEPI_RGB(0, 18, 28));
    d.drawBadge("ELV",  -4,  "deg",  8, 192, BEEPI_ORANGE, BEEPI_RGB(28, 12, 0));

    // Zoom bar — centred horizontally
    d.drawBarH(64, 215, 112, 8, 6, 20, BEEPI_GREEN, BEEPI_DARKGREY, BEEPI_WHITE);
    d.drawLabel("Z", 55, 215, BEEPI_WHITE, 1, BEEPI_RGB(6, 16, 10));

    draw_centred(d, "TRACKING", 120, 3,   BEEPI_YELLOW, 1, BEEPI_RGB(6, 16, 10));
    d.drawLabelAnchored("REC",   BEEPI_ANCHOR_BOTTOM_LEFT,  6, BEEPI_RED,   1, BEEPI_RGB(6, 16, 10));
    d.drawLabelAnchored("14:37", BEEPI_ANCHOR_BOTTOM_RIGHT, 6, BEEPI_WHITE, 1, BEEPI_RGB(6, 16, 10));

    report("HUD overlay", t);
}

// ---------------------------------------------------------------------------
// Pattern 10 — Radar sweep animation
// ---------------------------------------------------------------------------

static void pattern_radar(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);

    // Static background
    for (int16_t r = 24; r <= 112; r += 24)
        d.drawCircle(120, 120, r, BeePi_GC9A01A::dimColor(BEEPI_GREEN, 55));
    d.drawHLine(8, 120, 224, BeePi_GC9A01A::dimColor(BEEPI_GREEN, 55));
    d.drawVLine(120, 8, 224, BeePi_GC9A01A::dimColor(BEEPI_GREEN, 55));
    d.drawCircle(120, 120, 114, BeePi_GC9A01A::dimColor(BEEPI_GREEN, 110));
    draw_centred(d, "RADAR", 120, 4, BEEPI_GREEN, 1, BEEPI_BLACK);

    const float DEG2RAD   = (float)(M_PI / 180.0);
    const int   TRAIL     = 14;
    const float STEP_DEG  = 5.0f;

    // Two full rotations
    for (int frame = 0; frame < (int)(720.0f / STEP_DEG); frame++) {
        float sweep = frame * STEP_DEG;

        // Trail
        for (int tr = TRAIL; tr >= 0; tr--) {
            float rad = (sweep - tr * STEP_DEG) * DEG2RAD;
            uint8_t bright = (uint8_t)(200 - tr * (200 / (TRAIL + 1)));
            uint16_t col = BeePi_GC9A01A::rgb(0, (uint8_t)(bright / 2), 0);
            d.drawLine(120, 120,
                       (int16_t)(120 + 112 * cosf(rad)),
                       (int16_t)(120 + 112 * sinf(rad)),
                       col);
        }
        // Bright edge
        float rad = sweep * DEG2RAD;
        d.drawLine(120, 120,
                   (int16_t)(120 + 112 * cosf(rad)),
                   (int16_t)(120 + 112 * sinf(rad)),
                   BEEPI_GREEN);

        d.fillCircle(120, 120, 3, BEEPI_GREEN);
        delay_ms(14);
    }

    report("Radar sweep", t);
}

// ---------------------------------------------------------------------------
// Pattern 11 — Pulse rings animation
// ---------------------------------------------------------------------------

static void pattern_pulse(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    draw_centred(d, "PULSE", 120, 4, BEEPI_CYAN, 1, BEEPI_BLACK);

    const int RINGS  = 5;
    const int FRAMES = 60;
    const int MAX_R  = 112;

    for (int frame = 0; frame < FRAMES * 2; frame++) {
        for (int ring = 0; ring < RINGS; ring++) {
            int offset = frame - ring * (FRAMES / RINGS);
            if (offset < 0) offset += FRAMES;
            int r = (offset * MAX_R) / FRAMES;

            // Erase previous position
            if (r > 2) {
                d.drawCircle(120, 120, (int16_t)(r-1), BEEPI_BLACK);
                d.drawCircle(120, 120, (int16_t)(r-2), BEEPI_BLACK);
            }

            uint8_t bright = (uint8_t)(255 - (offset * 255 / FRAMES));
            uint16_t col = BeePi_GC9A01A::rgb(0, (uint8_t)(bright / 2), bright);
            if (r > 0 && r <= MAX_R)
                d.drawCircle(120, 120, (int16_t)r, col);
        }
        d.fillCircle(120, 120, 5, BEEPI_CYAN);
        delay_ms(18);
    }

    report("Pulse rings", t);
}

// ---------------------------------------------------------------------------
// Pattern 12 — Colour wheel
// ---------------------------------------------------------------------------

static void pattern_colour_wheel(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);

    const float DEG2RAD = (float)(M_PI / 180.0);
    for (int angle = 0; angle < 360; angle++) {
        float rad  = (float)angle * DEG2RAD;
        int sector = angle / 60;
        uint8_t frac = (uint8_t)((angle % 60) * 255 / 60);
        uint8_t r, g, b;
        switch (sector) {
            case 0:  r=255;       g=frac;      b=0;         break;
            case 1:  r=255-frac;  g=255;       b=0;         break;
            case 2:  r=0;         g=255;       b=frac;      break;
            case 3:  r=0;         g=255-frac;  b=255;       break;
            case 4:  r=frac;      g=0;         b=255;       break;
            default: r=255;       g=0;         b=255-frac;  break;
        }
        uint16_t col = BeePi_GC9A01A::rgb(r, g, b);
        for (int16_t rp = 42; rp < 114; rp++) {
            d.drawPixel(
                (int16_t)(120 + rp * cosf(rad)),
                (int16_t)(120 + rp * sinf(rad)),
                col
            );
        }
    }
    d.fillCircle(120, 120, 40, BEEPI_WHITE);
    draw_centred_xy(d, "HUE", 120, 120, BEEPI_BLACK, 2);

    report("Colour wheel", t);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    BeePiHALConfig cfg = make_config();
    BeePi_GC9A01A display(cfg);
    g_display = &display;

    printf("BeeBotix GC9A01A Test Pattern Suite\n");
    printf("=====================================\n");
    printf("Config:\n");
    printf("  SPI device   : %s\n", cfg.spi_device);
    printf("  SPI speed    : %u Hz (%u MHz)\n", cfg.spi_speed_hz, cfg.spi_speed_hz/1000000);
    printf("  GPIO chip    : %s\n", cfg.gpio_chip);
    printf("  DC  (BCM)    : %d\n", cfg.gpio_dc);
    printf("  RST (BCM)    : %d  (%s)\n", cfg.gpio_rst, cfg.gpio_rst < 0 ? "software reset" : "hardware reset");
    printf("  BL  (BCM)    : %d  (%s)\n", cfg.gpio_bl,  cfg.gpio_bl  < 0 ? "always on" : "GPIO controlled");
    printf("\n");

    printf("Initialising display...\n");
    if (!display.begin()) {
        fprintf(stderr, "ERROR: display.begin() failed\n"
                        "  Check: SPI enabled?   ls /dev/spidev0.0\n"
                        "  Check: Running as root?  sudo ./testpattern\n"
                        "  Check: DC/RST/BL pin numbers in make_config()\n");
        return 1;
    }
    printf("Display OK  (%dx%d)\n\n", display.width(), display.height());
    printf("  %-32s  %s\n", "Pattern", "Time");
    printf("  %s\n", "------------------------------------------------");

    int cycle = 0;
    for (;;) {
        cycle++;
        printf("\nCycle %d\n", cycle);

        pattern_splash(display);
        pattern_solid_fills(display);
        pattern_gradient(display);
        pattern_grid(display);
        pattern_rects(display);
        pattern_triangles(display);
        pattern_text(display);
        pattern_reticles(display);
        pattern_hud(display);
        pattern_radar(display);
        pattern_pulse(display);
        pattern_colour_wheel(display);

        printf("  --- cycle complete ---\n");
        delay_ms(2000);
    }

    display.fill(BEEPI_BLACK);
    display.end();
    return 0;
}