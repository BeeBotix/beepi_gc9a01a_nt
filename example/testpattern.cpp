/*!
 * @file testpattern.cpp
 *
 * BeeBotix GC9A01A — Milestone 1 Test Pattern (Linux / Raspberry Pi)
 *
 * Cycles through visual test patterns verifying:
 *   - SPI communication and GC9A01A init
 *   - Solid colour fills
 *   - Gradient colour interpolation
 *   - Grid lines + concentric circles
 *   - Rounded rectangles
 *   - Triangle fan
 *   - Text rendering (built-in 5x8 font, multiple scales)
 *   - All four reticle styles
 *   - Full HUD overlay (reticle + badges + bar + bearing arc)
 *   - Colour wheel (hue sweep)
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
 *   (sudo needed for GPIO access via lgpio on some Pi OS versions)
 *
 * Default wiring (BCM pin numbers):
 *   DC   -> BCM 25  (GPIO 25, physical pin 22)
 *   RST  -> BCM 27  (GPIO 27, physical pin 13)   [-1 to skip]
 *   BL   -> BCM 18  (GPIO 18, physical pin 12)   [-1 if always-on]
 *   MOSI -> BCM 10  (SPI0 MOSI, physical pin 19)
 *   SCLK -> BCM 11  (SPI0 SCLK, physical pin 23)
 *   CS   -> BCM 8   (SPI0 CE0,  physical pin 24)  [spidev0.0]
 *
 * Pi 5 note:
 *   Change gpio_chip to "/dev/gpiochip4"
 *   SPI device remains "/dev/spidev0.0" (or spidev1.0 for SPI1)
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
// Hardware configuration — edit for your wiring
// ---------------------------------------------------------------------------

static BeePiHALConfig make_config()
{
    BeePiHALConfig cfg = {};
    cfg.spi_device   = "/dev/spidev0.0";
    cfg.spi_speed_hz = 40000000u;       // 40 MHz — safe on Pi 3B+/4/5/Zero2W
    cfg.gpio_dc      = 25;              // GPIO 25 — Pin 22
    cfg.gpio_rst     = 24;              // GPIO 24 — Pin 18  (-1 if not wired)
    cfg.gpio_bl      = 18;              // GPIO 18 — Pin 12  (-1 if always on)
    cfg.gpio_chip    = "/dev/gpiochip0";// Pi 3/4/Zero2W: gpiochip0
                                        // Pi 5:          gpiochip4
    return cfg;
}

// ---------------------------------------------------------------------------
// Timing helper
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
    printf("  %-28s  %6llu us\n", name, (unsigned long long)elapsed);
    fflush(stdout);
    delay_ms(1400);
}

// ---------------------------------------------------------------------------
// Global display — cleaned up on Ctrl-C
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
// Pattern 1 — Solid colour fills
// ---------------------------------------------------------------------------

static void pattern_solid_fills(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    struct { uint16_t col; const char *name; } colours[] = {
        { BEEPI_RED,     "RED"     },
        { BEEPI_GREEN,   "GREEN"   },
        { BEEPI_BLUE,    "BLUE"    },
        { BEEPI_CYAN,    "CYAN"    },
        { BEEPI_MAGENTA, "MAGENTA" },
        { BEEPI_YELLOW,  "YELLOW"  },
        { BEEPI_WHITE,   "WHITE"   },
        { BEEPI_BLACK,   "BLACK"   },
    };
    for (auto &c : colours) {
        d.fill(c.col);
        uint16_t lc = (c.col == BEEPI_BLACK || c.col == BEEPI_BLUE ||
                       c.col == BEEPI_NAVY)
                      ? BEEPI_WHITE : BEEPI_BLACK;
        d.drawLabelAnchored(c.name, BEEPI_ANCHOR_CENTER, 0, lc, 2);
        delay_ms(350);
    }
    report("Solid fills", t);
}

// ---------------------------------------------------------------------------
// Pattern 2 — Gradient bars
// ---------------------------------------------------------------------------

static void pattern_gradient(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    for (int16_t x = 0; x < BEEPI_DISP_W; x++) {
        uint16_t col;
        if      (x < 80)  col = BeePi_GC9A01A::lerpColor(BEEPI_RED,   BEEPI_GREEN, (uint8_t)(x * 255 / 80));
        else if (x < 160) col = BeePi_GC9A01A::lerpColor(BEEPI_GREEN, BEEPI_BLUE,  (uint8_t)((x-80)  * 255 / 80));
        else               col = BeePi_GC9A01A::lerpColor(BEEPI_BLUE,  BEEPI_RED,   (uint8_t)((x-160) * 255 / 80));
        d.drawVLine(x, 0, BEEPI_DISP_H, col);
    }
    d.drawLabelAnchored("GRADIENT", BEEPI_ANCHOR_BOTTOM_LEFT, 8, BEEPI_WHITE, 1);
    report("Gradient bars", t);
}

// ---------------------------------------------------------------------------
// Pattern 3 — Grid + concentric circles
// ---------------------------------------------------------------------------

static void pattern_grid(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    for (int16_t y = 0; y < BEEPI_DISP_H; y += 20) d.drawHLine(0, y, BEEPI_DISP_W, BEEPI_DARKGREY);
    for (int16_t x = 0; x < BEEPI_DISP_W; x += 20) d.drawVLine(x, 0, BEEPI_DISP_H, BEEPI_DARKGREY);
    for (int16_t r = 20; r <= BEEPI_DISP_RADIUS; r += 20)
        d.drawCircle(BEEPI_DISP_CX, BEEPI_DISP_CY, r, BEEPI_DARKGREEN);
    d.fillCircle(BEEPI_DISP_CX, BEEPI_DISP_CY, 3, BEEPI_RED);
    d.drawLabel("GRID", 4, 4, BEEPI_CYAN, 1);
    report("Grid + circles", t);
}

// ---------------------------------------------------------------------------
// Pattern 4 — Rounded rectangles
// ---------------------------------------------------------------------------

static void pattern_rects(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    for (int16_t s = 10; s <= 110; s += 10) {
        uint16_t col = BeePi_GC9A01A::lerpColor(BEEPI_BLUE, BEEPI_YELLOW, (uint8_t)(s * 2));
        d.drawRoundRect(BEEPI_DISP_CX-s, BEEPI_DISP_CY-s, s*2, s*2, s/4, col);
    }
    d.drawLabel("RECTS", 80, 108, BEEPI_WHITE, 2);
    report("Rounded rects", t);
}

// ---------------------------------------------------------------------------
// Pattern 5 — Triangle fan
// ---------------------------------------------------------------------------

static void pattern_triangles(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    int16_t cx = BEEPI_DISP_CX, cy = BEEPI_DISP_CY;
    for (int16_t i = 0; i < 110; i += 8) {
        d.fillTriangle(cx, cy-i, cx-i, cy+i, cx+i, cy+i,
                       BeePi_GC9A01A::rgb(0, (uint8_t)(i*2), (uint8_t)i));
        d.drawTriangle(cx, cy-i, cx-i, cy+i, cx+i, cy+i,
                       BeePi_GC9A01A::rgb(0, (uint8_t)(i*2), 160));
    }
    d.drawLabel("TRIANGLES", 56, 108, BEEPI_WHITE, 1);
    report("Triangle fan", t);
}

// ---------------------------------------------------------------------------
// Pattern 6 — Text
// ---------------------------------------------------------------------------

static void pattern_text(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    d.drawLabel("BeeBotix",      20,  18, BEEPI_CYAN,    3);
    d.drawLabel("GC9A01A",       32,  44, BEEPI_YELLOW,  2);
    d.drawLabel("240x240 Round", 14,  66, BEEPI_WHITE,   1);
    d.drawLabel("RGB565",        80,  80, BEEPI_GREEN,   1);
    d.drawInt(3300,   "mV",      20,  98, BEEPI_ORANGE,  2);
    d.drawFloat(3.14159f, 3, "rad", 20, 122, BEEPI_PINK, 1);
    d.drawLabel("scale-3",       70, 148, BEEPI_MAGENTA, 3);
    d.drawLabel("v1.0-M1",      172, 210, BEEPI_DARKGREY,1);
    report("Text rendering", t);
}

// ---------------------------------------------------------------------------
// Pattern 7 — All reticle styles
// ---------------------------------------------------------------------------

static void pattern_reticles(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);
    d.drawHLine(0, 120, 240, BEEPI_DARKGREY);
    d.drawVLine(120, 0, 240, BEEPI_DARKGREY);

    d.drawReticle( 60,  60, 40, 10, BEEPI_RETICLE_CROSSHAIR,    BEEPI_RED,    1);
    d.drawLabel("XHAIR",  32, 103, BEEPI_RED,    1);

    d.drawReticle(180,  60, 40, 12, BEEPI_RETICLE_CROSS_GAP,    BEEPI_GREEN,  1);
    d.drawLabel("GAP",   160, 103, BEEPI_GREEN,  1);

    d.drawReticle( 60, 180, 40, 15, BEEPI_RETICLE_CIRCLE_CROSS, BEEPI_CYAN,   1);
    d.drawLabel("CIRC",   38, 126, BEEPI_CYAN,   1);

    d.drawReticle(180, 180, 40, 10, BEEPI_RETICLE_DOT,          BEEPI_YELLOW, 4);
    d.drawLabel("DOT",   162, 126, BEEPI_YELLOW, 1);

    report("Reticle styles", t);
}

// ---------------------------------------------------------------------------
// Pattern 8 — Full HUD overlay
// ---------------------------------------------------------------------------

static void pattern_hud(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();

    // Dark greenish background simulating a thermal/night-vision feed
    d.fill(BEEPI_RGB(8, 18, 12));

    // Outer boundary arc
    d.drawCircle(BEEPI_DISP_CX, BEEPI_DISP_CY, 118, BEEPI_DARKGREY);

    // Main targeting reticle
    d.drawReticle(BEEPI_DISP_CX, BEEPI_DISP_CY, 28, 8,
                  BEEPI_RETICLE_CROSS_GAP, BEEPI_GREEN, 2);

    // Range badge — top-left quadrant
    d.drawBadge("RNG", 847, "m",  8,  8, BEEPI_GREEN, BEEPI_RGB(0, 18, 0));

    // Azimuth badge — top-right quadrant
    d.drawBadge("AZI", 273, "d", 178,  8, BEEPI_CYAN, BEEPI_RGB(0, 18, 28));

    // Elevation badge — bottom-left
    d.drawBadge("ELV",  -4, "d",  8, 192, BEEPI_ORANGE, BEEPI_RGB(28, 12, 0));

    // Zoom bar — bottom centre
    d.drawBarH(72, 218, 96, 7, 6, 20, BEEPI_GREEN, BEEPI_DARKGREY, BEEPI_WHITE);
    d.drawLabel("Z", 60, 215, BEEPI_WHITE, 1);

    // Status labels
    d.drawLabel("TRACKING", 72,  2, BEEPI_YELLOW, 1);
    d.drawLabel("REC",       6, 216, BEEPI_RED,    1);
    d.drawLabel("14:37",   178, 216, BEEPI_WHITE,  1);

    // Bearing arc at r=106, 60-degree FOV, bearing=045
    d.drawBearingArc(45.0f, 60.0f, 106, BEEPI_DARKGREY, BEEPI_WHITE);

    report("HUD overlay", t);
}

// ---------------------------------------------------------------------------
// Pattern 9 — Colour wheel
// ---------------------------------------------------------------------------

static void pattern_colour_wheel(BeePi_GC9A01A &d)
{
    uint64_t t = now_us();
    d.fill(BEEPI_BLACK);

    const float DEG2RAD = (float)(M_PI / 180.0);
    for (int angle = 0; angle < 360; angle++) {
        float rad = (float)angle * DEG2RAD;
        int sector = angle / 60;
        uint8_t frac = (uint8_t)((angle % 60) * 255 / 60);
        uint8_t r, g, b;
        switch (sector) {
            case 0:  r=255;        g=frac;       b=0;          break;
            case 1:  r=255-frac;   g=255;        b=0;          break;
            case 2:  r=0;          g=255;        b=frac;       break;
            case 3:  r=0;          g=255-frac;   b=255;        break;
            case 4:  r=frac;       g=0;          b=255;        break;
            default: r=255;        g=0;          b=255-frac;   break;
        }
        uint16_t col = BeePi_GC9A01A::rgb(r, g, b);
        for (int16_t rp = 40; rp < 112; rp++) {
            d.drawPixel(
                (int16_t)(BEEPI_DISP_CX + rp * cosf(rad)),
                (int16_t)(BEEPI_DISP_CY + rp * sinf(rad)),
                col
            );
        }
    }
    d.fillCircle(BEEPI_DISP_CX, BEEPI_DISP_CY, 38, BEEPI_WHITE);
    d.drawLabel("HUE", 104, 112, BEEPI_BLACK, 1);
    report("Colour wheel", t);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    printf("BeeBotix GC9A01A Test Pattern — Milestone 1\n");
    printf("============================================\n");

    BeePiHALConfig cfg = make_config();
    BeePi_GC9A01A display(cfg);
    g_display = &display;

    printf("Initialising display...\n");
    if (!display.begin()) {
        fprintf(stderr, "ERROR: display.begin() failed — check SPI and GPIO config\n");
        return 1;
    }
    printf("Display OK  (%dx%d)\n\n", display.width(), display.height());
    printf("  %-28s  %s\n", "Pattern", "Time");
    printf("  %s\n", "-----------------------------------------------");

    int cycle = 0;
    for (;;) {
        cycle++;
        printf("\nCycle %d\n", cycle);

        pattern_solid_fills(display);
        pattern_gradient(display);
        pattern_grid(display);
        pattern_rects(display);
        pattern_triangles(display);
        pattern_text(display);
        pattern_reticles(display);
        pattern_hud(display);
        pattern_colour_wheel(display);

        printf("  --- cycle complete ---\n");
        delay_ms(2000);
    }

    // unreachable, but tidy
    display.fill(BEEPI_BLACK);
    display.end();
    return 0;
}