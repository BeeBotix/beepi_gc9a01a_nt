/*!
 * @file testpattern2.cpp
 *
 * BeeBotix GC9A01A — testpattern2
 *
 * Scenes (loop forever):
 *   1. Analog clock     — hour / minute / second hands, tick marks, live time
 *   2. Countdown timer  — 5 second countdown, xx:xxx format, arc progress ring
 *   3. RGB rainbow ring — smooth hue rotation filling the round display
 *   4. Bouncing balls   — 10 balls with physics, circular wall, colour trails
 *
 * Software framebuffer: all drawing goes into fb[], one pushFrame() per frame.
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
// Software framebuffer
// ---------------------------------------------------------------------------

static uint16_t fb[240 * 240];

static inline void fb_px(int x, int y, uint16_t c)
{
    if ((unsigned)x < 240u && (unsigned)y < 240u)
        fb[y * 240 + x] = c;
}

static void fb_clear(uint16_t c = BEEPI_BLACK)
{
    for (int i = 0; i < 240 * 240; i++) fb[i] = c;
}

static void fb_hline(int x, int y, int w, uint16_t c)
{
    for (int i = 0; i < w; i++) fb_px(x + i, y, c);
}

static void fb_fill_rect(int x, int y, int w, int h, uint16_t c)
{
    for (int dy = 0; dy < h; dy++)
        for (int dx = 0; dx < w; dx++)
            fb_px(x + dx, y + dy, c);
}

static void fb_line(int x0, int y0, int x1, int y1, uint16_t c)
{
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    for (;;) {
        fb_px(x0, y0, c);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

// Thick line — draw main line + one parallel offset
static void fb_thick_line(int x0, int y0, int x1, int y1, uint16_t c, int thick)
{
    fb_line(x0, y0, x1, y1, c);
    if (thick >= 2) {
        int dx = abs(x1-x0), dy = abs(y1-y0);
        if (dy > dx) {
            for (int t = 1; t < thick; t++) {
                fb_line(x0+t, y0, x1+t, y1, c);
                fb_line(x0-t, y0, x1-t, y1, c);
            }
        } else {
            for (int t = 1; t < thick; t++) {
                fb_line(x0, y0+t, x1, y1+t, c);
                fb_line(x0, y0-t, x1, y1-t, c);
            }
        }
    }
}

static void fb_fcircle(int cx, int cy, int r, uint16_t c)
{
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++)
            if (dx*dx + dy*dy <= r*r)
                fb_px(cx+dx, cy+dy, c);
}

static void fb_circle(int cx, int cy, int r, uint16_t c)
{
    int x = 0, y = r, d = 1 - r;
    while (x <= y) {
        fb_px(cx+x,cy+y,c); fb_px(cx-x,cy+y,c);
        fb_px(cx+x,cy-y,c); fb_px(cx-x,cy-y,c);
        fb_px(cx+y,cy+x,c); fb_px(cx-y,cy+x,c);
        fb_px(cx+y,cy-x,c); fb_px(cx-y,cy-x,c);
        if (d < 0) d += 2*x+3; else { d += 2*(x-y)+5; y--; }
        x++;
    }
}

// Draw a thick arc from angle a0 to a1 (radians), radius r, width w
static void fb_arc(int cx, int cy, int r, float a0, float a1, uint16_t c, int w)
{
    int steps = (int)(fabs(a1 - a0) * r) + 4;
    if (steps < 4) steps = 4;
    for (int i = 0; i <= steps; i++) {
        float a = a0 + (a1 - a0) * i / steps;
        for (int dr = 0; dr < w; dr++) {
            int rr = r - dr;
            fb_px(cx + (int)(rr * cosf(a)), cy + (int)(rr * sinf(a)), c);
        }
    }
}

// ---------------------------------------------------------------------------
// Colour helpers
// ---------------------------------------------------------------------------

static uint16_t hue565(int hue)
{
    hue = ((hue % 360) + 360) % 360;
    int s = hue / 60;
    uint8_t f = (uint8_t)((hue % 60) * 255 / 60);
    uint8_t r, g, b;
    switch (s) {
        case 0: r=255; g=f;     b=0;     break;
        case 1: r=255-f; g=255; b=0;     break;
        case 2: r=0;   g=255;   b=f;     break;
        case 3: r=0;   g=255-f; b=255;   break;
        case 4: r=f;   g=0;     b=255;   break;
        default:r=255; g=0;     b=255-f; break;
    }
    return BeePi_GC9A01A::rgb(r, g, b);
}

// Draw a centred string using the library (called after pushFrame, overlays on top)
static void overlay_centred(BeePi_GC9A01A &d, const char *text,
                             int16_t cx, int16_t y,
                             uint16_t col, uint8_t scale, uint16_t bg)
{
    int16_t tw = (int16_t)(strlen(text) * 6 * scale);
    d.drawLabel(text, cx - tw/2, y, col, scale, bg);
}

// ---------------------------------------------------------------------------
// Signal handler
// ---------------------------------------------------------------------------

static BeePi_GC9A01A *g_disp = nullptr;

static void sig_handler(int)
{
    if (g_disp) { g_disp->fill(BEEPI_BLACK); g_disp->setBacklight(false); g_disp->end(); }
    _exit(0);
}

// ===========================================================================
// Scene 1 — Analog Clock
//
// Draws a proper watch face: 12 hour numerals, 60 tick marks (long for hours,
// short for minutes), hour / minute / second hands, centre cap, date string.
// Runs for ~15 seconds of wall-clock time then returns.
// ===========================================================================

static void scene_clock(BeePi_GC9A01A &d)
{
    const float PI2     = (float)(2.0 * M_PI);
    const float DEG2RAD = (float)(M_PI / 180.0f);
    const int   CX = 120, CY = 120;
    const int   SECONDS = 15;    // how long to show the clock

    // Pre-draw the static face into a background buffer so we only
    // recompute the hands each frame
    static uint16_t face[240 * 240];

    // Build face
    memset(face, 0, sizeof(face));
    auto fpx = [&](int x, int y, uint16_t c) {
        if ((unsigned)x<240u&&(unsigned)y<240u) face[y*240+x]=c;
    };

    // Outer bezel rings
    for (int ri = 116; ri <= 118; ri++) {
        int x=0,y=ri,dd=1-ri;
        while(x<=y){
            auto dot=[&](int px2,int py2){ fpx(CX+px2,CY+py2,BEEPI_DARKGREY); fpx(CX-px2,CY+py2,BEEPI_DARKGREY); fpx(CX+px2,CY-py2,BEEPI_DARKGREY); fpx(CX-px2,CY-py2,BEEPI_DARKGREY); fpx(CX+py2,CY+px2,BEEPI_DARKGREY); fpx(CX-py2,CY+px2,BEEPI_DARKGREY); fpx(CX+py2,CY-px2,BEEPI_DARKGREY); fpx(CX-py2,CY-px2,BEEPI_DARKGREY); };
            dot(x,y);
            if(dd<0)dd+=2*x+3;else{dd+=2*(x-y)+5;y--;}x++;
        }
    }

    // Tick marks — 60 ticks, every 5th is a long hour tick
    for (int tick = 0; tick < 60; tick++) {
        float a   = tick * PI2 / 60.0f - (float)(M_PI / 2.0f);
        bool  hr  = (tick % 5 == 0);
        int   r0  = hr ? 100 : 108;
        int   r1  = 113;
        uint16_t tc = hr ? BEEPI_WHITE : BeePi_GC9A01A::dimColor(BEEPI_WHITE, 80);
        int  x0 = CX + (int)(r0 * cosf(a));
        int  y0 = CY + (int)(r0 * sinf(a));
        int  x1 = CX + (int)(r1 * cosf(a));
        int  y1 = CY + (int)(r1 * sinf(a));
        // Draw tick as 2px wide
        for (int dt = -1; dt <= 1; dt++) {
            fpx(x0+dt, y0,    tc); fpx(x0,    y0+dt, tc);
            fpx(x1+dt, y1,    tc); fpx(x1,    y1+dt, tc);
        }
        // Bresenham line on face
        int ddx=abs(x1-x0),ddy=abs(y1-y0),ssx=x0<x1?1:-1,ssy=y0<y1?1:-1,er=ddx-ddy;
        int lx=x0,ly=y0;
        for(;;){fpx(lx,ly,tc);if(lx==x1&&ly==y1)break;int e2=2*er;if(e2>-ddy){er-=ddy;lx+=ssx;}if(e2<ddx){er+=ddx;ly+=ssy;}}
    }

    // Hour numerals using library (we'll overlay after push)
    // Store positions for overlay
    struct NumPos { int16_t x, y; const char *label; } nums[12];
    for (int h = 1; h <= 12; h++) {
        float a = (h * PI2 / 12.0f) - (float)(M_PI / 2.0f);
        int r = 88;
        nums[h-1] = { (int16_t)(CX + (int)(r * cosf(a)) - 6),
                      (int16_t)(CY + (int)(r * sinf(a)) - 4),
                      nullptr };
    }
    static const char *hlabels[12]={"1","2","3","4","5","6","7","8","9","10","11","12"};
    for (int i=0;i<12;i++) nums[i].label = hlabels[i];

    uint64_t t_start = now_us();

    for (;;) {
        uint64_t now = now_us();
        if (now - t_start > (uint64_t)SECONDS * 1000000ull) break;

        // Get wall clock
        time_t tt = time(nullptr);
        struct tm *tm = localtime(&tt);
        int sec  = tm->tm_sec;
        int minn = tm->tm_min;
        int hour = tm->tm_hour % 12;

        // Copy face into fb
        memcpy(fb, face, sizeof(fb));

        // Compute hand angles (12 o'clock = -PI/2)
        float sa = sec  * PI2 / 60.0f                       - (float)(M_PI/2);
        float ma = minn * PI2 / 60.0f + sec  * PI2/3600.0f  - (float)(M_PI/2);
        float ha = hour * PI2 / 12.0f + minn * PI2/720.0f   - (float)(M_PI/2);

        // Hour hand — thick, short, white
        int hx = CX + (int)(62 * cosf(ha));
        int hy = CY + (int)(62 * sinf(ha));
        fb_thick_line(CX, CY, hx, hy, BEEPI_WHITE, 4);

        // Minute hand — medium thick, longer, light grey
        int mx2 = CX + (int)(85 * cosf(ma));
        int my2 = CY + (int)(85 * sinf(ma));
        fb_thick_line(CX, CY, mx2, my2, BEEPI_LIGHTGREY, 3);

        // Second hand — thin, longest, red with counterbalance tail
        int sx2 = CX + (int)(95 * cosf(sa));
        int sy2 = CY + (int)(95 * sinf(sa));
        int stx = CX - (int)(20 * cosf(sa));
        int sty = CY - (int)(20 * sinf(sa));
        fb_thick_line(stx, sty, sx2, sy2, BEEPI_RED, 2);

        // Centre cap
        fb_fcircle(CX, CY, 6, BEEPI_WHITE);
        fb_fcircle(CX, CY, 3, BEEPI_RED);

        d.pushFrame(fb);

        // Overlay numerals (library text on top of pushed frame)
        for (int i = 0; i < 12; i++) {
            uint8_t scale = (i==11||i==2||i==5||i==8) ? 1 : 1;
            d.drawLabel(nums[i].label, nums[i].x, nums[i].y,
                        BEEPI_WHITE, scale, BEEPI_BLACK);
        }

        // Time string at bottom
        char tbuf[12];
        snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d",
                 tm->tm_hour, minn, sec);
        overlay_centred(d, tbuf, 120, 192, BEEPI_CYAN, 1, BEEPI_BLACK);
    }

    printf("  Scene 1: Clock done\n");
    delay_ms(300);
}

// ===========================================================================
// Scene 2 — Countdown Timer  (5.000 seconds)
//
// Shows a large  SS:mmm  countdown (seconds : milliseconds).
// A cyan arc sweeps from full circle down to zero tracking elapsed time.
// When it hits 0 it flashes green "GO!" then returns.
// ===========================================================================

static void scene_timer(BeePi_GC9A01A &d)
{
    const float PI2      = (float)(2.0 * M_PI);
    const uint64_t TOTAL = 5000000ull;   // 5 seconds in microseconds
    const int CX = 120, CY = 120;

    uint64_t t_start = now_us();

    for (;;) {
        uint64_t elapsed = now_us() - t_start;
        if (elapsed > TOTAL + 800000ull) break;   // show GO for 0.8s

        fb_clear(BEEPI_BLACK);

        // Background rings
        fb_circle(CX, CY, 116, BeePi_GC9A01A::dimColor(BEEPI_DARKGREY, 60));
        fb_circle(CX, CY, 108, BeePi_GC9A01A::dimColor(BEEPI_DARKGREY, 40));

        bool done = elapsed >= TOTAL;
        float frac = done ? 0.0f : 1.0f - (float)elapsed / (float)TOTAL;

        if (!done) {
            // Arc from top (-PI/2) sweeping clockwise, length = frac * 2PI
            float a0 = -(float)(M_PI / 2.0f);
            float a1 = a0 + frac * PI2;

            // Draw arc in 6 px width, colour shifts red as time runs out
            uint16_t arc_col = BeePi_GC9A01A::lerpColor(
                BEEPI_RED, BEEPI_CYAN, (uint8_t)(frac * 255));
            fb_arc(CX, CY, 112, a0, a1, arc_col, 6);

            // Tick marks every 1 second remaining
            for (int s = 0; s <= 5; s++) {
                float ta = -(float)(M_PI/2) + s * PI2 / 5.0f;
                int x0 = CX + (int)(104 * cosf(ta));
                int y0 = CY + (int)(104 * sinf(ta));
                int x1 = CX + (int)(116 * cosf(ta));
                int y1 = CY + (int)(116 * sinf(ta));
                fb_line(x0, y0, x1, y1, BEEPI_WHITE);
            }

            // Remaining time
            uint64_t remaining_us = TOTAL - elapsed;
            uint32_t sec_part  = (uint32_t)(remaining_us / 1000000ull);
            uint32_t ms_part   = (uint32_t)((remaining_us % 1000000ull) / 1000);

            char buf[16];
            snprintf(buf, sizeof(buf), "%02u:%03u", sec_part, ms_part);

            d.pushFrame(fb);

            // Large centred time text
            overlay_centred(d, buf, 120, 104, arc_col, 3, BEEPI_BLACK);
            overlay_centred(d, "sec : ms", 120, 136, BEEPI_DARKGREY, 1, BEEPI_BLACK);
            overlay_centred(d, "COUNTDOWN", 120, 30, BEEPI_WHITE, 1, BEEPI_BLACK);

        } else {
            // GO! flash
            uint64_t flash_us = elapsed - TOTAL;
            bool on = (flash_us / 200000ull) % 2 == 0;
            d.pushFrame(fb);
            if (on) {
                overlay_centred(d, "GO!", 120, 100, BEEPI_GREEN, 4, BEEPI_BLACK);
            }
        }
    }

    printf("  Scene 2: Timer done\n");
    delay_ms(300);
}

// ===========================================================================
// Scene 3 — RGB Rainbow Ring
//
// The round display is filled with a spinning hue wheel. The hue origin
// rotates each frame producing a smooth colour wash animation.
// Runs for ~8 seconds.
// ===========================================================================

static void scene_rainbow(BeePi_GC9A01A &d)
{
    const int   CX = 120, CY = 120;
    const float PI2 = (float)(2.0 * M_PI);
    const int   SECONDS = 8;

    uint64_t t_start = now_us();
    int hue_offset = 0;

    for (;;) {
        if (now_us() - t_start > (uint64_t)SECONDS * 1000000ull) break;

        // Fill every pixel inside the circle with its angular hue
        for (int y = 0; y < 240; y++) {
            for (int x = 0; x < 240; x++) {
                int dx = x - CX, dy = y - CY;
                int r2 = dx*dx + dy*dy;
                if (r2 > 118*118) { fb[y*240+x] = BEEPI_BLACK; continue; }

                // Angle 0..359
                float angle = atan2f((float)dy, (float)dx);
                int hue = (int)(angle * 180.0f / (float)M_PI + 180.0f + hue_offset) % 360;

                // Radial brightness — bright at rim, slightly dimmer at centre
                float dist = sqrtf((float)r2);
                uint8_t bright = (uint8_t)(140 + dist * 115 / 118);

                uint16_t col = hue565(hue);
                // Apply brightness
                col = BeePi_GC9A01A::dimColor(col, bright);
                fb[y*240+x] = col;
            }
        }

        // White centre circle
        fb_fcircle(CX, CY, 28, BEEPI_WHITE);
        fb_fcircle(CX, CY, 24, BEEPI_RGB(20, 20, 20));

        d.pushFrame(fb);

        // Overlay "RGB" in centre
        overlay_centred(d, "RGB", 120, 116, BEEPI_WHITE, 1, BEEPI_RGB(20, 20, 20));

        hue_offset = (hue_offset + 3) % 360;
    }

    printf("  Scene 3: Rainbow done\n");
    delay_ms(300);
}

// ===========================================================================
// Scene 4 — Bouncing Balls
//
// 10 balls inside a circular arena. Trail via framebuffer decay.
// Each ball has velocity, bounces off the circular wall with damping.
// Runs for ~12 seconds.
// ===========================================================================

static void scene_balls(BeePi_GC9A01A &d)
{
    const int   N       = 10;
    const int   SECONDS = 12;
    const float WALL    = 110.0f;
    const float DAMP    = 0.975f;
    const float PI2     = (float)(2.0 * M_PI);
    const int   CX = 120, CY = 120;

    struct Ball {
        float x, y, vx, vy;
        int   r;
        uint16_t col;
    } balls[N];

    for (int i = 0; i < N; i++) {
        float a  = i * PI2 / N;
        float sp = 2.0f + (i % 5) * 0.5f;
        float va = a + (float)(M_PI * 0.5f) + (i % 2 ? 0.25f : -0.25f);
        balls[i].x   = CX + 40.0f * cosf(a);
        balls[i].y   = CY + 40.0f * sinf(a);
        balls[i].vx  = sp * cosf(va);
        balls[i].vy  = sp * sinf(va);
        balls[i].r   = 6 + (i % 4) * 2;
        balls[i].col = hue565(i * 36);
    }

    fb_clear();
    uint64_t t_start = now_us();

    while (now_us() - t_start < (uint64_t)SECONDS * 1000000ull) {
        // Decay previous frame — this creates the trail automatically
        for (int i = 0; i < 240*240; i++) {
            uint16_t c = fb[i];
            uint8_t r2 = (uint8_t)(((c>>11)&0x1F) * 12 / 16);
            uint8_t g2 = (uint8_t)(((c>>5 )&0x3F) * 12 / 16);
            uint8_t b2 = (uint8_t)(( c     &0x1F) * 12 / 16);
            fb[i] = (uint16_t)((r2<<11)|(g2<<5)|b2);
        }

        // Arena ring
        fb_circle(CX, CY, (int)WALL,     BeePi_GC9A01A::dimColor(BEEPI_CYAN, 50));
        fb_circle(CX, CY, (int)WALL - 1, BeePi_GC9A01A::dimColor(BEEPI_CYAN, 30));

        for (int i = 0; i < N; i++) {
            Ball &b = balls[i];

            b.x += b.vx;
            b.y += b.vy;

            // Circular wall collision
            float dx   = b.x - CX, dy = b.y - CY;
            float dist = sqrtf(dx*dx + dy*dy);
            float lim  = WALL - b.r;
            if (dist > lim) {
                float nx = dx / dist, ny = dy / dist;
                b.x = CX + nx * lim;
                b.y = CY + ny * lim;
                float dot = b.vx*nx + b.vy*ny;
                b.vx = (b.vx - 2.0f*dot*nx) * DAMP;
                b.vy = (b.vy - 2.0f*dot*ny) * DAMP;
                // Minimum speed so balls never die
                float spd = sqrtf(b.vx*b.vx + b.vy*b.vy);
                if (spd < 1.2f) { b.vx *= 1.2f/spd; b.vy *= 1.2f/spd; }
            }

            // Draw: outer glow ring, filled circle, bright highlight dot
            fb_circle((int)b.x, (int)b.y, b.r + 2,
                      BeePi_GC9A01A::dimColor(b.col, 60));
            fb_fcircle((int)b.x, (int)b.y, b.r, b.col);
            fb_circle((int)b.x,  (int)b.y, b.r,
                      BeePi_GC9A01A::dimColor(b.col, 180));
            // Specular highlight
            fb_fcircle((int)b.x - b.r/3, (int)b.y - b.r/3, 2,
                       BeePi_GC9A01A::lerpColor(b.col, BEEPI_WHITE, 160));
        }

        d.pushFrame(fb);
    }

    printf("  Scene 4: Bouncing balls done\n");
    delay_ms(300);
}

// ===========================================================================
// main
// ===========================================================================

int main()
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    BeePiHALConfig cfg = make_config();
    BeePi_GC9A01A display(cfg);
    g_disp = &display;

    printf("BeeBotix GC9A01A — testpattern2\n");
    printf("================================\n");
    printf("  SPI : %s  @ %u MHz\n", cfg.spi_device, cfg.spi_speed_hz/1000000);
    printf("  GPIO: %s  DC=%d RST=%d BL=%d\n\n",
           cfg.gpio_chip, cfg.gpio_dc, cfg.gpio_rst, cfg.gpio_bl);

    if (!display.begin()) {
        fprintf(stderr, "ERROR: display.begin() failed\n");
        return 1;
    }
    printf("Display OK (%dx%d)\n\n", display.width(), display.height());

    fb_clear();

    int cycle = 0;
    for (;;) {
        cycle++;
        printf("Cycle %d\n", cycle);
        scene_clock(display);
        scene_timer(display);
        scene_rainbow(display);
        scene_balls(display);
        printf("  --- cycle complete ---\n\n");
    }

    display.fill(BEEPI_BLACK);
    display.end();
    return 0;
}