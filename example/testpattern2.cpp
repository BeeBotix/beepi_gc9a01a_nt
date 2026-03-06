/*!
 * @file testpattern2.cpp
 *
 * BeeBotix GC9A01A — testpattern2
 *
 * Scenes (loop forever):
 *   1. Analog clock     — hour / minute / second hands, tick marks, live time
 *   2. Countdown timer  — 5 second countdown, SS:mmm format, arc progress ring
 *   3. RGB rainbow ring — smooth spinning hue wheel
 *   4. Bouncing balls   — 10 balls, physics, circular wall, colour trails
 *
 * Software framebuffer: all drawing into fb[], one pushFrame() per frame.
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
// Framebuffer
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

static void fb_line(int x0, int y0, int x1, int y1, uint16_t c)
{
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = x0<x1?1:-1, sy = y0<y1?1:-1, err = dx-dy;
    for(;;){
        fb_px(x0,y0,c);
        if(x0==x1&&y0==y1) break;
        int e2=2*err;
        if(e2>-dy){err-=dy;x0+=sx;}
        if(e2< dx){err+=dx;y0+=sy;}
    }
}

// Thick line by drawing parallel offsets
static void fb_thick_line(int x0, int y0, int x1, int y1, uint16_t c, int t)
{
    fb_line(x0, y0, x1, y1, c);
    int dx = abs(x1-x0), dy = abs(y1-y0);
    for (int i = 1; i < t; i++) {
        if (dy >= dx) {
            fb_line(x0+i,y0,x1+i,y1,c);
            fb_line(x0-i,y0,x1-i,y1,c);
        } else {
            fb_line(x0,y0+i,x1,y1+i,c);
            fb_line(x0,y0-i,x1,y1-i,c);
        }
    }
}

static void fb_fcircle(int cx, int cy, int r, uint16_t c)
{
    for (int dy=-r; dy<=r; dy++)
        for (int dx=-r; dx<=r; dx++)
            if (dx*dx+dy*dy<=r*r)
                fb_px(cx+dx,cy+dy,c);
}

static void fb_circle(int cx, int cy, int r, uint16_t c)
{
    int x=0,y=r,d=1-r;
    while(x<=y){
        fb_px(cx+x,cy+y,c); fb_px(cx-x,cy+y,c);
        fb_px(cx+x,cy-y,c); fb_px(cx-x,cy-y,c);
        fb_px(cx+y,cy+x,c); fb_px(cx-y,cy+x,c);
        fb_px(cx+y,cy-x,c); fb_px(cx-y,cy-x,c);
        if(d<0)d+=2*x+3;else{d+=2*(x-y)+5;y--;}x++;
    }
}

static void fb_arc(int cx, int cy, int r, float a0, float a1, uint16_t c, int w)
{
    int steps = (int)(fabsf(a1-a0) * r) + 4;
    for (int i=0; i<=steps; i++) {
        float a = a0 + (a1-a0)*i/steps;
        for (int dr=0; dr<w; dr++)
            fb_px(cx-(int)((r-dr)*cosf(a)), cy+(int)((r-dr)*sinf(a)), c);
    }
}

// ---------------------------------------------------------------------------
// Colour helpers
// ---------------------------------------------------------------------------

static uint16_t hue565(int hue)
{
    hue = ((hue%360)+360)%360;
    int s=hue/60; uint8_t f=(uint8_t)((hue%60)*255/60),r,g,b;
    switch(s){
        case 0:r=255;g=f;  b=0;  break; case 1:r=255-f;g=255;b=0;  break;
        case 2:r=0;  g=255;b=f;  break; case 3:r=0;  g=255-f;b=255;break;
        case 4:r=f;  g=0;  b=255;break; default:r=255;g=0;b=255-f;  break;
    }
    return BeePi_GC9A01A::rgb(r,g,b);
}

// ---------------------------------------------------------------------------
// Signal handler
// ---------------------------------------------------------------------------

static BeePi_GC9A01A *g_disp = nullptr;
static void sig_handler(int)
{
    if(g_disp){g_disp->fill(BEEPI_BLACK);g_disp->setBacklight(false);g_disp->end();}
    _exit(0);
}

// ===========================================================================
// Scene 1 — Analog Clock
//
// Clean watch face:
//   - 60 tick marks  (12 long hour ticks, 48 short minute ticks)
//   - Hour hand   : fat,  white,  length 55
//   - Minute hand : medium, light grey, length 80
//   - Second hand : thin,  red,    length 95  + 15px counterbalance tail
//   - Centre cap  : white outer, red inner
//   - No numerals — pure minimalist look
// ===========================================================================

static void scene_clock(BeePi_GC9A01A &d)
{
    const float PI2     = (float)(2.0 * M_PI);
    const float HALF_PI = (float)(M_PI / 2.0);
    const int   CX = 120, CY = 120;
    const int   RUN_SECS = 20;

    // Build the static face once into a separate buffer
    static uint16_t face[240 * 240];
    memset(face, 0, sizeof(face));

    auto fpx = [&](int x, int y, uint16_t c){
        if((unsigned)x<240u&&(unsigned)y<240u) face[y*240+x]=c;
    };
    auto fline = [&](int x0,int y0,int x1,int y1,uint16_t c){
        int dx=abs(x1-x0),dy=abs(y1-y0),sx=x0<x1?1:-1,sy=y0<y1?1:-1,err=dx-dy;
        for(;;){fpx(x0,y0,c);if(x0==x1&&y0==y1)break;int e2=2*err;if(e2>-dy){err-=dy;x0+=sx;}if(e2<dx){err+=dx;y0+=sy;}}
    };

    // Outer bezel — two concentric rings
    for (int ri = 114; ri <= 118; ri++) {
        uint16_t rc = (ri>=116) ? BeePi_GC9A01A::dimColor(BEEPI_WHITE, 55)
                                : BeePi_GC9A01A::dimColor(BEEPI_CYAN,  40);
        int x=0,y=ri,dd=1-ri;
        while(x<=y){
            auto p=[&](int px2,int py2){fpx(CX+px2,CY+py2,rc);fpx(CX-px2,CY+py2,rc);fpx(CX+px2,CY-py2,rc);fpx(CX-px2,CY-py2,rc);fpx(CX+py2,CY+px2,rc);fpx(CX-py2,CY+px2,rc);fpx(CX+py2,CY-px2,rc);fpx(CX-py2,CY-px2,rc);};
            p(x,y);
            if(dd<0)dd+=2*x+3;else{dd+=2*(x-y)+5;y--;}x++;
        }
    }

    // 60 tick marks
    for (int t = 0; t < 60; t++) {
        bool  is_hour = (t % 5 == 0);
        float a  = t * PI2 / 60.0f - HALF_PI;
        int   r0 = is_hour ? 97  : 107;
        int   r1 = 113;
        int   tw = is_hour ? 3   : 1;
        uint16_t tc = is_hour
            ? BEEPI_WHITE
            : BeePi_GC9A01A::dimColor(BEEPI_WHITE, 100);

        int x0 = CX-(int)(r0*cosf(a)), y0 = CY+(int)(r0*sinf(a));
        int x1 = CX-(int)(r1*cosf(a)), y1 = CY+(int)(r1*sinf(a));
        fline(x0,y0,x1,y1,tc);
        if(tw>1){
            fline(x0+1,y0,x1+1,y1,tc); fline(x0-1,y0,x1-1,y1,tc);
            fline(x0,y0+1,x1,y1+1,tc); fline(x0,y0-1,x1,y1-1,tc);
        }
    }

    // 4 cardinal dots at 12/3/6/9 positions (larger)
    for (int q = 0; q < 4; q++) {
        float a = q * PI2 / 4.0f - HALF_PI;
        int cx2 = CX-(int)(104*cosf(a)), cy2 = CY+(int)(104*sinf(a));
        for(int dy2=-3;dy2<=3;dy2++)
            for(int dx2=-3;dx2<=3;dx2++)
                if(dx2*dx2+dy2*dy2<=9) fpx(cx2+dx2,cy2+dy2,BEEPI_WHITE);
    }

    uint64_t t_start = now_us();

    for (;;) {
        if (now_us() - t_start > (uint64_t)RUN_SECS * 1000000ull) break;

        // Get current wall time
        time_t tt = time(nullptr);
        struct tm *tm2 = localtime(&tt);
        int sec  = tm2->tm_sec;
        int minn = tm2->tm_min;
        int hour = tm2->tm_hour % 12;

        // Copy static face
        memcpy(fb, face, sizeof(fb));

        // Hand angles — 12 o'clock is -PI/2
        // Hour:   12 positions + minute interpolation
        // Minute: 60 positions + second interpolation
        // Second: 60 positions, no sub-second (1Hz wall clock)
        float ha = (hour * PI2 / 12.0f)
                 + (minn * PI2 / 720.0f)   // minute contribution
                 - HALF_PI;
        float ma = (minn * PI2 / 60.0f)
                 + (sec  * PI2 / 3600.0f)  // second contribution
                 - HALF_PI;
        float sa = (sec  * PI2 / 60.0f) - HALF_PI;

        // -- Hour hand (fat, white, length 55) --
        int hx = CX - (int)(55 * cosf(ha));
        int hy = CY + (int)(55 * sinf(ha));
        fb_thick_line(CX, CY, hx, hy, BEEPI_WHITE, 5);
        // Rounded tip
        fb_fcircle(hx, hy, 3, BEEPI_WHITE);

        // -- Minute hand (medium, light grey, length 82) --
        int mx = CX - (int)(82 * cosf(ma));
        int my = CY + (int)(82 * sinf(ma));
        fb_thick_line(CX, CY, mx, my, BEEPI_LIGHTGREY, 3);
        fb_fcircle(mx, my, 2, BEEPI_LIGHTGREY);

        // -- Second hand (thin, red, length 95 + 15 tail) --
        int sx = CX - (int)(95 * cosf(sa));
        int sy = CY + (int)(95 * sinf(sa));
        int tx = CX + (int)(18 * cosf(sa));   // counterbalance tail
        int ty = CY - (int)(18 * sinf(sa));
        fb_thick_line(tx, ty, sx, sy, BEEPI_RED, 2);
        // Small lollipop at tip
        fb_fcircle(sx, sy, 3, BEEPI_RED);

        // -- Centre cap --
        fb_fcircle(CX, CY, 8, BEEPI_WHITE);
        fb_fcircle(CX, CY, 5, BEEPI_RED);
        fb_fcircle(CX, CY, 2, BEEPI_WHITE);

        d.pushFrame(fb);

        // Digital readout overlaid below centre via library text
        char tbuf[10];
        snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d", tm2->tm_hour, minn, sec);
        int16_t tw2 = (int16_t)(strlen(tbuf) * 6);
        d.drawLabel(tbuf, 120 - tw2/2, 160, BEEPI_CYAN, 1, BEEPI_BLACK);
    }

    printf("  Scene 1: Clock done\n");
    delay_ms(300);
}

// ===========================================================================
// Scene 2 — Countdown Timer  5.000 s
// ===========================================================================

static void scene_timer(BeePi_GC9A01A &d)
{
    const float    PI2   = (float)(2.0 * M_PI);
    const uint64_t TOTAL = 5000000ull;
    const int      CX = 120, CY = 120;

    uint64_t t_start = now_us();

    for (;;) {
        uint64_t elapsed = now_us() - t_start;
        if (elapsed > TOTAL + 1000000ull) break;

        fb_clear();

        fb_circle(CX,CY,116, BeePi_GC9A01A::dimColor(BEEPI_DARKGREY,55));
        fb_circle(CX,CY,108, BeePi_GC9A01A::dimColor(BEEPI_DARKGREY,35));

        bool done = (elapsed >= TOTAL);
        float frac = done ? 0.0f : 1.0f - (float)elapsed/(float)TOTAL;

        if (!done) {
            float a0 = -(float)(M_PI/2.0f);
            float a1 = a0 + frac * PI2;
            uint16_t arc_col = BeePi_GC9A01A::lerpColor(BEEPI_RED, BEEPI_CYAN,
                                   (uint8_t)(frac*255));
            fb_arc(CX,CY,112, a0,a1, arc_col, 7);

            // Second tick marks
            for (int s=0; s<=5; s++) {
                float ta = -(float)(M_PI/2) + s*PI2/5.0f;
                fb_line(CX-(int)(105*cosf(ta)), CY+(int)(105*sinf(ta)),
                        CX-(int)(116*cosf(ta)), CY+(int)(116*sinf(ta)),
                        BEEPI_WHITE);
            }

            uint64_t rem  = TOTAL - elapsed;
            uint32_t secs = (uint32_t)(rem / 1000000ull);
            uint32_t ms   = (uint32_t)((rem % 1000000ull) / 1000);

            d.pushFrame(fb);

            char buf[12];
            snprintf(buf, sizeof(buf), "%02u:%03u", secs, ms);
            int16_t tw = (int16_t)(strlen(buf)*6*3);
            d.drawLabel(buf, 120-tw/2, 100, arc_col, 3, BEEPI_BLACK);

            d.drawLabel("sec : ms", 120-24, 136, BEEPI_DARKGREY, 1, BEEPI_BLACK);
            d.drawLabel("COUNTDOWN", 120-27, 32, BEEPI_WHITE, 1, BEEPI_BLACK);
        } else {
            d.pushFrame(fb);
            uint64_t flash = elapsed - TOTAL;
            if ((flash / 250000ull) % 2 == 0)
                d.drawLabel("GO!", 120-36, 96, BEEPI_GREEN, 4, BEEPI_BLACK);
        }
    }

    printf("  Scene 2: Timer done\n");
    delay_ms(300);
}

// ===========================================================================
// Scene 3 — RGB Rainbow
// ===========================================================================

static void scene_rainbow(BeePi_GC9A01A &d)
{
    const int RUN_SECS = 8;
    int hue_offset = 0;
    uint64_t t_start = now_us();

    for (;;) {
        if (now_us() - t_start > (uint64_t)RUN_SECS * 1000000ull) break;

        for (int y=0; y<240; y++) {
            for (int x=0; x<240; x++) {
                int dx=x-120, dy=y-120;
                if (dx*dx+dy*dy > 118*118) { fb[y*240+x]=BEEPI_BLACK; continue; }
                float angle = atan2f((float)dy,-(float)dx);  // mirror X
                int hue = (int)(angle*180.0f/(float)M_PI + 180.0f + hue_offset) % 360;
                float dist = sqrtf((float)(dx*dx+dy*dy));
                uint8_t bright = (uint8_t)(130 + dist*125/118);
                fb[y*240+x] = BeePi_GC9A01A::dimColor(hue565(hue), bright);
            }
        }
        fb_fcircle(120,120,22,BEEPI_WHITE);
        fb_fcircle(120,120,18,BEEPI_BLACK);

        d.pushFrame(fb);
        d.drawLabel("RGB", 120-9, 116, BEEPI_WHITE, 1, BEEPI_BLACK);

        hue_offset = (hue_offset + 3) % 360;
    }

    printf("  Scene 3: Rainbow done\n");
    delay_ms(300);
}

// ===========================================================================
// Scene 4 — Bouncing Balls
// ===========================================================================

static void scene_balls(BeePi_GC9A01A &d)
{
    const int   N        = 10;
    const int   RUN_SECS = 12;
    const float WALL     = 110.0f;
    const float DAMP     = 0.975f;
    const float PI2      = (float)(2.0*M_PI);

    struct Ball { float x,y,vx,vy; int r; uint16_t col; } balls[N];

    for (int i=0; i<N; i++) {
        float a  = i*PI2/N;
        float sp = 2.0f+(i%5)*0.5f;
        float va = a+(float)(M_PI/2)+(i%2?0.25f:-0.25f);
        balls[i] = { 120-40*cosf(a), 120+40*sinf(a),
                     -sp*cosf(va), sp*sinf(va),
                     6+(i%4)*2, hue565(i*36) };
    }

    fb_clear();
    uint64_t t_start = now_us();

    while (now_us()-t_start < (uint64_t)RUN_SECS*1000000ull) {
        // Trail decay
        for (int i=0; i<240*240; i++) {
            uint16_t c=fb[i];
            fb[i]=(uint16_t)((((c>>11)&0x1F)*12/16)<<11)
                 |(uint16_t)((((c>>5 )&0x3F)*12/16)<<5)
                 |(uint16_t)(( (c    )&0x1F)*12/16);
        }

        fb_circle(120,120,(int)WALL,   BeePi_GC9A01A::dimColor(BEEPI_CYAN,50));
        fb_circle(120,120,(int)WALL-1, BeePi_GC9A01A::dimColor(BEEPI_CYAN,30));

        for (int i=0; i<N; i++) {
            Ball &b=balls[i];
            b.x+=b.vx; b.y+=b.vy;

            float dx=b.x-120, dy=b.y-120, dist=sqrtf(dx*dx+dy*dy);
            float lim=WALL-b.r;
            if (dist>lim) {
                float nx=dx/dist, ny=dy/dist;
                b.x=120+nx*lim; b.y=120+ny*lim;
                float dot=b.vx*nx+b.vy*ny;
                b.vx=(b.vx-2*dot*nx)*DAMP;
                b.vy=(b.vy-2*dot*ny)*DAMP;
                float spd=sqrtf(b.vx*b.vx+b.vy*b.vy);
                if(spd<1.2f){b.vx*=1.2f/spd;b.vy*=1.2f/spd;}
            }

            fb_circle((int)b.x,(int)b.y,b.r+2, BeePi_GC9A01A::dimColor(b.col,55));
            fb_fcircle((int)b.x,(int)b.y,b.r,  b.col);
            fb_circle((int)b.x,(int)b.y,b.r,   BeePi_GC9A01A::dimColor(b.col,180));
            fb_fcircle((int)b.x-b.r/3,(int)b.y-b.r/3, 2,
                       BeePi_GC9A01A::lerpColor(b.col,BEEPI_WHITE,160));
        }

        d.pushFrame(fb);
    }

    printf("  Scene 4: Balls done\n");
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
        fprintf(stderr,"ERROR: display.begin() failed\n");
        return 1;
    }
    printf("Display OK (%dx%d)\n\n", display.width(), display.height());

    fb_clear();

    int cycle=0;
    for(;;){
        cycle++;
        printf("Cycle %d\n",cycle);
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