/*!
 * @file beepi_gc9a01a_nt.cpp
 *
 * BeeBotix GC9A01A Non-Touch Round Display Library — Implementation
 *
 * All GC9A01A register communication goes through beepi_hal_* calls.
 * All geometry/drawing is self-contained (no Adafruit_GFX on Linux).
 * Font rendering uses the built-in beepi_font5x8.h glyph table.
 *
 * BeeBotix Autonomous Systems
 */

#include "beepi_gc9a01a_nt.h"
#include "fonts/beepi_font5x8.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// Swap helper
// ---------------------------------------------------------------------------

static inline void _swap16(int16_t &a, int16_t &b) { int16_t t = a; a = b; b = t; }
static inline int16_t _abs16(int16_t v)             { return v < 0 ? -v : v; }
static inline int16_t _min16(int16_t a, int16_t b)  { return a < b ? a : b; }
static inline int16_t _max16(int16_t a, int16_t b)  { return a > b ? a : b; }
static inline int32_t _clamp32(int32_t v, int32_t lo, int32_t hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

// ---------------------------------------------------------------------------
// GC9A01A initialisation sequence
//
// Format: { cmd, n_params, param0, param1, … } repeated.
// End marker: 0x00 cmd with 0xFF as the param count — impossible in normal use.
// If n_params has bit7 set (and isn't the end marker), insert 150 ms delay.
//
// NOTE: 0xFF is a valid GC9A01A command (inter-register enable group 3).
// We cannot use 0xFF as an end sentinel — it collides with that command.
// End marker is encoded as { 0x00, 0xFF } which is otherwise unused.
// ---------------------------------------------------------------------------

#define BEEPI_INIT_END  0x00, 0xFF   // end of sequence marker

static const uint8_t GC9A01A_INIT_SEQ[] = {
    0xEF, 0,
    0xEB, 1, 0x14,
    0xFE, 0,
    0xEF, 0,
    0xEB, 1, 0x14,
    0x84, 1, 0x40,
    0x85, 1, 0xFF,
    0x86, 1, 0xFF,
    0x87, 1, 0xFF,
    0x88, 1, 0x0A,
    0x89, 1, 0x21,
    0x8A, 1, 0x00,
    0x8B, 1, 0x80,
    0x8C, 1, 0x01,
    0x8D, 1, 0x01,
    0x8E, 1, 0xFF,
    0x8F, 1, 0xFF,
    0xB6, 2, 0x00, 0x00,
    GC9A01A_MADCTL, 1, (GC9A01A_MADCTL_MY | GC9A01A_MADCTL_MX | GC9A01A_MADCTL_BGR),
    GC9A01A_COLMOD, 1, 0x05,   // 16-bit RGB565
    0x90, 4, 0x08, 0x08, 0x08, 0x08,
    0xBD, 1, 0x06,
    0xBC, 1, 0x00,
    0xFF, 3, 0x60, 0x01, 0x04,  // inter-register enable — valid 0xFF command
    0xC3, 1, 0x13,  // POWER2
    0xC4, 1, 0x13,  // POWER3
    0xC9, 1, 0x22,  // POWER4
    0xBE, 1, 0x11,
    0xE1, 2, 0x10, 0x0E,
    0xDF, 3, 0x21, 0x0C, 0x02,
    0xF0, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,  // GAMMA1
    0xF1, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,  // GAMMA2
    0xF2, 6, 0x45, 0x09, 0x08, 0x08, 0x26, 0x2A,  // GAMMA3
    0xF3, 6, 0x43, 0x70, 0x72, 0x36, 0x37, 0x6F,  // GAMMA4
    0xED, 2, 0x1B, 0x0B,
    0xAE, 1, 0x77,
    0xCD, 1, 0x63,
    0xE8, 1, 0x34,  // FRAMERATE
    0x62, 12, 0x18, 0x0D, 0x71, 0xED, 0x70, 0x70,
              0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70,
    0x63, 12, 0x18, 0x11, 0x71, 0xF1, 0x70, 0x70,
              0x18, 0x13, 0x71, 0xF3, 0x70, 0x70,
    0x64,  7, 0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07,
    0x66, 10, 0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00,
    0x67, 10, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98,
    0x74,  7, 0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00,
    0x98,  2, 0x3E, 0x07,
    0x35, 0,           // TEON
    0x21, 0,           // INVON
    0x11, 0x80,        // SLPOUT + 150 ms delay
    0x29, 0x80,        // DISPON + 150 ms delay
    BEEPI_INIT_END
};

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

BeePi_GC9A01A::BeePi_GC9A01A(const BeePiHALConfig &cfg)
    : _cfg(cfg),
      _width(BEEPI_DISP_W),
      _height(BEEPI_DISP_H),
      _rotation(0),
      _initialised(false)
{}

BeePi_GC9A01A::~BeePi_GC9A01A()
{
    end();
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

bool BeePi_GC9A01A::begin()
{
    int rc = beepi_hal_init(&_cfg);
    if (rc < 0) {
        fprintf(stderr, "BeePi_GC9A01A::begin() HAL init failed: %d\n", rc);
        return false;
    }

    // Hardware reset
    if (_cfg.gpio_rst >= 0) {
        beepi_hal_rst(1);
        beepi_hal_delay_ms(10);
        beepi_hal_rst(0);
        beepi_hal_delay_ms(20);
        beepi_hal_rst(1);
        beepi_hal_delay_ms(150);
    } else {
        // Software reset
        beepi_hal_write_cmd(GC9A01A_SWRESET);
        beepi_hal_delay_ms(150);
    }

    _initSequence();

    _width  = BEEPI_DISP_W;
    _height = BEEPI_DISP_H;
    _initialised = true;

    // Backlight on if pin configured
    if (_cfg.gpio_bl >= 0) {
        beepi_hal_bl(1);
    }

    return true;
}

void BeePi_GC9A01A::end()
{
    if (_initialised) {
        beepi_hal_deinit();
        _initialised = false;
    }
}

void BeePi_GC9A01A::_initSequence()
{
    const uint8_t *p = GC9A01A_INIT_SEQ;
    for (;;) {
        uint8_t cmd     = *p++;
        uint8_t arg     = *p++;

        // End marker: cmd=0x00, arg=0xFF
        if (cmd == 0x00 && arg == 0xFF) break;

        uint8_t nparams = arg & 0x7F;
        bool    delay   = (arg & 0x80) != 0;

        beepi_hal_write_cmd(cmd);
        for (uint8_t i = 0; i < nparams; i++) {
            beepi_hal_write_data(*p++);
        }
        if (delay) {
            beepi_hal_delay_ms(150);
        }
    }
}

void BeePi_GC9A01A::setDisplayOn(bool on)
{
    beepi_hal_write_cmd(on ? GC9A01A_DISPON : GC9A01A_DISPOFF);
}

void BeePi_GC9A01A::invertDisplay(bool invert)
{
    beepi_hal_write_cmd(invert ? GC9A01A_INVON : GC9A01A_INVOFF);
}

void BeePi_GC9A01A::setRotation(uint8_t r)
{
    _rotation = r & 3;
    uint8_t madctl = 0;
    switch (_rotation) {
        case 0:
            madctl   = GC9A01A_MADCTL_MY | GC9A01A_MADCTL_MX | GC9A01A_MADCTL_BGR;
            _width   = BEEPI_DISP_W;
            _height  = BEEPI_DISP_H;
            break;
        case 1:
            madctl   = GC9A01A_MADCTL_MV | GC9A01A_MADCTL_MX | GC9A01A_MADCTL_BGR;
            _width   = BEEPI_DISP_H;
            _height  = BEEPI_DISP_W;
            break;
        case 2:
            madctl   = GC9A01A_MADCTL_BGR;
            _width   = BEEPI_DISP_W;
            _height  = BEEPI_DISP_H;
            break;
        case 3:
            madctl   = GC9A01A_MADCTL_MY | GC9A01A_MADCTL_MV | GC9A01A_MADCTL_BGR;
            _width   = BEEPI_DISP_H;
            _height  = BEEPI_DISP_W;
            break;
    }
    beepi_hal_write_cmd(GC9A01A_MADCTL);
    beepi_hal_write_data(madctl);
}

void BeePi_GC9A01A::setBacklight(bool on)
{
    if (_cfg.gpio_bl >= 0) beepi_hal_bl(on ? 1 : 0);
}

void BeePi_GC9A01A::sendCommand(uint8_t cmd)
{
    beepi_hal_write_cmd(cmd);
}

void BeePi_GC9A01A::sendCommand(uint8_t cmd, const uint8_t *params, uint8_t n)
{
    beepi_hal_write_cmd(cmd);
    for (uint8_t i = 0; i < n; i++) {
        beepi_hal_write_data(params[i]);
    }
}

// ---------------------------------------------------------------------------
// Address window
// ---------------------------------------------------------------------------

void BeePi_GC9A01A::_setAddrWindow(uint16_t x0, uint16_t y0,
                                    uint16_t x1, uint16_t y1)
{
    beepi_hal_write_cmd(GC9A01A_CASET);
    uint8_t caset[4] = {
        (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF),
        (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF)
    };
    beepi_hal_write_buf(caset, 4);

    beepi_hal_write_cmd(GC9A01A_RASET);
    uint8_t raset[4] = {
        (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF),
        (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF)
    };
    beepi_hal_write_buf(raset, 4);

    beepi_hal_write_cmd(GC9A01A_RAMWR);
}

// ---------------------------------------------------------------------------
// Core drawing — fill / pixel / line
// ---------------------------------------------------------------------------

void BeePi_GC9A01A::fill(uint16_t color)
{
    fillRect(0, 0, _width, _height, color);
}

void BeePi_GC9A01A::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                              uint16_t color)
{
    if (x >= _width || y >= _height || w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > _width)  w = _width  - x;
    if (y + h > _height) h = _height - y;

    _setAddrWindow((uint16_t)x, (uint16_t)y,
                   (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));

    size_t row_pixels = (size_t)w;
    uint16_t *row = (uint16_t *)malloc(row_pixels * sizeof(uint16_t));
    if (!row) return;
    for (size_t i = 0; i < row_pixels; i++) row[i] = color;
    for (int16_t r = 0; r < h; r++) {
        beepi_hal_write_pixels(row, row_pixels);
    }
    free(row);
}

void BeePi_GC9A01A::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= _width || y >= _height) return;
    _setAddrWindow((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)y);
    beepi_hal_write_pixels(&color, 1);
}

void BeePi_GC9A01A::drawHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    if (y < 0 || y >= _height || x >= _width || w <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > _width) w = _width - x;
    if (w <= 0) return;

    _setAddrWindow((uint16_t)x, (uint16_t)y,
                   (uint16_t)(x + w - 1), (uint16_t)y);
    uint16_t *row = (uint16_t *)malloc((size_t)w * sizeof(uint16_t));
    if (!row) return;
    for (int16_t i = 0; i < w; i++) row[i] = color;
    beepi_hal_write_pixels(row, (size_t)w);
    free(row);
}

void BeePi_GC9A01A::drawVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    fillRect(x, y, 1, h, color);
}

void BeePi_GC9A01A::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                              uint16_t color)
{
    if (x0 == x1) { drawVLine(x0, _min16(y0,y1), _abs16(y1-y0)+1, color); return; }
    if (y0 == y1) { drawHLine(_min16(x0,x1), y0, _abs16(x1-x0)+1, color); return; }

    // Bresenham
    bool steep = _abs16(y1 - y0) > _abs16(x1 - x0);
    if (steep)   { _swap16(x0,y0); _swap16(x1,y1); }
    if (x0 > x1) { _swap16(x0,x1); _swap16(y0,y1); }

    int16_t dx = x1 - x0;
    int16_t dy = _abs16(y1 - y0);
    int16_t err = dx / 2;
    int16_t ystep = (y0 < y1) ? 1 : -1;

    for (int16_t x = x0; x <= x1; x++) {
        if (steep) drawPixel(y0, x, color);
        else       drawPixel(x, y0, color);
        err -= dy;
        if (err < 0) { y0 += ystep; err += dx; }
    }
}

void BeePi_GC9A01A::drawRect(int16_t x, int16_t y, int16_t w, int16_t h,
                              uint16_t color)
{
    drawHLine(x, y,         w, color);
    drawHLine(x, y + h - 1, w, color);
    drawVLine(x,         y, h, color);
    drawVLine(x + w - 1, y, h, color);
}

// ---------------------------------------------------------------------------
// Circle drawing (Bresenham midpoint algorithm)
// ---------------------------------------------------------------------------

void BeePi_GC9A01A::_drawCircleHelper(int16_t cx, int16_t cy, int16_t r,
                                       uint8_t q, uint16_t color)
{
    int16_t f = 1 - r, ddx = 1, ddy = -2 * r, x = 0, y = r;
    while (x < y) {
        if (f >= 0) { y--; ddy += 2; f += ddy; }
        x++; ddx += 2; f += ddx;
        if (q & 0x4) { drawPixel(cx+x, cy+y, color); drawPixel(cx+y, cy+x, color); }
        if (q & 0x2) { drawPixel(cx+x, cy-y, color); drawPixel(cx+y, cy-x, color); }
        if (q & 0x8) { drawPixel(cx-y, cy+x, color); drawPixel(cx-x, cy+y, color); }
        if (q & 0x1) { drawPixel(cx-y, cy-x, color); drawPixel(cx-x, cy-y, color); }
    }
}

void BeePi_GC9A01A::drawCircle(int16_t cx, int16_t cy, int16_t r, uint16_t color)
{
    int16_t f = 1 - r, ddx = 1, ddy = -2 * r, x = 0, y = r;
    drawPixel(cx,   cy+r, color);
    drawPixel(cx,   cy-r, color);
    drawPixel(cx+r, cy,   color);
    drawPixel(cx-r, cy,   color);
    while (x < y) {
        if (f >= 0) { y--; ddy += 2; f += ddy; }
        x++; ddx += 2; f += ddx;
        drawPixel(cx+x, cy+y, color); drawPixel(cx-x, cy+y, color);
        drawPixel(cx+x, cy-y, color); drawPixel(cx-x, cy-y, color);
        drawPixel(cx+y, cy+x, color); drawPixel(cx-y, cy+x, color);
        drawPixel(cx+y, cy-x, color); drawPixel(cx-y, cy-x, color);
    }
}

void BeePi_GC9A01A::_fillCircleHelper(int16_t cx, int16_t cy, int16_t r,
                                       uint8_t sides, int16_t delta,
                                       uint16_t color)
{
    int16_t f = 1-r, ddx = 1, ddy = -2*r, x = 0, y = r;
    int16_t px = x, py = y;
    delta++;
    while (x < y) {
        if (f >= 0) { y--; ddy += 2; f += ddy; }
        x++; ddx += 2; f += ddx;
        if (x < y+1) {
            if (sides & 1) drawVLine(cx+x, cy-y, 2*y+delta, color);
            if (sides & 2) drawVLine(cx-x, cy-y, 2*y+delta, color);
        }
        if (y != py) {
            if (sides & 1) drawVLine(cx+py, cy-px, 2*px+delta, color);
            if (sides & 2) drawVLine(cx-py, cy-px, 2*px+delta, color);
            py = y;
        }
        px = x;
    }
}

void BeePi_GC9A01A::fillCircle(int16_t cx, int16_t cy, int16_t r, uint16_t color)
{
    drawVLine(cx, cy-r, 2*r+1, color);
    _fillCircleHelper(cx, cy, r, 3, 0, color);
}

// ---------------------------------------------------------------------------
// Triangles
// ---------------------------------------------------------------------------

void BeePi_GC9A01A::drawTriangle(int16_t x0, int16_t y0,
                                  int16_t x1, int16_t y1,
                                  int16_t x2, int16_t y2, uint16_t color)
{
    drawLine(x0,y0,x1,y1,color);
    drawLine(x1,y1,x2,y2,color);
    drawLine(x2,y2,x0,y0,color);
}

void BeePi_GC9A01A::fillTriangle(int16_t x0, int16_t y0,
                                  int16_t x1, int16_t y1,
                                  int16_t x2, int16_t y2, uint16_t color)
{
    // Sort by Y
    if (y0 > y1) { _swap16(y0,y1); _swap16(x0,x1); }
    if (y1 > y2) { _swap16(y2,y1); _swap16(x2,x1); }
    if (y0 > y1) { _swap16(y0,y1); _swap16(x0,x1); }

    if (y0 == y2) {
        int16_t a = _min16(x0,_min16(x1,x2));
        int16_t b = _max16(x0,_max16(x1,x2));
        drawHLine(a, y0, b-a+1, color);
        return;
    }

    int16_t dx01 = x1-x0, dy01 = y1-y0;
    int16_t dx02 = x2-x0, dy02 = y2-y0;
    int16_t dx12 = x2-x1, dy12 = y2-y1;
    int32_t sa = 0, sb = 0;

    int16_t last = (y1 == y2) ? y1 : y1-1;
    for (int16_t y = y0; y <= last; y++) {
        int16_t a = (int16_t)(x0 + sa/dy01);
        int16_t b = (int16_t)(x0 + sb/dy02);
        sa += dx01; sb += dx02;
        if (a > b) _swap16(a,b);
        drawHLine(a, y, b-a+1, color);
    }
    sa = (int32_t)dx12 * (last+1 - y1);
    sb = (int32_t)dx02 * (last+1 - y0);
    for (int16_t y = last+1; y <= y2; y++) {
        int16_t a = (int16_t)(x1 + sa/dy12);
        int16_t b = (int16_t)(x0 + sb/dy02);
        sa += dx12; sb += dx02;
        if (a > b) _swap16(a,b);
        drawHLine(a, y, b-a+1, color);
    }
}

// ---------------------------------------------------------------------------
// Rounded rectangle
// ---------------------------------------------------------------------------

void BeePi_GC9A01A::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                   int16_t r, uint16_t color)
{
    int16_t mr = _min16(w,h)/2;
    if (r > mr) r = mr;
    drawHLine(x+r,     y,         w-2*r, color);
    drawHLine(x+r,     y+h-1,     w-2*r, color);
    drawVLine(x,       y+r,       h-2*r, color);
    drawVLine(x+w-1,   y+r,       h-2*r, color);
    _drawCircleHelper(x+r,     y+r,     r, 1, color);
    _drawCircleHelper(x+w-r-1, y+r,     r, 2, color);
    _drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
    _drawCircleHelper(x+r,     y+h-r-1, r, 8, color);
}

void BeePi_GC9A01A::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                   int16_t r, uint16_t color)
{
    int16_t mr = _min16(w,h)/2;
    if (r > mr) r = mr;
    fillRect(x+r, y, w-2*r, h, color);
    _fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
    _fillCircleHelper(x+r,     y+r, r, 2, h-2*r-1, color);
}

// ---------------------------------------------------------------------------
// Text rendering — built-in 5x8 font
// ---------------------------------------------------------------------------

void BeePi_GC9A01A::_drawChar(int16_t x, int16_t y, char c,
                               uint16_t color, uint16_t bg, uint8_t scale)
{
    uint8_t uc = (uint8_t)c;
    if (uc < BEEPI_FONT_FIRST || uc > BEEPI_FONT_LAST) uc = '?';
    const uint8_t *glyph = BEEPI_FONT[uc - BEEPI_FONT_FIRST];

    bool transparent = (color == bg);

    for (int8_t col = 0; col < BEEPI_FONT_W; col++) {
        uint8_t bits = glyph[col];
        for (int8_t row = 0; row < BEEPI_FONT_H; row++) {
            uint16_t px_color = (bits & 0x80) ? color : bg;
            if (!(bits & 0x80) && transparent) { bits <<= 1; continue; }
            if (scale == 1) {
                drawPixel(x + col, y + row, px_color);
            } else {
                fillRect(x + col * scale, y + row * scale,
                         scale, scale, px_color);
            }
            bits <<= 1;
        }
    }
    // Right gap column
    if (!transparent) {
        fillRect(x + BEEPI_FONT_W * scale, y, scale, BEEPI_FONT_H * scale, bg);
    }
}

void BeePi_GC9A01A::drawLabel(const char *text, int16_t x, int16_t y,
                               uint16_t color, uint8_t scale, uint16_t bg)
{
    if (!text) return;
    int16_t cx = x;
    for (const char *p = text; *p; p++) {
        _drawChar(cx, y, *p, color, bg, scale);
        cx += BEEPI_FONT_ADVANCE * scale;
    }
}

void BeePi_GC9A01A::_textDimensions(const char *text, uint8_t scale,
                                     int16_t *outW, int16_t *outH) const
{
    *outW = (int16_t)(strlen(text) * BEEPI_FONT_ADVANCE * scale);
    *outH = (int16_t)(BEEPI_FONT_H * scale);
}

void BeePi_GC9A01A::drawLabelAnchored(const char *text, BeePiAnchor anchor,
                                       int16_t margin, uint16_t color,
                                       uint8_t scale, uint16_t bg)
{
    int16_t tw, th;
    _textDimensions(text, scale, &tw, &th);
    int16_t x = 0, y = 0;
    switch (anchor) {
        case BEEPI_ANCHOR_TOP_LEFT:     x = margin;          y = margin;             break;
        case BEEPI_ANCHOR_TOP_RIGHT:    x = _width-tw-margin; y = margin;             break;
        case BEEPI_ANCHOR_BOTTOM_LEFT:  x = margin;          y = _height-th-margin;  break;
        case BEEPI_ANCHOR_BOTTOM_RIGHT: x = _width-tw-margin; y = _height-th-margin; break;
        case BEEPI_ANCHOR_CENTER:       x = (_width-tw)/2;   y = (_height-th)/2;     break;
    }
    drawLabel(text, x, y, color, scale, bg);
}

void BeePi_GC9A01A::drawInt(int32_t value, const char *suffix,
                             int16_t x, int16_t y, uint16_t color, uint8_t scale)
{
    char buf[24];
    if (suffix) snprintf(buf, sizeof(buf), "%d%s", (int)value, suffix);
    else        snprintf(buf, sizeof(buf), "%d",   (int)value);
    drawLabel(buf, x, y, color, scale, color);  // transparent bg
}

void BeePi_GC9A01A::drawFloat(float value, uint8_t decimals, const char *suffix,
                               int16_t x, int16_t y, uint16_t color, uint8_t scale)
{
    char fmt[12], buf[32];
    snprintf(fmt, sizeof(fmt), "%%.%df%%s", (int)decimals);
    snprintf(buf, sizeof(buf), fmt, value, suffix ? suffix : "");
    drawLabel(buf, x, y, color, scale, color);
}

// ---------------------------------------------------------------------------
// HUD / Reticle
// ---------------------------------------------------------------------------

void BeePi_GC9A01A::_drawThickLine(int16_t x0, int16_t y0,
                                    int16_t x1, int16_t y1,
                                    uint16_t color, uint8_t thick)
{
    drawLine(x0, y0, x1, y1, color);
    if (thick >= 2) {
        if (_abs16(x1-x0) >= _abs16(y1-y0)) {
            drawLine(x0, y0-1, x1, y1-1, color);
            drawLine(x0, y0+1, x1, y1+1, color);
        } else {
            drawLine(x0-1, y0, x1-1, y1, color);
            drawLine(x0+1, y0, x1+1, y1, color);
        }
    }
}

void BeePi_GC9A01A::drawReticle(int16_t cx, int16_t cy, int16_t size,
                                 int16_t gap, BeePiReticleStyle style,
                                 uint16_t color, uint8_t thick)
{
    switch (style) {
        case BEEPI_RETICLE_CROSSHAIR:
            _drawThickLine(cx-size, cy,      cx+size, cy,      color, thick);
            _drawThickLine(cx,      cy-size, cx,      cy+size, color, thick);
            break;

        case BEEPI_RETICLE_CROSS_GAP:
            _drawThickLine(cx-size, cy,     cx-gap,  cy,     color, thick);
            _drawThickLine(cx+gap,  cy,     cx+size, cy,     color, thick);
            _drawThickLine(cx,      cy-size, cx,     cy-gap,  color, thick);
            _drawThickLine(cx,      cy+gap,  cx,     cy+size, color, thick);
            break;

        case BEEPI_RETICLE_CIRCLE_CROSS: {
            drawCircle(cx, cy, gap, color);
            int16_t tlen = _max16(size - gap, 2);
            _drawThickLine(cx-gap-tlen, cy,     cx-gap,      cy,     color, thick);
            _drawThickLine(cx+gap,      cy,     cx+gap+tlen, cy,     color, thick);
            _drawThickLine(cx,     cy-gap-tlen, cx,     cy-gap,      color, thick);
            _drawThickLine(cx,     cy+gap,      cx,     cy+gap+tlen, color, thick);
            break;
        }
        case BEEPI_RETICLE_DOT:
            fillCircle(cx, cy, thick > 1 ? thick : 2, color);
            break;
    }
}

void BeePi_GC9A01A::drawBearingArc(float bearing, float fovDeg,
                                    int16_t r, uint16_t arcColor,
                                    uint16_t tickColor)
{
    const float DEG2RAD = (float)(M_PI / 180.0);
    const int   STEPS   = 60;
    float half = fovDeg * 0.5f;

    for (int i = 0; i < STEPS; i++) {
        float a0 = (bearing - half + fovDeg * (float)i       / STEPS - 90.0f) * DEG2RAD;
        float a1 = (bearing - half + fovDeg * (float)(i + 1) / STEPS - 90.0f) * DEG2RAD;
        drawLine(
            (int16_t)(BEEPI_DISP_CX + r * cosf(a0)),
            (int16_t)(BEEPI_DISP_CY + r * sinf(a0)),
            (int16_t)(BEEPI_DISP_CX + r * cosf(a1)),
            (int16_t)(BEEPI_DISP_CY + r * sinf(a1)),
            arcColor
        );
    }
    float ta = (bearing - 90.0f) * DEG2RAD;
    drawLine(
        (int16_t)(BEEPI_DISP_CX + (r-6) * cosf(ta)),
        (int16_t)(BEEPI_DISP_CY + (r-6) * sinf(ta)),
        (int16_t)(BEEPI_DISP_CX + (r+6) * cosf(ta)),
        (int16_t)(BEEPI_DISP_CY + (r+6) * sinf(ta)),
        tickColor
    );
}

void BeePi_GC9A01A::drawBarH(int16_t x, int16_t y, int16_t w, int16_t h,
                              int32_t value, int32_t maxVal,
                              uint16_t barColor, uint16_t bgColor,
                              uint16_t border)
{
    value = _clamp32(value, 0, maxVal);
    int16_t filled = (maxVal > 0) ? (int16_t)((int32_t)w * value / maxVal) : 0;
    fillRect(x, y, w, h, bgColor);
    if (filled > 0) fillRect(x, y, filled, h, barColor);
    if (border != bgColor) drawRect(x, y, w, h, border);
}

void BeePi_GC9A01A::drawBadge(const char *label, int32_t value,
                               const char *unit,
                               int16_t x, int16_t y,
                               uint16_t fgColor, uint16_t bgColor)
{
    char valBuf[20];
    if (unit) snprintf(valBuf, sizeof(valBuf), "%d%s", (int)value, unit);
    else      snprintf(valBuf, sizeof(valBuf), "%d",   (int)value);

    int16_t labelW = (int16_t)(strlen(label)  * BEEPI_FONT_ADVANCE);
    int16_t valW   = (int16_t)(strlen(valBuf) * BEEPI_FONT_ADVANCE);
    int16_t boxW   = _max16(labelW, valW) + 6;
    int16_t boxH   = 20;

    fillRect(x, y, boxW, boxH, bgColor);
    drawRect(x, y, boxW, boxH, fgColor);
    drawLabel(label,  x+3, y+2,  dimColor(fgColor, 160), 1, bgColor);
    drawLabel(valBuf, x+3, y+11, fgColor,                  1, bgColor);
}

// ---------------------------------------------------------------------------
// Camera feed / image push
// ---------------------------------------------------------------------------

void BeePi_GC9A01A::pushFrame(const uint16_t *framebuf)
{
    _setAddrWindow(0, 0, BEEPI_DISP_W-1, BEEPI_DISP_H-1);
    beepi_hal_write_pixels(framebuf, (size_t)BEEPI_DISP_W * BEEPI_DISP_H);
}

void BeePi_GC9A01A::pushRegion(int16_t x, int16_t y, int16_t w, int16_t h,
                                const uint16_t *pixels)
{
    if (w <= 0 || h <= 0) return;
    _setAddrWindow((uint16_t)x, (uint16_t)y,
                   (uint16_t)(x+w-1), (uint16_t)(y+h-1));
    beepi_hal_write_pixels(pixels, (size_t)w * h);
}

void BeePi_GC9A01A::drawImage(int16_t x, int16_t y, const uint16_t *bitmap,
                               int16_t w, int16_t h)
{
    pushRegion(x, y, w, h, bitmap);
}

// ---------------------------------------------------------------------------
// Colour utilities
// ---------------------------------------------------------------------------

uint16_t BeePi_GC9A01A::rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((uint16_t)(r & 0xF8) << 8) |
                      ((uint16_t)(g & 0xFC) << 3) |
                       (uint16_t)(b >> 3));
}

uint16_t BeePi_GC9A01A::lerpColor(uint16_t c0, uint16_t c1, uint8_t t)
{
    uint8_t r0 = (c0>>11)&0x1F, g0 = (c0>>5)&0x3F, b0 = c0&0x1F;
    uint8_t r1 = (c1>>11)&0x1F, g1 = (c1>>5)&0x3F, b1 = c1&0x1F;
    uint8_t r  = (uint8_t)(r0 + ((int16_t)(r1-r0)*t)/255);
    uint8_t g  = (uint8_t)(g0 + ((int16_t)(g1-g0)*t)/255);
    uint8_t b  = (uint8_t)(b0 + ((int16_t)(b1-b0)*t)/255);
    return (uint16_t)((r<<11)|(g<<5)|b);
}

uint16_t BeePi_GC9A01A::dimColor(uint16_t color, uint8_t factor)
{
    uint8_t r = (uint8_t)(((color>>11)&0x1F) * factor / 255);
    uint8_t g = (uint8_t)(((color>>5 )&0x3F) * factor / 255);
    uint8_t b = (uint8_t)(( color     &0x1F) * factor / 255);
    return (uint16_t)((r<<11)|(g<<5)|b);
}