/*!
 * @file beepi_gc9a01a_nt.h
 *
 * BeeBotix GC9A01A Non-Touch Round Display Library — Public API
 *
 * Platform-agnostic wrapper for the 240x240 GC9A01A round TFT display.
 * Compiles on:
 *
 *   Linux / Raspberry Pi
 *     Pi 3B+   (ARMv8 32-bit, BCM2837)
 *     Pi Zero 2W (ARMv8 32/64-bit, BCM2837B0)
 *     Pi 4B    (ARMv8 64-bit, BCM2711)
 *     Pi 5     (ARMv8 64-bit, BCM2712 + RP1 southbridge)
 *
 *   Arduino-family  (ESP32, RP2040, SAMD, AVR, STM32, Teensy)
 *     Requires Adafruit_GFX + Adafruit_GC9A01A
 *     Define BEEPI_PLATFORM_ARDUINO before including this header
 *
 * No Arduino runtime, no PROGMEM, no avr/ headers required on Linux.
 * All types are standard stdint.h types.
 *
 * BeeBotix Autonomous Systems
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "hal/beepi_hal.h"

// ---------------------------------------------------------------------------
// Font geometry (built-in 5x8 bitmap font)
// ---------------------------------------------------------------------------

#define BEEPI_FONT_W        5   ///< Glyph width in pixels
#define BEEPI_FONT_H        8   ///< Glyph height in pixels
#define BEEPI_FONT_ADVANCE  6   ///< Pixels per character (W + 1px gap)

// ---------------------------------------------------------------------------
// Display geometry
// ---------------------------------------------------------------------------

#define BEEPI_DISP_W      240    ///< Display width pixels
#define BEEPI_DISP_H      240    ///< Display height pixels
#define BEEPI_DISP_CX     120    ///< Centre X
#define BEEPI_DISP_CY     120    ///< Centre Y
#define BEEPI_DISP_RADIUS 120    ///< Usable circle radius

// ---------------------------------------------------------------------------
// GC9A01A register addresses (for reference / direct sendCommand use)
// ---------------------------------------------------------------------------

#define GC9A01A_SWRESET   0x01
#define GC9A01A_SLPOUT    0x11
#define GC9A01A_NORON     0x13
#define GC9A01A_INVOFF    0x20
#define GC9A01A_INVON     0x21
#define GC9A01A_DISPOFF   0x28
#define GC9A01A_DISPON    0x29
#define GC9A01A_CASET     0x2A
#define GC9A01A_RASET     0x2B
#define GC9A01A_RAMWR     0x2C
#define GC9A01A_MADCTL    0x36
#define GC9A01A_COLMOD    0x3A
#define GC9A01A_INREGEN1  0xFE
#define GC9A01A_INREGEN2  0xEF

// MADCTL bits
#define GC9A01A_MADCTL_MY  0x80
#define GC9A01A_MADCTL_MX  0x40
#define GC9A01A_MADCTL_MV  0x20
#define GC9A01A_MADCTL_BGR 0x08

// ---------------------------------------------------------------------------
// RGB565 colour palette
// ---------------------------------------------------------------------------

#define BEEPI_BLACK      ((uint16_t)0x0000)
#define BEEPI_WHITE      ((uint16_t)0xFFFF)
#define BEEPI_RED        ((uint16_t)0x00F8)
#define BEEPI_GREEN      ((uint16_t)0xE007)
#define BEEPI_BLUE       ((uint16_t)0x1F00)
#define BEEPI_CYAN       ((uint16_t)0xFF07)
#define BEEPI_MAGENTA    ((uint16_t)0x1FF8)
#define BEEPI_YELLOW     ((uint16_t)0xE0FF)
#define BEEPI_ORANGE     ((uint16_t)0x20FD)
#define BEEPI_DARKGREY   ((uint16_t)0xEF7B)
#define BEEPI_LIGHTGREY  ((uint16_t)0x18C6)
#define BEEPI_NAVY       ((uint16_t)0x0F00)
#define BEEPI_DARKGREEN  ((uint16_t)0xE003)
#define BEEPI_MAROON     ((uint16_t)0x0078)
#define BEEPI_OLIVE      ((uint16_t)0xE07B)
#define BEEPI_PURPLE     ((uint16_t)0x0F78)
#define BEEPI_PINK       ((uint16_t)0x18FC)

/// Compile-time RGB888 → RGB565 conversion (r/g/b each 0–255)
#define BEEPI_RGB(r, g, b) \
    ((uint16_t)( (((uint16_t)(r) & 0xF8u) << 8) | \
                 (((uint16_t)(g) & 0xFCu) << 3) | \
                 ( (uint16_t)(b)           >> 3) ))

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Targeting reticle drawing style
typedef enum {
    BEEPI_RETICLE_CROSSHAIR    = 0,  ///< Full crosshair through centre
    BEEPI_RETICLE_CROSS_GAP    = 1,  ///< Crosshair with a clear gap at centre
    BEEPI_RETICLE_CIRCLE_CROSS = 2,  ///< Circle + four tick marks
    BEEPI_RETICLE_DOT          = 3,  ///< Filled dot only
} BeePiReticleStyle;

/// Text / label anchor position
typedef enum {
    BEEPI_ANCHOR_TOP_LEFT     = 0,
    BEEPI_ANCHOR_TOP_RIGHT    = 1,
    BEEPI_ANCHOR_BOTTOM_LEFT  = 2,
    BEEPI_ANCHOR_BOTTOM_RIGHT = 3,
    BEEPI_ANCHOR_CENTER       = 4,
} BeePiAnchor;

// ---------------------------------------------------------------------------
// BeePi_GC9A01A C++ class
// ---------------------------------------------------------------------------

#ifdef __cplusplus

#include <math.h>
#include <stdio.h>
#include <string.h>

/*!
 * @brief  BeeBotix GC9A01A display driver — main class.
 *
 * Construct with a filled BeePiHALConfig, call begin(), then draw.
 *
 * Linux / Raspberry Pi quick-start:
 * @code
 *   BeePiHALConfig cfg = {};
 *   cfg.spi_device  = "/dev/spidev0.0";
 *   cfg.spi_speed_hz = 40000000;
 *   cfg.gpio_dc      = 25;           // BCM 25
 *   cfg.gpio_rst     = 27;           // BCM 27 (-1 if not wired)
 *   cfg.gpio_bl      = 18;           // BCM 18 (-1 if always-on)
 *   cfg.gpio_chip    = "/dev/gpiochip0";  // Pi 5: "/dev/gpiochip4"
 *
 *   BeePi_GC9A01A display(cfg);
 *   display.begin();
 *   display.fill(BEEPI_BLACK);
 *   display.drawLabel("BeeBotix", 60, 110, BEEPI_GREEN, 2);
 * @endcode
 */
class BeePi_GC9A01A {
public:
    // ------------------------------------------------------------------
    // Constructor / destructor
    // ------------------------------------------------------------------

    /*!
     * @brief  Construct display object.
     * @param  cfg  Hardware configuration. Copied internally.
     */
    explicit BeePi_GC9A01A(const BeePiHALConfig &cfg);
    ~BeePi_GC9A01A();

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /*!
     * @brief  Initialise hardware and send GC9A01A init sequence.
     * @return true on success, false on HAL init failure.
     */
    bool begin();

    /// Release all HAL resources (SPI fd, GPIO handles).
    void end();

    /*!
     * @brief  Turn the display panel on or off.
     * @param  on  true = panel on, false = panel off (backlight unaffected).
     */
    void setDisplayOn(bool on);

    /*!
     * @brief  Invert all pixel colours.
     * @param  invert  true = inverted, false = normal.
     */
    void invertDisplay(bool invert);

    /*!
     * @brief  Set display rotation (0–3, each step = 90 degrees CW).
     * @param  r  Rotation index.
     */
    void setRotation(uint8_t r);

    /*!
     * @brief  Control backlight GPIO (if configured).
     * @param  on  true = backlight on.
     */
    void setBacklight(bool on);

    // ------------------------------------------------------------------
    // Core drawing primitives
    // ------------------------------------------------------------------

    /// Fill entire display with colour.
    void fill(uint16_t color);

    /// Fill rectangle.
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

    /// Draw rectangle outline.
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

    /// Draw a single pixel.
    void drawPixel(int16_t x, int16_t y, uint16_t color);

    /// Draw a line (Bresenham).
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);

    /// Draw horizontal line (fast DMA path).
    void drawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

    /// Draw vertical line (fast path).
    void drawVLine(int16_t x, int16_t y, int16_t h, uint16_t color);

    /// Draw circle outline (Bresenham midpoint).
    void drawCircle(int16_t cx, int16_t cy, int16_t r, uint16_t color);

    /// Draw filled circle.
    void fillCircle(int16_t cx, int16_t cy, int16_t r, uint16_t color);

    /// Draw triangle outline.
    void drawTriangle(int16_t x0, int16_t y0,
                      int16_t x1, int16_t y1,
                      int16_t x2, int16_t y2, uint16_t color);

    /// Draw filled triangle (scanline fill).
    void fillTriangle(int16_t x0, int16_t y0,
                      int16_t x1, int16_t y1,
                      int16_t x2, int16_t y2, uint16_t color);

    /// Draw rounded rectangle outline.
    void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                       int16_t r, uint16_t color);

    /// Draw filled rounded rectangle.
    void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                       int16_t r, uint16_t color);

    // ------------------------------------------------------------------
    // Text / OSD
    // ------------------------------------------------------------------

    /*!
     * @brief  Draw a string at pixel position (x, y).
     * @param  text    Null-terminated C string (printable ASCII).
     * @param  x       Left edge in pixels.
     * @param  y       Top edge in pixels.
     * @param  color   Text colour (RGB565).
     * @param  scale   Font scale (1 = 5x8 px, 2 = 10x16 px, …).
     * @param  bg      Background colour. Pass same value as color for
     *                 transparent (no background fill).
     */
    void drawLabel(const char *text, int16_t x, int16_t y,
                   uint16_t color, uint8_t scale = 1,
                   uint16_t bg = BEEPI_BLACK);

    /*!
     * @brief  Draw a string anchored to a screen corner or centre.
     * @param  text    Null-terminated C string.
     * @param  anchor  BeePiAnchor enum.
     * @param  margin  Pixel inset from display edge (corner anchors).
     * @param  color   Text colour.
     * @param  scale   Font scale.
     * @param  bg      Background colour.
     */
    void drawLabelAnchored(const char *text, BeePiAnchor anchor,
                           int16_t margin, uint16_t color,
                           uint8_t scale = 1, uint16_t bg = BEEPI_BLACK);

    /*!
     * @brief  Draw an integer with optional unit suffix.
     * @param  value   Value to display.
     * @param  suffix  Unit string (e.g. "m", " km/h") or nullptr.
     * @param  x       Left edge.
     * @param  y       Top edge.
     * @param  color   Text colour.
     * @param  scale   Font scale.
     */
    void drawInt(int32_t value, const char *suffix,
                 int16_t x, int16_t y, uint16_t color, uint8_t scale = 1);

    /*!
     * @brief  Draw a float with fixed decimal places.
     * @param  value     Value to display.
     * @param  decimals  Decimal places (0–6).
     * @param  suffix    Unit string or nullptr.
     * @param  x         Left edge.
     * @param  y         Top edge.
     * @param  color     Text colour.
     * @param  scale     Font scale.
     */
    void drawFloat(float value, uint8_t decimals, const char *suffix,
                   int16_t x, int16_t y, uint16_t color, uint8_t scale = 1);

    // ------------------------------------------------------------------
    // HUD / reticle primitives
    // ------------------------------------------------------------------

    /*!
     * @brief  Draw a targeting reticle.
     * @param  cx     Centre X.
     * @param  cy     Centre Y.
     * @param  size   Arm half-length in pixels (CROSSHAIR / CROSS_GAP / CIRCLE_CROSS).
     * @param  gap    Centre gap / circle radius in pixels.
     * @param  style  BeePiReticleStyle.
     * @param  color  Reticle colour.
     * @param  thick  Line thickness: 1 = single pixel, 2 = 3-pixel wide.
     */
    void drawReticle(int16_t cx, int16_t cy, int16_t size, int16_t gap,
                     BeePiReticleStyle style, uint16_t color, uint8_t thick = 1);

    /*!
     * @brief  Draw a bearing arc indicator (compass / PTU HUD).
     *
     * Draws a partial arc centred at the top of the display, spanning
     * fovDeg degrees of view, with a tick at the current bearing.
     *
     * @param  bearing    Current bearing in degrees (0–359).
     * @param  fovDeg     Field-of-view span shown on arc.
     * @param  r          Arc radius from display centre.
     * @param  arcColor   Arc line colour.
     * @param  tickColor  Bearing tick colour.
     */
    void drawBearingArc(float bearing, float fovDeg,
                        int16_t r, uint16_t arcColor, uint16_t tickColor);

    /*!
     * @brief  Draw a horizontal progress bar.
     * @param  x         Left edge.
     * @param  y         Top edge.
     * @param  w         Total bar width.
     * @param  h         Bar height.
     * @param  value     Current value.
     * @param  maxVal    Maximum (full) value.
     * @param  barColor  Filled-portion colour.
     * @param  bgColor   Empty-portion colour.
     * @param  border    Border colour (pass bgColor for no border).
     */
    void drawBarH(int16_t x, int16_t y, int16_t w, int16_t h,
                  int32_t value, int32_t maxVal,
                  uint16_t barColor, uint16_t bgColor, uint16_t border);

    /*!
     * @brief  Draw a two-row labelled value badge (e.g. range readout).
     * @param  label    Short label string (e.g. "RNG", "ALT").
     * @param  value    Integer value to display.
     * @param  unit     Unit string (e.g. "m") or nullptr.
     * @param  x        Top-left X.
     * @param  y        Top-left Y.
     * @param  fgColor  Foreground (text + border) colour.
     * @param  bgColor  Background fill colour.
     */
    void drawBadge(const char *label, int32_t value, const char *unit,
                   int16_t x, int16_t y,
                   uint16_t fgColor, uint16_t bgColor);

    // ------------------------------------------------------------------
    // Camera feed / image push
    // ------------------------------------------------------------------

    /*!
     * @brief  Push a full 240x240 RGB565 frame to the display.
     *
     * Primary path for camera feed display. Pixels must be in row-major
     * order (top-left to bottom-right), native host byte order.
     * Byte-swapping to big-endian wire format is done by the HAL layer.
     *
     * @param  framebuf  Pointer to 240*240 = 57600 uint16_t pixel values.
     */
    void pushFrame(const uint16_t *framebuf);

    /*!
     * @brief  Push a partial rectangular region.
     *
     * Use for OSD dirty-rect refreshes — e.g. update only the badge
     * area after receiving a new LRF range, without resending the full
     * camera frame.
     *
     * @param  x       Left edge.
     * @param  y       Top edge.
     * @param  w       Width.
     * @param  h       Height.
     * @param  pixels  Pointer to w*h uint16_t pixel values.
     */
    void pushRegion(int16_t x, int16_t y, int16_t w, int16_t h,
                    const uint16_t *pixels);

    /*!
     * @brief  Draw a static RGB565 image at arbitrary position.
     * @param  x       Top-left X.
     * @param  y       Top-left Y.
     * @param  bitmap  Pointer to w*h uint16_t pixel values.
     * @param  w       Image width.
     * @param  h       Image height.
     */
    void drawImage(int16_t x, int16_t y, const uint16_t *bitmap,
                   int16_t w, int16_t h);

    // ------------------------------------------------------------------
    // Colour utilities (static — usable without a display instance)
    // ------------------------------------------------------------------

    /// Pack R, G, B (0–255 each) into RGB565.
    static uint16_t rgb(uint8_t r, uint8_t g, uint8_t b);

    /// Linearly interpolate two RGB565 colours. t=0 → c0, t=255 → c1.
    static uint16_t lerpColor(uint16_t c0, uint16_t c1, uint8_t t);

    /// Dim an RGB565 colour. factor=255 → original, factor=0 → black.
    static uint16_t dimColor(uint16_t color, uint8_t factor);

    // ------------------------------------------------------------------
    // Display dimensions
    // ------------------------------------------------------------------

    int16_t width()  const { return _width;  }
    int16_t height() const { return _height; }

    // ------------------------------------------------------------------
    // Low-level direct access
    // ------------------------------------------------------------------

    /// Send a raw command byte (DC=0) directly to the display.
    void sendCommand(uint8_t cmd);

    /// Send a raw command byte followed by parameter bytes.
    void sendCommand(uint8_t cmd, const uint8_t *params, uint8_t paramLen);

private:
    BeePiHALConfig _cfg;
    int16_t        _width;
    int16_t        _height;
    uint8_t        _rotation;
    bool           _initialised;

    // Internal GC9A01A initialisation sequence
    void           _initSequence();

    // Set the RAM write window (CASET + RASET + RAMWR)
    void           _setAddrWindow(uint16_t x0, uint16_t y0,
                                  uint16_t x1, uint16_t y1);

    // Internal drawing helpers
    void           _drawThickLine(int16_t x0, int16_t y0,
                                  int16_t x1, int16_t y1,
                                  uint16_t color, uint8_t thick);

    void           _drawCircleHelper(int16_t cx, int16_t cy, int16_t r,
                                     uint8_t quadrant, uint16_t color);

    void           _fillCircleHelper(int16_t cx, int16_t cy, int16_t r,
                                     uint8_t sides, int16_t delta,
                                     uint16_t color);

    void           _drawChar(int16_t x, int16_t y, char c,
                              uint16_t color, uint16_t bg, uint8_t scale);

    void           _textDimensions(const char *text, uint8_t scale,
                                   int16_t *outW, int16_t *outH) const;
};

#endif // __cplusplus