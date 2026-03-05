/*!
 * @file beepi_hal_arduino.cpp
 *
 * BeeBotix GC9A01A — Arduino HAL shim
 *
 * Bridges the Adafruit_GC9A01A driver into the beepi_hal interface so
 * the same beepi_gc9a01a_nt.cpp core compiles unchanged on Arduino-family
 * boards (ESP32, RP2040, AVR, SAMD, STM32, Teensy, …).
 *
 * On Arduino we hand off SPI + GPIO management entirely to the Adafruit
 * driver.  The beepi_hal_write_* functions call into Adafruit_SPITFT
 * transaction wrappers which are already hardware-optimised per platform.
 *
 * To use on Arduino:
 *   1. Install Adafruit_GFX and Adafruit_GC9A01A via Library Manager.
 *   2. Add -DBEEPI_PLATFORM_ARDUINO to your build flags (or the sketch
 *      defines it via a config header).
 *   3. Call beepi_hal_init() with pin_cs / pin_dc / pin_rst populated.
 *      spi_device and gpio_chip are ignored.
 *
 * BeeBotix Autonomous Systems
 */

#include "beepi_hal.h"

#ifdef BEEPI_PLATFORM_ARDUINO

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

// Internal Adafruit driver instance — created in beepi_hal_init
static Adafruit_GC9A01A *s_tft = nullptr;
static int s_pin_dc = -1;

// ---------------------------------------------------------------------------
// beepi_hal_init  (Arduino)
// ---------------------------------------------------------------------------

int beepi_hal_init(const BeePiHALConfig *cfg)
{
    if (!cfg) return -1;

    s_pin_dc = cfg->gpio_dc;

    // Use software SPI if MOSI/SCLK are provided, otherwise hardware SPI
    if (cfg->pin_mosi >= 0 && cfg->pin_sclk >= 0) {
        s_tft = new Adafruit_GC9A01A(
            (int8_t)cfg->pin_cs,
            (int8_t)cfg->gpio_dc,
            (int8_t)cfg->pin_mosi,
            (int8_t)cfg->pin_sclk,
            (int8_t)cfg->gpio_rst,
            -1
        );
    } else {
        s_tft = new Adafruit_GC9A01A(
            (int8_t)cfg->pin_cs,
            (int8_t)cfg->gpio_dc,
            (int8_t)cfg->gpio_rst
        );
    }

    if (!s_tft) return -1;

    s_tft->begin(cfg->spi_speed_hz > 0 ? cfg->spi_speed_hz : 0);

    if (cfg->gpio_bl >= 0) {
        pinMode((uint8_t)cfg->gpio_bl, OUTPUT);
        digitalWrite((uint8_t)cfg->gpio_bl, HIGH);
    }

    return 0;
}

// ---------------------------------------------------------------------------
// beepi_hal_deinit  (Arduino)
// ---------------------------------------------------------------------------

void beepi_hal_deinit(void)
{
    delete s_tft;
    s_tft = nullptr;
}

// ---------------------------------------------------------------------------
// GPIO stubs — Adafruit driver owns the pins on Arduino
// ---------------------------------------------------------------------------

void beepi_hal_dc(int level)  { (void)level; /* managed by Adafruit */ }
void beepi_hal_rst(int level) { (void)level; }
void beepi_hal_bl(int level)
{
    // Caller may still use this to toggle backlight if BL pin is set
    (void)level;
}

void beepi_hal_delay_ms(uint32_t ms) { delay(ms); }

// ---------------------------------------------------------------------------
// SPI write — delegate to Adafruit_SPITFT
// ---------------------------------------------------------------------------

void beepi_hal_write_cmd(uint8_t cmd)
{
    if (!s_tft) return;
    s_tft->sendCommand(cmd);
}

void beepi_hal_write_data(uint8_t data)
{
    if (!s_tft) return;
    s_tft->startWrite();
    s_tft->spiWrite(data);
    s_tft->endWrite();
}

void beepi_hal_write_buf(const uint8_t *buf, size_t len)
{
    if (!s_tft) return;
    s_tft->startWrite();
    for (size_t i = 0; i < len; i++) {
        s_tft->spiWrite(buf[i]);
    }
    s_tft->endWrite();
}

void beepi_hal_write_pixels(const uint16_t *pixels, size_t count)
{
    if (!s_tft) return;
    // drawRGBBitmap handles all SPI transaction and byte-order details
    // We reconstruct the pointer cast that Adafruit expects
    s_tft->drawRGBBitmap(0, 0, pixels, 240, (int16_t)(count / 240));
}

// ---------------------------------------------------------------------------
// Expose the underlying Adafruit driver for direct GFX access
// (called by beepi_gc9a01a_nt.cpp when BEEPI_PLATFORM_ARDUINO is set)
// ---------------------------------------------------------------------------

Adafruit_GC9A01A *beepi_hal_get_adafruit_driver(void)
{
    return s_tft;
}

#endif // BEEPI_PLATFORM_ARDUINO