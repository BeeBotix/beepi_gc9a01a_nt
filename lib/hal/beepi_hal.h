/*!
 * @file beepi_hal.h
 *
 * BeeBotix GC9A01A — Hardware Abstraction Layer Interface
 *
 * This header defines the pure abstract interface that every platform port
 * must implement.  The library core (beepi_gc9a01a_nt.cpp) calls only these
 * functions — it never touches GPIO or SPI registers directly.
 *
 * Implemented by:
 *   beepi_hal_linux.cpp   — Raspberry Pi 3B+/Zero2W/4/5 via lgpio + spidev
 *   beepi_hal_arduino.cpp — Arduino-family boards via Adafruit_GC9A01A shim
 *
 * Platform selection is done at compile time:
 *   - BEEPI_PLATFORM_LINUX   : defined automatically when __linux__ is set
 *   - BEEPI_PLATFORM_ARDUINO : define manually for Arduino builds
 *
 * BeeBotix Autonomous Systems
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Compile-time platform selection
// ---------------------------------------------------------------------------

#if defined(__linux__)
#  define BEEPI_PLATFORM_LINUX 1
#elif defined(ARDUINO)
#  define BEEPI_PLATFORM_ARDUINO 1
#else
#  error "beepi_hal.h: unsupported platform. Define BEEPI_PLATFORM_LINUX or BEEPI_PLATFORM_ARDUINO."
#endif

// ---------------------------------------------------------------------------
// HAL configuration struct  (passed to beepi_hal_init)
// ---------------------------------------------------------------------------

/*!
 * @brief  Hardware configuration for the GC9A01A display.
 *
 * On Linux/RPi:
 *   spi_device   "/dev/spidev0.0"  (bus 0, CS 0)
 *   spi_speed_hz  40000000         (40 MHz — safe for all Pi models)
 *   gpio_dc       25               (BCM pin number)
 *   gpio_rst      27               (BCM pin number, -1 = not connected)
 *   gpio_bl       18               (BCM pin for backlight, -1 = always on)
 *   gpio_chip     "/dev/gpiochip0" (Pi 3/4/Zero: gpiochip0, Pi 5: gpiochip4)
 *
 * On Arduino:
 *   pin_cs, pin_dc, pin_rst, pin_mosi, pin_sclk filled in as Arduino pin nums.
 *   spi_speed_hz used as SPI freq.
 */
typedef struct {
    /* SPI */
    const char *spi_device;      /*!< Linux: "/dev/spidevX.Y"  Arduino: ignored */
    uint32_t    spi_speed_hz;    /*!< SPI clock frequency in Hz */

    /* GPIO — BCM numbers on Linux, Arduino pin numbers on Arduino */
    int         gpio_dc;         /*!< Data/Command select (required) */
    int         gpio_rst;        /*!< Reset (-1 = not connected) */
    int         gpio_bl;         /*!< Backlight control (-1 = always on) */

    /* Linux-specific */
    const char *gpio_chip;       /*!< lgpio chip path, e.g. "/dev/gpiochip0" */

    /* Arduino-specific (ignored on Linux) */
    int         pin_cs;          /*!< Arduino chip-select pin */
    int         pin_mosi;        /*!< Arduino MOSI pin (hw SPI: -1) */
    int         pin_sclk;        /*!< Arduino SCLK pin (hw SPI: -1) */
} BeePiHALConfig;

// ---------------------------------------------------------------------------
// HAL function prototypes  (implemented per-platform)
// ---------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief  Initialise SPI bus and GPIO pins.
 * @param  cfg   Pointer to filled BeePiHALConfig struct.
 * @return 0 on success, negative errno on failure.
 */
int  beepi_hal_init(const BeePiHALConfig *cfg);

/*!
 * @brief  Release all resources acquired by beepi_hal_init.
 */
void beepi_hal_deinit(void);

/*!
 * @brief  Set the DC (Data/Command) GPIO line.
 * @param  level  1 = data mode, 0 = command mode.
 */
void beepi_hal_dc(int level);

/*!
 * @brief  Set the RST (Reset) GPIO line.
 * @param  level  1 = released, 0 = held in reset.
 */
void beepi_hal_rst(int level);

/*!
 * @brief  Set the backlight GPIO line.
 * @param  level  1 = on, 0 = off.
 */
void beepi_hal_bl(int level);

/*!
 * @brief  Blocking delay.
 * @param  ms  Milliseconds to wait.
 */
void beepi_hal_delay_ms(uint32_t ms);

/*!
 * @brief  Write a single command byte (DC=0) to the display.
 * @param  cmd  Command byte.
 */
void beepi_hal_write_cmd(uint8_t cmd);

/*!
 * @brief  Write a single data byte (DC=1) to the display.
 * @param  data  Data byte.
 */
void beepi_hal_write_data(uint8_t data);

/*!
 * @brief  Write a block of data bytes (DC=1) to the display.
 *
 * Used for bulk pixel transfer. The driver sets DC=1 before calling this.
 * The HAL must NOT toggle DC.
 *
 * @param  buf   Pointer to byte buffer.
 * @param  len   Number of bytes to send.
 */
void beepi_hal_write_buf(const uint8_t *buf, size_t len);

/*!
 * @brief  Write a block of uint16_t pixels with automatic byte-swap.
 *
 * The GC9A01A expects big-endian (MSB first) pixel words on the wire.
 * On little-endian hosts (all ARM Linux RPi) each 16-bit value must be
 * byte-swapped before transmission.  The HAL implementation handles this
 * so the caller can pass native host-endian RGB565 values.
 *
 * @param  pixels   Pointer to RGB565 pixel array (host byte order).
 * @param  count    Number of pixels (not bytes).
 */
void beepi_hal_write_pixels(const uint16_t *pixels, size_t count);

#ifdef __cplusplus
}
#endif