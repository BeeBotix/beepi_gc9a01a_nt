/*!
 * @file beepi_hal_linux.cpp
 *
 * BeeBotix GC9A01A — Linux / Raspberry Pi HAL implementation
 *
 * Targets:
 *   Raspberry Pi 3B+   (ARMv8 32-bit, BCM2837, gpiochip0)
 *   Raspberry Pi Zero 2W (ARMv8 32/64-bit, BCM2837B0, gpiochip0)
 *   Raspberry Pi 4B     (ARMv8 64-bit, BCM2711, gpiochip0)
 *   Raspberry Pi 5      (ARMv8 64-bit, BCM2712+RP1, gpiochip4)
 *
 * SPI:  Linux kernel spidev via ioctl(SPI_IOC_MESSAGE)
 *       Single ioctl call transfers the entire pixel buffer in one DMA shot,
 *       which is significantly faster than byte-at-a-time writes.
 *
 * GPIO: lgpio (liblgpio) — the only GPIO library that works correctly on ALL
 *       Pi models including Pi 5 (where pigpio fails due to the RP1 chip).
 *       Install: sudo apt install liblgpio-dev
 *       Link:    -llgpio
 *
 * SPI speed notes:
 *   Pi 3B+    : tested stable at 40 MHz
 *   Pi Zero2W : tested stable at 32 MHz (conservative; bump to 40 if stable)
 *   Pi 4B     : tested stable at 64 MHz
 *   Pi 5      : tested stable at 64 MHz (RP1 SPI controller)
 *   Default in BeePiHALConfig: 40 MHz (safe across all models)
 *
 * Byte-swap note:
 *   All ARM Linux Pi boards are little-endian. The GC9A01A expects big-endian
 *   RGB565 on the wire. We use __builtin_bswap16() — zero-cost on ARMv8
 *   (it maps to REV16 instruction) — inside the pixel write path.
 *
 * BeeBotix Autonomous Systems
 */

#include "beepi_hal.h"

#ifdef BEEPI_PLATFORM_LINUX

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <lgpio.h>

// ---------------------------------------------------------------------------
// Internal state
// ---------------------------------------------------------------------------

static int      s_spi_fd    = -1;   // spidev file descriptor
static int      s_gpio_h    = -1;   // lgpio chip handle
static int      s_pin_dc    = -1;
static int      s_pin_rst   = -1;
static int      s_pin_bl    = -1;

// SPI transfer scratch buffer — avoids malloc on the pixel-push hot path.
// 240*240*2 = 115200 bytes (< 128 KB, fine on all Pi RAM configs).
// We byte-swap into this buffer then fire one ioctl.
#define SPI_BUF_BYTES  (240 * 240 * 2)
static uint8_t s_spi_buf[SPI_BUF_BYTES];

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static int gpio_out(int chip_h, int pin, int initial_level)
{
    if (pin < 0) return 0;
    int rc = lgGpioClaimOutput(chip_h, 0, pin, initial_level);
    if (rc < 0) {
        fprintf(stderr, "beepi_hal: lgGpioClaimOutput pin %d failed: %d\n",
                pin, rc);
        return rc;
    }
    return 0;
}

static void gpio_write(int chip_h, int pin, int level)
{
    if (pin < 0 || chip_h < 0) return;
    lgGpioWrite(chip_h, pin, level);
    // Allow the GPIO pin state to settle before the next SPI transaction.
    // Without this, the DC line may not be stable when the SPI ioctl fires,
    // causing command/data framing errors (symptom: blank or corrupted display).
    struct timespec ts = { 0, 1000L };   // 1 microsecond
    nanosleep(&ts, NULL);
}

// ---------------------------------------------------------------------------
// beepi_hal_init
// ---------------------------------------------------------------------------

int beepi_hal_init(const BeePiHALConfig *cfg)
{
    if (!cfg) return -EINVAL;

    // --- Open lgpio chip ------------------------------------------------
    const char *chip_path = cfg->gpio_chip ? cfg->gpio_chip : "/dev/gpiochip0";

    // lgOpenChip takes the chip number extracted from the path.
    // We parse the trailing digit for generality (gpiochip0, gpiochip4, …).
    int chip_num = 0;
    const char *p = chip_path + strlen(chip_path) - 1;
    while (p > chip_path && *p >= '0' && *p <= '9') p--;
    chip_num = atoi(p + 1);

    s_gpio_h = lgGpiochipOpen(chip_num);
    if (s_gpio_h < 0) {
        fprintf(stderr, "beepi_hal: lgGpiochipOpen(%d) failed: %d\n"
                        "  Check chip path: %s\n"
                        "  Pi 5 needs gpiochip4, Pi 3/4/Zero: gpiochip0\n",
                chip_num, s_gpio_h, chip_path);
        return s_gpio_h;
    }

    s_pin_dc  = cfg->gpio_dc;
    s_pin_rst = cfg->gpio_rst;
    s_pin_bl  = cfg->gpio_bl;

    if (gpio_out(s_gpio_h, s_pin_dc,  0) < 0) { beepi_hal_deinit(); return -EIO; }
    if (gpio_out(s_gpio_h, s_pin_rst, 1) < 0) { beepi_hal_deinit(); return -EIO; }
    if (gpio_out(s_gpio_h, s_pin_bl,  0) < 0) { beepi_hal_deinit(); return -EIO; }

    // --- Open spidev ----------------------------------------------------
    const char *spi_dev = cfg->spi_device ? cfg->spi_device : "/dev/spidev0.0";
    s_spi_fd = open(spi_dev, O_RDWR);
    if (s_spi_fd < 0) {
        fprintf(stderr, "beepi_hal: open(%s) failed: %s\n",
                spi_dev, strerror(errno));
        beepi_hal_deinit();
        return -errno;
    }

    // SPI mode 0 (CPOL=0, CPHA=0) — GC9A01A requirement
    uint8_t mode = SPI_MODE_0;
    if (ioctl(s_spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        fprintf(stderr, "beepi_hal: SPI_IOC_WR_MODE failed: %s\n",
                strerror(errno));
        beepi_hal_deinit();
        return -errno;
    }

    // 8 bits per word
    uint8_t bits = 8;
    if (ioctl(s_spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        fprintf(stderr, "beepi_hal: SPI_IOC_WR_BITS_PER_WORD failed: %s\n",
                strerror(errno));
        beepi_hal_deinit();
        return -errno;
    }

    // SPI speed
    // Pi 3B+ SPI clock divider rounds to the nearest power-of-2 divisor of
    // 400 MHz.  40 MHz is unreliable on jumper wires / breadboards.
    // 20 MHz (divisor 20) is rock-solid on all Pi models.
    // Increase to 32 MHz or 40 MHz only after confirming stable operation.
    uint32_t speed = cfg->spi_speed_hz > 0 ? cfg->spi_speed_hz : 20000000u;
    if (ioctl(s_spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        fprintf(stderr, "beepi_hal: SPI_IOC_WR_MAX_SPEED_HZ failed: %s\n",
                strerror(errno));
        beepi_hal_deinit();
        return -errno;
    }

    return 0;
}

// ---------------------------------------------------------------------------
// beepi_hal_deinit
// ---------------------------------------------------------------------------

void beepi_hal_deinit(void)
{
    if (s_spi_fd >= 0) { close(s_spi_fd); s_spi_fd = -1; }
    if (s_gpio_h >= 0) {
        if (s_pin_dc  >= 0) lgGpioFree(s_gpio_h, s_pin_dc);
        if (s_pin_rst >= 0) lgGpioFree(s_gpio_h, s_pin_rst);
        if (s_pin_bl  >= 0) lgGpioFree(s_gpio_h, s_pin_bl);
        lgGpiochipClose(s_gpio_h);
        s_gpio_h = -1;
    }
    s_pin_dc = s_pin_rst = s_pin_bl = -1;
}

// ---------------------------------------------------------------------------
// GPIO helpers
// ---------------------------------------------------------------------------

void beepi_hal_dc(int level)  { gpio_write(s_gpio_h, s_pin_dc,  level); }
void beepi_hal_rst(int level) { gpio_write(s_gpio_h, s_pin_rst, level); }
void beepi_hal_bl(int level)  { gpio_write(s_gpio_h, s_pin_bl,  level); }

// ---------------------------------------------------------------------------
// Delay
// ---------------------------------------------------------------------------

void beepi_hal_delay_ms(uint32_t ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000u;
    ts.tv_nsec = (long)(ms % 1000u) * 1000000L;
    nanosleep(&ts, NULL);
}

// ---------------------------------------------------------------------------
// SPI write helpers
// ---------------------------------------------------------------------------

static void spi_transfer(const uint8_t *buf, size_t len)
{
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf        = (unsigned long)buf;
    tr.rx_buf        = 0;
    tr.len           = (uint32_t)len;
    tr.speed_hz      = 0;       // use device default set at open
    tr.delay_usecs   = 0;
    tr.bits_per_word = 8;
    tr.cs_change     = 0;

    if (ioctl(s_spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        fprintf(stderr, "beepi_hal: SPI_IOC_MESSAGE failed: %s\n",
                strerror(errno));
    }
}

void beepi_hal_write_cmd(uint8_t cmd)
{
    gpio_write(s_gpio_h, s_pin_dc, 0);   // DC low = command
    spi_transfer(&cmd, 1);
}

void beepi_hal_write_data(uint8_t data)
{
    gpio_write(s_gpio_h, s_pin_dc, 1);   // DC high = data
    spi_transfer(&data, 1);
}

void beepi_hal_write_buf(const uint8_t *buf, size_t len)
{
    gpio_write(s_gpio_h, s_pin_dc, 1);   // DC high = data
    spi_transfer(buf, len);
}

void beepi_hal_write_pixels(const uint16_t *pixels, size_t count)
{
    // Byte-swap each pixel from host little-endian to wire big-endian.
    // Process in chunks that fit in our scratch buffer.
    size_t remaining = count;
    const uint16_t *src = pixels;

    while (remaining > 0) {
        size_t chunk = remaining;
        size_t max_pixels = SPI_BUF_BYTES / 2;
        if (chunk > max_pixels) chunk = max_pixels;

        // __builtin_bswap16 → REV16 on ARMv6T2+ — zero overhead
        uint8_t *dst = s_spi_buf;
        for (size_t i = 0; i < chunk; i++) {
            uint16_t swapped = __builtin_bswap16(src[i]);
            dst[0] = (uint8_t)(swapped >> 8);
            dst[1] = (uint8_t)(swapped & 0xFF);
            dst += 2;
        }

        gpio_write(s_gpio_h, s_pin_dc, 1);
        spi_transfer(s_spi_buf, chunk * 2);

        src       += chunk;
        remaining -= chunk;
    }
}

#endif // BEEPI_PLATFORM_LINUX