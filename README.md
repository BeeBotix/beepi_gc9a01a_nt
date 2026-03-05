# BeeBotix GC9A01A Non-Touch Round Display Library

**beepi_gc9a01a_nt** — Platform-native C++ driver for the 240×240 GC9A01A
round TFT display, targeting Raspberry Pi Linux. No Arduino runtime, no
Adafruit dependency on Linux. Clean HAL layer keeps it portable.

---

## File tree

```
beepi_gc9a01a_lib/
│
├── CMakeLists.txt                  Build system (cmake)
├── README.md                       This file
│
├── lib/                            ← Include this whole directory
│   │
│   ├── beepi_gc9a01a_nt.h          PUBLIC API — include this in your code
│   ├── beepi_gc9a01a_nt.cpp        Implementation (all drawing, init seq, font)
│   │
│   ├── hal/
│   │   ├── beepi_hal.h             HAL interface (platform-agnostic contract)
│   │   ├── beepi_hal_linux.cpp     RPi implementation: lgpio + spidev ioctl
│   │   └── beepi_hal_arduino.cpp   Arduino shim (wraps Adafruit_GC9A01A)
│   │
│   └── fonts/
│       └── beepi_font5x8.h         Built-in 5×8 ASCII bitmap font (no deps)
│
├── example/
│   ├── testpattern.cpp             Milestone 1 — 9 visual test patterns
│   ├── imgview.cpp                 Milestone 2 — static image display
│   └── videoview.cpp               Milestone 2 — animated GIF playback
│
└── assets/
    ├── media2hex.py                PNG/GIF → C RGB565 header converter
    ├── img.png                     Place your 240×240 test image here
    └── video.gif                   Place your 240×240 test GIF here
```

---

## Supported platforms

| Board | OS | Arch | SPI | GPIO chip |
|---|---|---|---|---|
| Raspberry Pi 3B+ | Raspberry Pi OS 32-bit (Bullseye/Bookworm) | ARMv8 32-bit | `/dev/spidev0.0` | `/dev/gpiochip0` |
| Raspberry Pi Zero 2W | Raspberry Pi OS 32-bit or 64-bit | ARMv8 32/64 | `/dev/spidev0.0` | `/dev/gpiochip0` |
| Raspberry Pi 4B | Raspberry Pi OS 64-bit (Bookworm recommended) | ARMv8 64-bit | `/dev/spidev0.0` | `/dev/gpiochip0` |
| Raspberry Pi 5 | Raspberry Pi OS 64-bit Bookworm | ARMv8 64-bit | `/dev/spidev0.0` | **`/dev/gpiochip4`** |
| Arduino / ESP32 | Arduino IDE / PlatformIO | any | hardware SPI | Adafruit driver |

---

## Wiring (BCM pin numbers)

```
GC9A01A pin   BCM (physical)   Notes
-----------   ---------------  -------
VCC           3.3V             DO NOT connect to 5V
GND           GND
SCL / SCLK    BCM 11  (pin 23) SPI0 clock
SDA / MOSI    BCM 10  (pin 19) SPI0 MOSI
RES / RST     BCM 27  (pin 13) Optional — set gpio_rst=-1 if unconnected
DC            BCM 25  (pin 22) Required
CS            BCM 8   (pin 24) SPI0 CE0 — handled by spidev kernel driver
BLK / BL      BCM 18  (pin 12) Optional — set gpio_bl=-1 if hardwired HIGH
```

> **Pi 5 note:** The RP1 southbridge chip moves GPIO to `/dev/gpiochip4`.
> Change `gpio_chip = "/dev/gpiochip4"` in `make_config()` inside your example.
> SPI and spidev paths stay the same.

---

## Dependencies

```bash
# Install lgpio (works on ALL Pi models including Pi 5)
sudo apt update
sudo apt install liblgpio-dev

# Enable SPI kernel module if not already enabled
sudo raspi-config   # Interface Options → SPI → Enable
# or manually:
echo "dtparam=spi=on" | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

> **Why lgpio instead of pigpio?**
> pigpio accesses GPIO via direct `/dev/mem` register mapping. The Raspberry
> Pi 5 uses a new RP1 southbridge chip — its GPIO registers are not at the
> BCM283x addresses that pigpio expects. lgpio uses the standard Linux GPIO
> character device interface (`/dev/gpiochipN`) which works on all Pi models
> via the same code path.

---

## Building

```bash
# Clone / copy the library to your Pi
cd beepi_gc9a01a_lib

# Configure
mkdir build && cd build
cmake ..

# Build (uses all CPU cores)
make -j$(nproc)

# Run test patterns
sudo ./testpattern
```

### Cross-compilation (build on x86 laptop, run on Pi)

```bash
# Install ARM toolchains
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu    # for Pi 64-bit
sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf # for Pi 32-bit

# 64-bit cross build
mkdir build64 && cd build64
cmake .. \
  -DCMAKE_TOOLCHAIN_FILE=../toolchain-aarch64.cmake \
  -DCMAKE_SYSTEM_PROCESSOR=aarch64
make -j$(nproc)
```

---

## Quick start code

```cpp
#include "beepi_gc9a01a_nt.h"

int main()
{
    BeePiHALConfig cfg = {};
    cfg.spi_device   = "/dev/spidev0.0";
    cfg.spi_speed_hz = 40000000;
    cfg.gpio_dc      = 25;
    cfg.gpio_rst     = 27;
    cfg.gpio_bl      = 18;
    cfg.gpio_chip    = "/dev/gpiochip0";  // Pi 5: "/dev/gpiochip4"

    BeePi_GC9A01A display(cfg);
    display.begin();
    display.fill(BEEPI_BLACK);

    // Reticle + HUD
    display.drawReticle(120, 120, 30, 8, BEEPI_RETICLE_CROSS_GAP, BEEPI_GREEN, 2);
    display.drawBadge("RNG", 847, "m",  8,  8, BEEPI_GREEN, BEEPI_BLACK);
    display.drawBarH(72, 218, 96, 7, 6, 20, BEEPI_GREEN, BEEPI_DARKGREY, BEEPI_WHITE);

    // Camera frame push (your RGB565 buffer from V4L2/GStreamer appsink)
    // uint16_t frame[240*240] = { ... };
    // display.pushFrame(frame);

    pause();
    display.end();
}
```

---

## API reference

### Configuration

```cpp
BeePiHALConfig cfg = {};
cfg.spi_device   = "/dev/spidev0.0";   // SPI bus
cfg.spi_speed_hz = 40000000;           // 40 MHz — safe on all Pi
cfg.gpio_dc      = 25;                 // BCM pin
cfg.gpio_rst     = 27;                 // BCM pin (-1 = skip)
cfg.gpio_bl      = 18;                 // BCM pin (-1 = skip)
cfg.gpio_chip    = "/dev/gpiochip0";   // Pi 5: "/dev/gpiochip4"
```

### Lifecycle

| Method | Description |
|---|---|
| `begin()` | Init HAL, reset display, run init sequence. Returns `false` on error. |
| `end()` | Release SPI fd and GPIO handles. |
| `setDisplayOn(bool)` | Toggle panel output (DISPON/DISPOFF command). |
| `invertDisplay(bool)` | Invert all colours. |
| `setRotation(0-3)` | Rotate 90° per step. |
| `setBacklight(bool)` | Drive BL GPIO (if configured). |

### Drawing primitives

`fill`, `fillRect`, `drawRect`, `drawPixel`, `drawLine`, `drawHLine`, `drawVLine`,
`drawCircle`, `fillCircle`, `drawTriangle`, `fillTriangle`,
`drawRoundRect`, `fillRoundRect`

### Text / OSD

| Method | Description |
|---|---|
| `drawLabel(text, x, y, color, scale, bg)` | Draw at pixel position. `color==bg` → transparent background. |
| `drawLabelAnchored(text, anchor, margin, color, scale, bg)` | Anchor to corner or centre. |
| `drawInt(value, suffix, x, y, color, scale)` | Integer + unit string. |
| `drawFloat(value, decimals, suffix, x, y, color, scale)` | Float with fixed decimals. |

`BeePiAnchor`: `BEEPI_ANCHOR_TOP_LEFT`, `TOP_RIGHT`, `BOTTOM_LEFT`, `BOTTOM_RIGHT`, `CENTER`

### HUD / reticle

| Method | Description |
|---|---|
| `drawReticle(cx, cy, size, gap, style, color, thick)` | Targeting reticle. |
| `drawBearingArc(bearing, fovDeg, r, arcColor, tickColor)` | Compass arc + tick. |
| `drawBarH(x,y,w,h, value, maxVal, barColor, bgColor, border)` | Horizontal bar. |
| `drawBadge(label, value, unit, x, y, fgColor, bgColor)` | Two-row value badge. |

`BeePiReticleStyle`: `BEEPI_RETICLE_CROSSHAIR`, `CROSS_GAP`, `CIRCLE_CROSS`, `DOT`

### Camera / image push

| Method | Description |
|---|---|
| `pushFrame(buf)` | Full 240×240 RGB565 frame from RAM buffer. One `ioctl` DMA transfer. |
| `pushRegion(x,y,w,h, pixels)` | Partial dirty-rect update. |
| `drawImage(x,y, bitmap, w, h)` | Arbitrary-size RGB565 bitmap. |

### Colour utilities (static)

```cpp
uint16_t col = BeePi_GC9A01A::rgb(255, 128, 0);         // RGB888 → RGB565
uint16_t mid = BeePi_GC9A01A::lerpColor(BEEPI_RED, BEEPI_BLUE, 128); // 50%
uint16_t dim = BeePi_GC9A01A::dimColor(BEEPI_GREEN, 100);             // ~40%
// compile-time macro:
uint16_t col2 = BEEPI_RGB(255, 128, 0);
```

---

## Integrating a camera feed

With GStreamer `appsink` (your existing pipeline on FURY V2 / BeeBotix camera):

```cpp
// In GStreamer appsink callback — framebuf is the RGB565 data pointer
// from a GstMapInfo after converting via videoconvert + video/x-raw,format=RGB16
void on_new_sample(uint8_t *data, size_t size)
{
    if (size == 240 * 240 * 2) {
        display.pushFrame(reinterpret_cast<const uint16_t *>(data));
    }
}
```

The `pushFrame` path sets the full address window and fires a single
`ioctl(SPI_IOC_MESSAGE)` call, transferring all 115,200 bytes in one
kernel DMA shot. At 40 MHz SPI, a full frame transfer takes ~23 ms
(≈ 43 fps theoretical maximum, real fps gated by your camera source).

---

## Milestone roadmap

| Milestone | Status | Files |
|---|---|---|
| M1 | Done | `/lib/` complete + `testpattern.cpp` |
| M2 | Planned | `imgview.cpp`, `videoview.cpp`, `/assets/` usage examples |

---

## Licence

BeeBotix Autonomous Systems internal use.