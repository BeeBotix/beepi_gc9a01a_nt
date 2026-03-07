# BeeBotix GC9A01A Non-Touch Round Display Library

**beepi_gc9a01a_nt** — Platform-native C++ driver for the 240×240 GC9A01A
round TFT display, targeting Raspberry Pi Linux. No Arduino runtime, no
Adafruit dependency on Linux. Clean HAL layer keeps it portable.

> For full usage walkthroughs see [UserGuide.md](UserGuide.md).

---

## File tree

```
beepi_gc9a01a_nt/
│
├── CMakeLists.txt                  Build system (CMake 3.16+)
├── README.md                       This file
├── UserGuide.md                    Step-by-step usage guide
│
├── lib/                            Core library — include this directory
│   ├── beepi_gc9a01a_nt.h          PUBLIC API — the only header you need
│   ├── beepi_gc9a01a_nt.cpp        Implementation (drawing, init, font)
│   ├── hal/
│   │   ├── beepi_hal.h             HAL interface (platform-agnostic)
│   │   ├── beepi_hal_linux.cpp     RPi: lgpio + spidev ioctl
│   │   └── beepi_hal_arduino.cpp   Arduino shim (wraps Adafruit_GC9A01A)
│   └── fonts/
│       └── beepi_font5x8.h         Built-in 5×8 ASCII bitmap font
│
├── example/
│   ├── testpattern.cpp             9 visual test patterns
│   ├── testpattern2.cpp            Extended pattern set
│   ├── imgview.cpp                 Static PNG image display
│   ├── videoview.cpp               Animated GIF / video playback
│   └── videoview_osd.cpp           Video playback with live FPS OSD overlay
│
└── assets/
    ├── png_to_rgb565.py            PNG → C header (uint16_t RGB565 array)
    ├── gif_to_frames.py            Animated GIF → BPGF binary (Pillow-based)
    ├── video_to_frames.py          Any video (MP4/MKV/AVI…) → BPGF binary
    ├── img.png                     Place your 240×240 test image here
    ├── ocean.gif                   Place your test GIF here
    └── input.mp4                   Place your test video here (triggers videoplay target)
```

---

## Supported platforms

| Board | OS | SPI device | GPIO chip |
|---|---|---|---|
| Raspberry Pi 3B+ | Pi OS 32-bit Bullseye / Bookworm | `/dev/spidev0.0` | `/dev/gpiochip0` |
| Raspberry Pi Zero 2W | Pi OS 32 or 64-bit | `/dev/spidev0.0` | `/dev/gpiochip0` |
| Raspberry Pi 4B | Pi OS 64-bit Bookworm (recommended) | `/dev/spidev0.0` | `/dev/gpiochip0` |
| Raspberry Pi 5 | Pi OS 64-bit Bookworm | `/dev/spidev0.0` | **`/dev/gpiochip4`** |
| Arduino / ESP32 | Arduino IDE / PlatformIO | hardware SPI | Adafruit driver |

---

## Wiring

| Display Pin | Raspberry Pi GPIO | Physical Pin | Function |
|---|---|---|---|
| VCC | 3.3 V | Pin 1 | Power — **3.3 V only, never 5 V** |
| GND | GND | Pin 6 | Ground |
| SCL (CLK) | GPIO 11 | Pin 23 | SPI Clock |
| SDA (MOSI) | GPIO 10 | Pin 19 | SPI Data |
| CS | GPIO 8 (CE0) | Pin 24 | Chip Select (kernel-driven) |
| DC | GPIO 25 | Pin 22 | Data / Command |
| RES (RST) | GPIO 24 | Pin 18 | Reset (set `gpio_rst=-1` if unconnected) |
| BLK (BL) | GPIO 18 | Pin 12 | Backlight (set `gpio_bl=-1` if hardwired ON) |

> CS is driven by the spidev kernel driver. Do not drive it as a software GPIO.

> **Pi 5:** GPIO moved to `/dev/gpiochip4` due to the RP1 southbridge.
> Set `cfg.gpio_chip = "/dev/gpiochip4"`. Pin numbers and SPI path unchanged.

---

## Dependencies

```bash
# lgpio — GPIO library (works on all Pi models including Pi 5)
sudo apt update
sudo apt install liblgpio-dev

# ffmpeg + Pillow — required for asset converters only
sudo apt install ffmpeg python3-pil

# Enable SPI if not already on
sudo raspi-config   # Interface Options → SPI → Enable
# or:
echo "dtparam=spi=on" | sudo tee -a /boot/firmware/config.txt && sudo reboot
```

---

## Building

```bash
cd beepi_gc9a01a_nt
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Targets built by default

| Target | Binary | Description |
|---|---|---|
| `testpattern` | `./testpattern` | 9-step visual self-test |
| `testpattern2` | `./testpattern2` | Extended pattern set |
| `imgview` | `./imgview` | Display `assets/img.png` |
| `ocean_bin` | `assets/ocean.bin` | Auto-converts `ocean.gif` → BPGF |
| `videoview` | `./videoview` | GIF playback (no OSD) |
| `videoview_osd` | `./videoview_osd` | GIF playback + live FPS counter |
| `videoplay` | `./videoplay` | Video playback (only if `assets/input.mp4` present) |

---

## Performance

All measurements on Raspberry Pi 4B, SPI at 62.5 MHz.

| Mode | Achieved FPS | Notes |
|---|---|---|
| `pushFrame` raw throughput | ~68 fps | Physics limit: 240×240×16 bits ÷ 62.5 MHz |
| GIF playback (`videoview`) | 20 fps | Throttled by GIF frame duration metadata |
| Video playback (`videoplay`) | ~50 fps | ffmpeg-resampled to target fps |
| OSD overlay (`videoview_osd`) | ~50 fps | FPS counter drawn in fb[] before pushFrame, negligible overhead |

To push higher fps, increase `spi_speed_hz`. The GC9A01A and RPi 4 SPI bus
are rated up to 125 MHz; 62.5 MHz is the tested-stable value.

---

## BPGF binary format

Both `gif_to_frames.py` and `video_to_frames.py` produce this format:

```
Offset   Size    Field
0        4       Magic: "BPGF"
4        2       Width  (always 240)
6        2       Height (always 240)
8        4       frame_count  (uint32)
12       2       avg_fps      (uint16)
14       N×2     per-frame duration_ms[]  (uint16 each)
?        pad     Zero-padding to next 512-byte boundary
data_off N×115200  Raw pixel data, one frame after another
                   Each frame: 240×240×2 = 115200 bytes, big-endian RGB565
```

---

## Quick start

```cpp
#include "beepi_gc9a01a_nt.h"

int main()
{
    BeePiHALConfig cfg = {};
    cfg.spi_device   = "/dev/spidev0.0";
    cfg.spi_speed_hz = 62500000u;
    cfg.gpio_dc      = 25;
    cfg.gpio_rst     = 24;
    cfg.gpio_bl      = 18;
    cfg.gpio_chip    = "/dev/gpiochip0";

    BeePi_GC9A01A display(cfg);
    if (!display.begin()) return 1;

    display.fill(BEEPI_BLACK);
    display.drawLabel("BeeBotix", 60, 112, BEEPI_GREEN, 2);
    display.drawReticle(120, 120, 30, 8, BEEPI_RETICLE_CROSS_GAP, BEEPI_GREEN, 2);

    pause();
    display.end();
}
```

---

## Milestones achieved

| Milestone | Description | Key files |
|---|---|---|
| M1 | Core library, HAL, font, 9 test patterns | `lib/`, `testpattern.cpp` |
| M2 | PNG image display via C header | `imgview.cpp`, `png_to_rgb565.py` |
| M3 | Animated GIF playback (BPGF, mmap, per-frame timing) | `videoview.cpp`, `gif_to_frames.py` |
| M4 | Live FPS OSD overlay (white text, solid bar, correct orientation) | `videoview_osd.cpp` |
| M5 | Full video playback — any format, centre-crop, 50 fps stress test | `video_to_frames.py`, `videoplay` target |

---

## Licence

BeeBotix Autonomous Systems — internal use.