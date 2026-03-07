# BeeBotix GC9A01A — User Guide

This guide walks through every feature of the library from first boot to
live video playback with OSD overlay. Follow the sections in order for a
new setup, or jump directly to the feature you need.

---

## Table of contents

1. [Hardware setup and wiring](#1-hardware-setup-and-wiring)
2. [Software dependencies](#2-software-dependencies)
3. [Building the library](#3-building-the-library)
4. [Running the test patterns](#4-running-the-test-patterns)
5. [Displaying a static PNG image](#5-displaying-a-static-png-image)
6. [Playing an animated GIF](#6-playing-an-animated-gif)
7. [Playing a video file](#7-playing-a-video-file)
8. [FPS OSD overlay](#8-fps-osd-overlay)
9. [Drawing primitives and HUD elements](#9-drawing-primitives-and-hud-elements)
10. [Integrating a live camera feed](#10-integrating-a-live-camera-feed)
11. [Configuration reference](#11-configuration-reference)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Hardware setup and wiring

The GC9A01A is a 240×240 round SPI TFT. It runs on **3.3 V only** — never
connect VCC to a 5 V pin.

### Pin connections (Raspberry Pi)

| Display pin | RPi GPIO (BCM) | Physical pin | Notes |
|---|---|---|---|
| VCC | 3.3 V | Pin 1 | Power |
| GND | GND | Pin 6 | Ground |
| SCL / CLK | GPIO 11 | Pin 23 | SPI0 SCLK |
| SDA / MOSI | GPIO 10 | Pin 19 | SPI0 MOSI |
| CS | GPIO 8 | Pin 24 | SPI0 CE0 — kernel-driven, do not use as software GPIO |
| DC | GPIO 25 | Pin 22 | Data/Command select |
| RES / RST | GPIO 24 | Pin 18 | Hardware reset — optional, tie to 3.3 V if unused |
| BLK / BL | GPIO 18 | Pin 12 | Backlight — optional, tie to 3.3 V for always-on |

### Raspberry Pi 5 note

The RP1 southbridge moves GPIO to a different chip device. Change one config
field:

```cpp
cfg.gpio_chip = "/dev/gpiochip4";   // Pi 5 only, all others use gpiochip0
```

Everything else — BCM pin numbers, SPI device path, physical pin numbers —
is identical.

### Enabling SPI

```bash
sudo raspi-config
# Navigate to: Interface Options → SPI → Enable → reboot
```

Or manually:

```bash
echo "dtparam=spi=on" | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

Verify it worked:

```bash
ls /dev/spidev0.*
# Should show: /dev/spidev0.0  /dev/spidev0.1
```

---

## 2. Software dependencies

```bash
sudo apt update
sudo apt install liblgpio-dev cmake build-essential ffmpeg python3-pil
```

| Package | Used for |
|---|---|
| `liblgpio-dev` | GPIO control (works on all Pi models including Pi 5) |
| `cmake`, `build-essential` | Building the C++ library and examples |
| `ffmpeg` | Video decoding in `video_to_frames.py` |
| `python3-pil` | Pillow image library — GIF and video converters |

---

## 3. Building the library

```bash
cd beepi_gc9a01a_nt
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Expected output (abbreviated):

```
-- Configuring done
-- Build files have been written to: .../build
[ 16%] Built target ocean_bin
[ 50%] Built target beepi_gc9a01a_nt
[ 66%] Built target testpattern
[ 83%] Built target imgview
[100%] Built target videoview_osd
```

If `assets/input.mp4` exists, a `videoplay` target is also built and the
video is auto-converted to `assets/video.bin` during the build.

---

## 4. Running the test patterns

`testpattern` is the first thing to run on a new hardware setup. It cycles
through nine visual patterns: solid colours, gradients, concentric circles,
lines, text, and a full-colour sweep.

```bash
cd build
sudo ./testpattern
```

Each pattern is held for ~2 seconds. If any pattern looks wrong (colours
shifted, no display, partial image) see the [Troubleshooting](#12-troubleshooting)
section before proceeding.

`testpattern2` runs an extended set of patterns targeting HUD primitives:
reticles, badges, bearing arcs, and bars.

```bash
sudo ./testpattern2
```

---

## 5. Displaying a static PNG image

### Step 1 — Prepare your image

The converter accepts any PNG. For best results use a square image; non-square
images are scaled and cropped automatically.

```bash
# Convert PNG to C header
cd beepi_gc9a01a_nt
python3 assets/png_to_rgb565.py your_image.png assets/logo_rgb565.h logo_rgb565
```

This creates `assets/logo_rgb565.h` containing:

```cpp
const uint16_t logo_rgb565[57600];   // 240*240 big-endian RGB565 pixels
const uint16_t logo_rgb565_width  = 240;
const uint16_t logo_rgb565_height = 240;
```

### Step 2 — Build and run

```bash
cd build
cmake ..          # re-run cmake so it picks up the new header
make imgview -j$(nproc)
sudo ./imgview
```

`imgview` draws the image as a full-screen static frame, then overlays a
live animated HUD (reticle, range badge, compass arc, health bar) as a
demonstration of mixing image and OSD drawing.

### How it works internally

The display uses `MY|MX` MADCTL — rows are scanned bottom-to-top and columns
right-to-left. To correct this, `imgview` copies the image into a frame
buffer with the rows flipped before calling `pushFrame`:

```cpp
for (int r = 0; r < 240; r++)
    memcpy(fb + r * 240, logo_rgb565 + (239 - r) * 240, 240 * sizeof(uint16_t));
display.pushFrame(fb);
```

You do not need to handle this yourself when using the built-in drawing API
(`drawLabel`, `fillRect`, etc.) — it is only relevant when passing raw pixel
buffers through `pushFrame`.

---

## 6. Playing an animated GIF

### Step 1 — Convert your GIF

```bash
cd beepi_gc9a01a_nt
python3 assets/gif_to_frames.py assets/ocean.gif assets/ocean.bin
```

The converter uses Pillow to decode all GIF frames reliably (handles
transparency, disposal methods, and partial-frame GIFs). Output is a BPGF
binary — see the format description in README.md.

GIFs of any size are supported. Non-240×240 input is scaled and centre-cropped
automatically.

### Step 2 — Play

```bash
cd build
sudo ./videoview                          # plays assets/ocean.bin
sudo ./videoview /path/to/other.bin       # custom bin file
```

The player mmaps the bin file (no heap allocation for frame data), reads
per-frame duration from the metadata, and sleeps precisely between frames.

### Converter options

```bash
python3 assets/gif_to_frames.py --help
```

No extra options are required for standard GIFs. The FPS stored in the output
bin matches the GIF's own frame duration metadata.

---

## 7. Playing a video file

`video_to_frames.py` accepts any format that ffmpeg can decode: MP4, MKV,
AVI, MOV, WebM, and so on.

### Step 1 — Convert

```bash
cd beepi_gc9a01a_nt

# Basic conversion at source fps (capped to 60)
python3 assets/video_to_frames.py input.mp4 assets/video.bin

# Target 50 fps explicitly (recommended for stress testing)
python3 assets/video_to_frames.py input.mp4 assets/video.bin --fps 50

# Limit to first 500 frames (quick test)
python3 assets/video_to_frames.py input.mp4 assets/video.bin --fps 50 --max-frames 500
```

The converter pipeline:

```
ffmpeg decode (any codec)
    → raw RGB24 frames
    → Python centre-crop to 1:1 aspect ratio
    → Lanczos resize to 240×240
    → big-endian RGB565 pack
    → BPGF binary
```

Progress is printed every 30 frames. A 60 fps 1-minute video produces roughly
~166 MB of bin data.

### Step 2 — Play

```bash
cd build
sudo ./videoplay ../assets/video.bin
```

`videoplay` is the same binary as `videoview_osd` — it includes the FPS OSD
overlay. You will see `Display FPS: XX (target: XX)` printed to the terminal
every second, and `FPS:XX` overlaid on the display itself in the top bar.

### Automatic build integration

If you place your video at `assets/input.mp4` before running cmake, the build
system converts it automatically:

```bash
cp your_video.mp4 beepi_gc9a01a_nt/assets/input.mp4
cd build && cmake .. && make videoplay -j$(nproc)
sudo ./videoplay
```

---

## 8. FPS OSD overlay

`videoview_osd` (and `videoplay`) draw a live FPS counter at the top of the
screen while playing any BPGF binary.

### What it shows

- A solid black bar at the top of the display, 28 pixels tall
- White text centred in the bar: `FPS:XX` where XX is the actual measured
  display fps over the last one-second window
- Terminal output: `Display FPS: XX  (target: XX)` updated every second

### Expected fps values

| SPI speed | Theoretical max | Typical achieved |
|---|---|---|
| 20 MHz | ~21 fps | 20 fps |
| 40 MHz | ~43 fps | 38–40 fps |
| 62.5 MHz | ~68 fps | 50–55 fps |

The SPI speed is set in `make_config()` in the example source files:

```cpp
cfg.spi_speed_hz = 62500000u;   // 62.5 MHz
```

### Orientation notes for custom OSD drawing

The display uses `MY|MX` MADCTL. When drawing directly into a frame buffer
that will be passed to `pushFrame`, keep these rules in mind:

- Rows: `fb[r * 240 + c]` appears at screen row `(239 - r)`. The row-flip
  corrects this — draw at fb row `(239 - desired_screen_row)`.
- Columns: sequential writes fill left-to-right correctly regardless of MX.
  Draw at `fb[fy * 240 + fx]` for screen column `fx` directly — no x-mirror
  needed.

The OSD bar targets the screen top. Since fb row `239` = screen row `0` (top),
the bar is drawn at `fb rows (240 - BAR_H) .. 239`.

---

## 9. Drawing primitives and HUD elements

These methods are available on any `BeePi_GC9A01A` instance and do not
require a frame buffer. They write directly to display RAM via SPI.

### Basic shapes

```cpp
display.fill(BEEPI_BLACK);
display.fillRect(10, 10, 100, 50, BEEPI_NAVY);
display.drawRect(10, 10, 100, 50, BEEPI_WHITE);
display.drawCircle(120, 120, 60, BEEPI_GREEN);
display.fillCircle(120, 120, 10, BEEPI_RED);
display.drawLine(0, 0, 239, 239, BEEPI_YELLOW);
display.drawRoundRect(20, 20, 80, 40, 8, BEEPI_CYAN);
```

### Text labels

The built-in 5×8 bitmap font supports printable ASCII. Scale multiplies the
glyph size: scale 1 = 5×8 px, scale 2 = 10×16 px, scale 3 = 15×24 px.

```cpp
// Absolute position
display.drawLabel("HELLO", 60, 112, BEEPI_GREEN, 2);

// Corner-anchored with margin
display.drawLabelAnchored("TOP LEFT", BEEPI_ANCHOR_TOP_LEFT,    4, BEEPI_WHITE, 1);
display.drawLabelAnchored("CENTRE",   BEEPI_ANCHOR_CENTER,      0, BEEPI_YELLOW, 2);
display.drawLabelAnchored("BOT RIGHT",BEEPI_ANCHOR_BOTTOM_RIGHT,4, BEEPI_CYAN, 1);

// Numeric values with units
display.drawInt(847, " m", 8, 8, BEEPI_GREEN, 1);
display.drawFloat(3.14f, 2, " rad", 8, 20, BEEPI_WHITE, 1);
```

Background parameter: pass the same colour as the foreground for a
transparent background (no fill drawn behind the text).

```cpp
display.drawLabel("TRANSPARENT", 20, 60, BEEPI_RED, 1, BEEPI_RED);  // transparent bg
display.drawLabel("SOLID BG",    20, 80, BEEPI_RED, 1, BEEPI_BLACK); // black bg
```

### Targeting reticle

```cpp
// Cross with centre gap — common targeting style
display.drawReticle(120, 120, 30, 8, BEEPI_RETICLE_CROSS_GAP, BEEPI_GREEN, 2);

// Full crosshair
display.drawReticle(120, 120, 40, 0, BEEPI_RETICLE_CROSSHAIR, BEEPI_WHITE, 1);

// Circle with tick marks
display.drawReticle(120, 120, 25, 12, BEEPI_RETICLE_CIRCLE_CROSS, BEEPI_RED, 1);

// Simple dot
display.drawReticle(120, 120, 0, 4, BEEPI_RETICLE_DOT, BEEPI_YELLOW, 1);
```

Parameters: `(cx, cy, arm_length, gap_or_radius, style, colour, thickness)`

### Range / value badge

Draws a two-row labelled box — label on top, value + unit on the bottom.

```cpp
display.drawBadge("RNG", 847, "m",   8,   8, BEEPI_GREEN, BEEPI_BLACK);
display.drawBadge("ALT", 1250, "m",  8,  40, BEEPI_CYAN,  BEEPI_BLACK);
display.drawBadge("SPD", 32,  "kts", 8,  72, BEEPI_WHITE, BEEPI_BLACK);
```

### Horizontal bar (health / progress)

```cpp
// drawBarH(x, y, width, height, value, max_value, bar_colour, bg_colour, border_colour)
display.drawBarH(72, 218, 96, 7, 6, 20, BEEPI_GREEN, BEEPI_DARKGREY, BEEPI_WHITE);
```

### Bearing arc (compass HUD)

```cpp
// drawBearingArc(bearing_deg, fov_deg, radius, arc_colour, tick_colour)
display.drawBearingArc(45.0f, 90.0f, 110, BEEPI_WHITE, BEEPI_YELLOW);
```

Draws a partial arc spanning `fov_deg` degrees around the display edge,
with a tick mark at the current bearing.

### Colour utilities

```cpp
// Runtime RGB888 → RGB565
uint16_t orange = BeePi_GC9A01A::rgb(255, 128, 0);

// Compile-time macro (use in constant expressions)
uint16_t teal = BEEPI_RGB(0, 180, 180);

// Interpolate between two colours
uint16_t mid = BeePi_GC9A01A::lerpColor(BEEPI_RED, BEEPI_BLUE, 128);  // 50%

// Dim a colour
uint16_t dim = BeePi_GC9A01A::dimColor(BEEPI_GREEN, 80);  // ~31% brightness
```

---

## 10. Integrating a live camera feed

`pushFrame` transfers a full 240×240 RGB565 buffer in a single SPI DMA
call — this is the primary path for live camera display.

### Minimum integration

```cpp
BeePiHALConfig cfg = {};
cfg.spi_device   = "/dev/spidev0.0";
cfg.spi_speed_hz = 62500000u;
cfg.gpio_dc      = 25;
cfg.gpio_rst     = 24;
cfg.gpio_bl      = 18;
cfg.gpio_chip    = "/dev/gpiochip0";

BeePi_GC9A01A display(cfg);
display.begin();

uint16_t fb[240 * 240];

while (running) {
    // Fill fb[] from your camera source (V4L2, GStreamer appsink, etc.)
    get_camera_frame(fb);
    display.pushFrame(fb);
}
```

### With GStreamer appsink

Configure your GStreamer pipeline to output `video/x-raw,format=RGB16` at
240×240. In the appsink callback:

```cpp
void on_new_sample(GstAppSink *sink, void *user_data)
{
    BeePi_GC9A01A *display = static_cast<BeePi_GC9A01A *>(user_data);
    GstSample *sample = gst_app_sink_pull_sample(sink);
    GstBuffer *buf    = gst_sample_get_buffer(sample);
    GstMapInfo map;

    if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
        if (map.size == 240 * 240 * 2)
            display->pushFrame(reinterpret_cast<const uint16_t *>(map.data));
        gst_buffer_unmap(buf, &map);
    }
    gst_sample_unref(sample);
}
```

### Partial region update

For overlays that update infrequently (range readout, status text) use
`pushRegion` to avoid resending the whole frame:

```cpp
// Update only the top-left 80×20 px badge area
uint16_t badge_pixels[80 * 20];
render_badge(badge_pixels);
display.pushRegion(0, 0, 80, 20, badge_pixels);
```

---

## 11. Configuration reference

```cpp
BeePiHALConfig cfg = {};

// SPI
cfg.spi_device   = "/dev/spidev0.0";  // SPI bus device node
cfg.spi_speed_hz = 62500000u;         // SPI clock in Hz
                                       //   20 MHz — safe with long jumper wires
                                       //   40 MHz — safe on all Pi with short wires
                                       //   62.5 MHz — maximum tested-stable on Pi 4

// GPIO (BCM pin numbers)
cfg.gpio_dc   = 25;                   // Data/Command pin (required)
cfg.gpio_rst  = 24;                   // Reset pin (-1 to skip)
cfg.gpio_bl   = 18;                   // Backlight pin (-1 if hardwired on)
cfg.gpio_chip = "/dev/gpiochip0";     // GPIO chip — Pi 5 must use "/dev/gpiochip4"
```

### SPI speed vs fps

```
62.5 MHz  →  ~68 fps theoretical  →  ~50–55 fps real
40.0 MHz  →  ~43 fps theoretical  →  ~38–40 fps real
20.0 MHz  →  ~21 fps theoretical  →  ~20 fps real
```

---

## 12. Troubleshooting

### No display output at all

- Verify wiring. The most common error is swapping DC and RST.
- Check SPI is enabled: `ls /dev/spidev0.*`
- Run `sudo ./testpattern` — if it exits without error but screen is blank,
  check the BLK/backlight pin. Set `gpio_bl = -1` if the backlight is
  hardwired.
- Check that `gpio_chip` matches your Pi model.

### All colours appear blue / wrong colours

The pixel data path requires big-endian RGB565 storage in the bin/header
files. The HAL byte-swaps pixels on transmission. If you are writing your
own converter, pack as:

```python
v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)  # standard RGB565
v = ((v & 0xFF) << 8) | (v >> 8)                        # byte-swap to big-endian
```

### Image is upside-down

The `copy_frame_to_fb` row-flip is required before every `pushFrame` call
when using raw pixel buffers. Make sure your display loop includes:

```cpp
for (int r = 0; r < 240; r++)
    memcpy(fb + r * 240, src + (239 - r) * 240, 240 * sizeof(uint16_t));
display.pushFrame(fb);
```

The built-in drawing API (`drawLabel`, `fill`, etc.) is not affected — this
only applies to raw buffer pushes.

### OSD text appears mirrored

Do not apply any x-mirror compensation when writing OSD pixels into `fb[]`.
Write directly to `fb[fy * 240 + fx]`. The MX MADCTL flag reverses the GRAM
scan direction, but sequential pixel writes from `pushFrame` always fill
left-to-right on screen regardless.

### OSD text appears upside-down

The OSD must be drawn to match the `MY` row-flip. Target the correct fb rows:

- Screen top = fb rows `(240 - BAR_H) .. 239`
- Font bit 0 (glyph top on screen) = highest fb row in the glyph:
  draw from `y_top` downward in fb index as row index increases.

### Build fails: `liblgpio not found`

```bash
sudo apt install liblgpio-dev
```

### Build fails: `python3-pil not found` (asset converters)

```bash
sudo apt install python3-pil
# or:
pip3 install Pillow --break-system-packages
```

### Low fps / stuttering video

- Increase `spi_speed_hz` in `make_config()`. Try `40000000` then `62500000`.
- Check that the bin file is on a fast storage path (SD card Class 10 or USB SSD).
- For the video converter, verify that ffmpeg resampled to the target fps:
  the terminal output shows `target X fps` and `Display FPS: X` should match.

### `begin()` returns false

The HAL failed to open the SPI device or configure GPIO. Common causes:

- Not running with `sudo` (spidev and lgpio both require elevated privileges
  or udev rules granting access).
- Wrong `gpio_chip` for your Pi model.
- SPI not enabled in raspi-config.