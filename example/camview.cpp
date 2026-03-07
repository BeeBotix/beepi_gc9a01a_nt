/*!
 * @file camview.cpp
 * BeeBotix GC9A01A — Live USB Camera Feed
 *
 * Pipeline:
 *   V4L2 capture (640x480 YUYV)
 *     -> centre-crop 480x480 (skip 80px left/right)
 *     -> 2:1 nearest-neighbour downsample to 240x240
 *     -> RGB565 pack
 *     -> row-flip (MY|MX orientation)
 *     -> FPS OSD overlay
 *     -> pushFrame (single SPI DMA ioctl)
 *
 * Threading:
 *   Capture thread  — V4L2 DQBUF, YUYV->RGB565 crop+scale, write ping-pong slot
 *   Display thread  — row-flip, OSD draw, pushFrame, fps measurement
 *   Lock-free handoff via std::atomic<int> slot index + std::atomic<bool> fresh flag
 *
 * BeeBotix Autonomous Systems
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <csignal>
#include <cassert>
#include <atomic>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

#include "beepi_gc9a01a_nt.h"

// ---------------------------------------------------------------------------
// Tunable constants
// ---------------------------------------------------------------------------

static const char   *CAM_DEVICE    = "/dev/video0";
static const int     CAM_W         = 640;
static const int     CAM_H         = 480;
static const int     CAM_FPS       = 30;
static const int     V4L2_NBUF     = 2;      // kernel mmap buffer count

static const int     DST_W         = 240;
static const int     DST_H         = 240;

// ---------------------------------------------------------------------------
// Hardware config
// ---------------------------------------------------------------------------

static BeePiHALConfig make_config()
{
    BeePiHALConfig cfg = {};
    cfg.spi_device   = "/dev/spidev0.0";
    cfg.spi_speed_hz = 62500000u;
    cfg.gpio_dc      = 25;
    cfg.gpio_rst     = 24;
    cfg.gpio_bl      = 18;
    cfg.gpio_chip    = "/dev/gpiochip0";
    return cfg;
}

// ---------------------------------------------------------------------------
// Global shutdown flag
// ---------------------------------------------------------------------------

static volatile std::sig_atomic_t g_quit = 0;
static void on_signal(int) { g_quit = 1; }

// ---------------------------------------------------------------------------
// Timing helpers
// ---------------------------------------------------------------------------

static uint64_t now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000ULL);
}

// ---------------------------------------------------------------------------
// V4L2 context
// ---------------------------------------------------------------------------

struct V4L2Buf {
    void   *start;
    size_t  length;
};

struct V4L2Ctx {
    int       fd;
    V4L2Buf   bufs[V4L2_NBUF];
    uint32_t  width;
    uint32_t  height;
    uint32_t  stride;   // bytes per line (may be > width*2 for YUYV)
};

static bool v4l2_open(V4L2Ctx &ctx, const char *dev)
{
    ctx.fd = open(dev, O_RDWR | O_NONBLOCK);
    if (ctx.fd < 0) {
        perror(dev);
        return false;
    }

    // Verify capability
    struct v4l2_capability cap = {};
    if (ioctl(ctx.fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP");
        close(ctx.fd);
        return false;
    }
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "%s: not a capture device\n", dev);
        close(ctx.fd);
        return false;
    }

    // Request YUYV 640x480
    struct v4l2_format fmt = {};
    fmt.type                         = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width                = (uint32_t)CAM_W;
    fmt.fmt.pix.height               = (uint32_t)CAM_H;
    fmt.fmt.pix.pixelformat          = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field                = V4L2_FIELD_NONE;
    if (ioctl(ctx.fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        close(ctx.fd);
        return false;
    }

    // Read back negotiated format
    if (ioctl(ctx.fd, VIDIOC_G_FMT, &fmt) < 0) {
        perror("VIDIOC_G_FMT");
        close(ctx.fd);
        return false;
    }
    ctx.width  = fmt.fmt.pix.width;
    ctx.height = fmt.fmt.pix.height;
    ctx.stride = fmt.fmt.pix.bytesperline;

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
        fprintf(stderr, "Camera did not accept YUYV format\n");
        close(ctx.fd);
        return false;
    }

    printf("  Camera : %s  %ux%u YUYV  stride=%u\n",
           dev, ctx.width, ctx.height, ctx.stride);

    // Request fps (best-effort, many cams ignore this)
    struct v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(ctx.fd, VIDIOC_G_PARM, &parm) == 0) {
        parm.parm.capture.timeperframe.numerator   = 1;
        parm.parm.capture.timeperframe.denominator = (uint32_t)CAM_FPS;
        ioctl(ctx.fd, VIDIOC_S_PARM, &parm);
    }

    // Request kernel mmap buffers
    struct v4l2_requestbuffers req = {};
    req.count  = V4L2_NBUF;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(ctx.fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        close(ctx.fd);
        return false;
    }

    // Map and enqueue each buffer
    for (uint32_t i = 0; i < (uint32_t)V4L2_NBUF; i++) {
        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (ioctl(ctx.fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            close(ctx.fd);
            return false;
        }
        ctx.bufs[i].length = buf.length;
        ctx.bufs[i].start  = mmap(nullptr, buf.length,
                                   PROT_READ | PROT_WRITE,
                                   MAP_SHARED, ctx.fd, buf.m.offset);
        if (ctx.bufs[i].start == MAP_FAILED) {
            perror("mmap");
            close(ctx.fd);
            return false;
        }
        if (ioctl(ctx.fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            close(ctx.fd);
            return false;
        }
    }

    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(ctx.fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        close(ctx.fd);
        return false;
    }

    return true;
}

static void v4l2_close(V4L2Ctx &ctx)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(ctx.fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < V4L2_NBUF; i++)
        munmap(ctx.bufs[i].start, ctx.bufs[i].length);
    close(ctx.fd);
}

// Block until a frame is available (handles EAGAIN from O_NONBLOCK fd)
static int v4l2_wait_frame(int fd)
{
    fd_set fds;
    struct timeval tv;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    tv.tv_sec  = 2;
    tv.tv_usec = 0;
    return select(fd + 1, &fds, nullptr, nullptr, &tv);
}

// ---------------------------------------------------------------------------
// YUYV -> RGB565  centre-crop + 2:1 downsample
//
// Source: CAM_W x CAM_H YUYV (2 bytes per pixel, packed as Y0 U Y1 V per macropixel)
// Crop:   centre 480x480 region -> skip (CAM_W - CAM_H)/2 = 80 columns on each side
// Scale:  pick every 2nd source pixel -> output 240x240
//
// One pass: iterate over 240 output rows and 240 output columns,
// map back to source coordinates, read YUYV macropixel, convert.
//
// BT.601 integer YUV->RGB (fast, no floats):
//   C = Y - 16
//   D = U - 128
//   E = V - 128
//   R = clip((298*C         + 409*E + 128) >> 8)
//   G = clip((298*C - 100*D - 208*E + 128) >> 8)
//   B = clip((298*C + 516*D         + 128) >> 8)
// ---------------------------------------------------------------------------

static inline uint8_t yuv_clip(int v)
{
    return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
}

static inline uint16_t yuv_to_rgb565(int y_raw, int u_raw, int v_raw)
{
    int C = y_raw - 16;
    int D = u_raw - 128;
    int E = v_raw - 128;

    uint8_t r = yuv_clip((298 * C           + 409 * E + 128) >> 8);
    uint8_t g = yuv_clip((298 * C - 100 * D - 208 * E + 128) >> 8);
    uint8_t b = yuv_clip((298 * C + 516 * D           + 128) >> 8);

    uint16_t px = (uint16_t)(((r & 0xF8u) << 8) | ((g & 0xFCu) << 3) | (b >> 3));
    // pushFrame expects big-endian RGB565 (same format as BEEPI_* colour constants).
    // Our pack produces standard LE RGB565, so byte-swap before storing.
    return (uint16_t)((px >> 8) | (px << 8));
}

static void yuyv_crop_scale_to_rgb565(
    const uint8_t *src,   // raw YUYV frame from V4L2
    uint32_t       src_w, // actual camera width  (e.g. 640)
    uint32_t       src_h, // actual camera height (e.g. 480)
    uint32_t       stride,// bytes per source line
    uint16_t      *dst)   // output: DST_W * DST_H RGB565 pixels
{
    // Centre-crop square side = min(src_w, src_h)
    const uint32_t crop_side = (src_w < src_h) ? src_w : src_h;
    const uint32_t crop_x0   = (src_w - crop_side) / 2;  // 80 for 640x480
    const uint32_t crop_y0   = (src_h - crop_side) / 2;  // 0  for 640x480

    // Scale factor: source pixels per output pixel
    // crop_side = 480, DST = 240 -> step = 2 exactly
    // Use fixed-point (16.16) for non-integer ratios
    const uint32_t step_fp = (crop_side << 16) / (uint32_t)DST_W;

    for (int oy = 0; oy < DST_H; oy++) {
        // Source row for this output row
        uint32_t sy = crop_y0 + (((uint32_t)oy * step_fp) >> 16);
        const uint8_t *srow = src + sy * stride;

        for (int ox = 0; ox < DST_W; ox++) {
            // Source column for this output column
            uint32_t sx = crop_x0 + (((uint32_t)ox * step_fp) >> 16);

            // YUYV: each macropixel (2 source pixels) = 4 bytes
            // macropixel index = sx / 2, byte offset = macropixel * 4
            uint32_t mp   = sx / 2;
            uint32_t base = mp * 4;

            int y = (sx & 1) ? (int)srow[base + 2] : (int)srow[base + 0];
            int u = (int)srow[base + 1];   // YUYV byte 1 = U (Cb)
            int v = (int)srow[base + 3];   // YUYV byte 3 = V (Cr)

            dst[oy * DST_W + ox] = yuv_to_rgb565(y, u, v);
        }
    }
}

// ---------------------------------------------------------------------------
// Ping-pong frame slot  (lock-free producer/consumer)
//
// Two buffers: capture thread writes to slot[write_idx],
// display thread reads from slot[1 - write_idx].
// write_idx is flipped atomically after each capture.
// fresh flag tells display thread a new frame is available.
// ---------------------------------------------------------------------------

static uint16_t g_slot[2][DST_W * DST_H];
static std::atomic<int>  g_write_idx{0};
static std::atomic<bool> g_fresh{false};

// Fps counters shared between threads and reporting
static std::atomic<uint32_t> g_cap_fps_count{0};
static std::atomic<uint32_t> g_disp_fps_count{0};

// ---------------------------------------------------------------------------
// OSD  (identical logic to videoview_osd, adapted for two fps values)
// ---------------------------------------------------------------------------

static const uint8_t FONT5x8[][5] = {
    /* [0]  ' ' */ {0x00,0x00,0x00,0x00,0x00},
    /* [1]  'F' */ {0x7F,0x09,0x09,0x01,0x01},
    /* [2]  'P' */ {0x7F,0x09,0x09,0x09,0x06},
    /* [3]  'S' */ {0x46,0x49,0x49,0x49,0x31},
    /* [4]  ':' */ {0x00,0x36,0x36,0x00,0x00},
    /* [5]  '0' */ {0x3E,0x51,0x49,0x45,0x3E},
    /* [6]  '1' */ {0x00,0x42,0x7F,0x40,0x00},
    /* [7]  '2' */ {0x42,0x61,0x51,0x49,0x46},
    /* [8]  '3' */ {0x21,0x41,0x45,0x4B,0x31},
    /* [9]  '4' */ {0x18,0x14,0x12,0x7F,0x10},
    /* [10] '5' */ {0x27,0x45,0x45,0x45,0x39},
    /* [11] '6' */ {0x3C,0x4A,0x49,0x49,0x30},
    /* [12] '7' */ {0x01,0x71,0x09,0x05,0x03},
    /* [13] '8' */ {0x36,0x49,0x49,0x49,0x36},
    /* [14] '9' */ {0x06,0x49,0x49,0x29,0x1E},
};

static int font_idx(char c)
{
    switch (c) {
        case ' ': return 0;
        case 'F': return 1;  case 'P': return 2;  case 'S': return 3;
        case ':': return 4;
        case '0': return 5;  case '1': return 6;  case '2': return 7;
        case '3': return 8;  case '4': return 9;  case '5': return 10;
        case '6': return 11; case '7': return 12; case '8': return 13;
        case '9': return 14;
        default:  return 0;
    }
}

static const int OSD_BAR_H = 28;
static const int OSD_SCALE = 2;
static const int OSD_GLYPH_W = (5 + 1) * OSD_SCALE;  // 12 px per char

static void osd_draw_char(uint16_t *fb, char c, int px, int py, uint16_t fg)
{
    const uint8_t *bits   = FONT5x8[font_idx(c)];
    const int      glyph_h = 8 * OSD_SCALE;
    const int      y_top   = py + glyph_h - 1;   // fb row of glyph screen-top

    for (int col = 0; col < 5; col++) {
        uint8_t colbits = bits[col];
        for (int row = 0; row < 8; row++) {
            if (!(colbits & (1 << row))) continue;
            for (int sy = 0; sy < OSD_SCALE; sy++) {
                for (int sx = 0; sx < OSD_SCALE; sx++) {
                    int fx = px + col * OSD_SCALE + sx;
                    int fy = y_top - row * OSD_SCALE - sy;
                    if (fx >= 0 && fx < DST_W && fy >= 0 && fy < DST_H)
                        fb[fy * DST_W + fx] = fg;
                }
            }
        }
    }
}

// Draw "FPS:XX" bar at screen top.
// Screen top in row-flipped fb = rows (DST_H - OSD_BAR_H) .. (DST_H - 1).
static void draw_osd(uint16_t *fb, uint32_t disp_fps)
{
    const int bar_y0 = DST_H - OSD_BAR_H;

    // Solid black bar
    for (int fy = bar_y0; fy < DST_H; fy++)
        for (int fx = 0; fx < DST_W; fx++)
            fb[fy * DST_W + fx] = 0x0000u;

    // Build label
    char buf[24];
    snprintf(buf, sizeof(buf), "FPS:%u", disp_fps);
    int len = 0;
    while (buf[len]) len++;

    const int glyph_h = 8 * OSD_SCALE;
    const int tw      = len * OSD_GLYPH_W;
    const int x0      = (DST_W - tw) / 2;
    const int y0      = bar_y0 + (OSD_BAR_H - glyph_h) / 2;

    for (int i = 0; i < len; i++)
        osd_draw_char(fb, buf[i], x0 + i * OSD_GLYPH_W, y0, 0xFFFFu);
}

// ---------------------------------------------------------------------------
// Row-flip  (MY orientation correction)
// ---------------------------------------------------------------------------

static void row_flip(const uint16_t *src, uint16_t *dst)
{
    for (int r = 0; r < DST_H; r++)
        memcpy(dst + r * DST_W, src + (DST_H - 1 - r) * DST_W,
               DST_W * sizeof(uint16_t));
}

// ---------------------------------------------------------------------------
// Capture thread
// ---------------------------------------------------------------------------

static void capture_thread_fn(V4L2Ctx *ctx)
{
    while (!g_quit) {
        // Wait for frame with 2s timeout
        int ret = v4l2_wait_frame(ctx->fd);
        if (ret <= 0) {
            if (ret == 0) fprintf(stderr, "capture: frame timeout\n");
            continue;
        }

        struct v4l2_buffer buf = {};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(ctx->fd, VIDIOC_DQBUF, &buf) < 0) {
            perror("VIDIOC_DQBUF");
            continue;
        }

        // Write into the slot that display is NOT currently reading
        int widx = g_write_idx.load(std::memory_order_relaxed);

        yuyv_crop_scale_to_rgb565(
            static_cast<const uint8_t *>(ctx->bufs[buf.index].start),
            ctx->width, ctx->height, ctx->stride,
            g_slot[widx]);

        // Flip write index and signal fresh frame
        g_write_idx.store(1 - widx, std::memory_order_release);
        g_fresh.store(true, std::memory_order_release);

        g_cap_fps_count.fetch_add(1, std::memory_order_relaxed);

        // Return buffer to kernel
        if (ioctl(ctx->fd, VIDIOC_QBUF, &buf) < 0)
            perror("VIDIOC_QBUF");
    }
}

// ---------------------------------------------------------------------------
// Display thread
// ---------------------------------------------------------------------------

static uint16_t g_fb_flip[DST_W * DST_H];   // row-flipped + OSD working buffer

static void display_thread_fn(BeePi_GC9A01A *display)
{
    uint32_t disp_fps_display = 0;
    uint32_t disp_count       = 0;
    uint64_t win_start        = now_us();

    // Show black until first frame arrives
    display->fill(BEEPI_BLACK);

    while (!g_quit) {
        // Spin-wait for fresh frame (low latency; camera is ~33ms so this
        // burns very little CPU — display would otherwise sleep here)
        if (!g_fresh.load(std::memory_order_acquire)) {
            struct timespec ts = {0, 500000};   // yield 0.5 ms
            nanosleep(&ts, nullptr);
            continue;
        }
        g_fresh.store(false, std::memory_order_release);

        // Read from the slot that capture is NOT currently writing to
        int ridx = 1 - g_write_idx.load(std::memory_order_acquire);

        // Row-flip into working buffer
        row_flip(g_slot[ridx], g_fb_flip);

        // OSD overlay
        draw_osd(g_fb_flip, disp_fps_display);

        // Push to display
        display->pushFrame(g_fb_flip);
        disp_count++;

        // Update fps every second
        uint64_t now = now_us();
        if (now - win_start >= 1000000ULL) {
            disp_fps_display = disp_count;
            disp_count       = 0;
            win_start        = now;

            uint32_t cap_fps = g_cap_fps_count.exchange(0, std::memory_order_relaxed);
            printf("  Capture: %2u fps   Display: %2u fps\r",
                   cap_fps, disp_fps_display);
            fflush(stdout);
        }
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    const char *dev = (argc > 1) ? argv[1] : CAM_DEVICE;

    printf("BeeBotix GC9A01A — camview\n");
    printf("  Device : %s\n", dev);

    // Open V4L2
    V4L2Ctx cam = {};
    if (!v4l2_open(cam, dev))
        return 1;

    // Open display
    BeePiHALConfig cfg = make_config();
    BeePi_GC9A01A  display(cfg);
    if (!display.begin()) {
        fprintf(stderr, "ERROR: display.begin() failed\n");
        v4l2_close(cam);
        return 1;
    }

    printf("  SPI    : %.0f MHz\n", cfg.spi_speed_hz / 1e6);
    printf("  Press Ctrl+C to stop\n\n");

    // Launch threads
    std::thread cap_thread(capture_thread_fn, &cam);
    std::thread disp_thread(display_thread_fn, &display);

    cap_thread.join();
    disp_thread.join();

    printf("\nStopped.\n");

    display.fill(BEEPI_BLACK);
    display.end();
    v4l2_close(cam);

    return 0;
}