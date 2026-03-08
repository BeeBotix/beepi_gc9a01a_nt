/*!
 * @file picamview.cpp
 * BeeBotix GC9A01A — Pi Camera Live Feed
 *
 * Uses GStreamer + libcamerasrc (the correct way to access Pi cameras).
 * Direct V4L2 does not work because the Pi camera stack (libcamera) exposes
 * ISP sub-devices on /dev/video0..N — none of which are directly streamable
 * via VIDIOC_STREAMON. libcamerasrc handles the full ISP pipeline internally.
 *
 * Pipeline:
 *   libcamerasrc
 *     -> video/x-raw WxH @ fps
 *     -> videoconvert
 *     -> video/x-raw,format=BGR          (BGR24, 3 bytes per pixel)
 *     -> appsink
 *
 * Display pipeline (in appsink callback):
 *   BGR24 frame
 *     -> centre-crop 1:1
 *     -> 2:1 (or N:1) bilinear downsample to 240x240
 *     -> BGR -> RGB565 pack + bswap16    (pushFrame big-endian contract)
 *     -> row-flip                        (MY|MX orientation)
 *     -> FPS OSD bar
 *     -> pushFrame (single SPI DMA ioctl)
 *
 * Threading:
 *   GStreamer internal thread  -> appsink callback -> writes ping-pong slot
 *   Display thread             -> reads slot, flips, OSD, pushFrame
 *
 * BeeBotix Autonomous Systems
 */

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <csignal>
#include <cstdlib>
#include <atomic>
#include <thread>
#include <string>

extern "C" {
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include "beepi_gc9a01a_nt.h"

// ---------------------------------------------------------------------------
// Tunable constants
// ---------------------------------------------------------------------------

static const int CAM_W   = 640;
static const int CAM_H   = 480;
static const int CAM_FPS = 30;

static const int DST_W   = 240;
static const int DST_H   = 240;

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
// Shutdown flag
// ---------------------------------------------------------------------------

static volatile std::sig_atomic_t g_quit = 0;
static GstElement* g_pipeline = nullptr;

static void on_signal(int)
{
    g_quit = 1;
    if (g_pipeline)
        gst_element_send_event(g_pipeline, gst_event_new_eos());
}

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------

static uint64_t now_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)(ts.tv_nsec / 1000ULL);
}

// ---------------------------------------------------------------------------
// BGR24 -> RGB565 + bswap  centre-crop + downsample
//
// Source: CAM_W x CAM_H BGR24 (3 bytes per pixel, row-major)
// Crop:   centre square, side = min(CAM_W, CAM_H) = 480 for 640x480
// Scale:  fixed-point nearest-neighbour to 240x240
//
// pushFrame contract: big-endian RGB565
//   pack = ((R&0xF8)<<8)|((G&0xFC)<<3)|(B>>3)   (standard LE RGB565)
//   store = bswap16(pack)                         (big-endian for pushFrame)
//
// Note: GStreamer videoconvert outputs BGR (not RGB), so bytes are B,G,R.
// ---------------------------------------------------------------------------

static void bgr24_crop_scale_to_rgb565(
    const uint8_t *src,    // BGR24 frame from GStreamer appsink
    int            src_w,
    int            src_h,
    uint16_t      *dst)    // DST_W * DST_H big-endian RGB565
{
    const int crop_side = (src_w < src_h) ? src_w : src_h;
    const int crop_x0   = (src_w - crop_side) / 2;
    const int crop_y0   = (src_h - crop_side) / 2;
    const int src_stride = src_w * 3;   // BGR24: 3 bytes per pixel

    // Fixed-point step (16.16)
    const uint32_t step_fp = ((uint32_t)crop_side << 16) / (uint32_t)DST_W;

    for (int oy = 0; oy < DST_H; oy++) {
        int sy = crop_y0 + (int)(((uint32_t)oy * step_fp) >> 16);
        const uint8_t *srow = src + sy * src_stride;

        for (int ox = 0; ox < DST_W; ox++) {
            int sx = crop_x0 + (int)(((uint32_t)ox * step_fp) >> 16);
            const uint8_t *px = srow + sx * 3;

            uint8_t b = px[0];
            uint8_t g = px[1];
            uint8_t r = px[2];

            uint16_t le = (uint16_t)(((r & 0xF8u) << 8) |
                                     ((g & 0xFCu) << 3)  |
                                      (b >> 3));
            // bswap16 to big-endian (pushFrame contract)
            dst[oy * DST_W + ox] = (uint16_t)((le >> 8) | (le << 8));
        }
    }
}

// ---------------------------------------------------------------------------
// Ping-pong frame slot  (lock-free, same model as camview.cpp)
// ---------------------------------------------------------------------------

static uint16_t g_slot[2][DST_W * DST_H];
static std::atomic<int>  g_write_idx{0};
static std::atomic<bool> g_fresh{false};

static std::atomic<uint32_t> g_cap_fps_count{0};
static std::atomic<uint32_t> g_disp_fps_count{0};

// ---------------------------------------------------------------------------
// OSD  (identical to camview.cpp)
// ---------------------------------------------------------------------------

static const uint8_t FONT5x8[][5] = {
    /* ' ' */ {0x00,0x00,0x00,0x00,0x00},
    /* 'F' */ {0x7F,0x09,0x09,0x01,0x01},
    /* 'P' */ {0x7F,0x09,0x09,0x09,0x06},
    /* 'S' */ {0x46,0x49,0x49,0x49,0x31},
    /* ':' */ {0x00,0x36,0x36,0x00,0x00},
    /* '0' */ {0x3E,0x51,0x49,0x45,0x3E},
    /* '1' */ {0x00,0x42,0x7F,0x40,0x00},
    /* '2' */ {0x42,0x61,0x51,0x49,0x46},
    /* '3' */ {0x21,0x41,0x45,0x4B,0x31},
    /* '4' */ {0x18,0x14,0x12,0x7F,0x10},
    /* '5' */ {0x27,0x45,0x45,0x45,0x39},
    /* '6' */ {0x3C,0x4A,0x49,0x49,0x30},
    /* '7' */ {0x01,0x71,0x09,0x05,0x03},
    /* '8' */ {0x36,0x49,0x49,0x49,0x36},
    /* '9' */ {0x06,0x49,0x49,0x29,0x1E},
};

static int font_idx(char c)
{
    switch (c) {
        case ' ': return 0; case 'F': return 1; case 'P': return 2;
        case 'S': return 3; case ':': return 4; case '0': return 5;
        case '1': return 6; case '2': return 7; case '3': return 8;
        case '4': return 9; case '5': return 10; case '6': return 11;
        case '7': return 12; case '8': return 13; case '9': return 14;
        default:  return 0;
    }
}

static const int OSD_BAR_H  = 28;
static const int OSD_SCALE  = 2;
static const int OSD_GLYPH_W = (5 + 1) * OSD_SCALE;

static void osd_draw_char(uint16_t *fb, char c, int px, int py)
{
    const uint8_t *bits   = FONT5x8[font_idx(c)];
    const int      y_top  = py + 8 * OSD_SCALE - 1;

    for (int col = 0; col < 5; col++) {
        uint8_t colbits = bits[col];
        for (int row = 0; row < 8; row++) {
            if (!(colbits & (1 << row))) continue;
            for (int sy = 0; sy < OSD_SCALE; sy++)
                for (int sx = 0; sx < OSD_SCALE; sx++) {
                    int fx = px + col * OSD_SCALE + sx;
                    int fy = y_top - row * OSD_SCALE - sy;
                    if (fx >= 0 && fx < DST_W && fy >= 0 && fy < DST_H)
                        fb[fy * DST_W + fx] = 0xFFFFu;   // white
                }
        }
    }
}

static void draw_osd(uint16_t *fb, uint32_t disp_fps)
{
    const int bar_y0  = DST_H - OSD_BAR_H;

    for (int fy = bar_y0; fy < DST_H; fy++)
        for (int fx = 0; fx < DST_W; fx++)
            fb[fy * DST_W + fx] = 0x0000u;

    char buf[16];
    snprintf(buf, sizeof(buf), "FPS:%u", disp_fps);
    int len = 0; while (buf[len]) len++;

    const int glyph_h = 8 * OSD_SCALE;
    const int x0      = (DST_W - len * OSD_GLYPH_W) / 2;
    const int y0      = bar_y0 + (OSD_BAR_H - glyph_h) / 2;

    for (int i = 0; i < len; i++)
        osd_draw_char(fb, buf[i], x0 + i * OSD_GLYPH_W, y0);
}

// ---------------------------------------------------------------------------
// Row-flip
// ---------------------------------------------------------------------------

static uint16_t g_fb_flip[DST_W * DST_H];

static void row_flip(const uint16_t *src, uint16_t *dst)
{
    for (int r = 0; r < DST_H; r++)
        memcpy(dst + r * DST_W, src + (DST_H - 1 - r) * DST_W,
               DST_W * sizeof(uint16_t));
}

// ---------------------------------------------------------------------------
// GStreamer appsink callback
// Called from GStreamer's internal streaming thread on every new frame.
// ---------------------------------------------------------------------------

static GstFlowReturn on_new_sample(GstAppSink *appsink, gpointer user_data)
{
    int *dims  = static_cast<int *>(user_data);
    int  src_w = dims[0];
    int  src_h = dims[1];

    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample) return GST_FLOW_ERROR;

    GstBuffer  *buffer = gst_sample_get_buffer(sample);
    GstMapInfo  map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    // Write into the slot capture is NOT reading
    int widx = g_write_idx.load(std::memory_order_relaxed);

    bgr24_crop_scale_to_rgb565(
        static_cast<const uint8_t *>(map.data),
        src_w, src_h,
        g_slot[widx]);

    g_write_idx.store(1 - widx, std::memory_order_release);
    g_fresh.store(true, std::memory_order_release);
    g_cap_fps_count.fetch_add(1, std::memory_order_relaxed);

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// Display thread
// ---------------------------------------------------------------------------

static void display_thread_fn(BeePi_GC9A01A *display)
{
    uint32_t disp_fps_shown = 0;
    uint32_t disp_count     = 0;
    uint64_t win_start      = now_us();

    display->fill(BEEPI_BLACK);

    while (!g_quit) {
        if (!g_fresh.load(std::memory_order_acquire)) {
            struct timespec ts = {0, 500000};
            nanosleep(&ts, nullptr);
            continue;
        }
        g_fresh.store(false, std::memory_order_release);

        int ridx = 1 - g_write_idx.load(std::memory_order_acquire);

        row_flip(g_slot[ridx], g_fb_flip);
        draw_osd(g_fb_flip, disp_fps_shown);
        display->pushFrame(g_fb_flip);
        disp_count++;

        uint64_t now = now_us();
        if (now - win_start >= 1000000ULL) {
            disp_fps_shown = disp_count;
            disp_count     = 0;
            win_start      = now;

            uint32_t cap_fps = g_cap_fps_count.exchange(0, std::memory_order_relaxed);
            printf("  Capture: %2u fps   Display: %2u fps\r",
                   cap_fps, disp_fps_shown);
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

    int width  = (argc >= 2) ? atoi(argv[1]) : CAM_W;
    int height = (argc >= 3) ? atoi(argv[2]) : CAM_H;
    int fps    = (argc >= 4) ? atoi(argv[3]) : CAM_FPS;

    printf("BeeBotix GC9A01A — picamview\n");
    printf("  Camera : libcamerasrc  %dx%d @ %d fps\n", width, height, fps);

    // GStreamer init
    gst_init(&argc, &argv);

    // Build pipeline string — identical source to rtsp.cpp
    std::string pipeline_str =
        "libcamerasrc ! "
        "video/x-raw,width="      + std::to_string(width)  +
        ",height="                 + std::to_string(height) +
        ",framerate="              + std::to_string(fps)    + "/1 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink name=sink emit-signals=false max-buffers=1 drop=true sync=false";

    printf("  Pipeline: %s\n", pipeline_str.c_str());

    GError     *err      = nullptr;
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), &err);
    g_pipeline           = pipeline;

    if (!pipeline || err) {
        fprintf(stderr, "ERROR: failed to create pipeline: %s\n",
                err ? err->message : "unknown");
        return 1;
    }

    // Wire up appsink callback
    GstElement *sink_elem = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
    GstAppSink *appsink   = GST_APP_SINK(sink_elem);

    static int dims[2];
    dims[0] = width;
    dims[1] = height;

    GstAppSinkCallbacks cbs = {};
    cbs.new_sample = on_new_sample;
    gst_app_sink_set_callbacks(appsink, &cbs, dims, nullptr);

    // Open display
    BeePiHALConfig cfg = make_config();
    BeePi_GC9A01A  display(cfg);
    if (!display.begin()) {
        fprintf(stderr, "ERROR: display.begin() failed\n");
        gst_object_unref(pipeline);
        return 1;
    }
    printf("  SPI    : %.0f MHz\n", cfg.spi_speed_hz / 1e6);
    printf("  Press Ctrl+C to stop\n\n");

    // Launch display thread
    std::thread disp_thread(display_thread_fn, &display);

    // Start pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        fprintf(stderr, "ERROR: failed to start pipeline\n");
        g_quit = 1;
        disp_thread.join();
        gst_object_unref(pipeline);
        display.end();
        return 1;
    }

    // GStreamer bus loop — blocks until EOS or error (Ctrl+C sends EOS)
    GstBus *bus = gst_element_get_bus(pipeline);
    while (true) {
        GstMessage *msg = gst_bus_timed_pop_filtered(
            bus, GST_SECOND,
            static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (msg) {
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR: {
                    GError *gerr = nullptr; gchar *info = nullptr;
                    gst_message_parse_error(msg, &gerr, &info);
                    fprintf(stderr, "\nGStreamer error: %s\n", gerr->message);
                    g_error_free(gerr); g_free(info);
                    gst_message_unref(msg);
                    goto cleanup;
                }
                case GST_MESSAGE_EOS:
                    gst_message_unref(msg);
                    goto cleanup;
                default:
                    gst_message_unref(msg);
                    break;
            }
        }
        if (g_quit) break;
    }

cleanup:
    g_quit = 1;
    printf("\nStopped.\n");

    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);

    disp_thread.join();

    display.fill(BEEPI_BLACK);
    display.end();

    return 0;
}