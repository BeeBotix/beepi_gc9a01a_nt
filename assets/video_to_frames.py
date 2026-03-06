#!/usr/bin/env python3
"""
video_to_frames.py  --  Convert any video to a BPGF binary frame file.

Pipeline:
  1. ffmpeg decodes the video to raw RGB frames (any format/codec supported)
  2. Each frame is centre-cropped to 1:1 aspect ratio
  3. Scaled to 240x240 using Lanczos (high quality)
  4. Packed as big-endian RGB565 (matching imgview/videoview format)
  5. Written to BPGF binary

Usage:
  python3 video_to_frames.py input.mp4 output.bin [--fps 50] [--max-frames 0]

Options:
  --fps N          Target playback fps (default: use source fps, capped at 60)
  --max-frames N   Stop after N frames (0 = all frames, default 0)
  --no-osd         Skip printing progress

Binary format (BPGF) — same as gif_to_frames.py:
  [0..3]   magic "BPGF"
  [4..5]   uint16 width  (240)
  [6..7]   uint16 height (240)
  [8..11]  uint32 frame_count
  [12..13] uint16 avg_fps
  [14..]   uint16[frame_count] per-frame duration_ms
  [pad to 512-byte boundary]
  [pixel data] frame_count x 240 x 240 x 2 bytes  big-endian RGB565

Requirements:
  pip3 install Pillow --break-system-packages
  sudo apt install ffmpeg
"""

import sys
import os
import struct
import subprocess
import argparse
import json

DST_W = 240
DST_H = 240


def probe_video(path):
    """Return (width, height, fps_num, fps_den, nb_frames) via ffprobe."""
    cmd = [
        "ffprobe", "-v", "quiet",
        "-select_streams", "v:0",
        "-show_entries", "stream=width,height,r_frame_rate,nb_frames,duration",
        "-of", "json",
        path
    ]
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError:
        print("ERROR: ffprobe failed. Is ffmpeg installed?  sudo apt install ffmpeg",
              file=sys.stderr)
        sys.exit(1)

    info = json.loads(out)
    stream = info["streams"][0]
    w = int(stream["width"])
    h = int(stream["height"])

    num, den = stream["r_frame_rate"].split("/")
    fps = float(num) / float(den)

    nb = int(stream.get("nb_frames") or 0)
    if nb == 0:
        dur = float(stream.get("duration") or 0)
        if dur > 0:
            nb = int(dur * fps + 0.5)

    return w, h, fps, nb


def centre_crop_scale(pil_img):
    """Centre-crop to square then scale to 240x240 with Lanczos."""
    from PIL import Image
    w, h = pil_img.size
    side = min(w, h)
    left = (w - side) // 2
    top  = (h - side) // 2
    img  = pil_img.crop((left, top, left + side, top + side))
    if img.size != (DST_W, DST_H):
        img = img.resize((DST_W, DST_H), Image.LANCZOS)
    return img


def frame_to_rgb565(pil_img):
    """Convert 240x240 RGB Pillow image to big-endian RGB565 bytearray."""
    img    = pil_img.convert("RGB")
    pixels = img.tobytes()   # packed RGBRGB... 3 bytes per pixel
    out    = bytearray(DST_W * DST_H * 2)
    for i in range(DST_W * DST_H):
        r = pixels[i * 3]
        g = pixels[i * 3 + 1]
        b = pixels[i * 3 + 2]
        v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        # Byte-swap to big-endian to match imgview pack_be (HAL bswaps on send)
        v = ((v & 0xFF) << 8) | (v >> 8)
        out[i * 2]     = v & 0xFF
        out[i * 2 + 1] = (v >> 8) & 0xFF
    return out


def convert(src_path, dst_path, target_fps=0, max_frames=0):
    try:
        from PIL import Image
    except ImportError:
        print("ERROR: Pillow not installed.  pip3 install Pillow --break-system-packages",
              file=sys.stderr)
        sys.exit(1)

    src_w, src_h, src_fps, nb_frames = probe_video(src_path)
    print(f"  Source : {src_path}")
    print(f"           {src_w}x{src_h}  {src_fps:.2f} fps  (~{nb_frames} frames)")

    # Determine output fps
    out_fps = float(target_fps) if target_fps > 0 else min(src_fps, 60.0)
    out_fps = min(out_fps, 60.0)   # hard cap — display physics
    dur_ms  = max(16, int(1000.0 / out_fps + 0.5))   # ms per output frame
    print(f"  Output : 240x240  {out_fps:.1f} fps  ({dur_ms} ms/frame)")

    if max_frames > 0:
        print(f"  Limit  : {max_frames} frames")

    # ffmpeg: decode video, output raw RGB24 frames on stdout
    # -vf fps=N resamples to target fps, crop and scale done in Python for accuracy
    vf = f"fps={out_fps}"
    cmd = [
        "ffmpeg", "-i", src_path,
        "-vf", vf,
        "-pix_fmt", "rgb24",
        "-f",  "rawvideo",
        "-",
    ]
    print(f"  Running ffmpeg ...")

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    frame_size = src_w * src_h * 3   # raw RGB24 bytes per frame from ffmpeg
    durations  = []
    frame_data = []
    count      = 0

    while True:
        raw = proc.stdout.read(frame_size)
        if len(raw) < frame_size:
            break

        from PIL import Image as PILImage
        img = PILImage.frombytes("RGB", (src_w, src_h), raw)
        img = centre_crop_scale(img)

        durations.append(dur_ms)
        frame_data.append(frame_to_rgb565(img))
        count += 1

        if count % 30 == 0 or (max_frames > 0 and count == max_frames):
            pct = f"{count}/{max_frames}" if max_frames > 0 else f"{count}"
            print(f"  encoded {pct} frames ...", flush=True)

        if max_frames > 0 and count >= max_frames:
            proc.terminate()
            break

    proc.wait()
    n = len(frame_data)

    if n == 0:
        print("ERROR: no frames decoded. Check the input file.", file=sys.stderr)
        sys.exit(1)

    print(f"  Total  : {n} frames encoded")

    # Write BPGF binary
    hdr_size = 14 + n * 2
    pad      = (512 - hdr_size % 512) % 512
    data_off = hdr_size + pad

    with open(dst_path, "wb") as f:
        f.write(struct.pack("<4sHHIH", b"BPGF", DST_W, DST_H, n, int(out_fps)))
        for d in durations:
            f.write(struct.pack("<H", d))
        f.write(b"\x00" * pad)
        for fd in frame_data:
            f.write(fd)

    total_mb = (data_off + n * DST_W * DST_H * 2) / 1e6
    print(f"  Wrote  : {dst_path}  ({total_mb:.1f} MB)")
    print(f"  Done.")


def main():
    ap = argparse.ArgumentParser(description="Convert video to BPGF for GC9A01A display")
    ap.add_argument("input",                          help="Input video file")
    ap.add_argument("output",                         help="Output .bin file")
    ap.add_argument("--fps",        type=float, default=0,
                    help="Target playback fps (default: source fps capped at 60)")
    ap.add_argument("--max-frames", type=int,   default=0,
                    help="Stop after N frames (0 = all)")
    args = ap.parse_args()

    if not os.path.exists(args.input):
        print(f"ERROR: input file not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    convert(args.input, args.output, args.fps, args.max_frames)


if __name__ == "__main__":
    main()