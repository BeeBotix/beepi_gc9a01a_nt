#!/usr/bin/env python3
"""
gif_to_frames.py  --  Convert an animated GIF to a BPGF binary frame file.

Uses Pillow for reliable GIF decode (handles all disposal methods, LZW variants,
transparency, interlacing, and partial frames automatically).

Binary format (BPGF):
  [0..3]   magic "BPGF"
  [4..5]   uint16 width  (240)
  [6..7]   uint16 height (240)
  [8..11]  uint32 frame_count
  [12..13] uint16 avg_fps
  [14..]   uint16[frame_count] per-frame duration_ms
  [pad to 512-byte boundary]
  [pixel data] frame_count x 240 x 240 x 2 bytes  native LE RGB565
"""

import sys
import struct

DST_W = 240
DST_H = 240


def frame_to_rgb565(pil_image):
    """
    Convert a Pillow image (any mode) to a bytearray of 240*240*2 bytes
    in native LE RGB565, matching the packing used by png_to_rgb565.py:
      v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    stored low-byte first.

    If the source is exactly 240x240 no scaling is done.
    Otherwise it is scaled to fit then centre-cropped.
    """
    img = pil_image

    # Resize only if needed
    if img.size != (DST_W, DST_H):
        # Scale to cover, then centre-crop
        scale = max(DST_W / img.width, DST_H / img.height)
        new_w = max(DST_W, int(img.width  * scale + 0.5))
        new_h = max(DST_H, int(img.height * scale + 0.5))
        img = img.resize((new_w, new_h))
        left = (new_w - DST_W) // 2
        top  = (new_h - DST_H) // 2
        img  = img.crop((left, top, left + DST_W, top + DST_H))

    # Convert to plain RGB (no alpha, no palette)
    img = img.convert("RGB")

    pixels = img.tobytes()   # packed RGBRGB... bytes, 3 bytes per pixel

    out = bytearray(DST_W * DST_H * 2)
    for i in range(DST_W * DST_H):
        r = pixels[i * 3]
        g = pixels[i * 3 + 1]
        b = pixels[i * 3 + 2]
        # Pack as big-endian RGB565 to match imgview/logo header format.
        # imgview stores pixels byte-swapped (pack_be); HAL bswaps on send
        # so the wire value is correct RGB565.
        v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        v = ((v & 0xFF) << 8) | (v >> 8)   # byte-swap to big-endian
        out[i * 2]     = v & 0xFF
        out[i * 2 + 1] = (v >> 8) & 0xFF
    return out


def convert(src_path, dst_path):
    try:
        from PIL import Image
    except ImportError:
        print("ERROR: Pillow not installed.  Run:  pip3 install Pillow", file=sys.stderr)
        sys.exit(1)

    img = Image.open(src_path)
    if not hasattr(img, 'n_frames') or img.n_frames < 1:
        print("ERROR: not an animated GIF", file=sys.stderr)
        sys.exit(1)

    n = img.n_frames
    print(f"  {src_path}: {img.size[0]}x{img.size[1]}  {n} frames")

    durations  = []
    frame_data = []

    # Use a persistent RGBA canvas for correct GIF disposal compositing
    canvas = Image.new("RGBA", (img.size[0], img.size[1]), (0, 0, 0, 255))

    for i in range(n):
        img.seek(i)

        # Convert current frame to RGBA and paste onto canvas
        frame = img.convert("RGBA")
        canvas.paste(frame, (0, 0), frame)

        # Flatten canvas to RGB for encoding (black background)
        flat = Image.new("RGB", canvas.size, (0, 0, 0))
        flat.paste(canvas.convert("RGB"), (0, 0))

        dur = img.info.get('duration', 40)
        dur = max(dur, 20)
        durations.append(dur)
        frame_data.append(frame_to_rgb565(flat))

        if (i + 1) % 10 == 0 or i == n - 1:
            print(f"  encoded {i + 1}/{n} ...", flush=True)

    avg_ms  = sum(durations) / len(durations)
    avg_fps = max(1, round(1000.0 / avg_ms))
    print(f"  avg {avg_ms:.1f} ms/frame  (~{avg_fps} fps)")

    hdr_size = 14 + n * 2
    pad      = (512 - hdr_size % 512) % 512
    data_off = hdr_size + pad

    with open(dst_path, 'wb') as f:
        f.write(struct.pack('<4sHHIH', b'BPGF', DST_W, DST_H, n, avg_fps))
        for d in durations:
            f.write(struct.pack('<H', d))
        f.write(b'\x00' * pad)
        for fd in frame_data:
            f.write(fd)

    total = data_off + n * DST_W * DST_H * 2
    print(f"  wrote {dst_path}  ({total / 1e6:.1f} MB)")


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} input.gif output.bin")
        sys.exit(1)
    convert(sys.argv[1], sys.argv[2])


if __name__ == '__main__':
    main()