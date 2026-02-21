#!/usr/bin/env python3
"""
convert_and_send.py
依赖:
  pip install pillow pyserial imageio imageio-ffmpeg

用法:
  python3 convert_and_send.py <serial_port> <video_file> <fps> [max_frames]
例:
  python3 convert_and_send.py /dev/cu.usbserial-1420 sample.mp4 6 50

TEST:
    python3 convert_and_send.py /dev/cu.usbserial-1420 BAD_APPLE.mp4 6 50
说明:
 - 将视频抽帧并缩放到 128x64 单色 (1-bit)，按 SSD1306 页格式打包（1024 字节/帧）
 - 发送协议: header "VID1"(4) | fps(1) | frame_count(4 little) | frame_size(2 little)
 - 然后逐帧发送每帧 1024 字节
"""
import sys, time
import imageio.v2 as imageio
from PIL import Image
import serial

WIDTH = 128
HEIGHT = 64
FRAME_SIZE = WIDTH * (HEIGHT // 8)

def frame_to_ssd1306_bytes(img):
    pixels = img.load()
    buf = bytearray(FRAME_SIZE)
    for page in range(HEIGHT // 8):
        for x in range(WIDTH):
            byte = 0
            for bit in range(8):
                y = page * 8 + bit
                p = pixels[x, y]
                if p != 0:
                    byte |= (1 << bit)
            buf[page * WIDTH + x] = byte
    return buf

def extract_frames(video_path, fps, max_frames=0):
    reader = imageio.get_reader(video_path, 'ffmpeg')
    meta = reader.get_meta_data()
    src_fps = meta.get('fps', 30)
    print("Source fps:", src_fps)
    frames = []
    # sample step
    step = max(1, int(round(src_fps / fps))) if src_fps > fps else 1
    for i, im in enumerate(reader):
        if i % step == 0:
            pil = Image.fromarray(im).convert('L')
            pil = pil.resize((WIDTH, HEIGHT), Image.BICUBIC)
            pil = pil.point(lambda p: 255 if p > 128 else 0, '1')
            frames.append(pil)
            if max_frames and len(frames) >= max_frames:
                break
    reader.close()
    print("Extracted frames:", len(frames))
    return frames

def send_frames(port, baud, frames, fps):
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(1)
    frame_count = len(frames)
    hdr = b'VID1' + bytes([int(fps)]) + frame_count.to_bytes(4,'little') + FRAME_SIZE.to_bytes(2,'little')
    print("Sending header:", hdr)
    ser.write(hdr)
    time.sleep(0.02)
    for i, f in enumerate(frames):
        buf = frame_to_ssd1306_bytes(f)
        ser.write(buf)
        # wait short ack (line starting with 'F')
        start = time.time()
        while time.time() - start < 1.0:
            if ser.in_waiting:
                line = ser.readline().decode(errors='ignore').strip()
                if line.startswith('F'):
                    break
            else:
                time.sleep(0.005)
        print("Sent frame", i+1, "/", frame_count)
    ser.close()
    print("Done.")

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: convert_and_send.py <serial_port> <video_file> <fps> [max_frames]")
        sys.exit(1)
    port = sys.argv[1]
    vfile = sys.argv[2]
    fps = int(sys.argv[3])
    maxf = int(sys.argv[4]) if len(sys.argv) >=5 else 0
    frames = extract_frames(vfile, fps, maxf)
    send_frames(port, 115200, frames, fps)