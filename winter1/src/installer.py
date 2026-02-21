#!/usr/bin/env python3
"""
installer.py (以前名为 convert_and_send.py)

功能:
  - 将视频抽帧并缩放到 128x64 单色 (1-bit)，按 SSD1306 页面格式打包（1024 字节/帧）
  - 发送协议: header "VID1"(4) | fps(1) | frame_count(4 little) | frame_size(2 little)
  - 然后逐帧通过串口发送每帧 1024 字节，并在每帧等待简单 ACK

依赖:
  pip install Pillow pyserial imageio imageio-ffmpeg
系统依赖:
  ffmpeg （若没有，imageio-ffmpeg 插件通常会自动下载，或用系统包管理器安装）

用法:
  python3 installer.py <serial_port> <video_file> <fps> [--max-frames N]

示例:
  python3 installer.py /dev/cu.usbserial-1420 sample.mp4 6 --max-frames 50
"""

import sys
import os
import time
import argparse
import logging
from typing import List

try:
    import imageio.v2 as imageio
except Exception:
    import imageio

from PIL import Image
import serial

WIDTH = 128
HEIGHT = 64
FRAME_SIZE = WIDTH * (HEIGHT // 8)

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')


def frame_to_ssd1306_bytes(img: Image.Image) -> bytearray:
    pixels = img.load()
    buf = bytearray(FRAME_SIZE)
    for page in range(HEIGHT // 8):
        for x in range(WIDTH):
            byte = 0
            for bit in range(8):
                y = page * 8 + bit
                p = pixels[x, y]
                # 对于 '1' 模式，黑=0 白=255
                if p != 0:
                    byte |= (1 << bit)
            buf[page * WIDTH + x] = byte
    return buf


def extract_frames(video_path: str, fps: int, max_frames: int = 0) -> List[Image.Image]:
    if not os.path.exists(video_path):
        raise FileNotFoundError(f"视频文件未找到: {video_path}")

    try:
        reader = imageio.get_reader(video_path, 'ffmpeg')
    except Exception as e:
        logging.warning("使用 ffmpeg 插件打开视频失败: %s", e)
        logging.info("尝试不指定插件打开视频... 若失败，请安装 ffmpeg 或 imageio-ffmpeg")
        reader = imageio.get_reader(video_path)

    meta = {}
    try:
        meta = reader.get_meta_data()
    except Exception:
        logging.debug("无法获取视频元数据，使用默认 fps=30")

    src_fps = float(meta.get('fps', 30))
    logging.info("源视频 fps: %s", src_fps)

    frames: List[Image.Image] = []
    # 采样步长
    step = max(1, int(round(src_fps / fps))) if src_fps > fps else 1
    logging.info("采样步长: %d，目标 fps: %d", step, fps)

    for i, im in enumerate(reader):
        if i % step == 0:
            pil = Image.fromarray(im).convert('L')
            pil = pil.resize((WIDTH, HEIGHT), Image.BICUBIC)
            # 二值化阈值 128 -> 0/255, 并保留为 '1' 模式
            pil = pil.point(lambda p: 255 if p > 128 else 0, '1')
            frames.append(pil)
            if max_frames and len(frames) >= max_frames:
                break
    try:
        reader.close()
    except Exception:
        pass

    logging.info("抽取帧数: %d", len(frames))
    return frames


def send_frames(port: str, baud: int, frames: List[Image.Image], fps: int):
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except Exception as e:
        raise ConnectionError(f"打开串口失败 {port}: {e}")

    time.sleep(0.5)
    frame_count = len(frames)
    hdr = b'VID1' + bytes([int(fps)]) + frame_count.to_bytes(4, 'little') + FRAME_SIZE.to_bytes(2, 'little')
    logging.info("发送 header: %s", hdr)
    ser.write(hdr)
    time.sleep(0.02)

    for i, f in enumerate(frames):
        buf = frame_to_ssd1306_bytes(f)
        ser.write(buf)

        # 等待简单 ACK：行以 'F' 开头
        start = time.time()
        while time.time() - start < 1.0:
            try:
                if ser.in_waiting:
                    line = ser.readline().decode(errors='ignore').strip()
                    if line.startswith('F'):
                        break
                else:
                    time.sleep(0.005)
            except Exception:
                break

        logging.info("已发送帧 %d / %d", i + 1, frame_count)

    try:
        ser.close()
    except Exception:
        pass

    logging.info("全部发送完成。")


def make_argparser():
    p = argparse.ArgumentParser(description='Convert video to SSD1306 frames and send over serial')
    p.add_argument('serial_port', help='serial port (e.g. /dev/ttyUSB0 or COM3)')
    p.add_argument('video_file', help='input video file')
    p.add_argument('fps', type=int, help='target fps to extract')
    p.add_argument('--max-frames', type=int, default=0, help='maximum frames to extract (0 = all)')
    p.add_argument('--baud', type=int, default=115200, help='serial baudrate')
    return p


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    args = make_argparser().parse_args(argv)

    if args.fps <= 0:
        logging.error('fps 必须大于 0')
        return 2

    try:
        frames = extract_frames(args.video_file, args.fps, args.max_frames)
    except Exception as e:
        logging.error('抽帧失败: %s', e)
        return 3

    if not frames:
        logging.error('未提取到任何帧，检查输入文件/参数')
        return 4

    try:
        send_frames(args.serial_port, args.baud, frames, args.fps)
    except Exception as e:
        logging.error('发送失败: %s', e)
        return 5

    return 0


if __name__ == '__main__':
    sys.exit(main())