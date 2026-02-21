import sys, time, subprocess
import imageio.v2 as imageio
from PIL import Image, ImageEnhance, ImageOps
import serial
import numpy as np
import os
import argparse

WIDTH = 128
HEIGHT = 64
FRAME_SIZE = WIDTH * (HEIGHT // 8)
SAMPLE_RATE = 8000 # 8kHz 采样率

def frame_to_ssd1306_bytes(img):
    # 使用 numpy 向量化操作
    arr = np.array(img.convert('L'))
    arr = (arr > 127).astype(np.uint8)
    arr = arr.reshape(8, 8, 128)
    weights = np.array([1, 2, 4, 8, 16, 32, 64, 128], dtype=np.uint8).reshape(1, 8, 1)
    packed = (arr * weights).sum(axis=1).astype(np.uint8) 
    return packed.tobytes()

def extract_audio_ffmpeg(video_path, sample_rate=8000):
    """
    针对 5W/4Ω 高素质喇叭调优
    """
    print(f"Extracting audio (Optimized for 5W/4Ohm Speaker)...")
    
    # 释放低音：lowpass 提高到 4kHz，增加 300Hz 处的中音厚度
    filter_chain = "lowpass=f=4000,anequalizer=c0 f=300 w=100 g=3,volume=0.8"
    
    cmd = [
        'ffmpeg', '-i', video_path,
        '-vn', '-ac', '1', '-ar', str(sample_rate),
        '-acodec', 'pcm_u8',
        '-af', filter_chain,
        '-f', 'u8', 'pipe:1'
    ]

    try:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        audio_data, _ = process.communicate()
        if not audio_data:
            return None
        return np.frombuffer(audio_data, dtype=np.uint8)
    except Exception as e:
        print(f"FFmpeg audio extraction failed: {e}")
        return None

def stream_and_send(port, baud, video_path, fps, max_frames=0, version='VID3'):
    if not os.path.exists(video_path):
        print(f"Error: Video file {video_path} not found.")
        return

    print(f"Opening video {video_path}...")
    
    # 使用新方法提取音频
    audio_samples = extract_audio_ffmpeg(video_path, SAMPLE_RATE)
    if audio_samples is not None:
        print(f"Audio extraction done. {len(audio_samples)} samples.")
    else:
        print("Continuing without audio...")
    
    ser = serial.Serial(port, baud, timeout=0.1)
    
    print("Waiting for STM32 READY signal (2s timeout)...")
    start_wait = time.time()
    while time.time() - start_wait < 2.0:
        line = ser.readline().decode('ascii', errors='ignore').strip()
        if "READY" in line:
            print("STM32 is READY!")
            break
    
    ser.timeout = 2 # 恢复长超时
    
    reader = imageio.get_reader(video_path, 'ffmpeg')
    meta = reader.get_meta_data()
    src_fps = meta.get('fps', 30)
    step = max(1, int(round(src_fps / fps))) if src_fps > fps else 1

    print("Skipping frame count for speed...")
    count = 9999
    audio_per_frame = SAMPLE_RATE // fps
    
    reader = imageio.get_reader(video_path, 'ffmpeg')
    # 发送 4 字节 Magic
    ser.write(version.encode()) 
    # 发送协议头
    ser.write(bytes([int(fps)]))
    ser.write((count).to_bytes(4, 'little'))
    ser.write((FRAME_SIZE).to_bytes(2, 'little'))
    ser.write((audio_per_frame).to_bytes(2, 'little')) # 新增
    time.sleep(0.05)

    start_time = time.time()
    idx = 0
    audio_per_frame = SAMPLE_RATE // fps
    last_frame_data = None

    print("Starting stream...")
    for i, im in enumerate(reader):
        if i % step != 0: continue
        if max_frames and idx >= max_frames: break

        pil = Image.fromarray(im).convert('L')
        pil = ImageOps.pad(pil, (WIDTH, HEIGHT), method=Image.Resampling.LANCZOS, color=0)
        enhancer = ImageEnhance.Contrast(pil)
        pil = enhancer.enhance(2.0) 
        
        final_frame = pil.point(lambda p: 255 if p > 127 else 0, '1')
        buf = frame_to_ssd1306_bytes(final_frame)
        
        payload = bytearray()
        payload.extend([0xA5, 0x5A]) # 强化版双字节同步

        if version == 'VID3':
            # 发送音频数据
            if audio_samples is not None:
                # 改进同步：使用浮点计算当前帧应对应的精确音频位置，防止累积偏差
                asrt = int(idx * SAMPLE_RATE / fps)
                aend = int((idx + 1) * SAMPLE_RATE / fps)
                a_chunk = audio_samples[asrt:aend]
                
                # 检查 STM32 端预期的样点数 (audio_per_frame) 并补齐，保持协议对齐
                if len(a_chunk) < audio_per_frame:
                    a_chunk = np.pad(a_chunk, (0, audio_per_frame - len(a_chunk)), constant_values=128)
                elif len(a_chunk) > audio_per_frame:
                    a_chunk = a_chunk[:audio_per_frame]
                
                payload.extend(a_chunk.tobytes())
            else:
                payload.extend(bytes([128] * audio_per_frame))

            # 差分视频


            mask = 0
            diff_pages = []
            force_full = (idx % 50 == 0) 
            for p in range(8):
                s = p * 128
                e = s + 128
                curr_page = buf[s:e]
                if force_full or last_frame_data is None or curr_page != last_frame_data[s:e]:
                    mask |= (1 << p)
                    diff_pages.append(curr_page)
            payload.append(mask)
            for p_data in diff_pages:
                payload.extend(p_data)
        else:
            payload.extend(buf)

        ser.write(payload)
        ser.flush()
        last_frame_data = buf

        # 等待反馈
        ack_start = time.time()
        got_ack = False
        while time.time() - ack_start < 1.0:
            if ser.in_waiting:
                char = ser.read(1)
                if char == b'F':
                    got_ack = True
                    break
                elif char == b'E':
                    print(f"\nFrame {idx}: Error from STM32")
                    break
            time.sleep(0.001)

        idx += 1
        if idx % 10 == 0:
            elapsed = time.time() - start_time
            cur_fps = idx / elapsed
            print(f"Sent {idx}/{count} | Sync: OK | FPS: {cur_fps:.1f}")



    reader.close()
    ser.close()
    print(f"Done. Total time: {time.time() - start_time:.2f}s")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='OLED Streaming Tool')
    parser.add_argument('--port', required=True, help='Serial port (e.g. /dev/cu.usbserial-10)')
    parser.add_argument('--video', required=True, help='Path to video file')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second (default: 30)')
    parser.add_argument('--max-frames', type=int, default=0, help='Max frames to send (0 for unlimited)')
    parser.add_argument('--version', default='VID3', choices=['VID1', 'VID2', 'VID3'], help='Protocol version')
    parser.add_argument('--baud', type=int, default=460800, help='Serial baud rate (default: 460800)')

    args = parser.parse_args()
    
    stream_and_send(
        args.port, 
        args.baud, 
        args.video, 
        args.fps, 
        args.max_frames, 
        version=args.version
    )
