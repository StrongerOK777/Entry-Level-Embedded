#include <Arduino.h>
#include <Wire.h>

// SSD1306 OLED 显示屏配置
#define SSD1306_ADDR 0x3C
#define WIDTH 128
#define HEIGHT 64
#define PAGE_COUNT (HEIGHT/8)
#define FRAME_SIZE (WIDTH * PAGE_COUNT)
#define BAUDRATE 460800 

// --- 音频配置 ---
#define AUDIO_PIN PA8
#define AUDIO_BUF_SIZE 2048 // 增大到 2KB，提供更多抖动缓冲空间
volatile uint8_t audioRingBuf[AUDIO_BUF_SIZE];
volatile uint16_t audioHead = 0;
volatile uint16_t audioTail = 0;

HardwareTimer *TimerPWM;
HardwareTimer *TimerSample;

// 计算缓冲区已用大小
uint16_t audio_buf_used() {
    return (audioHead >= audioTail) ? (audioHead - audioTail) : (AUDIO_BUF_SIZE - audioTail + audioHead);
}

void play_startup_tones() {
    uint16_t notes[] = {523, 659, 784}; // C5, E5, G5
    for (int n = 0; n < 3; n++) {
        uint32_t period_us = 1000000 / notes[n];
        uint32_t startTime = millis();
        while (millis() - startTime < 150) {
            // 针对 5W 大喇叭增强摆幅
            TimerPWM->setCaptureCompare(1, 210, TICK_COMPARE_FORMAT); 
            delayMicroseconds(period_us / 2);
            TimerPWM->setCaptureCompare(1, 46, TICK_COMPARE_FORMAT);
            delayMicroseconds(period_us / 2);
        }
        TimerPWM->setCaptureCompare(1, 128, TICK_COMPARE_FORMAT);
        delay(50);
    }
}


void audio_init() {
    // PA8 是 TIM1_CH1
    TimerPWM = new HardwareTimer(TIM1);
    TimerPWM->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, AUDIO_PIN);
    TimerPWM->setOverflow(255, TICK_FORMAT); // 281kHz carrier
    TimerPWM->setCaptureCompare(1, 128, TICK_COMPARE_FORMAT);
    TimerPWM->resume();

    // 在开启采样定时器前，播放开机测试音
    play_startup_tones();
    
    // 测试音结束后，务必将 PWM 设回 128 (中点)，避免持续电流导致嗡嗡声
    TimerPWM->setCaptureCompare(1, 128, TICK_COMPARE_FORMAT);

    // 定时器 4 用于 8kHz 定时中断
    // 手动显式配置预分频和溢出，确保 8kHz 绝对精确，不受库版本差异影响
    TimerSample = new HardwareTimer(TIM4);
    TimerSample->setPrescaleFactor(72);    // 72MHz / 72 = 1MHz 基频
    TimerSample->setOverflow(125, TICK_FORMAT); // 1MHz / 125 = 8000Hz 采样率
    TimerSample->attachInterrupt([]() {
        uint32_t h = audioHead;
        uint32_t t = audioTail;
        if (t != h) {
            TimerPWM->setCaptureCompare(1, audioRingBuf[t], TICK_COMPARE_FORMAT);
            audioTail = (t + 1) % AUDIO_BUF_SIZE;
        } else {
            // 当缓冲区流空时，不再强行写入 128 (避免载波抖动引起的兹兹声)
            // 保持当前电平或等待新数据
        }
    });
    TimerSample->resume();
}



static uint8_t frameBuf[FRAME_SIZE]; 

// --- 底层驱动函数 ---

void ssd1306_command(uint8_t c) {
    Wire.beginTransmission(SSD1306_ADDR);
    Wire.write(0x00); 
    Wire.write(c);
    Wire.endTransmission();
}

void ssd1306_write_data_chunk(uint8_t* data, size_t len) {
    const size_t CHUNK = 31; // 恢复至 31 字节：这是 STM32 I2C 硬件缓冲区的理想大小，减少中断次数
    for (size_t sent = 0; sent < len; sent += CHUNK) {
        size_t tosend = (len - sent) > CHUNK ? CHUNK : (len - sent);
        Wire.beginTransmission(SSD1306_ADDR);
        Wire.write(0x40); 
        Wire.write(data + sent, tosend);
        Wire.endTransmission();
    }
}

void ssd1306_refresh_page(uint8_t page, uint8_t *data) {
    if (page > 7) return;
    ssd1306_command(0xB0 | page); 
    ssd1306_command(0x00);        
    ssd1306_command(0x10);        
    
    // 提升到 31 字节分片（STM32 Wire 库缓冲区上限）
    // 并使用更高效的批量写入函数
    for (int i = 0; i < 128; i += 31) {
        int len = (128 - i) > 31 ? 31 : (128 - i);
        Wire.beginTransmission(SSD1306_ADDR);
        Wire.write(0x40);
        Wire.write(data + i, len);
        Wire.endTransmission();
    }
}

// 快速全屏刷新 (仅适用于水平寻址模式)
void ssd1306_fast_display(uint8_t *buf) {
    // 设置列地址从0到127，行地址从0到7。设置一次即可，后续只需持续推数据
    ssd1306_command(0x21); ssd1306_command(0); ssd1306_command(127);
    ssd1306_command(0x22); ssd1306_command(0); ssd1306_command(7);
    ssd1306_write_data_chunk(buf, 1024);
}

void ssd1306_init() {
    delay(500); 
    Wire.begin();
    Wire.setClock(400000); // 恢复 400kHz 稳定性，通过 Pipeline 优化来补足速度
    
    uint8_t seq1[] = {
        0xAE,           // Display OFF
        0xD5, 0x80,     // Clock
        0xA8, 0x3F,     // MUX
        0xD3, 0x00,     // Offset
        0x40,           // Start Line
        0x8D, 0x14,     // Charge Pump
        0x20, 0x02,     // 必须是 0x02 (Page Mode)
        0xA1, 0xC8,     // Flip
        0xDA, 0x12,     // COM Pins
        0x81, 0xCF,     // Contrast
        0xD9, 0xF1,     // Pre-charge
        0xDB, 0x40,     // VCOMH
        0xA4, 0xA6, 0xAF // ON
    };
    for(uint8_t c : seq1) ssd1306_command(c);
    delay(100);
}

void setup() {
    Serial.begin(BAUDRATE);
    
    ssd1306_init();
    audio_init();
    
    // 强制执行开机彻底清屏和坐标复位
    memset(frameBuf, 0x00, FRAME_SIZE);
    for(uint8_t p = 0; p < 8; p++) {
        ssd1306_refresh_page(p, frameBuf + p * 128);
    }
    Serial.println("READY"); // 发送就绪信号
}


// 更高效的大块数据读取函数
bool read_bytes_fixed(uint8_t* target, size_t len) {
    if (len == 0) return true;
    size_t got = 0;
    uint32_t startTime = millis();
    while (got < len) {
        if (Serial.available()) {
            target[got++] = Serial.read();
        }
        if (millis() - startTime > 1000) return false; 
    }
    return true;
}

uint32_t read_u32_le() {
    uint8_t b[4];
    if(!read_bytes_fixed(b, 4)) return 0;
    return (uint32_t)(b[0] | (b[1]<<8) | (b[2]<<16) | (b[3]<<24));
}

uint16_t read_u16_le() {
    uint8_t b[2];
    if(!read_bytes_fixed(b, 2)) return 0;
    return (uint16_t)(b[0] | (b[1] << 8));
}

void loop() {
    if (Serial.available() >= 13) {
        int head = Serial.peek();
        if (head == 'V') {
            char magic[4];
            if (!read_bytes_fixed((uint8_t*)magic, 4)) return;
            
            if (magic[0]=='V' && magic[1]=='I' && magic[2]=='D' && magic[3]=='3') {
                uint8_t fps;
                if (!read_bytes_fixed(&fps, 1)) return;
                uint32_t frameCount = read_u32_le();
                uint16_t frameSize = read_u16_le();
                uint16_t audioPerFrame = read_u16_le(); 

                for (uint32_t f = 0; f < frameCount; f++) {
                    // 1. 同步校验：寻找 0xA5 0x5A
                    bool found = false;
                    uint32_t timeout = millis();
                    while(millis() - timeout < 2000) {
                        if(Serial.available() > 0) {
                            if(Serial.read() == 0xA5) {
                                while(Serial.available() == 0 && (millis() - timeout < 2000));
                                if(Serial.peek() == 0x5A) {
                                    Serial.read(); // consume 0x5A
                                    found = true;
                                    break;
                                }
                            }
                        }
                    }
                    if(!found) break;

                    // 2. 接收音频并存入环路缓存
                    {
                        uint8_t sample;
                        for(int i=0; i < audioPerFrame; i++) {
                            while(Serial.available() == 0);
                            sample = Serial.read();
                            uint16_t nextHead = (audioHead + 1) % AUDIO_BUF_SIZE;
                            // 如果缓冲区满了，硬等（会有短暂杂音但能保证同步）
                            while(nextHead == audioTail); 
                            audioRingBuf[audioHead] = sample;
                            audioHead = nextHead;
                        }
                    }

                    // 3. 接收 Mask
                    uint8_t mask;
                    if (!read_bytes_fixed(&mask, 1)) goto frame_err;

                    // 4. 接收变更页到 RAM
                    for (uint8_t p = 0; p < 8; p++) {
                        if (mask & (1 << p)) {
                            if (!read_bytes_fixed(frameBuf + (p * 128), 128)) goto frame_err;
                        }
                    }
                    
                    // 【同步锁机制】
                    // 如果音频缓冲区太满（超过 3/4），说明视频发快了，我们需要等待音频消耗掉一部分
                    // 这样视频会被“拽”到和音频一样的物理播放速度上
                    while(audio_buf_used() > (AUDIO_BUF_SIZE * 3 / 4)) {
                        delayMicroseconds(100); 
                    }

                    Serial.write('F'); 

                    // 5. 将 RAM 刷新到 OLED
                    for (uint8_t p = 0; p < 8; p++) {
                        if (mask & (1 << p)) {
                            ssd1306_refresh_page(p, frameBuf + (p * 128));
                        }
                    }
                    continue;

                frame_err:
                    Serial.write('E'); 
                    break;
                }
            }
        } else {
            Serial.read(); // 吞掉无效字节
        }
    }
}
