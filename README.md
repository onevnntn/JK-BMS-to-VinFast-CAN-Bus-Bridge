# JK-BMS to VinFast Klara S1 CAN Bus Bridge

Mã nguồn và hướng dẫn kết nối **JK-BMS** sang hệ thống **CAN Bus của xe máy điện VinFast Klara S1** sử dụng vi điều khiển **STM32F103C8T6 (Blue Pill)** và module **MCP2515 CAN SPI**.

Dự án này giúp thay thế pin nguyên bản của VinFast Klara S1 bằng khối pin độ dùng mạch **JK-BMS**, cho phép xe nhận diện đầy đủ thông số (SOC, Điện áp, Dòng điện, Nhiệt độ, Điện áp từng Cell) và **hỗ trợ cắm sạc bình thường**.

---

## 🛠️ 1. Sơ đồ đấu nối phần cứng (Hardware Wiring)

### A. Kết nối STM32F103 (Blue Pill) <-> Module MCP2515 (CAN SPI)
| Chân MCP2515 | Chân STM32F103 | Ghi chú |
| :--- | :--- | :--- |
| **VCC** | **5V** | Nguồn 5V cho MCP2515 |
| **GND** | **GND** | Nối đất chung |
| **CS** | **PA4** | Chip Select SPI |
| **SCK** | **PA5** | SPI1 Clock |
| **SO (MISO)** | **PA6** | SPI1 MISO |
| **SI (MOSI)** | **PA7** | SPI1 MOSI |
| **INT** | Không nối (hoặc PA1) | Tùy chọn ngắt |

> **Mạng CAN Xe:** Nối `CAN_H` và `CAN_L` của MCP2515 vào cổng CAN Bus trên xe Klara S1.

### B. Kết nối STM32F103 <-> JK-BMS (UART Serial1)
Sử dụng mạch chuyển đổi mức tín hiệu (MAX3222 / TTL-RS485/UART) nối từ cổng UART của JK-BMS sang STM32:

| Chân JK-BMS Adapter | Chân STM32F103 | Ghi chú |
| :--- | :--- | :--- |
| **TX** | **PA10 (RX1)** | Nhận dữ liệu UART từ JK-BMS |
| **RX** | **PA9 (TX1)** | Gửi dữ liệu UART sang JK-BMS |
| **GND** | **GND** | Mass chung |

---

## 📑 2. Danh sách CAN ID Klara S1 (Reverse Engineered)

| CAN ID | Chức năng | Chu kỳ / Ghi chú |
| :--- | :--- | :--- |
| **`0x303`** | BMS Heartbeat & Relay status | 100ms (Rolling Counter Byte 0) |
| **`0x309`** | Điện áp tổng, Dòng xả/sạc, Trạng thái sạc | Cập nhật theo dữ liệu BMS |
| **`0x30E`** | Nhiệt độ cảm biến (3 cảm biến) | Cập nhật theo dữ liệu BMS |
| **`0x310` - `0x314`** | Điện áp chi tiết Cell 1 đến Cell 20 | Chia làm 4 block CAN |
| **`0x326`** | Gateway Heartbeat | 100ms |
| **`0x340`, `0x342`, `0x343`, `0x33F`** | Khung truyền tín hiệu nhận sạc (Charging Status) | Kích hoạt khi dòng điện > 0A |

---

## 💻 3. Mã nguồn Arduino (STM32F1)

```cpp
/*
 * JK_BMS_to_Klara_CAN.ino
 * STM32F1 (Blue Pill) - UART read JK-BMS (MAX3222) → CAN (MCP2515)
 * Gửi CAN frame cho VinFast Klara S1 theo log thực tế, bao gồm tín hiệu sạc
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// --- UART cho JK-BMS ---
#define JK_SERIAL Serial1
#define JK_BAUD 115200

// --- CAN SPI ---
#define CAN_CS PA4
MCP2515 mcp2515(CAN_CS);

// --- CAN IDs của Klara S1 (từ log) ---
#define CAN_ID_303  0x303   // BMS Heartbeat & Relay
#define CAN_ID_309  0x309   // SOC, Voltage, Current
#define CAN_ID_30E  0x30E   // Nhiệt độ
#define CAN_ID_310  0x310   // Cell 1-4
#define CAN_ID_311  0x311   // Cell 5-8
#define CAN_ID_312  0x312   // Cell 9-12
#define CAN_ID_313  0x313   // Cell 13-16
#define CAN_ID_314  0x314   // Cell 17-20
#define CAN_ID_326  0x326   // Gateway Heartbeat

// --- CAN IDs liên quan đến sạc ---
#define CAN_ID_340  0x340
#define CAN_ID_342  0x342
#define CAN_ID_343  0x343
#define CAN_ID_33F  0x33F

// --- Biến lưu dữ liệu từ JK-BMS ---
uint8_t  jk_soc = 0;
uint16_t jk_pack_voltage_10mV = 0;  // đơn vị 10mV (từ JK-BMS)
int16_t  jk_current_10mA = 0;       // mA, dương = sạc, âm = xả
int16_t  jk_temp[3] = {0,0,0};      // °C * 10
uint16_t jk_cell_mV[16] = {0};      // mV

// Rolling counter cho 0x303
uint8_t roll_counter = 0;

// --- Hàm parse JK-BMS frame (dựa trên cấu trúc thực tế) ---
void parseJKReply(uint8_t *buf, uint16_t len) {
    if (len < 10 || buf[0] != 0x4E || buf[1] != 0x57) return;

    // Tìm token 0x80
    uint8_t *p = buf;
    uint8_t *end = buf + len - 10;
    while (p < end) {
        if (*p == 0x80) break;
        p++;
    }
    if (p >= end) p = buf + 61; // fallback

    uint8_t *ptr = p + 1;
    jk_temp[0] = (int16_t)((ptr[0] << 8) | ptr[1]); ptr += 2;
    ptr++;
    jk_temp[1] = (int16_t)((ptr[0] << 8) | ptr[1]); ptr += 2;
    ptr++;
    jk_temp[2] = (int16_t)((ptr[0] << 8) | ptr[1]); ptr += 2;
    ptr++;
    jk_pack_voltage_10mV = (ptr[0] << 8) | ptr[1]; ptr += 2;
    ptr++;
    uint16_t raw_current = (ptr[0] << 8) | ptr[1]; ptr += 2;
    if (raw_current & 0x8000) {
        jk_current_10mA = raw_current & 0x7FFF;
    } else {
        jk_current_10mA = -(int16_t)(raw_current & 0x7FFF);
    }
    ptr++;
    jk_soc = *ptr;

    // Đọc cell
    uint8_t *cell_ptr = buf + 11;
    if (*cell_ptr == 0x79 && cell_ptr[1] == 0x30) {
        cell_ptr += 2;
        uint8_t num_cells = buf[0xA9];
        if (num_cells > 16) num_cells = 16;
        for (int i = 0; i < num_cells; i++) {
            uint16_t raw = (cell_ptr[i*3 + 1] << 8) | cell_ptr[i*3 + 2];
            jk_cell_mV[i] = raw / 10;
        }
    }
}

// --- Hàm gửi các frame CAN liên quan đến sạc ---
void sendChargingFrames() {
    struct can_frame frame;

    // Xác định trạng thái sạc: dòng > 0 => đang sạc
    bool isCharging = (jk_current_10mA > 0);

    // 1. Frame 0x340 - Yêu cầu dòng sạc (gửi liên tục)
    frame.can_id = CAN_ID_340;
    frame.can_dlc = 8;
    frame.data[0] = 0x73;
    frame.data[1] = 0x4D;
    frame.data[2] = 0x43;
    frame.data[3] = 0x5F;
    frame.data[4] = 0x00;
    frame.data[5] = 0xCF;
    frame.data[6] = 0x00;
    frame.data[7] = 0x05;
    mcp2515.sendMessage(&frame);
    delayMicroseconds(200);

    // 2. Frame 0x342 - Dòng/áp sạc thực tế
    frame.can_id = CAN_ID_342;
    frame.can_dlc = 8;
    if (isCharging) {
        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        uint16_t currentRaw = (uint16_t)jk_current_10mA;
        frame.data[2] = currentRaw >> 8;
        frame.data[3] = currentRaw & 0xFF;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        uint16_t voltageRaw = jk_pack_voltage_10mV * 10;
        frame.data[6] = voltageRaw >> 8;
        frame.data[7] = voltageRaw & 0xFF;
    } else {
        frame.data[0] = 0xFF;
        frame.data[1] = 0xFF;
        frame.data[2] = 0xD6;
        frame.data[3] = 0x28;
        frame.data[4] = 0x00;
        frame.data[5] = 0x00;
        frame.data[6] = 0xB7;
        frame.data[7] = 0xC0;
    }
    mcp2515.sendMessage(&frame);
    delayMicroseconds(200);

    // 3. Frame 0x343 - Dữ liệu bổ sung
    frame.can_id = CAN_ID_343;
    frame.can_dlc = 8;
    frame.data[0] = 0xE1;
    frame.data[1] = 0xA1;
    frame.data[2] = 0xF0;
    frame.data[3] = 0xC4;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    mcp2515.sendMessage(&frame);
    delayMicroseconds(200);

    // 4. Frame 0x33F - Trạng thái sạc
    frame.can_id = CAN_ID_33F;
    frame.can_dlc = 8;
    frame.data[0] = 0x12;
    frame.data[1] = 0x11;
    frame.data[2] = isCharging ? 0x01 : 0x00; // 0x01 = Đang sạc
    frame.data[3] = 0x51;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x04;
    frame.data[7] = 0x57;
    mcp2515.sendMessage(&frame);
}

// --- Hàm gửi các frame CAN cơ bản (BMS) ---
void sendCANFrames() {
    struct can_frame frame;

    // 0x309 – SOC, Voltage, Current
    frame.can_id = CAN_ID_309;
    frame.can_dlc = 8;
    frame.data[0] = 0x11;
    frame.data[1] = 0x00;
    uint16_t vraw = (uint32_t)jk_pack_voltage_10mV * 480 / 100;
    frame.data[2] = vraw >> 8;
    frame.data[3] = vraw & 0xFF;
    frame.data[4] = 0x00;
    frame.data[5] = (jk_current_10mA > 0) ? 0x01 : 0x00;
    int16_t iraw = jk_current_10mA;
    frame.data[6] = (iraw >> 8) & 0xFF;
    frame.data[7] = iraw & 0xFF;
    mcp2515.sendMessage(&frame);
    delayMicroseconds(200);

    // 0x30E – Nhiệt độ
    frame.can_id = CAN_ID_30E;
    frame.can_dlc = 8;
    for (int i = 0; i < 3; i++) {
        int16_t t = jk_temp[i];
        frame.data[i*2]   = (t >> 8) & 0xFF;
        frame.data[i*2+1] = t & 0xFF;
    }
    frame.data[6] = 0;
    frame.data[7] = 0;
    mcp2515.sendMessage(&frame);
    delayMicroseconds(200);

    // 0x310 – 0x314: Cell Voltages
    for (int block = 0; block < 4; block++) {
        int base = block * 4;
        if (base >= 16) break;
        frame.can_id = 0x310 + block;
        frame.can_dlc = 8;
        for (int j = 0; j < 4; j++) {
            uint16_t cell = jk_cell_mV[base + j] * 10;
            frame.data[j*2]   = cell >> 8;
            frame.data[j*2+1] = cell & 0xFF;
        }
        mcp2515.sendMessage(&frame);
        delayMicroseconds(200);
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    JK_SERIAL.begin(JK_BAUD);

    SPI.begin();
    mcp2515.reset();
    if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) == MCP2515::ERROR_OK) {
        Serial.println("CAN init OK");
    } else {
        Serial.println("CAN init FAIL");
    }
    mcp2515.setNormalMode();

    Serial.println("JK-BMS to Klara CAN Bridge started (with charging support)");
}

// --- Loop ---
void loop() {
    static uint8_t rxBuf[350];
    static uint16_t idx = 0;

    while (JK_SERIAL.available()) {
        uint8_t c = JK_SERIAL.read();
        if (idx == 0 && c != 0x4E) continue;
        if (idx == 1 && c != 0x57) { idx = 0; continue; }
        rxBuf[idx++] = c;
        if (idx >= 4) {
            uint16_t frameLen = (rxBuf[2] << 8) | rxBuf[3];
            if (frameLen > 0 && idx >= frameLen + 2) {
                parseJKReply(rxBuf, idx);
                sendCANFrames();
                sendChargingFrames();
                idx = 0;
            }
        }
        if (idx >= sizeof(rxBuf)) idx = 0;
    }

    // Heartbeat (0x303 và 0x326) mỗi 100ms
    static uint32_t lastHeartbeat = 0;
    if (millis() - lastHeartbeat >= 100) {
        lastHeartbeat = millis();

        struct can_frame hb303;
        hb303.can_id = CAN_ID_303;
        hb303.can_dlc = 8;
        hb303.data[0] = roll_counter++;
        hb303.data[1] = 0x30;
        hb303.data[2] = 0x00;
        hb303.data[3] = 0x00;
        hb303.data[4] = 0x01;
        hb303.data[5] = 0x01;
        hb303.data[6] = 0x06;
        hb303.data[7] = 0x00;
        mcp2515.sendMessage(&hb303);
        delayMicroseconds(200);

        struct can_frame hb326;
        hb326.can_id = CAN_ID_326;
        hb326.can_dlc = 8;
        hb326.data[0] = 0x00;
        hb326.data[1] = 0x00;
        hb326.data[2] = 0x00;
        hb326.data[3] = 0x00;
        hb326.data[4] = 0x01;
        hb326.data[5] = 0x00;
        hb326.data[6] = 0x00;
        hb326.data[7] = 0x00;
        mcp2515.sendMessage(&hb326);
    }

    delay(5);
}
```

---

## ☕ Ủng hộ tác giả (Donate)

Nếu dự án này giúp ích cho bạn trong việc đóng pin và giải mã thành công xe VinFast Klara S1, bạn có thể ủng hộ mình một ly cà phê qua tài khoản bên dưới:

* **Chủ tài khoản:** TRAN DUY THO
* **Số tài khoản:** `3013 2838 69`
* **Ngân hàng:** Techcombank

![Mã QR Donate Techcombank](./qr_donate.png)
Dự án phục vụ mục đích nghiên cứu học thuật và tham khảo. Tác giả không chịu trách nhiệm đối với bất kỳ rủi ro, hư hỏng thiết bị hoặc mất an toàn giao thông nào phát sinh khi người dùng áp dụng thực tế
