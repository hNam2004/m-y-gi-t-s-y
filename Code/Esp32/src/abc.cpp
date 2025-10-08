#include <Arduino.h>

// --- Config phần cứng (chỉnh theo board của bạn) ---
#define RS485_TX 17 // TX của ESP32 -> tới DI của MAX485 (hoặc RX của device nếu TTL)
#define RS485_RX 16 // RX của ESP32 <- RO của MAX485 (hoặc TX của device nếu TTL)
#define RS485_DE -1 // Driver enable (HIGH = transmit), chỉnh nếu bạn dùng RS485; nếu TTL để -1
#define BAUDRATE 9600
#define SLAVE_ID 0x01

HardwareSerial RS485Serial(2);

// --- CRC16 Modbus ---
uint16_t modbus_crc16(const uint8_t *buf, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; pos++)
  {
    crc ^= (uint16_t)buf[pos];
    for (int i = 0; i < 8; i++)
    {
      if (crc & 0x0001)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
        crc >>= 1;
    }
  }
  return crc;
}

// --- RS485 helper ---
void rs485_tx_enable()
{
  if (RS485_DE >= 0)
    digitalWrite(RS485_DE, HIGH);
}
void rs485_rx_enable()
{
  if (RS485_DE >= 0)
    digitalWrite(RS485_DE, LOW);
}

// tính trễ tối thiểu để dữ liệu shift hết theo baud:
// số bit ~ (len bytes * 11 bits per byte (start+8+parity+stop))
// ms = bits/baud * 1000
uint32_t tx_delay_ms_for_len(size_t len)
{
  unsigned long bits = (unsigned long)len * 11UL;
  unsigned long ms = (bits * 1000UL) / BAUDRATE + 2;
  return (uint32_t)ms;
}

// --- Gửi frame raw, tuỳ chọn chờ phản hồi ---
bool send_frame_and_maybe_read(const uint8_t *frame, size_t flen,
                               uint8_t *respBuf, size_t respMaxLen,
                               size_t *respLen, unsigned long timeout_ms,
                               bool expectResponse)
{
  // drive RS485 DE
  rs485_tx_enable();
  RS485Serial.write(frame, flen);
  RS485Serial.flush();              // đợi hết buffer -> NOTE: trên ESP32 flush nên chờ tiếp
  delay(tx_delay_ms_for_len(flen)); // đảm bảo shift hết trên dây
  rs485_rx_enable();

  if (!expectResponse)
  {
    if (respLen)
      *respLen = 0;
    return true;
  }

  // đọc response (non-blocking loop)
  unsigned long start = millis();
  size_t idx = 0;
  while (millis() - start < timeout_ms && idx < respMaxLen)
  {
    while (RS485Serial.available() && idx < respMaxLen)
    {
      respBuf[idx++] = RS485Serial.read();
    }
  }
  if (respLen)
    *respLen = idx;
  return (idx > 0);
}

// --- Build & send Modbus Write Single Register style via Function 0x10 (Write Multiple Registers with count=1) ---
bool send_write_register(uint8_t slave, uint16_t regAddr, uint16_t value,
                         uint8_t *respBuf = nullptr, size_t *respLen = nullptr, unsigned long timeout_ms = 200)
{
  uint8_t frame[11];
  frame[0] = slave;
  frame[1] = 0x10; // function 16
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] = regAddr & 0xFF;
  frame[4] = 0x00;
  frame[5] = 0x01; // quantity 1 register
  frame[6] = 0x02; // byte count
  frame[7] = (value >> 8) & 0xFF;
  frame[8] = value & 0xFF;
  uint16_t crc = modbus_crc16(frame, 9);
  frame[9] = crc & 0xFF;         // CRC low
  frame[10] = (crc >> 8) & 0xFF; // CRC high

  return send_frame_and_maybe_read(frame, sizeof(frame), respBuf, 256, respLen, timeout_ms, false);
}

// --- Generic read holding registers (func 0x03) ---
bool send_read_holding(uint8_t slave, uint16_t regAddr, uint16_t qty,
                       uint8_t *respBuf, size_t *respLen, unsigned long timeout_ms = 300)
{
  uint8_t frame[8];
  frame[0] = slave;
  frame[1] = 0x03;
  frame[2] = (regAddr >> 8) & 0xFF;
  frame[3] = regAddr & 0xFF;
  frame[4] = (qty >> 8) & 0xFF;
  frame[5] = qty & 0xFF;
  uint16_t crc = modbus_crc16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;
  return send_frame_and_maybe_read(frame, sizeof(frame), respBuf, 256, respLen, timeout_ms, true);
}

// --- Wrapper cho lệnh bạn liệt kê ---
// Lưu ý: regAddr và value theo bảng bạn đã cung cấp (ví dụ 0x012E cho Start, 0x0131 cho coin simulate)
bool cmd_start_machine()
{
  return send_write_register(SLAVE_ID, 0x012E, 0x0001);
}
bool cmd_next_step()
{
  return send_write_register(SLAVE_ID, 0x012F, 0x0001);
}
bool cmd_mute_alarm()
{
  return send_write_register(SLAVE_ID, 0x012C, 0x0001);
}
bool cmd_simulate_coins(uint16_t coins)
{
  // coins: 1/2/3
  return send_write_register(SLAVE_ID, 0x0131, coins & 0xFFFF);
}
bool cmd_select_program(uint16_t programNo)
{
  // programNo: 1..6 (but in your frames value stored might be program mapping; adjust if needed)
  return send_write_register(SLAVE_ID, 0x0132, programNo & 0xFFFF);
}
bool cmd_read_status(uint8_t *resp, size_t *rlen)
{
  // example frame you gave used Function 0x01 reading coils from 0x0000 qty 0x00A0
  // we'll implement a generic read-holding-version (func 0x03) too if needed
  return send_frame_and_maybe_read((uint8_t[]){SLAVE_ID, 0x01, 0x00, 0x00, 0x00, 0xA0, 0x3C, 0x72},
                                   8, resp, 256, rlen, 500, true);
}

// --- Coin counting & main flow ---
volatile int coinCount = 0;
volatile unsigned long lastInterrupt = 0;

void IRAM_ATTR coinISR()
{
  unsigned long t = millis();
  if (t - lastInterrupt > 40)
  { // 40 ms debounce (tinh chỉnh)
    coinCount++;
    lastInterrupt = t;
  }
}

void setup()
{
  Serial.begin(115200);
  RS485Serial.begin(BAUDRATE, SERIAL_8N1, RS485_RX, RS485_TX);

  if (RS485_DE >= 0)
  {
    pinMode(RS485_DE, OUTPUT);
    rs485_rx_enable();
  }

  // coin inputs (theo schematic IN_SIG1->GPIO19, IN_SIG2->GPIO18). Bạn quyết 1 hay 2
  pinMode(18, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), coinISR, FALLING);

  pinMode(19, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(19), coinISR, FALLING);

  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);

  Serial.println("System ready");
}

void loop()
{
  // atomically copy coinCount
  noInterrupts();
  int c = coinCount;
  interrupts();

  if (c >= 3)
  {
    Serial.println("Đủ 3 xu: kích hoạt chức năng CHẠY + gửi lệnh Start");
    // 1) gửi lệnh Start (không có phản hồi theo mô tả)
    if (cmd_start_machine())
    {
      Serial.println("Lệnh Start gửi xong");
    }
    else
    {
      Serial.println("Lỗi gửi lệnh Start");
    }
    // 2) bật các chân control (nếu cần)
    digitalWrite(25, HIGH);
    digitalWrite(26, HIGH);
    delay(5000);
    digitalWrite(25, LOW);
    digitalWrite(26, LOW);

    // reset counter atomically
    noInterrupts();
    coinCount = 0;
    interrupts();
  }

  // you can poll status periodically, e.g. every 2s
  static unsigned long lastPoll = 0;
  if (millis() - lastPoll > 2000)
  {
    lastPoll = millis();
    uint8_t resp[256];
    size_t rlen = 0;
    if (cmd_read_status(resp, &rlen))
    {
      Serial.printf("Status read %u bytes\n", (unsigned)rlen);
      for (size_t i = 0; i < rlen; i++)
        Serial.printf("%02X ", resp[i]);
      Serial.println();
      // TODO: parse the bytes according to machine's format you provide
    }
  }

  delay(10);
}
