#include <Arduino.h>
#include <ModbusRTU.h>
#include <Modbus.h>
#include <ArduinoJson.h>

ModbusRTU mb;
#define RXD2 16
#define TXD2 17
#define RE_DE 4

struct commandMap {
  const char* name;   // "t" trong JSON
  uint8_t slave;      // địa chỉ node
  uint8_t func;       // function code (6,16,3,1…)
  uint16_t addr;      // địa chỉ thanh ghi
  uint16_t numregis;  // số lượng thanh ghi
  uint16_t numofbytes; // số byte dữ liệu
  uint16_t data;    // giá trị ghi vào thanh ghi
};

commandMap* currentCmd = nullptr;

commandMap commands[] = {
  {.name="c", .slave=1, .func=16, .addr=0x012E, .numregis=1, .numofbytes=2, .data=0x0000},
  {.name="p", .slave=1, .func=16, .addr=0x0132, .numregis=1, .numofbytes=2, .data=0x0000},
  {.name="m", .slave=1, .func=16, .addr=0x0131, .numregis=1, .numofbytes=2, .data=0x0000},
  {.name="n1", .slave=1, .func=16, .addr=0x012F, .numregis=1, .numofbytes=2, .data=0x0000},
  {.name="n2", .slave=1, .func=16, .addr=0x012C, .numregis=1, .numofbytes=2, .data=0x0000},
  {.name="i", .slave=1, .func=16, .addr=0x0133, .numregis=1, .numofbytes=2, .data=0x0000}
};

// --- Hàm in frame hex ---
void printHexFrame(const uint8_t* buf, size_t len, const char* tag) {
  Serial.printf("[%s] ", tag);
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();
}

// --- Callback truyền/nhận raw frame ---
void onTx(const uint8_t* buf, uint16_t len) {
  printHexFrame(buf, len, "TX");
}
void onRx(const uint8_t* buf, uint16_t len) {
  printHexFrame(buf, len, "RX");
}


// --- Callback Modbus ---
bool cbModbusTransaction(Modbus::ResultCode event, uint16_t transactionId, void* data) {
    commandMap* cmd = currentCmd;
    StaticJsonDocument<128> resp;

   if (event == Modbus::EX_SUCCESS) {
    resp["status"] = "ok";
    resp["t"] = cmd->name;
  

    // Nếu là đọc thì lấy giá trị đọc được
    if (cmd->func == 3) {
      uint16_t val;
      mb.readHreg(cmd->slave, cmd->addr, &val, 1);
      char buf[10];
      sprintf(buf, "0x%04X", val);
      resp["v"] = buf;
    } else {
      char buf[10];
      sprintf(buf, "0x%04X", cmd->data);
      resp["v"] = buf;
    }
  } else {
    resp["status"] = "error";
    resp["t"] = cmd->name;
    char buf[10];
    sprintf(buf, "0x%02X", event);
    resp["code"] = buf;
  }

  serializeJson(resp, Serial);
  Serial.println();
  return true;
}

// Hàm tính CRC16 Modbus
uint16_t modbusCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte vào CRC
    for (int i = 8; i != 0; i--) {      // Lặp 8 lần
      if ((crc & 0x0001) != 0) {        // Nếu bit LSB = 1
        crc >>= 1;
        crc ^= 0xA001;
      } else
        crc >>= 1;
    }
  }
  return crc;
}


bool sendCommand(const char* name, uint16_t value = 0) {
  uint8_t frame[16];  // tối đa frame dài 16 byte cho func=16
  uint8_t frameLen = 0;
  for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
    if (strcmp(commands[i].name, name) == 0) {
      commandMap* cmd = &commands[i];
      cmd->data = value;
      bool ok = false;
      currentCmd = cmd;

      // In ra frame Modbus chuẩn để debug
      switch (cmd->func) {
        case 3:{ // Đọc Holding Register
          ok = mb.readHreg(cmd->slave, cmd->addr, &cmd->data, cmd->numregis, cbModbusTransaction);
          frame[0] = cmd->slave;
          frame[1] = cmd->func;
          frame[2] = highByte(cmd->addr);
          frame[3] = lowByte(cmd->addr);
          frame[4] = highByte(cmd->numregis);
          frame[5] = lowByte(cmd->numregis);
          frame[6] = cmd->numregis * 2; // Byte count
          frame[7] = highByte(cmd->data);
          frame[8] = lowByte(cmd->data);

          uint16_t crc = modbusCRC(frame, 9); // tính CRC cho 9 byte đầu
          frame[9]  = crc & 0xFF;   // CRC Low
          frame[10] = crc >> 8;     // CRC High
          frameLen = 11;
          break;
        }

        case 6:{ // Ghi 1 thanh ghi
          ok = mb.writeHreg(cmd->slave, cmd->addr, cmd->data, cbModbusTransaction);
          frame[0] = cmd->slave;
          frame[1] = cmd->func;
          frame[2] = highByte(cmd->addr);
          frame[3] = lowByte(cmd->addr);
          frame[4] = highByte(cmd->numregis);
          frame[5] = lowByte(cmd->numregis);
          frame[6] = cmd->numregis * 2; // Byte count
          frame[7] = highByte(cmd->data);
          frame[8] = lowByte(cmd->data);

          uint16_t crc = modbusCRC(frame, 9); // tính CRC cho 9 byte đầu
          frame[9]  = crc & 0xFF;   // CRC Low
          frame[10] = crc >> 8;     // CRC High
          frameLen = 11;
          break;
        }

        case 16:{ // Ghi nhiều thanh ghi
          ok = mb.writeHreg(cmd->slave, cmd->addr, &cmd->data, cmd->numregis, cbModbusTransaction);
         frame[0] = cmd->slave;
          frame[1] = cmd->func;
          frame[2] = highByte(cmd->addr);
          frame[3] = lowByte(cmd->addr);
          frame[4] = highByte(cmd->numregis);
          frame[5] = lowByte(cmd->numregis);
          frame[6] = cmd->numregis * 2; // Byte count
          frame[7] = highByte(cmd->data);
          frame[8] = lowByte(cmd->data);

          uint16_t crc = modbusCRC(frame, 9); // tính CRC cho 9 byte đầu
          frame[9]  = crc & 0xFF;   // CRC Low
          frame[10] = crc >> 8;     // CRC High
          frameLen = 11;
          break;
        }
        default:
          Serial.printf("[ERROR] Function code %d chưa hỗ trợ!\n", cmd->func);
          return false;
      }
      Serial.printf("[TX] ");
      for (int j = 0; j < frameLen; j++) Serial.printf("%02X ", frame[j]);
      Serial.println();

      if (!ok) {
        Serial.println("[ERROR] Gửi lệnh Modbus thất bại!");
        return false;
      }
      return true;
    }
  }
  Serial.println("[ERROR] Không tìm thấy lệnh trong bảng mapping!");
  return false;
}


void setup() {
  Serial.begin(115200);
  delay(500);  // cho ổn định kết nối USB


  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Cấu hình chân DE/RE (nếu bạn dùng RS485)
  mb.begin(&Serial2, RE_DE);   // khởi tạo Modbus RTU trên Serial2 với chân RE/DE
  mb.master();
  
  Serial.println("Hệ thống sẵn sàng. Nhập JSON dạng: {\"t\":\"c\",\"v\":\"5\"}");

   
}

void loop() {

  sendCommand("c", 1);
  delay(1000);
  sendCommand("c", 0);
  delay(1000);
  sendCommand("p", 1);
  delay(1000);
  sendCommand("p", 1);
  delay(1000);
  sendCommand("p", 2);
  delay(1000);

  mb.task();

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.isEmpty()) return;

    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, input);
    if (err) {
      Serial.println("[ERROR] JSON không hợp lệ!");
      return;
    }

    const char* name = doc["t"];
    if (!name) {
      Serial.println("[ERROR] Thiếu trường 't'!");
      return;
    }

    // Kiểm tra nếu có giá trị 'v' thì là ghi, ngược lại là đọc
    if (doc.containsKey("v")) {
      uint16_t value = doc["v"];
      sendCommand(name, value);
    } else {
      sendCommand(name);
    }
  }
}