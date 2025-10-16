
#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PubSubClient.h>
#include "sys_capserver.hpp"
#include "sys_wifi.hpp"
#include <Wire.h>
#include <ESP32Ping.h>
#include <ArduinoJson.h>

// --- Cấu hình MQTT ---
const char *mqtt_server = "109.237.65.252";
const int mqtt_port = 7183;
const char *mqtt_topic_info = "Kdev/1234/info";
const char *mqtt_topic_cmd = "Kdev/1234/cmd";
const char *mqtt_topic_status = "Kdev/1234/status";
const char *mqtt_topic_control = "Kdev/1234/control";
const char *FIRMWARE_VERSION = "1.0.0";
// --- Cấu hình chân GPIO ---
#define BOOT_PIN 99
#define RST_PIN 23
#define LED_PIN 21
#define COIN_PIN 12
#define IN_SIG2 18
// --- Khai báo biến toàn cục ---
WiFiClient espClient;
PubSubClient client(espClient);
unsigned int pressTime = 0;
bool press = false;
TaskHandle_t task2_handle = NULL;
volatile uint8_t Interupt_Flag = 0;
volatile bool coinPulseLoopActive = false; // Cờ để bật/tắt vòng lặp
volatile bool connectWifiPing = false;
 
// Khai bao byte modbus
byte requestFrame01[] = {0x01, 0x10, 0x01, 0x2E, 0x00, 0x01, 0x02, 0x00, 0x01, 0x71, 0x1E};    
byte requestFrame02[] = {0x01, 0x10, 0x01, 0x2F, 0x00, 0x01, 0x02, 0x00, 0x01, 0x70, 0xCF};
byte requestFrame03[] = {0x01, 0x10, 0x01, 0x2C, 0x00, 0x01, 0x02, 0x00, 0x01, 0x70, 0xFC};

// Nhóm lệnh "Giả lập coin"
byte requestFrame04[] = {0x01, 0x10, 0x01, 0x31, 0x00, 0x01, 0x02, 0x00, 0x01, 0x73, 0x71};
byte requestFrame05[] = {0x01, 0x10, 0x01, 0x31, 0x00, 0x01, 0x02, 0x00, 0x02, 0x33, 0x70};
byte requestFrame06[] = {0x01, 0x10, 0x01, 0x31, 0x00, 0x01, 0x02, 0x00, 0x03, 0xF2, 0xB0};

// Nhóm lệnh "Chọn chương trình"
byte requestFrame07[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02, 0x00, 0x01, 0xB3, 0x82}; 
byte requestFrame08[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02, 0x00, 0x02, 0x33, 0x43};
byte requestFrame09[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02, 0x00, 0x03, 0xF2, 0x83};
byte requestFrame10[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02, 0x00, 0x04, 0xB3, 0x41};
byte requestFrame11[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02, 0x00, 0x05, 0x72, 0x81};
byte requestFrame12[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02, 0x00, 0x06, 0x40, 0x83};
byte requestFrame13[] = {0x01, 0x03, 0x03, 0x20, 0x00, 0x46, 0xC5, 0xB6};
byte machineStatusCommand[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0xA0, 0x3C, 0x72};

const int RESPONSE_BUFFER_SIZE = 145;
byte responseBuffer[RESPONSE_BUFFER_SIZE];

struct MachineData {
  int temperature;
  int warningCount;
  long totalCoins;
  long coinsInBox;
  int runCount;
  char modelName[20];
  bool isValid;
};
MachineData machineInfo;

uint16_t calculateCRC(const byte* data, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return (crc << 8) | (crc >> 8);
}

bool parseResponse(const byte* buffer, int len) {
  if (len < 5) return false;
  if (buffer[0] != requestFrame13[0] || buffer[1] != requestFrame13[1]) return false;
  int dataByteCount = buffer[2];
  if (len != 3 + dataByteCount + 2) return false;
  uint16_t receivedCRC = (buffer[len - 2] << 8) | buffer[len - 1];
  uint16_t calculatedCRC = calculateCRC(buffer, len - 2);
  if (receivedCRC != calculatedCRC) return false;
  
  machineInfo.temperature = (buffer[3+2] << 8) | buffer[3+3];
  machineInfo.warningCount = (buffer[3+8] << 8) | buffer[3+9];
  machineInfo.totalCoins = (long)(buffer[3+22] << 24) | (long)(buffer[3+23] << 16) | (buffer[3+24] << 8) | buffer[3+25];
  machineInfo.coinsInBox = (long)(buffer[3+58] << 24) | (long)(buffer[3+59] << 16) | (buffer[3+60] << 8) | buffer[3+61];
  machineInfo.runCount = (buffer[3+64] << 8) | buffer[3+65];
  int modelStartIndex = 3 + 70;
  int modelLen = 0;
  while(modelLen < 19 && (modelStartIndex + modelLen) < (len - 2) && buffer[modelStartIndex + modelLen] != 0) {
    machineInfo.modelName[modelLen] = (char)buffer[modelStartIndex + modelLen];
    modelLen++;
  }
  machineInfo.modelName[modelLen] = '\0';
  machineInfo.isValid = true;
  return true;
}
int getMachineStatus() {
    while(Serial.available()) Serial.read();
    Serial.write(machineStatusCommand, sizeof(machineStatusCommand));
    
    unsigned long startTime = millis();
    int bytesRead = 0;
    byte statusBuffer[30];
    while (millis() - startTime < 1000 && bytesRead < 30) {
        if (Serial.available()) {
            statusBuffer[bytesRead++] = Serial.read();
        }
    }

    if (bytesRead >= 6) {
        uint16_t receivedCRC = (statusBuffer[bytesRead - 2] << 8) | statusBuffer[bytesRead - 1];
        if (calculateCRC(statusBuffer, bytesRead - 2) == receivedCRC) {
            if (statusBuffer[5] == 0x01) return 1;
            else return 0;
        }
    }
    return -1;
}
// (ADDED) Hàm đóng gói việc gửi lệnh và nhận phản hồi
bool readAndParseMachineData() {
    // Xóa bộ đệm cũ trước khi đọc
    while(Serial.available()) Serial.read();

    Serial.write(requestFrame13, sizeof(requestFrame13));

    unsigned long startTime = millis();
    int bytesRead = 0;
    while (millis() - startTime < 1500 && bytesRead < RESPONSE_BUFFER_SIZE) { // Tăng timeout lên 1.5s cho chắc chắn
        if (Serial.available()) {
            responseBuffer[bytesRead] = Serial.read();
            bytesRead++;
        }
    }

    if (bytesRead > 0) {
        return parseResponse(responseBuffer, bytesRead);
    }
    
    // Không nhận được phản hồi
    machineInfo.isValid = false;
    return false;
}
void Runmachine(void)
{
   Serial.write(requestFrame01, sizeof(requestFrame01));
   client.publish(mqtt_topic_cmd,"{\"t\":\"c\", \"v\":\"1\"}");
}

void Nextstep(void)
{
   Serial.write(requestFrame02, sizeof(requestFrame02));
}
void info(void){
    Serial.write(requestFrame13, sizeof(requestFrame13));
}
void Mute(void )
{
   Serial.write(requestFrame03, sizeof(requestFrame03));
} 
void Takecoin1(void)
{
   Serial.write(requestFrame04, sizeof(requestFrame04));
   client.publish(mqtt_topic_cmd,"{\"t\":\"m\", \"v\":\"1\"}");
} 

void Takecoin2(void)
{
   Serial.write(requestFrame05, sizeof(requestFrame05));
   client.publish(mqtt_topic_cmd,"{\"t\":\"m\", \"v\":\"2\"}");

} 

void Takecoin3(void)
{
   Serial.write(requestFrame06, sizeof(requestFrame06));
   client.publish(mqtt_topic_cmd,"{\"t\":\"m\", \"v\":\"3\"}");

} 

void Program1(void)
{
   Serial.write(requestFrame07, sizeof(requestFrame07));
   client.publish(mqtt_topic_cmd,"{\"t\":\"p\", \"v\":\"1\"}");

} 

void Program2(void )
{
   Serial.write(requestFrame08, sizeof(requestFrame08));
      client.publish(mqtt_topic_cmd,"{\"t\":\"p\", \"v\":\"2\"}");

} 

void Program3(void)
{
   Serial.write(requestFrame09, sizeof(requestFrame09));
      client.publish(mqtt_topic_cmd,"{\"t\":\"p\", \"v\":\"3\"}");

} 

void Program4(void )
{
   Serial.write(requestFrame10, sizeof(requestFrame10));
      client.publish(mqtt_topic_cmd,"{\"t\":\"p\", \"v\":\"4\"}");

} 

void Program5(void)
{
   Serial.write(requestFrame11, sizeof(requestFrame11));
      client.publish(mqtt_topic_cmd,"{\"t\":\"p\", \"v\":\"5\"}");

} 

void Program6(void)
{
   Serial.write(requestFrame12, sizeof(requestFrame12));
      client.publish(mqtt_topic_cmd,"{\"t\":\"p\", \"v\":\"6\"}");

} 
// --- Khai báo các hàm ---
void mqttCallback(char *topic, byte *payload, unsigned int length);
void reconnect();

// Hàm callback khi nhận được tin nhắn từ MQTT
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);

    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.print("Message: ");
    Serial.println(message);

    if (strcmp(topic, mqtt_topic_control) == 0)
    {
        Serial.print("Control command: ");
        Serial.println(message);
        StaticJsonDocument<128> control;
        DeserializationError error = deserializeJson(control, payload, length);
        const char *t_value = control["t"];
        const char *v_value = control["v"];
        if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "1") == 0)
        {
            
            coinPulseLoopActive = false;
            digitalWrite(COIN_PIN, HIGH);
            Takecoin1();
        }
        else if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "2") == 0)
        {
            coinPulseLoopActive = true;
            Takecoin2();
        }
        else if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "3") == 0)
        {
            coinPulseLoopActive = false;
            digitalWrite(COIN_PIN, LOW);
            Takecoin3();
        }
        else if (t_value && v_value && strcmp(t_value, "c") == 0 && strcmp(v_value, "1") == 0)
        {
            Runmachine();
        }
        else if (t_value && v_value && strcmp(t_value, "c") == 0 && strcmp(v_value, "0") == 0)
        {

        }
        else if (t_value && v_value && strcmp(t_value, "p") == 0 && strcmp(v_value, "1") == 0)
        {
            Program1();
        }
        else if (t_value && v_value && strcmp(t_value, "p") == 0 && strcmp(v_value, "2") == 0)
        {
            Program2();
        }
        else if (t_value && v_value && strcmp(t_value, "p") == 0 && strcmp(v_value, "3") == 0)
        {
            Program3();
        }   
        else if (t_value && v_value && strcmp(t_value, "p") == 0 && strcmp(v_value, "4") == 0)
        {   
            Program4();
        }    
        else if (t_value && v_value && strcmp(t_value, "p") == 0 && strcmp(v_value, "5") == 0)
        {   
            Program5();
        } 
        else if (t_value && v_value && strcmp(t_value, "p") == 0 && strcmp(v_value, "6") == 0)
        {
            Program6();
        }                                                     
    }
    else if (strcmp(topic, mqtt_topic_status) == 0)
    {
        StaticJsonDocument<128> doc;
        DeserializationError error = deserializeJson(doc, payload, length);

        if (error)
        {
            Serial.print("deserializeJson() failed: ");
            Serial.println(error.c_str());
            return;
        }

        const char *t_value = doc["t"];
        const char *v_value = doc["v"];

        // KIỂM TRA QUAN TRỌNG: Đảm bảo cả hai con trỏ không phải là NULL trước khi dùng strcmp
        if (t_value && v_value && strcmp(t_value, "i") == 0 && strcmp(v_value, "1") == 0)
        {
            Serial.print("Status command OK. Received: ");
            char message[length + 1];
            memcpy(message, payload, length);
            message[length] = '\0';
            Serial.println(message);
            client.publish(mqtt_topic_status, FIRMWARE_VERSION);
        }
        else
        {
            // Cung cấp thông tin debug hữu ích khi lệnh không thành công
            Serial.println("--- MQTT Command Failed ---");

            char message[length + 1];
            memcpy(message, payload, length);
            message[length] = '\0';
            Serial.print("Received payload: ");
            Serial.println(message);

            if (!t_value)
            {
                Serial.println("Reason: JSON key 't' is missing or not a string.");
            }
            if (!v_value)
            {
                Serial.println("Reason: JSON key 'v' is not a string.");
            }
            Serial.println("---------------------------");
        }
    }
}

void reconnect()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESp_DLG";
        if (client.connect(clientId.c_str()))
        {
            Serial.println("connected");
            client.subscribe(mqtt_topic_cmd);
            client.subscribe(mqtt_topic_control);
            client.subscribe(mqtt_topic_info);
            client.subscribe(mqtt_topic_status);
            client.publish(mqtt_topic_status, FIRMWARE_VERSION, true);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

// This function will be called when the BOOT pin transitions from LOW to HIGH (rising edge)
void bootInterruptHandler()
{
    Serial.println("Interupt ocur");
    Interupt_Flag = 1;
}

void task1Function(void *parameter)
{
    while (true)
    {
        sys_capserver_proc();
        if (Interupt_Flag)
        {
            clearWiFiCredentials();
            Interupt_Flag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    Serial.print(wifiState);
}

// QUẢN LÝ LED VÀ KIỂM TRA MẠNG =======
void task2Function(void *parameter)
{
    pinMode(LED_PIN, OUTPUT);
    long lastPingTime = 0;
    const int pingInterval = 30000; // Kiểm tra ping mỗi 30 giây

    while (true)
    {
        switch (wifiState)
        {
        case WIFI_CONNECTED:
            if (!connectWifiPing)
            {
                digitalWrite(LED_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, HIGH);
                connectWifiPing = true;
            }
            if (millis() - lastPingTime > pingInterval)
            {
                Serial.println("[Network] Dang kiem tra ket noi mang...");
                if (!Ping.ping("www.google.com", 1))
                {
                    Serial.println("[Network] PING FAILED. Mat ket noi Internet.");
                    wifiState = WIFI_CONFIGURED_NOT_CONNECTED;
                }
                else
                {
                    Serial.println("[Network] Ping OK. Ket noi Internet on dinh.");
                }
                lastPingTime = millis(); // Cập nhật lại thời điểm ping cuối cùng
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            break;

        case WIFI_CONFIGURED_NOT_CONNECTED:
            // Đã có cấu hình nhưng không kết nối được -> NHÁY CHẬM.
            digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Đảo trạng thái LED
            vTaskDelay(pdMS_TO_TICKS(1000));              // Nháy mỗi 1 giây
            break;

        case WIFI_NOT_CONFIGURED:
            digitalWrite(LED_PIN, LOW); // luôn sáng
            vTaskDelay(pdMS_TO_TICKS(200));
            connectWifiPing = false;
            break;
        }
    }
}

void task3Function(void *parameter)
{
    while (true)
    {
        int buttonState = digitalRead(RST_PIN);

        if (buttonState == LOW)
        {
            if (!press)
            {
                press = true;
                pressTime = millis();
                digitalWrite(LED_PIN, LOW);
                vTaskDelay(300);
                digitalWrite(LED_PIN, HIGH);
                vTaskDelay(300);
                digitalWrite(LED_PIN, LOW);
                vTaskDelay(300);
                digitalWrite(LED_PIN, HIGH);
                vTaskDelay(300);
                digitalWrite(LED_PIN, LOW);
                vTaskDelay(300);
                digitalWrite(LED_PIN, HIGH);
            }
            else
            {
                if (millis() - pressTime >= 5000)
                {
                    Serial.println(">>> Nut RST giu 5s, xoa WiFi <<<");
                    clearWiFiCredentials();
                    press = false;
                }
            }
        }
        else
        {
            press = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void task5Function(void *parameter)
{
    long lastMsgTime = 0;
    const int publishInterval = 90000; // Gửi tin nhắn mỗi 90 giây
    long lastMqttAttempt = 0;
    long lastWifiReconnectAttempt = 0;
    const int wifiReconnectInterval = 15000; // Thử lại Wi-Fi mỗi 15 giây

    while (true)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            if (wifiState != WIFI_CONNECTED)
            {
                Serial.println("[Task 5] WiFi Connected!");
                wifiState = WIFI_CONNECTED;
            }
            if (!client.connected())
            {
                if (millis() - lastMqttAttempt > 5000)
                {
                    Serial.print("Attempting MQTT connection...");
                    String clientId = "ESp_DLG";
                    if (client.connect(clientId.c_str()))
                    {
                        Serial.println("MQTT connected");
                        // Subscribe lại các topic cần thiết
                        client.subscribe(mqtt_topic_cmd);
                        client.subscribe(mqtt_topic_control);
                        client.subscribe(mqtt_topic_info);
                        client.subscribe(mqtt_topic_status);
                    }
                    else
                    {
                        Serial.print("failed, rc=");
                        Serial.println(client.state());
                    }
                    lastMqttAttempt = millis();
                }
            }
            else // Nếu MQTT đã kết nối thành công
            {
                client.loop();

                long now = millis();
                if (now - lastMsgTime > publishInterval)
                {
                    lastMsgTime = now;
                    int currentStatus = -1;
                    Serial.println("[Task 5] Reading machine data...");
                    // Gọi hàm đọc và phân tích dữ liệu từ máy
                    bool infoReadSuccess = false;
                    currentStatus = getMachineStatus();
                    infoReadSuccess = readAndParseMachineData();
                    StaticJsonDocument<200> doc;    
                    JsonObject obj = doc.to<JsonObject>();
                    if (currentStatus){
                        obj["s"] = 1; 
                    }
                    else obj["s"] = 0;
                    if (infoReadSuccess) {
                        
                        String v_value = String(machineInfo.temperature) + "," + String(machineInfo.totalCoins) + "," + String(machineInfo.runCount);
                        obj["v"] = v_value;
                        obj["st"] = 0;
                    } else {
                        obj["v"] = "0,0,0";
                        obj["st"] = "02";
                    }
                     char jsonBuffer[512];
                     serializeJson(doc, jsonBuffer);
                     client.publish(mqtt_topic_info, jsonBuffer);

                     Serial.print("Published info: ");
                      Serial.println(jsonBuffer);

                      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                      vTaskDelay(pdMS_TO_TICKS(100));
                      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                }
            }
        }
        // 2. XỬ LÝ KHI MẤT KẾT NỐI WIFI (ĐÃ CÓ CẤU HÌNH)
        else if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED)
        {
            // Nếu mất kết nối Wi-Fi, thử kết nối lại định kỳ
            if (millis() - lastWifiReconnectAttempt > wifiReconnectInterval)
            {
                Serial.println("[Task 5] WiFi lost. Attempting to reconnect...");
                WiFi.reconnect();
                digitalWrite(LED_PIN, LOW);
                digitalWrite(LED_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, LOW);
                lastWifiReconnectAttempt = millis();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
void task6Function(void *parameter)
{
    while (true)
    {
        if (coinPulseLoopActive) // Nếu cờ được bật
        {
            digitalWrite(COIN_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(5000)); // Nghỉ 5 giây

            if (coinPulseLoopActive)
            {
                digitalWrite(COIN_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(55000)); // Nghỉ 55 giây
            }
        }
        else // Nếu cờ đang tắt
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void setup()
{
    Serial.begin(9600,SERIAL_8N1);
    delay(1000);
    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);
    pinMode(RST_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(COIN_PIN,OUTPUT);                                                       // Configure BOOT pin as input with internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING); // Attach interrupt handler to rising edge of BOOT pin
    Serial.println("All Done!");
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback); 

    xTaskCreate(
        task1Function, // Task function
        "Task 1",      // Task name
        10000,         // Stack size (bytes)        //Interrupt mạng
        NULL,          // Task parameters
        1,             // Task priority
        NULL           // Task handle
    );
    xTaskCreate(
        task2Function, // Task function
        "Task 2",      // Task name
        10000,         // Stack size (bytes)        //Nháy led và ping check mạng
        NULL,          // Task parameters
        1,             // Task priority
        NULL           // Task handle
    );
    xTaskCreate(
        task3Function,
        "Task 3",
        10000, // Giữ 5 giây vào IO23 để reset mạng
        NULL,
        1,
        NULL);

    xTaskCreate(
        task5Function,
        "Task 5", /// MQTT
        10000,
        NULL,
        1,
        NULL);
    xTaskCreate(
        task6Function,
        "Task 6", // Coin LOOP
        10000,
        NULL,
        1,
        NULL);
}
void loop()
{
}
