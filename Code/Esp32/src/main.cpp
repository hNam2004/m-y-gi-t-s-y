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

#define DEVICE_ID "TEWD43472L55"

const char *mqtt_server = "devices.koisolutions.vn";
const int mqtt_port = 7183;
const char *FIRMWARE_VERSION = "1.0.0";

const char *mqtt_topic_info = "Kdev/" DEVICE_ID "/info";
const char *mqtt_topic_cmd = "Kdev/" DEVICE_ID "/cmd";
const char *mqtt_topic_status = "Kdev/" DEVICE_ID "/status";
const char *mqtt_topic_control = "Kdev/" DEVICE_ID "/control";

// --- Cấu hình chân GPIO ---
#define BOOT_PIN 99
#define RST_PIN 23
#define LED_PIN 21
#define COIN_PIN 12

WiFiClient espClient;
PubSubClient client(espClient);
unsigned int pressTime = 0;
bool press = false;
TaskHandle_t task2_handle = NULL;
volatile uint8_t Interupt_Flag = 0;
volatile bool connectWifiPing = false;


// --- Buffer và Struct cho dữ liệu máy ---
const int RESPONSE_BUFFER_SIZE = 145;
byte responseBuffer[RESPONSE_BUFFER_SIZE];

struct MachineData
{
    int temperature;
    int warningCount;
    long totalCoins;
    long coinsInBox;
    int runCount;
    char modelName[20];
    bool isValid;
};
MachineData machineInfo;

// =================================================================
// ==== BỘ CÔNG CỤ TẠO VÀ GỬI LỆNH MODBUS (ĐÃ TÍCH HỢP) ====
// =================================================================

// --- HÀM TÍNH TOÁN MODBUS CRC-16 ---
unsigned int ModRTU_CRC(byte buf[], int len)
{
    unsigned int crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (unsigned int)buf[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void printByteAsBinary(byte data)
{
    // Lặp từ bit 7 (MSB) về bit 0 (LSB)
    for (int i = 7; i >= 0; i--)
    {
        Serial.print(bitRead(data, i));
        if (i == 4)
        {
            Serial.print(" ");
        }
    }
}
// --- KHUNG TRUYỀN MẪU CHO CÁC LỆNH MODBUS GHI DỮ LIỆU (Write Register) ---
// Định dạng: [SlaveID, FuncCode, StartAddr_Hi, StartAddr_Lo, NumReg_Hi, NumReg_Lo, ByteCount]
const byte frameTemplate_C[] = {0x01, 0x10, 0x01, 0x2E, 0x00, 0x01, 0x02}; // Start/Stop/Next...
const byte frameTemplate_M[] = {0x01, 0x10, 0x01, 0x31, 0x00, 0x01, 0x02}; // Giả lập coin
const byte frameTemplate_P[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02}; // Chọn chương trình
// HÀM GỬI DỮ LIỆU QUA CỔNG SERIAL
void send_to_serial(byte _msg[], int _len)
{
    while (Serial.available())
        Serial.read(); // Xóa bộ đệm nhận trước khi gửi
    Serial.write(_msg, _len);
    Serial.flush(); // Đợi cho đến khi tất cả dữ liệu được gửi đi
}

/**
 * @brief Xây dựng và gửi một khung truyền Modbus RTU để ghi 1 thanh ghi (Function Code 16)
 * @param commandType Loại lệnh ("c", "m", hoặc "p") để chọn thanh ghi đích.
 * @param commandValue Giá trị (dưới dạng chuỗi) để ghi vào thanh ghi.
 */
void sendMobus(const char *commandType, const char *commandValue)
{
    const byte *templateFrame = NULL;
    size_t templateSize = 0;
    uint16_t dataValue = 0; // Giá trị sẽ được ghi vào thanh ghi

    // Chuyển đổi giá trị từ chuỗi sang số (sẽ dùng cho trường hợp chung)
    uint16_t valueFromStr = atoi(commandValue);

    // --- LOGIC CHỌN TEMPLATE ĐÃ SỬA ĐỔI ---
    if (strcmp(commandType, "c") == 0)
    {
        templateFrame = frameTemplate_C;
        templateSize = sizeof(frameTemplate_C);
        dataValue = valueFromStr; // Dùng giá trị từ input
    }
    else if (strcmp(commandType, "m") == 0)
    {
        templateFrame = frameTemplate_M;
        templateSize = sizeof(frameTemplate_M);
        dataValue = valueFromStr; // Dùng giá trị từ input
    }
    else if (strcmp(commandType, "p") == 0)
    {
        templateFrame = frameTemplate_P;
        templateSize = sizeof(frameTemplate_P);
        dataValue = valueFromStr; // Dùng giá trị từ input
    }
    else
    {
        Serial.printf("Loi: Loai lenh khong xac dinh '%s'\n", commandType);
        return;
    }
    // --- HẾT LOGIC SỬA ĐỔI ---

    // Kích thước cuối cùng = template (7) + 2 byte dữ liệu + 2 byte CRC = 11 bytes
    const size_t finalFrameSize = templateSize + 4;
    byte finalFrame[finalFrameSize];

    // 1. Sao chép khung mẫu vào
    memcpy(finalFrame, templateFrame, templateSize);

    // 2. Gắn 2 byte dữ liệu vào (high byte first)
    //    (dataValue bây giờ đã được gán đúng ở logic bên trên)
    finalFrame[templateSize] = dataValue >> 8;
    finalFrame[templateSize + 1] = dataValue & 0xFF;

    // 3. Tính CRC cho phần dữ liệu đã có (template + 2 byte data)
    //    (Giả sử hàm ModRTU_CRC() tồn tại)
    unsigned int crc = ModRTU_CRC(finalFrame, templateSize + 2);

    // 4. Gắn 2 byte CRC vào cuối (low byte first)
    finalFrame[templateSize + 2] = crc & 0xFF;
    finalFrame[templateSize + 3] = crc >> 8;

    Serial.print("[MODBUS TX]: ");
    for (size_t i = 0; i < finalFrameSize; i++)
    {
        Serial.printf("%02X ", finalFrame[i]);
    }
    Serial.println();

    // (Giả sử hàm send_to_serial() tồn tại)
    send_to_serial(finalFrame, finalFrameSize);

    // Sau khi gửi lệnh, publish lại trạng thái lệnh lên MQTT
    // (Giả sử 'client' và 'mqtt_topic_cmd' tồn tại)
    char jsonMsg[50];
    sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", commandType, commandValue);
    client.publish(mqtt_topic_cmd, jsonMsg);
}

// =================================================================
// ==== CÁC HÀM XỬ LÝ PHẢN HỒI MODBUS (CỦA BẠN) ====
// =================================================================

uint16_t calculateCRC_Response(const byte *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return (crc << 8) | (crc >> 8); // CRC cho phản hồi có thể bị đảo byte
}

bool parseResponse(const byte *buffer, int len)
{
    if (len < 5)
        return false;
    if (buffer[0] != requestReadInfo[0] || buffer[1] != requestReadInfo[1])
        return false;

    int dataByteCount = buffer[2];
    if (len != 3 + dataByteCount + 2) 
        return false;

    uint16_t receivedCRC = (buffer[len - 2] << 8) | buffer[len - 1];
    uint16_t calculatedCRC = calculateCRC_Response(buffer, len - 2);

    machineInfo.temperature = (buffer[3 + 2] << 8) | buffer[3 + 3];
    machineInfo.warningCount = (buffer[3 + 8] << 8) | buffer[3 + 9];
    machineInfo.totalCoins = (long)(buffer[3 + 25] << 24) | (long)(buffer[3 + 26] << 16) | (long)(buffer[3 + 27] << 8) | buffer[3 + 28];
    machineInfo.coinsInBox = (long)(buffer[3 + 68] << 24) | (long)(buffer[3 + 69] << 16) | (long)(buffer[3 + 70] << 8) | buffer[3 + 71];

    machineInfo.runCount = (buffer[3 + 72] << 8) | buffer[3 + 73];

    int modelStartIndex = 3 + 80;
    int modelLen = 0;
    while (modelLen < 19 && (modelStartIndex + modelLen) < (len - 2) && buffer[modelStartIndex + modelLen] != 0)
    {
        machineInfo.modelName[modelLen] = (char)buffer[modelStartIndex + modelLen];
        modelLen++;
    }
    machineInfo.modelName[modelLen] = '\0';

    machineInfo.isValid = true;
    return true;
}

int getMachineStatus()
{
    Serial.write(requestReadStatus, sizeof(requestReadStatus));
    unsigned long startTime = millis();
    int bytesRead = 0;
    byte statusBuffer[30];
    while (millis() - startTime < 1000 && bytesRead < 30)
    {
        if (Serial.available())
        {
            statusBuffer[bytesRead++] = Serial.read();
        }
    }

    if (bytesRead >= 6)
    {
        byte statusByte = statusBuffer[6];
        uint16_t receivedCRC = (statusBuffer[bytesRead - 2] << 8) | statusBuffer[bytesRead - 1];
        if (calculateCRC_Response(statusBuffer, bytesRead - 2) == receivedCRC)
        {
            if (statusByte!= 0x00)
                return 1; // Máy đang chạy
            else
                return 0; // Máy đang dừng
        }
    }
    return -1; // Lỗi hoặc timeout
}

bool readAndParseMachineData()
{
    Serial.write(requestReadInfo, sizeof(requestReadInfo));
    unsigned long startTime = millis();
    int bytesRead = 0;
    while (millis() - startTime < 500 && bytesRead < RESPONSE_BUFFER_SIZE)
    {
        if (Serial.available())
        {
            responseBuffer[bytesRead++] = Serial.read();
        }
    }

    if (bytesRead > 0)
    {
        return parseResponse(responseBuffer, bytesRead);
    }
    machineInfo.isValid = false;
    return false;
}

// =================================================================
// ==== HÀM XỬ LÝ MQTT VÀ CÁC TASK FreeRTOS ====
// =================================================================

// Hàm callback khi nhận được tin nhắn từ MQTT
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.printf("Message arrived on topic: %s. Message: %s\n", topic, message);

    if (strcmp(topic, mqtt_topic_control) == 0)
    {
        StaticJsonDocument<128> doc;
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error)
        {
            Serial.println("Failed to parse JSON");
            return;
        }

        const char *t_value = doc["t"];
        const char *v_value = doc["v"];

        if (t_value && v_value)
        {
            if (strcmp(t_value, "c") == 0 || strcmp(t_value, "m") == 0 || strcmp(t_value, "p") == 0)
            {
                sendMobus(t_value, v_value);
            }
            else if (strcmp(t_value, "s") == 0 && strcmp(v_value, "1") == 0)
            {
                char jsonMsg[50];
                sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", t_value, v_value);
                client.publish(mqtt_topic_cmd, jsonMsg);
                int currentStatus = getMachineStatus();
                vTaskDelay(pdMS_TO_TICKS(200));
                while (Serial.available())
                    Serial.read();
                bool infoReadSuccess = readAndParseMachineData();
                StaticJsonDocument<256> docs;
                docs["s"] = (currentStatus == 1) ? 1 : 0;
                if (infoReadSuccess)
                {
                    char v_buffer[100];
                    sprintf(v_buffer, "%d,0,0,0", machineInfo.temperature);
                    docs["v"] = v_buffer;
                    docs["st"] = 0;
                }
                else
                {
                    docs["v"] = "0,0,0,0";
                    docs["st"] = "02";
                }
                char jsonBuffer[256];
                serializeJson(docs, jsonBuffer);
                client.publish(mqtt_topic_info, jsonBuffer);
            }
            else if (strcmp(t_value, "i") == 0 && strcmp(v_value, "1") == 0)
            {
                char jsonMsg[50];
                sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", t_value, v_value);
                client.publish(mqtt_topic_cmd, jsonMsg);

                // 2. Gửi phiên bản firmware lên INFO
                client.publish(mqtt_topic_info, FIRMWARE_VERSION);
            }
            else
            {
                Serial.printf("Invalid command type 't': %s\n", t_value);
            }
        }
        else
        {
            Serial.println("Invalid JSON format in control topic");
        }
    }
}

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
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    Serial.print(wifiState);
}

void task2Function(void *parameter)
{
    pinMode(LED_PIN, OUTPUT);
    long lastPingTime = 0;
    const int pingInterval = 30000;

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
                connectWifiPing = true;
            }
            if (millis() - lastPingTime > pingInterval)
            {
                if (!Ping.ping("www.google.com", 1))
                {
                    wifiState = WIFI_CONFIGURED_NOT_CONNECTED;
                }
                lastPingTime = millis();
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        case WIFI_CONFIGURED_NOT_CONNECTED:
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        case WIFI_NOT_CONFIGURED:
            digitalWrite(LED_PIN, LOW);
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
        if (digitalRead(RST_PIN) == LOW)
        {
            if (!press)
            {
                press = true;
                pressTime = millis();
            }
            else
            {
                if (millis() - pressTime >= 5000)
                {
                    Serial.println(">>> Nut RST giu 5s, xoa WiFi <<<");
                    clearWiFiCredentials();
                    press = false; // Reset cờ để tránh lặp lại
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
    // Logic gốc của bạn
    long lastMsgTime = 0;
    const int publishInterval = 3000;
    long lastMqttAttempt = 0;
    long lastWifiReconnectAttempt = 0;
    const int wifiReconnectInterval = 15000;

    while (true)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            if (wifiState != WIFI_CONNECTED)
            {
                wifiState = WIFI_CONNECTED;
            }
            if (!client.connected())
            {
                if (millis() - lastMqttAttempt > 5000)
                {
                    Serial.print("Attempting MQTT connection...");
                    if (client.connect(DEVICE_ID))
                    {
                        Serial.println("MQTT connected");
                        client.subscribe(mqtt_topic_control);
                        client.subscribe(mqtt_topic_info);
                    }
                    else
                    {
                        Serial.printf("failed, rc=%d\n", client.state());
                    }
                    lastMqttAttempt = millis();
                }
            }
            else
            {
                client.loop();
                if (millis() - lastMsgTime > publishInterval)
                {
                    lastMsgTime = millis();

                    // Xóa buffer trước khi thực hiện chuỗi giao tiếp mới
                    while (Serial.available())
                        Serial.read();

                    while (Serial.available())
                        Serial.read();

                    // GỌI HÀM TỪ readbyte.cpp
                    int currentStatus = getMachineStatus();

                    // Xóa buffer một lần nữa để chắc chắn
                    while (Serial.available())
                        Serial.read();

                    while (Serial.available())
                        Serial.read();

                    // GỌI HÀM TỪ readbyte.cpp
                    bool infoReadSuccess = readAndParseMachineData();

                    StaticJsonDocument<256> doc;
                    doc["s"] = (currentStatus == 1) ? 1 : 0;
                    if (infoReadSuccess)
                    {
                        char v_buffer[100];
                        sprintf(v_buffer, "%d,0,0,0", machineInfo.temperature);
                        doc["v"] = v_buffer;
                        doc["st"] = 0;
                    }
                    else
                    {
                        doc["v"] = "0,0,0,0";
                        doc["st"] = "02";
                    }
                    char jsonBuffer[256];
                    serializeJson(doc, jsonBuffer);
                    client.publish(mqtt_topic_info, jsonBuffer);
                    Serial.print("Published info: ");
                    Serial.println(jsonBuffer);
                }
            }
        }
        else if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED)
        {
            if (millis() - lastWifiReconnectAttempt > wifiReconnectInterval)
            {
                Serial.println("[Task 5] WiFi lost. Attempting to reconnect...");
                WiFi.reconnect();
                lastWifiReconnectAttempt = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void setup()
{
    Serial.begin(9600, SERIAL_8N1);
    delay(1000);
    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);
    pinMode(RST_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(COIN_PIN, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING);
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);

    xTaskCreate(task1Function, "Task 1", 10000, NULL, 1, NULL); // Interrupt mạng
    xTaskCreate(task2Function, "Task 2", 10000, NULL, 1, NULL); // Nháy led và ping check mạng
    xTaskCreate(task3Function, "Task 3", 10000, NULL, 1, NULL); // Giữ 5 giây vào IO23 để reset mạng
    xTaskCreate(task5Function, "Task 5", 10000, NULL, 1, NULL); // MQTT
}

void loop()
{
}