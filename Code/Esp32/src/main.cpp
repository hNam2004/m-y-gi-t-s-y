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
#include <Preferences.h>
// --- Cấu hình thiết bị (sẽ được load từ Preferences) ---
char deviceID[32];
char mqttServer[100];
const int mqtt_port = 7183;
const char *FIRMWARE_VERSION = "1.0.0";

// --- Các topic MQTT (sẽ được xây dựng tự động) ---
char mqtt_topic_info[150];
char mqtt_topic_cmd[150];
char mqtt_topic_status[150];
char mqtt_topic_control[150];

// --- Cấu hình chân GPIO ---
#define BOOT_PIN 99
#define RST_PIN 23
#define LED_PIN 21
#define COIN_PIN 12
#define IN_SIG2 18

// --- Khai báo biến toàn cục ---
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
String serialCmdBuffer = "";
unsigned int pressTime = 0;
bool press = false;
TaskHandle_t task2_handle = NULL;
volatile uint8_t Interupt_Flag = 0;
volatile bool connectWifiPing = false;

// --- Khai báo các lệnh Modbus cố định ---
// Lệnh đọc thông tin máy (đọc nhiều thanh ghi)
byte requestReadInfo[] = {0x01, 0x03, 0x03, 0x20, 0x00, 0x46, 0xC5, 0xB6};
// Lệnh đọc trạng thái máy (đọc coil)
byte requestReadStatus[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0xA0, 0x3C, 0x72};

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

    if (bytesRead >= 12)
    {
        uint16_t receivedCRC = (statusBuffer[bytesRead - 2] << 8) | statusBuffer[bytesRead - 1];
        if (calculateCRC_Response(statusBuffer, bytesRead - 2) == receivedCRC)
        {
            int statusSum = statusBuffer[6] + statusBuffer[9];

            if (statusSum != 0)
            {
                return 1;
            }
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

#define FC03_EXPECTED_DATA_BYTES 140
#define FC03_FULL_RESPONSE_LENGTH (3 + FC03_EXPECTED_DATA_BYTES + 2)

    int expectedLength = FC03_FULL_RESPONSE_LENGTH;

    while (millis() - startTime < 1000)
    {
        if (Serial.available())
        {
            if (bytesRead < RESPONSE_BUFFER_SIZE)
            {
                responseBuffer[bytesRead++] = Serial.read();
            }
            else
            {
                while (Serial.available())
                    Serial.read();
                break;
            }
        }
        if (bytesRead == expectedLength)
        {
            break;
        }
    }

    if (bytesRead == expectedLength)
    {
        return parseResponse(responseBuffer, bytesRead);
    }

    machineInfo.isValid = false;
    return false;
}
void buildMqttTopics()
{
    sprintf(mqtt_topic_info, "Kdev/%s/info", deviceID);
    sprintf(mqtt_topic_cmd, "Kdev/%s/cmd", deviceID);
    sprintf(mqtt_topic_status, "Kdev/%s/status", deviceID);
    sprintf(mqtt_topic_control, "Kdev/%s/control", deviceID);

    Serial.println("--- MQTT Topics Đã Xây Dựng ---");
    Serial.printf("Info: %s\n", mqtt_topic_info);
    Serial.printf("Control: %s\n", mqtt_topic_control);
    Serial.println("-------------------------------");
}

/**
 * @brief Lưu giá trị mới vào Preferences và khởi động lại.
 */
void saveAndRestart(const char* key, String value)
{
    preferences.begin("config", false); // Mở namespace 'config' ở chế độ read-write
    preferences.putString(key, value);
    preferences.end();

    Serial.printf("Đã lưu %s = %s\n", key, value.c_str());
    Serial.println("Khởi động lại sau 2 giây...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();
}

/**
 * @brief Tải cấu hình từ Preferences vào các biến toàn cục.
 * Được gọi trong setup().
 */
void loadConfig()
{
    preferences.begin("config", true); // Mở namespace 'config' ở chế độ read-only
    
    // Tải Device ID, nếu không có thì dùng "1234"
    String id = preferences.getString("deviceID", "1234");
    // Tải MQTT Server, nếu không có thì dùng "devices.koisolutions.vn"
    String server = preferences.getString("mqttServer", "devices.koisolutions.vn");
    
    preferences.end();

    // Sao chép vào các biến char array toàn cục
    strcpy(deviceID, id.c_str());
    strcpy(mqttServer, server.c_str());

    Serial.println("--- Cấu Hình Đã Tải ---");
    Serial.printf("Device ID: %s\n", deviceID);
    Serial.printf("MQTT Server: %s\n", mqttServer);
    Serial.println("-------------------------");

    // Xây dựng các topic MQTT sau khi đã có deviceID
    buildMqttTopics();
}

/**
 * @brief Kiểm tra và xử lý các lệnh đến từ Serial.
 * Sẽ được gọi liên tục trong một task (ví dụ Task 5).
 */
void checkSerialCommands()
{
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r') // Khi nhấn Enter
        {
            serialCmdBuffer.trim(); // Xóa khoảng trắng
            if (serialCmdBuffer.length() > 0)
            {
                Serial.printf("[Serial CMD] Nhận được: %s\n", serialCmdBuffer.c_str());

                // Phân tích lệnh
                if (serialCmdBuffer.startsWith("ID_"))
                {
                    String newID = serialCmdBuffer.substring(3); // Lấy phần sau "ID_"
                    if (newID.length() > 0) {
                        saveAndRestart("deviceID", newID); // Hàm này sẽ tự restart
                    } else {
                        Serial.println("Lỗi: ID không được rỗng.");
                    }
                }
                else if (serialCmdBuffer.startsWith("SV_"))
                {
                    String newServer = serialCmdBuffer.substring(3); // Lấy phần sau "SV_"
                     if (newServer.length() > 0) {
                        saveAndRestart("mqttServer", newServer); // Hàm này sẽ tự restart
                    } else {
                        Serial.println("Lỗi: Server không được rỗng.");
                    }
                }
                else
                {
                    Serial.println("Lỗi: Lệnh không xác định. Dùng: ID_<value> hoặc SV_<value>");
                }
            }
            serialCmdBuffer = ""; // Xóa buffer cho lệnh tiếp theo
        }
        else if (isPrintable(c)) // Chỉ thêm các ký tự in được
        {
            serialCmdBuffer += c;
        }
    }
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
                if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "1") == 0)
                {
                    sendMobus(t_value, v_value);
                    digitalWrite(COIN_PIN, LOW);
                    
                }
                else if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "2") == 0)
                {
                    sendMobus(t_value, v_value);
                    digitalWrite(COIN_PIN, HIGH);
                    vTaskDelay(pdMS_TO_TICKS(100)); // Nghỉ 5 giây
                    digitalWrite(COIN_PIN, LOW);
                }
                else if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "3") == 0)
                {
                    sendMobus(t_value, v_value);
                    digitalWrite(COIN_PIN, HIGH);
                }

                else
                {
                    sendMobus(t_value, v_value);
                }
            }
            else if (strcmp(t_value, "s") == 0 && strcmp(v_value, "1") == 0)
            {
                char jsonMsg[50];
                sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", t_value, v_value);
                client.publish(mqtt_topic_cmd, jsonMsg); // 'client' và 'mqtt_topic_cmd' có sẵn
                
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
    unsigned long task3_lastBlinkTime = 0;
    const int BLINK_INTERVAL = 150; // Tốc độ nháy LED (ms)
    const int RESET_HOLD_TIME = 5000; // 5 giây

    while (true)
    {
        if (digitalRead(RST_PIN) == LOW) 
        {
            if (!press)
            {
                press = true; 
                pressTime = millis(); 
                task3_lastBlinkTime = millis(); 
                Serial.println("[Task 3] Bắt đầu giữ nút RST (timer 5s)...");
            }


            if (millis() - task3_lastBlinkTime >= BLINK_INTERVAL)
            {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Đảo trạng thái LED
                task3_lastBlinkTime = millis();
            }


            if (press && (millis() - pressTime >= RESET_HOLD_TIME))
            {
                Serial.println("[Task 3] >>> Nut RST giu 5s, xoa WiFi <<<");
                clearWiFiCredentials(); 
                press = false; 

                while (digitalRead(RST_PIN) == LOW)
                {
                    if (millis() - task3_lastBlinkTime >= BLINK_INTERVAL)
                    {
                        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                        task3_lastBlinkTime = millis();
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            }
        }
        else // Nút đang được thả (HIGH)
        {
            if (press)
            {
                Serial.println("[Task 3] Đã thả nút RST (trước 5s).");
                digitalWrite(LED_PIN, HIGH); // Kéo LED lên HIGH theo yêu cầu
            }
            press = false; 
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void task5Function(void *parameter)
{
    long lastMsgTime = 0;
    const int publishInterval = 3000;
    long lastMqttAttempt = 0;
    long lastWifiReconnectAttempt = 0;
    const int wifiReconnectInterval = 15000;

    while (true)
    {
        checkSerialCommands();
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
                    if (client.connect(deviceID))
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

                    int currentStatus = getMachineStatus();

                    // Xóa buffer một lần nữa để chắc chắn
                    while (Serial.available())
                        Serial.read();

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
    loadConfig();
    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);
    pinMode(RST_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(COIN_PIN, OUTPUT);
    client.setServer(mqttServer, mqtt_port);
    client.setCallback(mqttCallback);

    xTaskCreate(task1Function, "Task 1", 10000, NULL, 1, NULL); // Interrupt mạng
    xTaskCreate(task2Function, "Task 2", 10000, NULL, 1, NULL); // Nháy led và ping check mạng
    xTaskCreate(task3Function, "Task 3", 10000, NULL, 1, NULL); // Giữ 5 giây vào IO23 để reset mạng
    xTaskCreate(task5Function, "Task 5", 10000, NULL, 1, NULL); // MQTT
}

void loop()
{
}