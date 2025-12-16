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
#include <EEPROM.h> 
#include "sys_eeprom.hpp" 

// --- [OTA] Thư viện OTA ---
#include <SPIFFS.h>
#include <Update.h>
#include <WiFiClientSecure.h> 
#include <CRC32.h>
#include <HTTPClient.h>

char deviceID[32];
char mqttServer[100];
const int mqtt_port = 7183;
const char *FIRMWARE_VERSION = "1.0.0";

char mqtt_topic_info[150];
char mqtt_topic_cmd[150];
char mqtt_topic_status[150];
char mqtt_topic_control[150];
char mqtt_topic_ota_status[150]; 

#define BOOT_PIN 99
#define RST_PIN 23
#define LED_PIN 21
#define COIN_PIN 12
#define IN_SIG2 18
#define OUT_SIG1 14

WiFiClient espClient;
PubSubClient client(espClient);
WiFiClientSecure httpsClient;

String serialCmdBuffer = "";
unsigned int pressTime = 0;
bool press = false;
TaskHandle_t task2_handle = NULL;
volatile uint8_t Interupt_Flag = 0;
volatile bool connectWifiPing = false;

int machineType = 1; 

// --- [OTA] Biến toàn cục ---
volatile bool g_otaInProgress = false;
TaskHandle_t task6_handle = NULL; 
char g_otaUrl[256]; 

byte requestReadInfo[] = {0x01, 0x03, 0x03, 0x20, 0x00, 0x46, 0xC5, 0xB6};
byte requestReadStatus[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0xA0, 0x3C, 0x72};

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
    for (int i = 7; i >= 0; i--)
    {
        Serial.print(bitRead(data, i));
        if (i == 4)
        {
            Serial.print(" ");
        }
    }
}
const byte frameTemplate_C[] = {0x01, 0x10, 0x01, 0x2E, 0x00, 0x01, 0x02};
const byte frameTemplate_M[] = {0x01, 0x10, 0x01, 0x31, 0x00, 0x01, 0x02};
const byte frameTemplate_P[] = {0x01, 0x10, 0x01, 0x32, 0x00, 0x01, 0x02};

void send_to_serial(byte _msg[], int _len)
{
    while (Serial.available())
        Serial.read(); 
    Serial.write(_msg, _len);
    Serial.flush(); 
}

void sendMobus(const char *commandType, const char *commandValue)
{
    const byte *templateFrame = NULL;
    size_t templateSize = 0;
    uint16_t dataValue = 0; 

    uint16_t valueFromStr = atoi(commandValue);

    if (strcmp(commandType, "c") == 0)
    {
        templateFrame = frameTemplate_C;
        templateSize = sizeof(frameTemplate_C);
        dataValue = valueFromStr; 
    }
    else if (strcmp(commandType, "m") == 0)
    {
        templateFrame = frameTemplate_M;
        templateSize = sizeof(frameTemplate_M);
        dataValue = valueFromStr; 
    }
    else if (strcmp(commandType, "p") == 0)
    {
        templateFrame = frameTemplate_P;
        templateSize = sizeof(frameTemplate_P);
        dataValue = valueFromStr; 
    }
    else
    {
        Serial.printf("Loi: Loai lenh khong xac dinh '%s'\n", commandType);
        return;
    }
    
    const size_t finalFrameSize = templateSize + 4;
    byte finalFrame[finalFrameSize];

    memcpy(finalFrame, templateFrame, templateSize);

    finalFrame[templateSize] = dataValue >> 8;
    finalFrame[templateSize + 1] = dataValue & 0xFF;

    unsigned int crc = ModRTU_CRC(finalFrame, templateSize + 2);

    finalFrame[templateSize + 2] = crc & 0xFF;
    finalFrame[templateSize + 3] = crc >> 8;

    Serial.print("[MODBUS TX]: ");
    for (size_t i = 0; i < finalFrameSize; i++)
    {
        Serial.printf("%02X ", finalFrame[i]);
    }
    Serial.println();

    send_to_serial(finalFrame, finalFrameSize);

    char jsonMsg[50];
    sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", commandType, commandValue);

    client.publish(mqtt_topic_cmd, jsonMsg);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(LED_PIN,HIGH);
}

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
    return (crc << 8) | (crc >> 8); 
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
    machineInfo.runCount = (buffer[3 + 72] << 8) | buffer[3 + 73];
    uint16_t conditionValue = (buffer[3 + 66] << 8) | buffer[3 + 67];
    if (conditionValue > 8)
    {
        machineInfo.coinsInBox = (buffer[3 + 68] << 8) | buffer[3 + 69]; 
    }
    else
    {
        machineInfo.coinsInBox = conditionValue;
    }

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
                return 0; 
        }
    }
    return -1; 
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
    sprintf(mqtt_topic_ota_status, "Kdev/%s/ota_status", deviceID);

    Serial.println("--- MQTT Topics Đã Xây Dựng ---");
    Serial.printf("Info: %s\n", mqtt_topic_info);
    Serial.printf("Control: %s\n", mqtt_topic_control);
    Serial.printf("OTA Status: %s\n", mqtt_topic_ota_status);
    Serial.println("-------------------------------");
}

void saveAndRestart(const char* key, String value)
{
    saveConfigDataToEEPROM(key, value.c_str());

    Serial.printf("Đã lưu %s = %s vào EEPROM\n", key, value.c_str());
    Serial.println("Khởi động lại sau 2 giây...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP.restart();
}

void loadConfig()
{
    readConfigDataFromEEPROM(); 
    
    String id = String(sys_eeprom_deviceID);
    String server = String(sys_eeprom_mqttServer);
    String typeStr = String(sys_eeprom_machineType);
    
    if (id.length() == 0) {
        id = "1234";
    }
    if (server.length() == 0) {
        server = "devices.koisolutions.vn";
    }
    if (typeStr.length() == 0) {
        typeStr = "1";
    }
    
    machineType = typeStr.toInt(); 
    if (machineType != 1 && machineType != 2)
    {
        machineType = 1; 
    }
    
    strcpy(deviceID, id.c_str());
    strcpy(mqttServer, server.c_str());

    Serial.println("--- Cấu Hình Đã Tải (từ EEPROM) ---");
    Serial.printf("Device ID: %s\n", deviceID);
    Serial.printf("MQTT Server: %s\n", mqttServer);
    Serial.printf("Machine Type: %d (%s)\n", machineType, (machineType == 1) ? "Modbus" : "Coin Pin");
    Serial.println("-------------------------");

    buildMqttTopics();
}

int MQTT_Publish_Status(const char* _t, const char* _s) {
    StaticJsonDocument<128> doc;
    if (_t != "null") doc["t"] = _t;
    if (_s != "null") doc["v"] = _s;
    
    char strPub[128];
    size_t len = serializeJson(doc, strPub, sizeof(strPub));
    
    if (client.connected()) {
        client.publish(mqtt_topic_cmd, strPub, len); 
        return 1;
    } else {
        return 0;
    }
}

void printPercent(uint32_t readLength, uint32_t contentLength) {
    if (contentLength != 0) {
        Serial.print("\r [OTA] ");
        Serial.print((100.0 * readLength) / contentLength);
        Serial.print('%');
    } else {
        Serial.println(readLength);
    }
}

int downloadFirmWare(String server, String resource)
{
    const char* firmware_filename = "/firmware.bin"; 

    if (SPIFFS.exists(firmware_filename)) {
        SPIFFS.remove(firmware_filename);
        Serial.println("[OTA] Đã xóa tệp firmware cũ.");
    }

    HTTPClient http;
    WiFiClientSecure client_for_http;
    client_for_http.setInsecure();

    String full_url = "https://" + server + "/" + resource;
    Serial.print(F("[OTA] Đang kết nối: "));
    Serial.println(full_url);

    if (!http.begin(client_for_http, full_url)) {
        Serial.println("[OTA] Lỗi khởi tạo HTTPClient.");
        return 0;
    }

    http.addHeader("User-Agent", "ESP32-OTA-Downloader");
    http.setTimeout(10000); 
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) { 
        Serial.printf("[OTA] HTTP GET thất bại: %d\n", httpCode);
        http.end();
        return 0;
    }

    int contentLength = http.getSize();
    Serial.printf("[OTA] Kích thước file: %d bytes\n", contentLength);

    File file = SPIFFS.open(firmware_filename, FILE_WRITE);
    if (!file) {
        Serial.println(F("[OTA] Lỗi mở file SPIFFS"));
        http.end();
        return 0;
    }

    WiFiClient* stream = http.getStreamPtr();

    uint32_t readLength = 0;
    CRC32 crc;
    unsigned long timeout = millis();
    unsigned long lastUpdate = millis();
    
    // Buffer 2KB
    uint8_t buff[2048] = { 0 };
    
    printPercent(readLength, contentLength);

    while (http.connected() && (readLength < contentLength || contentLength == -1))
    {
        if (millis() - timeout > 10000L) {
            Serial.println("\n[OTA] Lỗi: Timeout khi tải dữ liệu!");
            break;
        }

        size_t size = stream->available();
        if (size > 0) 
        {
            int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
            file.write(buff, c);
            for(int i=0; i<c; i++) {
                crc.update(buff[i]);
            }
            readLength += c;
            timeout = millis();
            if (millis() - lastUpdate > 200) {
                printPercent(readLength, contentLength);
                lastUpdate = millis();
            }
        }
        else {
             vTaskDelay(pdMS_TO_TICKS(1)); 
        }
    }

    file.close();
    http.end(); 
    Serial.println(); 
    Serial.print("[OTA] Tải xong. CRC32: 0x"); Serial.println(crc.finalize(), HEX);

    if (readLength != contentLength && contentLength != -1) {
        Serial.printf("[OTA] Lỗi: Chỉ tải được %d / %d bytes\n", readLength, contentLength);
        return 0;
    }
    Serial.println(F("[OTA] Tải tệp thành công."));
    return 1;
}

int updateFirmWare() {
    const char* firmware_filename = "/firmware.bin";

    File file = SPIFFS.open(firmware_filename);
    if (!file) {
        Serial.println("[OTA] Không thể mở tệp /firmware.bin");
        return 0;
    }
    size_t fileSize = file.size();
    if (fileSize == 0) {
        Serial.println("[OTA] Lỗi: Tệp firmware rỗng.");
        file.close();
        return 0;
    }
    Serial.println("[OTA] Bắt đầu quá trình cập nhật...");
    if (!Update.begin(fileSize)) {
        Serial.println("[OTA] Không đủ dung lượng để cập nhật");
        file.close();
        return 0;
    };
    Update.writeStream(file);
    if (Update.end()) {
        Serial.println("[OTA] Cập nhật thành công!");
        file.close();
        SPIFFS.remove(firmware_filename); 
        return 1;
    } else {
        Serial.println("Lỗi Occurred: " + String(Update.getError()));
        file.close();
        return 0;
    }
}

void checkSerialCommands()
{
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r') 
        {
            serialCmdBuffer.trim(); 
            if (serialCmdBuffer.length() > 0)
            {
                Serial.printf("[Serial CMD] Nhận được: %s\n", serialCmdBuffer.c_str());

                if (serialCmdBuffer.startsWith("@"))
                {
                    String command = serialCmdBuffer.substring(1); 
                    int firstSlash = command.indexOf('/');
                    int secondSlash = command.indexOf('/', firstSlash + 1); 

                    if (firstSlash > 0 && secondSlash > (firstSlash + 1))
                    {
                        String newID = command.substring(0, firstSlash);
                        String newServer = command.substring(firstSlash + 1, secondSlash);
                        String newTypeStr = command.substring(secondSlash + 1);

                        int newType = newTypeStr.toInt();
                        bool idValid = newID.length() > 0 && newID.length() < MAX_DEVICE_ID_LENGTH;
                        bool serverValid = newServer.length() > 0 && newServer.length() < MAX_MQTT_SERVER_LENGTH;
                        bool typeValid = (newType == 1 || newType == 2); 

                        if (idValid && serverValid && typeValid)
                        {
                            Serial.println("OK: Cấu hình hợp lệ. Đang tiến hành lưu...");
                            saveConfigDataToEEPROM("deviceID", newID.c_str());
                            saveConfigDataToEEPROM("mqttServer", newServer.c_str());
                            saveConfigDataToEEPROM("machineType", newTypeStr.c_str());
                            Serial.println("Lưu hoàn tất! Sẽ khởi động lại trong 2 giây...");
                            delay(2000);
                            ESP.restart(); 
                        }
                        else
                        {
                            Serial.println("Lỗi: Cấu hình không hợp lệ.");
                        }
                    }
                    else
                    {
                        Serial.println("Lỗi: Định dạng sai. Phải là: @ID/SERVER/TYPE");
                    }
                }
                else
                {
                    Serial.println("Lỗi: Lệnh không xác định. Lệnh cấu hình phải bắt đầu bằng @");
                }
            }
            serialCmdBuffer = ""; 
        }
        else if (isPrintable(c)) 
        {
            if (serialCmdBuffer.length() < (MAX_DEVICE_ID_LENGTH + MAX_MQTT_SERVER_LENGTH + MAX_MACHINE_TYPE_LENGTH + 10))
            {
                 serialCmdBuffer += c;
            }
        }
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';
    Serial.printf("Message arrived on topic: %s. Message: %s\n", topic, message);

    if (strcmp(topic, mqtt_topic_control) == 0)
    {
        StaticJsonDocument<256> doc; 
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
            if (g_otaInProgress && strcmp(t_value, "f_update") != 0) {
                Serial.println("[OTA] Đang bận cập nhật, từ chối lệnh xen ngang!");
                MQTT_Publish_Status(t_value, "busy_ota");
                return; 
            }

            if (strcmp(t_value, "c") == 0 || strcmp(t_value, "p") == 0)
            {
                Serial.println("[Control] Lệnh 'c'/'p'. Dùng Modbus.");
                sendMobus(t_value, v_value);
            }
            else if (strcmp(t_value, "m") == 0)
            {
                if (machineType == 1)
                {
                    Serial.printf("[Control] Lệnh 'm' (Type 1). Dùng Modbus (v=%s).\n", v_value);
                    sendMobus(t_value, v_value);
                }
                else if (machineType == 2)
                {
                    Serial.printf("[Control] Lệnh 'm' (Type 2). Dùng Coin Pin (v=%s).\n", v_value);
                    char jsonMsg[50];
                    sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", t_value, v_value);
                    client.publish(mqtt_topic_cmd, jsonMsg);
                    int count = atoi(v_value);                  
                    for (int i=0;i<count;i++)
                    {
                        digitalWrite(COIN_PIN,HIGH);
                        vTaskDelay(100);
                        digitalWrite(COIN_PIN,LOW);
                        vTaskDelay(100);
                    }
                    digitalWrite(LED_PIN, LOW);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    digitalWrite(LED_PIN, HIGH);
                }
            }
            else if (strcmp(t_value, "s") == 0 && strcmp(v_value, "1") == 0)
            {
                char jsonMsg[50];
                sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", t_value, v_value);
                client.publish(mqtt_topic_cmd, jsonMsg); 
                
                // Logic đọc tức thời khi nhận lệnh 's'
                int currentStatus = getMachineStatus();
                vTaskDelay(pdMS_TO_TICKS(200));
                while (Serial.available()) Serial.read();
                bool infoReadSuccess = readAndParseMachineData();
                
                StaticJsonDocument<256> doc_resp;
                doc_resp["s"] = (currentStatus == 1) ? 1 : 0;
                if (digitalRead(IN_SIG2)){
                    doc_resp["s"] = 1;
                }
                if (infoReadSuccess)
                {
                    char v_buffer[100];
                    sprintf(v_buffer, "%d,0,%d,0", machineInfo.temperature,machineInfo.coinsInBox);
                    doc_resp["v"] = v_buffer;
                    doc_resp["st"] = 0; 
                }
                else
                {
                    doc_resp["v"] = "0,0,0,0";
                    doc_resp["st"] = "02";
                }
                // Sửa lỗi logic gán (=) thành so sánh (==)
                if (machineType == 2){
                    doc_resp["st"] = "0";
                }
                char jsonBuffer[256];
                serializeJson(doc_resp, jsonBuffer);
                client.publish(mqtt_topic_info, jsonBuffer);
                digitalWrite(LED_PIN, LOW);
                digitalWrite(LED_PIN,HIGH);
            }
            else if (strcmp(t_value, "i") == 0 && strcmp(v_value, "1") == 0)
            {
                char jsonMsg[50];
                sprintf(jsonMsg, "{\"t\":\"%s\", \"v\":\"%s\"}", t_value, v_value);
                client.publish(mqtt_topic_cmd, jsonMsg);
                client.publish(mqtt_topic_info, FIRMWARE_VERSION);
            }
            else if (strcmp(t_value, "f_update") == 0)
            {
                if (g_otaInProgress) 
                {
                    Serial.println("[OTA] Đang bận cập nhật, từ chối lệnh!");
                    MQTT_Publish_Status("f_update", "busy_ota");
                }
                else
                {
                    Serial.println("[OTA] Nhận lệnh cập nhật...");
                    g_otaInProgress = true; 
                    strncpy(g_otaUrl, v_value, sizeof(g_otaUrl) - 1);
                    g_otaUrl[sizeof(g_otaUrl) - 1] = '\0'; 
                    MQTT_Publish_Status("f_update", "starting");
                    if (task6_handle != NULL) vTaskResume(task6_handle);
                }
            }
        }
    }
}

void task1Function(void *parameter)
{
    while (true)
    {
        sys_capserver_proc();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void task2Function(void *parameter)
{
    pinMode(LED_PIN, OUTPUT);
    long lastPingTime = 0;
    const int pingInterval = 90000;
    while (true)
    {
        switch (wifiState)
        {
        case WIFI_CONNECTED:
            if (!connectWifiPing)
            {
                digitalWrite(LED_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(300));
                digitalWrite(LED_PIN, HIGH); connectWifiPing = true;
            }
            if (millis() - lastPingTime > pingInterval)
            {
                if (!Ping.ping("www.google.com", 1)) wifiState = WIFI_CONFIGURED_NOT_CONNECTED;
                lastPingTime = millis();
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        case WIFI_CONFIGURED_NOT_CONNECTED:
            digitalWrite(LED_PIN, !digitalRead(LED_PIN)); vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        case WIFI_NOT_CONFIGURED:
            digitalWrite(LED_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(200)); connectWifiPing = false;
            break;
        }
    }
}

void task3Function(void *parameter)
{
    unsigned long task3_lastBlinkTime = 0;
    const int RESET_HOLD_TIME = 5000; 
    while (true)
    {
        if (digitalRead(RST_PIN) == LOW) 
        {
            if (!press) { press = true; pressTime = millis(); Serial.println("[Task 3] Bắt đầu giữ nút RST..."); }
            if (press && (millis() - pressTime >= RESET_HOLD_TIME))
            {
                Serial.println("[Task 3] >>> Nut RST giu 5s, xoa WiFi <<<");
                clearWiFiCredentials(); press = false; 
                digitalWrite(LED_PIN,HIGH); vTaskDelay(pdMS_TO_TICKS(100)); digitalWrite(LED_PIN,LOW);
            }
        }
        else { if (press) digitalWrite(LED_PIN, HIGH); press = false; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- [QUAN TRỌNG] TASK 5 KHÔI PHỤC LOGIC CŨ + BẢO VỆ OTA ---
void task5Function(void *parameter)
{
    long lastMsgTime = 0;
    const int publishInterval = 3000;
    long lastMqttAttempt = 0;
    long lastWifiReconnectAttempt = 0;
    const int wifiReconnectInterval = 15000;

    while (true)
    {
        checkSerialCommands(); // Kiểm tra lệnh từ Serial

        // 1. Kiểm tra Wifi
        if (WiFi.status() == WL_CONNECTED)
        {
            if (wifiState != WIFI_CONNECTED)
            {
                wifiState = WIFI_CONNECTED;
            }

            // 2. Kiểm tra MQTT Client
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
                // MQTT đã kết nối -> Xử lý gửi tin
                client.loop();
                
                if (millis() - lastMsgTime > publishInterval)
                {
                    if (g_otaInProgress) {
                         vTaskDelay(10); // Ngủ đông chờ OTA xong
                    } else {
                        lastMsgTime = millis();

                        // Xóa buffer rác trước khi xử lý
                        while (Serial.available()) Serial.read();

                        // --- KHỞI TẠO JSON ---
                        StaticJsonDocument<256> doc;

                        // ==========================================
                        // LOGIC CHO MACHINE TYPE 1
                        // ==========================================
                        if (machineType == 1) 
                        {
                            int currentStatus = getMachineStatus();
                            
                            // Xóa buffer lần nữa trước khi đọc data dài
                            while (Serial.available()) Serial.read();

                            bool infoReadSuccess = readAndParseMachineData();

                            // Gán trạng thái S (Status hoạt động)
                            doc["s"] = (currentStatus == 1) ? 1 : 0;

                            // Gán trạng thái ST (Status lỗi/kết nối)
                            // Nếu đọc thành công và trạng thái máy hợp lệ -> st = 0 (OK)
                            if (infoReadSuccess && currentStatus != -1) {
                                 doc["st"] = 0; 
                            } else {
                                 doc["st"] = "02"; // Lỗi kết nối mạch điều khiển
                            }

                            // Gán giá trị V (Data)
                            if (infoReadSuccess)
                            {
                                char v_buffer[100];
                                sprintf(v_buffer, "%d,0,%d,0", machineInfo.temperature, machineInfo.coinsInBox);
                                doc["v"] = v_buffer;
                            }
                            else
                            {
                                doc["v"] = "0,0,0,0";
                            }
                        }
                        // ==========================================
                        // LOGIC CHO MACHINE TYPE 2 (Đã sửa lỗi cú pháp)
                        // ==========================================
                        else if (machineType == 2)
                        {
                            // Đọc tín hiệu Input
                            if (digitalRead(IN_SIG2) == HIGH) {
                                doc["s"] = 1;
                            } else {
                                doc["s"] = 0;
                            }
                            
                            // LƯU Ý: Bạn đang set cứng st="02" (thường là lỗi). 
                            // Nếu muốn báo OK thì sửa thành doc["st"] = 0;
                            doc["st"] = "02"; 

                            // NÊN THÊM: Gán giá trị mặc định cho v để JSON đủ trường
                            doc["v"] = "0,0,0,0"; 
                        }

                        // ==========================================
                        // GỬI MQTT
                        // ==========================================
                        char jsonBuffer[256];
                        serializeJson(doc, jsonBuffer);
                        client.publish(mqtt_topic_info, jsonBuffer);
                        
                        // Nháy đèn báo hiệu đã gửi
                        digitalWrite(LED_PIN, LOW);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        digitalWrite(LED_PIN, HIGH);
                        
                        Serial.print("Published info: ");
                        Serial.println(jsonBuffer);
                    }
                }
            }
        }
        else if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED)
        {
            if (millis() - lastWifiReconnectAttempt > wifiReconnectInterval)
            {
                WiFi.reconnect();
                lastWifiReconnectAttempt = millis();
            }
        }
        
        // Delay cho Task để tránh chiếm dụng CPU
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void task6Function(void *parameter)
{
    Serial.println("[Task 6] Khởi tạo. Sẵn sàng nhận lệnh OTA.");
    while (true)
    {
        vTaskSuspend(NULL);
        Serial.println("[Task 6] Đã được kích hoạt. Bắt đầu quá trình OTA...");
        client.publish(mqtt_topic_ota_status, "starting");
        
        String cmd_firmware = String(g_otaUrl); 
        String _Data[2]; 
        int firstSlash = cmd_firmware.indexOf('/');
        if (firstSlash == -1 || firstSlash == 0) {
            client.publish(mqtt_topic_ota_status, "failed_format");
            g_otaInProgress = false; continue; 
        }
        _Data[0] = cmd_firmware.substring(0, firstSlash);
        _Data[1] = cmd_firmware.substring(firstSlash + 1);

        int ret = downloadFirmWare(_Data[0], _Data[1]);
        if (ret == 0) {
            client.publish(mqtt_topic_ota_status, "failed_download");
            g_otaInProgress = false; continue; 
        }
        
        ret = updateFirmWare();
        if (ret == 1) {
            client.publish(mqtt_topic_ota_status, "success");
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP.restart(); 
        } else {
            client.publish(mqtt_topic_ota_status, "failed_update");
            g_otaInProgress = false; continue; 
        }
    }
}

void setup()
{
    Serial.begin(9600, SERIAL_8N1);
    delay(1000);
    if(!SPIFFS.begin(true)){ return; }
    
    EEPROM.begin(512); 
    loadConfig(); 
    sys_wifi_init(); 
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);
    pinMode(RST_PIN, INPUT_PULLUP);
    pinMode(IN_SIG2, INPUT_PULLDOWN);
    pinMode(LED_PIN, OUTPUT);
    pinMode(COIN_PIN, OUTPUT);

    pinMode(OUT_SIG1, OUTPUT);
    digitalWrite(OUT_SIG1, LOW);

    client.setServer(mqttServer, mqtt_port);
    client.setCallback(mqttCallback);

    xTaskCreate(task1Function, "Task 1", 10000, NULL, 1, NULL); 
    xTaskCreate(task2Function, "Task 2", 10000, NULL, 1, NULL); 
    xTaskCreate(task3Function, "Task 3", 3000, NULL, 1, NULL); 
    xTaskCreate(task5Function, "Task 5", 10000, NULL, 1, NULL); 
    xTaskCreate(task6Function, "Task 6 OTA", 8192, NULL, 2, &task6_handle); 
}

void loop() 
{

}