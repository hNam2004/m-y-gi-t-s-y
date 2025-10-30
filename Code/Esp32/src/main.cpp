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
#include "readbyte.hpp"

#define DEVICE_ID "1234"

const char *mqtt_server = "devices.koisolutions.vn";
const int mqtt_port = 7183;
const char *FIRMWARE_VERSION = "1.0.0";

const char *mqtt_topic_info = "Kdev/" DEVICE_ID "/info";
const char *mqtt_topic_cmd = "Kdev/" DEVICE_ID "/cmd";
const char *mqtt_topic_status = "Kdev/" DEVICE_ID "/status";
const char *mqtt_topic_control = "Kdev/" DEVICE_ID "/control";

#define BOOT_PIN 99
#define RST_PIN 23
#define LED_PIN 21
#define COIN_PIN 12
#define IN_SIG2 18

WiFiClient espClient;
PubSubClient client(espClient);
unsigned int pressTime = 0;
bool press = false;
TaskHandle_t task2_handle = NULL;
volatile uint8_t Interupt_Flag = 0;
volatile bool connectWifiPing = false;


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
            
                    digitalWrite(COIN_PIN, LOW);
                }
                else if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "2") == 0)
                {
                    digitalWrite(COIN_PIN, HIGH);
                    vTaskDelay(pdMS_TO_TICKS(100)); // Nghỉ 5 giây
                    digitalWrite(COIN_PIN, LOW);
                }
                else if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "3") == 0)
                {
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
                    // DÙNG BIẾN 'machineInfo' (từ readbyte.cpp)
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

// ... (Các hàm task1, task2, task3, bootInterruptHandler giữ nguyên) ...
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



// --- Task 5 (MQTT & Modbus Polling) ---
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

                    while (Serial.available())
                        Serial.read();

                    // GỌI HÀM TỪ readbyte.cpp
                    int currentStatus = getMachineStatus();

                    while (Serial.available())
                        Serial.read();

                    // GỌI HÀM TỪ readbyte.cpp
                    bool infoReadSuccess = readAndParseMachineData();

                    StaticJsonDocument<256> doc;
                    doc["s"] = (currentStatus == 1) ? 1 : 0;
                    if (infoReadSuccess)
                    {
                        char v_buffer[100];
                        // DÙNG BIẾN 'machineInfo' (từ readbyte.cpp)
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

    xTaskCreate(task1Function, "Task 1", 10000, NULL, 1, NULL); 
    xTaskCreate(task2Function, "Task 2", 10000, NULL, 1, NULL); 
    xTaskCreate(task3Function, "Task 3", 10000, NULL, 1, NULL); 
    xTaskCreate(task5Function, "Task 5", 10000, NULL, 1, NULL); 
}

void loop()
{
}