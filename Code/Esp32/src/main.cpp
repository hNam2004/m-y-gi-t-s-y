
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
// --- Khai báo biến toàn cục ---
WiFiClient espClient;
PubSubClient client(espClient);
unsigned int pressTime = 0;
bool press = false;
TaskHandle_t task2_handle = NULL;
volatile uint8_t Interupt_Flag = 0;
volatile bool coinPulseLoopActive = false; // Cờ để bật/tắt vòng lặp
volatile bool connectWifiPing = false;
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

    // if (strcmp(topic, mqtt_topic_cmd) == 0)                   ///Đoạn lệnh điều khiển khi nào Hiến xong thì copy vào
    // {
    //     if (strcmp(message, "1") == 0)
    //     {
    //         client.publish(mqtt_topic_cmd, "{\"t\":\"c\",\"v\":\"value\"}");
    //     }
    //     if (strcmp(message, "2") == 0)
    //     {
    //         client.publish(mqtt_topic_cmd, "{\"t\":\"p\",\"v\":\"value\"}");
    //     }
    //     if (strcmp(message, "3") == 0)
    //     {
    //         client.publish(mqtt_topic_cmd, "{\"t\":\"m\",\"v\":\"value\"}");
    //     }
    // }
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
        }
        else if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "2") == 0)
        {
            coinPulseLoopActive = true;
        }
        else if (t_value && v_value && strcmp(t_value, "m") == 0 && strcmp(v_value, "3") == 0)
        {
            coinPulseLoopActive = false;
            digitalWrite(COIN_PIN, LOW);
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
                        client.publish(mqtt_topic_status, FIRMWARE_VERSION, true);
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
                    float temp_value = 0;
                    int coin_value = 0;
                    int cycles_value = 0;
                    lastMsgTime = now;

                    // Gửi thông tin thiết bị (dưới dạng JSON)
                    StaticJsonDocument<200> doc;
                    doc["s"] = 0;
                    String v_value = String(temp_value) + "," + String(coin_value) + "," + String(cycles_value);
                    doc["v"] = v_value;
                    doc["st"] = 0;
                    char jsonBuffer[512];
                    serializeJson(doc, jsonBuffer);
                    client.publish(mqtt_topic_info, jsonBuffer);

                    Serial.print("Published info: ");
                    Serial.println(jsonBuffer);

                    // Nháy đèn báo hiệu đã gửi tin
                    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                    vTaskDelay(pdMS_TO_TICKS(100)); // Nháy nhanh
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
    Serial.begin(115200);
    delay(1000);
    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);
    pinMode(RST_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);                                                       // Configure BOOT pin as input with internal pull-up resistor
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
