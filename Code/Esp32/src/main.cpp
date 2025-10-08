#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PubSubClient.h>
#include "sys_capserver.hpp"
#include "sys_wifi.hpp"
#include <Wire.h>

const char *mqtt_ssid = "testuser";
const char *mqtt_password = "wukdeg-9pimty-zomCew";
const char *mqtt_server = "10.102.62.76";
const int mqtt_port = 7183; // MQTT default port
const char *mqtt_topic_info = "Kdev/1234/info";
const char *mqtt_topic_cmd = "Kdev/1234/cmd";
const char *mqtt_topic_status = "Kdev/1234/status";
const char *mqtt_topic_control = "Kdev/1234/control";

#define BOOT_PIN 0
#define RST_PIN 27

WiFiClient espClient;
PubSubClient client(espClient);
volatile uint8_t Interupt_Flag = 0;
QueueHandle_t sensor1Queue;
QueueHandle_t sensor2Queue;
unsigned int pressTime = 0;
bool press = false;
// This function will be called when the BOOT pin transitions from LOW to HIGH (rising edge)
void bootInterruptHandler()
{
    Serial.println("Interupt ocur");
    Interupt_Flag = 1;
}

void task3Function(void *parameter)
{
    while (true)
    {
        sys_capserver_proc();
        if (Interupt_Flag)
        {
            clearWiFiCredentials();
            Interupt_Flag = 0;
        }
        if (wifiState == WIFI_CONNECTED)
        {
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void task4Function(void *parameter)
{
    while (true)
    {
        if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED)
        {
            pinMode(2, OUTPUT);
            digitalWrite(2, !digitalRead(2)); // Toggle the LED pin state
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
void task5Function(void *parameter)
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
            }
            else
            {
                if (millis() - pressTime >= 5000)
                {
                    Serial.println(">>> Nut RST giu 5s, xoa WiFi <<<");
                    clearWiFiCredentials();
                    sys_capserver_proc();

                    press = false;
                }
            }
        }
        else
        {
            press = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // delay để tránh chiếm CPU
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    sys_wifi_init();
    sys_capserver_init();
    pinMode(BOOT_PIN, INPUT_PULLUP);
    pinMode(RST_PIN, INPUT_PULLUP);                                                 // Configure BOOT pin as input with internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), bootInterruptHandler, RISING); // Attach interrupt handler to rising edge of BOOT pin
    Serial.println("All Done!");
    client.setServer(mqtt_server, mqtt_port);

    xTaskCreate(
        task3Function, // Task function
        "Task 3",      // Task name
        10000,         // Stack size (bytes)
        NULL,          // Task parameters
        1,             // Task priority
        NULL           // Task handle
    );
    xTaskCreate(
        task4Function, // Task function
        "Task 4",      // Task name
        10000,         // Stack size (bytes)
        NULL,          // Task parameters
        1,             // Task priority
        NULL           // Task handle
    );
    xTaskCreate(
        task5Function,
        "Task 5",
        10000,
        NULL,
        1,
        NULL);
}

void loop()
{
}
