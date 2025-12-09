#include "sys_wifi.hpp"
#include "sys_eeprom.hpp"
#include <WiFi.h>
#include <string.h> 
#include <ESP32Ping.h>
#include <Arduino.h>
#include "esp_wifi.h" // [QUAN TRỌNG] Thêm thư viện này để can thiệp sâu vào WiFi
#define LED_PIN 21

WiFiState wifiState = WIFI_NOT_CONFIGURED;

void connectToWiFi(const char *ssid, const char *password)
{
    Serial.println("Connecting to WiFi...");
    
    // [FIX 1] Đảm bảo chuyển về chế độ Station (Khách) hoàn toàn
    WiFi.mode(WIFI_STA); 
    
    // [FIX 2 - RẤT QUAN TRỌNG] Tắt chế độ tiết kiệm năng lượng TRƯỚC khi connect
    // Giúp ESP32 luôn bật Radio, Router sẽ không đá thiết bị ra.
    WiFi.setSleep(false); 
    esp_wifi_set_ps(WIFI_PS_NONE); // Dùng thêm lệnh ESP-IDF cho chắc chắn

    WiFi.begin(ssid, password);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    { 
        // [FIX 3] Dùng vTaskDelay thay vì delay để thân thiện với RTOS hơn
        vTaskDelay(pdMS_TO_TICKS(500)); 
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());

        // Disable AP mode (để tiết kiệm RAM và tránh nhiễu)
        WiFi.softAPdisconnect(true);
        
        // [FIX 4] Nhắc lại lệnh tắt sleep một lần nữa sau khi đã connect thành công
        WiFi.setSleep(false);

        // Turn on built-in LED
        pinMode(LED_PIN, OUTPUT); // Hãy đảm bảo LED_PIN được define hoặc thay bằng số 2
        digitalWrite(LED_PIN, HIGH); 
        wifiState = WIFI_CONNECTED;
    }
    else
    {
        Serial.println("\nWiFi connection FAILED.");
        wifiState = WIFI_CONFIGURED_NOT_CONNECTED;
    }
}

void saveWiFiCredentials(const char *newSSID, const char *newPassword)
{
    saveWiFiCredentialsToEEPROM(newSSID, newPassword);
}
    
void sys_wifi_init()
{
    readWiFiCredentialsFromEEPROM();

    if (strlen(sys_eeprom_ssid) > 0 && strlen(sys_eeprom_password) > 0)
    {
        // Đã có cấu hình WiFi, tiến hành kết nối
        connectToWiFi(sys_eeprom_ssid, sys_eeprom_password);
    }
    else
    {
        Serial.println("No WiFi credentials found. Starting AP mode for configuration.");
        
        // [FIX 5] Khi bật AP, cũng nên set mode rõ ràng là AP Only hoặc AP_STA
        WiFi.mode(WIFI_AP);

        if (sys_eeprom_deviceID[0] == '\0') 
        {
            WiFi.softAP("ESP32_CONFIG_AP"); 
            Serial.println("Warning: deviceID not set. Using default 'ESP32_CONFIG_AP'.");
        } 
        else 
        {
            WiFi.softAP(sys_eeprom_deviceID);
            Serial.printf("Soft AP started: %s\n", sys_eeprom_deviceID);
        }
        
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
        
        // Khi ở chế độ AP, trạng thái là chưa cấu hình
        wifiState = WIFI_NOT_CONFIGURED; 
    }
}

void clearWiFiCredentials()
{
    clearWiFiCredentialsInEEPROM();
    
    // Ngắt kết nối hiện tại
    WiFi.disconnect(true, true); // Xóa config lưu trong flash của ESP (NVS) luôn cho sạch
    WiFi.mode(WIFI_OFF);
    
    wifiState = WIFI_NOT_CONFIGURED;
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); 
    
    // Khởi tạo lại (sẽ vào mode AP do EEPROM đã xóa)
    vTaskDelay(pdMS_TO_TICKS(500));
    sys_wifi_init();
}

bool checkNetworkConnectivity()
{
    // Hàm này OK, không ảnh hưởng lỗi rớt mạng
    const char *testHost = "www.google.com";
    const int pingCount = 2; 

    Serial.printf("[WiFi Check] Pinging %s (%d times)...\n", testHost, pingCount);

    if (Ping.ping(testHost, pingCount)) {
        Serial.println("[WiFi Check] Ping SUCCESS.");
        return true;
    } else {
        Serial.println("[WiFi Check] Ping FAILED.");
        return false; 
    }
}