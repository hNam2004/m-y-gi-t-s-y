#include <sys_wifi.hpp>
#include <sys_eeprom.hpp>
#include <WiFi.h>
#include <string.h> 
#include <ESP32Ping.h> 
#include <Arduino.h>

// --- THÊM DÒNG NÀY: Mật khẩu cho Captive Portal ---
// Lưu ý: Mật khẩu bắt buộc phải >= 8 ký tự. 
// Nếu ngắn hơn, ESP32 sẽ bỏ qua và phát Wifi không pass.
#define AP_CONFIG_PASSWORD  "12345678" 

WiFiState wifiState = WIFI_NOT_CONFIGURED;

void connectToWiFi(const char *ssid, const char *password)
{
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    { 
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());

        // Disable AP mode
        WiFi.softAPdisconnect(true);

        // Turn on built-in LED
        pinMode(2, OUTPUT);
        digitalWrite(2, HIGH); 
        wifiState = WIFI_CONNECTED;
    }
    else
    {
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
        // Không có cấu hình WiFi, bật chế độ AP để cài đặt
        Serial.println("No WiFi credentials found. Starting AP mode for configuration.");
        WiFi.mode(WIFI_AP);

        // --- ĐÂY LÀ PHẦN SỬA ĐỔI ĐỂ CÓ PASSWORD ---

        // Kiểm tra xem deviceSerial trong EEPROM có rỗng không
        if (sys_eeprom_deviceSerial[0] == '\0') 
        {
            // Nếu rỗng, dùng tên mặc định VÀ password
            WiFi.softAP("ESP32_CONFIG_AP", AP_CONFIG_PASSWORD); 
            Serial.println("Warning: deviceSerial not set. Using default AP name 'ESP32_CONFIG_AP' with password.");
        } 
        else 
        {
            // Nếu đã có deviceSerial, dùng nó làm tên AP VÀ password
            WiFi.softAP(sys_eeprom_deviceSerial, AP_CONFIG_PASSWORD);
            Serial.print("Soft AP name set from deviceSerial: ");
            Serial.println(sys_eeprom_deviceSerial);
            Serial.println("Password set: " AP_CONFIG_PASSWORD);
        }
        
        // --- HẾT PHẦN SỬA ĐỔI ---

        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
    }
}

void clearWiFiCredentials()
{
    clearWiFiCredentialsInEEPROM();
    wifiState = WIFI_NOT_CONFIGURED;
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW); 
    sys_wifi_init();
}

bool checkNetworkConnectivity()
{
    const char *testHost = "www.google.com";
    const int pingCount = 2; 

    Serial.printf("[WiFi Check] Pinging %s (%d times)...\n", testHost, pingCount);

    if (Ping.ping(testHost, pingCount)) {
        Serial.println("[WiFi Check] Ping SUCCESS. Network is UP and IP is resolved.");
        return true;
    } else {
        Serial.println("[WiFi Check] Ping FAILED. IP resolution or network access issue.");
        return false; 
    }
}