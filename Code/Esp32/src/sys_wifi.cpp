#include <sys_wifi.hpp>
#include <sys_eeprom.hpp>
#include <WiFi.h>

WiFiState wifiState = WIFI_NOT_CONFIGURED;

void connectToWiFi(const char *ssid, const char *password)
{
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    { // Try to connect for up to 10 seconds
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
        digitalWrite(2, HIGH); // Assuming built-in LED is active low
        wifiState = WIFI_CONNECTED;
    }
    
    else
    {
        wifiState = WIFI_CONFIGURED_NOT_CONNECTED;
    }
}

void saveWiFiCredentials(const char *newSSID, const char *newPassword)
{
    // Save new WiFi credentials to EEPROM
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

        // --- ĐÂY LÀ THAY ĐỔI QUAN TRỌNG ---
        
        // Kiểm tra xem deviceID trong EEPROM có rỗng không (ví dụ: lần chạy đầu tiên)
        if (sys_eeprom_deviceID[0] == '\0') 
        {
            // Nếu rỗng, dùng một tên AP mặc định an toàn
            WiFi.softAP("ESP32_CONFIG_AP"); 
            Serial.println("Warning: deviceID not set in EEPROM. Using default AP name 'ESP32_CONFIG_AP'.");
        } 
        else 
        {
            // Nếu đã có deviceID, dùng nó làm tên Soft AP
            WiFi.softAP(sys_eeprom_deviceID);
            Serial.print("Soft AP name set from deviceID: ");
            Serial.println(sys_eeprom_deviceID);
        }
        
        // --- HẾT THAY ĐỔI ---

        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
    }
}

void clearWiFiCredentials()
{
    clearWiFiCredentialsInEEPROM();
    wifiState = WIFI_NOT_CONFIGURED;
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW); // Assuming built-in LED is active low
    sys_wifi_init();
}