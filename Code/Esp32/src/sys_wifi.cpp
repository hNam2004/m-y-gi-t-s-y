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
        connectToWiFi(sys_eeprom_ssid, sys_eeprom_password);
    }
    else
    {
        Serial.println("No WiFi credentials found. Please configure WiFi.");
        Serial.println("\n Restarting AP mode...");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("esp-captive");
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
