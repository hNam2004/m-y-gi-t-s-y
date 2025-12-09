#include <EEPROM.h>
#include <sys_eeprom.hpp>
#include <string.h> // Cần cho strcmp

// --- Định nghĩa các biến toàn cục ---
char sys_eeprom_ssid[MAX_SSID_LENGTH];
char sys_eeprom_password[MAX_PASSWORD_LENGTH];
char sys_eeprom_deviceID[MAX_DEVICE_ID_LENGTH];
char sys_eeprom_mqttServer[MAX_MQTT_SERVER_LENGTH];
char sys_eeprom_machineType[MAX_MACHINE_TYPE_LENGTH];


// === CÁC HÀM WIFI (Từ code của bạn) ===

void clearWiFiCredentialsInEEPROM() {
    // EEPROM.begin(512) đã được gọi trong setup() của main.cpp
    
    // Clear SSID
    for (int i = 0; i < MAX_SSID_LENGTH; i++) {
        EEPROM.write(ADDR_SSID + i, '\0');
    }
    // Clear password
    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++) {
        EEPROM.write(ADDR_PASSWORD + i, '\0');
    }
    EEPROM.commit();
}

void readWiFiCredentialsFromEEPROM()
{
    for (int i = 0; i < MAX_SSID_LENGTH; i++)
    {
        sys_eeprom_ssid[i] = EEPROM.read(ADDR_SSID + i);
        if (sys_eeprom_ssid[i] == '\0') break;
    }

    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++)
    {
        sys_eeprom_password[i] = EEPROM.read(ADDR_PASSWORD + i);
        if (sys_eeprom_password[i] == '\0') break;
    }
}

void saveWiFiCredentialsToEEPROM(const char *ssid, const char *password)
{
    for (int i = 0; i < MAX_SSID_LENGTH; i++)
    {
        EEPROM.write(ADDR_SSID + i, ssid[i]);
        sys_eeprom_ssid[i] = ssid[i];
        if (ssid[i] == '\0') break;
    }

    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++)
    {
        EEPROM.write(ADDR_PASSWORD + i, password[i]);
        sys_eeprom_password[i] = password[i];
        if (password[i] == '\0') break;
    }
    EEPROM.commit();
}


// === CÁC HÀM CẤU HÌNH (MỚI) ===

/**
 * @brief Đọc Device ID, Server, Type từ EEPROM vào biến toàn cục
 */
void readConfigDataFromEEPROM()
{
    // EEPROM.begin(512) đã được gọi trong setup() của main.cpp
    for (int i = 0; i < MAX_DEVICE_ID_LENGTH; i++) {
        sys_eeprom_deviceID[i] = EEPROM.read(ADDR_DEVICE_ID + i);
        if (sys_eeprom_deviceID[i] == '\0') break;
    }

    for (int i = 0; i < MAX_MQTT_SERVER_LENGTH; i++) {
        sys_eeprom_mqttServer[i] = EEPROM.read(ADDR_MQTT_SERVER + i);
        if (sys_eeprom_mqttServer[i] == '\0') break;
    }

    for (int i = 0; i < MAX_MACHINE_TYPE_LENGTH; i++) {
        sys_eeprom_machineType[i] = EEPROM.read(ADDR_MACHINE_TYPE + i);
        if (sys_eeprom_machineType[i] == '\0') break;
    }
}

/**
 * @brief Lưu một cặp key-value vào đúng địa chỉ EEPROM
 */
void saveConfigDataToEEPROM(const char *key, const char *value)
{
    int addr = -1;
    int maxLen = -1;
    char* globalVar = NULL;

    if (strcmp(key, "deviceID") == 0) {
        addr = ADDR_DEVICE_ID;
        maxLen = MAX_DEVICE_ID_LENGTH;
        globalVar = sys_eeprom_deviceID;
    } else if (strcmp(key, "mqttServer") == 0) {
        addr = ADDR_MQTT_SERVER;
        maxLen = MAX_MQTT_SERVER_LENGTH;
        globalVar = sys_eeprom_mqttServer;
    } else if (strcmp(key, "machineType") == 0) {
        addr = ADDR_MACHINE_TYPE;
        maxLen = MAX_MACHINE_TYPE_LENGTH;
        globalVar = sys_eeprom_machineType;
    }

    if (addr != -1 && globalVar != NULL) {
        for (int i = 0; i < maxLen; i++) {
            EEPROM.write(addr + i, value[i]);
            globalVar[i] = value[i]; // Cập nhật biến global ngay lập tức
            if (value[i] == '\0') break;
        }
        EEPROM.commit();
    }
}
