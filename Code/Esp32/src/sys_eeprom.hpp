#ifndef SYS_EEPROM_H
#define SYS_EEPROM_H

#include <Arduino.h>

// ==========================================
// 1. ĐỊNH NGHĨA ĐỘ DÀI (LENGTH)
// ==========================================
#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64

// [ĐÃ SỬA] Đổi tên cho khớp với main.cpp và sys_eeprom.cpp
#define MAX_DEVICE_SERIAL_LENGTH 32 

#define MAX_MQTT_SERVER_LENGTH 100
#define MAX_MACHINE_TYPE_LENGTH 10 

// ==========================================
// 2. ĐỊNH NGHĨA ĐỊA CHỈ (ADDRESS)
// ==========================================
#define ADDR_SSID 0
#define ADDR_PASSWORD (ADDR_SSID + MAX_SSID_LENGTH) // Bắt đầu từ 32

// [ĐÃ SỬA] Đổi tên ADDR_DEVICE_ID -> ADDR_DEVICE_SERIAL
#define ADDR_DEVICE_SERIAL (ADDR_PASSWORD + MAX_PASSWORD_LENGTH) // Bắt đầu từ 96

// [ĐÃ SỬA] Cập nhật công thức tính (dùng MAX_DEVICE_SERIAL_LENGTH)
#define ADDR_MQTT_SERVER (ADDR_DEVICE_SERIAL + MAX_DEVICE_SERIAL_LENGTH) // Bắt đầu từ 128

#define ADDR_MACHINE_TYPE (ADDR_MQTT_SERVER + MAX_MQTT_SERVER_LENGTH) // Bắt đầu từ 228

// ==========================================
// 3. KHAI BÁO BIẾN TOÀN CỤC (EXTERN)
// ==========================================
extern char sys_eeprom_ssid[MAX_SSID_LENGTH];
extern char sys_eeprom_password[MAX_PASSWORD_LENGTH];

// [ĐÃ SỬA] Đổi tên biến extern thành deviceSerial
extern char sys_eeprom_deviceSerial[MAX_DEVICE_SERIAL_LENGTH];

extern char sys_eeprom_mqttServer[MAX_MQTT_SERVER_LENGTH];
extern char sys_eeprom_machineType[MAX_MACHINE_TYPE_LENGTH];

// ==========================================
// 4. KHAI BÁO HÀM (FUNCTIONS)
// ==========================================
extern void readWiFiCredentialsFromEEPROM();
extern void clearWiFiCredentialsInEEPROM();
extern void saveWiFiCredentialsToEEPROM(const char *ssid, const char *password);

// Hàm kiểm tra mạng (nếu có định nghĩa trong cpp)
extern bool checkNetworkConnectivity();

extern void readConfigDataFromEEPROM();
extern void saveConfigDataToEEPROM(const char *key, const char *value);

#endif