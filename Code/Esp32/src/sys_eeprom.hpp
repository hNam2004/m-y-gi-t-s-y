#ifndef SYS_EEPROM_H
#define SYS_EEPROM_H

#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64
#define MAX_DEVICE_ID_LENGTH 32
#define MAX_MQTT_SERVER_LENGTH 100
#define MAX_MACHINE_TYPE_LENGTH 8 // Đủ để lưu "1" hoặc "2"

#define ADDR_SSID 0
#define ADDR_PASSWORD (ADDR_SSID + MAX_SSID_LENGTH) // Bắt đầu từ 32
#define ADDR_DEVICE_ID (ADDR_PASSWORD + MAX_PASSWORD_LENGTH) // Bắt đầu từ 96
#define ADDR_MQTT_SERVER (ADDR_DEVICE_ID + MAX_DEVICE_ID_LENGTH) // Bắt đầu từ 128
#define ADDR_MACHINE_TYPE (ADDR_MQTT_SERVER + MAX_MQTT_SERVER_LENGTH) // Bắt đầu từ 228

extern char sys_eeprom_ssid[MAX_SSID_LENGTH];
extern char sys_eeprom_password[MAX_PASSWORD_LENGTH];

extern char sys_eeprom_deviceID[MAX_DEVICE_ID_LENGTH];
extern char sys_eeprom_mqttServer[MAX_MQTT_SERVER_LENGTH];
extern char sys_eeprom_machineType[MAX_MACHINE_TYPE_LENGTH];

extern void readWiFiCredentialsFromEEPROM();
extern void clearWiFiCredentialsInEEPROM();
extern void saveWiFiCredentialsToEEPROM(const char *ssid, const char *password);
extern bool checkNetworkConnectivity();
extern void readConfigDataFromEEPROM();
extern void saveConfigDataToEEPROM(const char *key, const char *value);

#endif
