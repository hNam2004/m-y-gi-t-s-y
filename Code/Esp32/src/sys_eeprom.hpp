#ifndef SYS_EEPROM_H
#define SYS_EEPROM_H

#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64

#define SSID_EEPROM_ADDR 0
#define PASSWORD_EEPROM_ADDR (SSID_EEPROM_ADDR + MAX_SSID_LENGTH)

#define MAX_DEVICE_ID_LENGTH 32
#define MAX_SERVER_LENGTH 64

#define DEVICE_ID_EEPROM_ADDR   (PASSWORD_EEPROM_ADDR + MAX_DEVICE_ID_LENGTH) 
#define SERVER_EEPROM_ADDR      (DEVICE_ID_EEPROM_ADDR + MAX_DEVICE_ID_LENGTH) 
extern char sys_eeprom_ssid[MAX_SSID_LENGTH];
extern char sys_eeprom_password[MAX_PASSWORD_LENGTH];
extern char sys_eeprom_device_id[MAX_DEVICE_ID_LENGTH];
extern char sys_eeprom_server[MAX_SERVER_LENGTH];

extern void readConfigFromEEPROM();
extern void saveConfigToEEPROM(const char *device_id, const char *mqtt_server);
extern void readWiFiCredentialsFromEEPROM();
extern void clearWiFiCredentialsInEEPROM();
extern void saveWiFiCredentialsToEEPROM(const char *ssid, const char *password);
#endif