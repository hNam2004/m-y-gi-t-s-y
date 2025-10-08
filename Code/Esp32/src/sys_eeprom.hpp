#ifndef SYS_EEPROM_H
#define SYS_EEPROM_H

#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64

#define SSID_EEPROM_ADDR 0
#define PASSWORD_EEPROM_ADDR (SSID_EEPROM_ADDR + MAX_SSID_LENGTH)

extern char sys_eeprom_ssid[MAX_SSID_LENGTH];
extern char sys_eeprom_password[MAX_PASSWORD_LENGTH];

extern void readWiFiCredentialsFromEEPROM();
extern void clearWiFiCredentialsInEEPROM();
extern void saveWiFiCredentialsToEEPROM(const char *ssid, const char *password);
#endif