#include <EEPROM.h>
#include <sys_eeprom.hpp>

char sys_eeprom_ssid[MAX_SSID_LENGTH];
char sys_eeprom_password[MAX_PASSWORD_LENGTH];

void clearWiFiCredentialsInEEPROM() {
    EEPROM.begin(512);
    
    // Clear SSID
    for (int i = 0; i < MAX_SSID_LENGTH; i++) {
        EEPROM.write(SSID_EEPROM_ADDR + i, '\0');
    }

    // Clear password
    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++) {
        EEPROM.write(PASSWORD_EEPROM_ADDR + i, '\0');
    }

    EEPROM.commit();
}


void readWiFiCredentialsFromEEPROM()
{
    EEPROM.begin(512);
    for (int i = 0; i < MAX_SSID_LENGTH; i++)
    {
        sys_eeprom_ssid[i] = EEPROM.read(SSID_EEPROM_ADDR + i);
        if (sys_eeprom_ssid[i] == '\0')
        {
            break;
        }
    }

    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++)
    {
        sys_eeprom_password[i] = EEPROM.read(PASSWORD_EEPROM_ADDR + i);
        if (sys_eeprom_password[i] == '\0')
        {
            break;
        }
    }
}

void saveWiFiCredentialsToEEPROM(const char *ssid, const char *password)
{
    for (int i = 0; i < MAX_SSID_LENGTH; i++)
    {
        EEPROM.write(SSID_EEPROM_ADDR + i, ssid[i]);
        sys_eeprom_ssid[i] = ssid[i];
        if (ssid[i] == '\0')
        {
            break;
        }
    }

    for (int i = 0; i < MAX_PASSWORD_LENGTH; i++)
    {
        EEPROM.write(PASSWORD_EEPROM_ADDR + i, password[i]);
        sys_eeprom_password[i] = password[i];
        if (password[i] == '\0')
        {
            break;
        }
    }

    EEPROM.commit();
}

