#include <EEPROM.h>
#include <sys_eeprom.hpp>

char sys_eeprom_ssid[MAX_SSID_LENGTH];
char sys_eeprom_password[MAX_PASSWORD_LENGTH];
char sys_eeprom_device_id[MAX_DEVICE_ID_LENGTH];
char sys_eeprom_server[MAX_SERVER_LENGTH];

void clearWiFiCredentialsInEEPROM() {
    
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
void readConfigFromEEPROM()
{

    // Read DEVICE_ID
    for (int i = 0; i < MAX_DEVICE_ID_LENGTH; i++)
    {
        sys_eeprom_device_id[i] = EEPROM.read(DEVICE_ID_EEPROM_ADDR + i);
        if (sys_eeprom_device_id[i] == '\0')
        {
            break;
        }
    }
    
    sys_eeprom_device_id[MAX_DEVICE_ID_LENGTH - 1] = '\0';

    for (int i = 0; i < MAX_SERVER_LENGTH; i++)
    {
        sys_eeprom_server[i] = EEPROM.read(SERVER_EEPROM_ADDR + i);
        if (sys_eeprom_server[i] == '\0')
        {
            break;
        }
    }
    sys_eeprom_server[MAX_SERVER_LENGTH - 1] = '\0';

}
void readWiFiCredentialsFromEEPROM()
{
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
void saveConfigToEEPROM(const char *device_id, const char *mqtt_server)
{
    
    // Ghi DEVICE_ID
    int id_len = strnlen(device_id, MAX_DEVICE_ID_LENGTH - 1); // Tránh tràn bộ nhớ
    for (int i = 0; i < MAX_DEVICE_ID_LENGTH; i++)
    {
        char data = (i <= id_len) ? device_id[i] : '\0'; // Thêm null sau khi ghi
        EEPROM.write(DEVICE_ID_EEPROM_ADDR + i, data);
        sys_eeprom_device_id[i] = data;
        if (data == '\0') break;
    }

    // Ghi MQTT_SERVER
    int server_len = strnlen(mqtt_server, MAX_SERVER_LENGTH - 1);
    for (int i = 0; i < MAX_SERVER_LENGTH; i++)
    {
        char data = (i <= server_len) ? mqtt_server[i] : '\0';
        EEPROM.write(SERVER_EEPROM_ADDR + i, data);
        sys_eeprom_server[i] = data;
        if (data == '\0') break;
    }

    EEPROM.commit(); // Ghi dữ liệu thực sự vào Flash
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

