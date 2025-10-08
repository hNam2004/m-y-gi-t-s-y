#ifndef SYS_WIFI_H
#define SYS_WIFI_H

enum WiFiState {
  WIFI_NOT_CONFIGURED,
  WIFI_CONFIGURED_NOT_CONNECTED,
  WIFI_CONNECTED
};

extern WiFiState wifiState;
extern void saveWiFiCredentials(const char *newSSID, const char *newPassword);
extern void clearWiFiCredentials();
extern void connectToWiFi(const char *ssid, const char *password);
extern void sys_wifi_init();
#endif