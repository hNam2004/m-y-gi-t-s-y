#ifndef SYS_WIFI_H
#define SYS_WIFI_H

// [THÊM 1] Cần include Arduino.h để hiểu các kiểu dữ liệu như bool, char...
#include <Arduino.h> 
#include <WiFi.h>

// [THÊM 2] Định nghĩa chân LED
// Vì trong file .cpp mới tôi dùng biến LED_PIN thay vì số cứng.
// Bạn kiểm tra lại mạch thật, nếu dùng chân 2 thì sửa 21 thành 2.
#ifndef LED_PIN
#define LED_PIN 21 
#endif

enum WiFiState {
  WIFI_NOT_CONFIGURED,
  WIFI_CONFIGURED_NOT_CONNECTED,
  WIFI_CONNECTED,
  WIFI_CONNECTING,
};

// Các khai báo hàm (Giữ nguyên như cũ của bạn)
extern bool checkNetworkConnectivity();
extern WiFiState wifiState;
extern void saveWiFiCredentials(const char *newSSID, const char *newPassword);
extern void clearWiFiCredentials();
extern void connectToWiFi(const char *ssid, const char *password);
extern void sys_wifi_init();

#endif