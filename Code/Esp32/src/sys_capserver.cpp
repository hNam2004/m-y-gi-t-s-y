#include "sys_capserver.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"
#include "sys_eeprom.hpp"
#include "sys_wifi.hpp"

// --- GIAO DIỆN HTML MỚI (Card Design) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>Kdev Config</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; background-color: #eef2f5; margin: 0; padding: 0; display: flex; justify-content: center; align-items: center; min-height: 100vh; }
    .container { background-color: #fff; width: 100%; max-width: 380px; border-radius: 12px; box-shadow: 0 10px 25px rgba(0,0,0,0.1); overflow: hidden; margin: 20px; }
    .header { background: linear-gradient(135deg, #007bff, #0056b3); padding: 25px; text-align: center; color: white; }
    .header h2 { margin: 0; font-size: 24px; font-weight: 600; }
    .header p { margin: 5px 0 0; font-size: 14px; opacity: 0.8; }
    form { padding: 25px; }
    
    .form-group { margin-bottom: 20px; }
    label { font-weight: 600; color: #333; display: block; margin-bottom: 8px; font-size: 14px; }
    
    input[type="text"], input[type="password"], select { 
      width: 100%; padding: 12px; border: 1px solid #ddd; border-radius: 8px; box-sizing: border-box; font-size: 16px; transition: border-color 0.3s;
      background-color: #fafafa;
    }
    input:focus, select:focus { border-color: #007bff; outline: none; background-color: #fff; }
    
    .divider { height: 1px; background-color: #eee; margin: 25px 0; }
    .section-title { color: #007bff; font-size: 12px; text-transform: uppercase; letter-spacing: 1px; font-weight: bold; margin-bottom: 15px; display: block; }

    input[type="submit"] { 
      background-color: #28a745; color: white; border: none; cursor: pointer; width: 100%; padding: 15px; border-radius: 8px; font-size: 16px; font-weight: bold; text-transform: uppercase; transition: background 0.3s; box-shadow: 0 4px 6px rgba(40, 167, 69, 0.2);
    }
    input[type="submit"]:hover { background-color: #218838; transform: translateY(-1px); }
    input[type="submit"]:active { transform: translateY(1px); }
    
    .note { font-size: 12px; color: #888; margin-top: 5px; font-style: italic; }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h2>KDEV CONFIG</h2>
      <p>System Setup & Configuration</p>
    </div>
    
    <form action="/get">
      
      <span class="section-title">Network Settings</span>
      <div class="form-group">
        <label for="wifi_id">WiFi Name (SSID)</label>
        <input type="text" id="wifi_id" name="wifi_id" placeholder="Ex: MyHome_Wifi" required>
      </div>
      
      <div class="form-group">
        <label for="wifi_password">WiFi Password</label>
        <input type="password" id="wifi_password" name="wifi_password" placeholder="Enter Password">
      </div>

      <div class="divider"></div>

      <span class="section-title">Device Settings</span>
      <div class="form-group">
        <label for="device_id">Device ID</label>
        <input type="text" id="device_id" name="device_id" placeholder="Ex: KDEV_001" required>
      </div>

      <div class="form-group">
        <label for="machine_type">Machine Type</label>
        <select name="machine_type" id="machine_type">
          <option value="1">Type 1 - Modbus System</option>
          <option value="2">Type 2 - Relay System</option>
        </select>
        <div class="note">* Select the operating mode appropriate for the hardware.</div>
      </div>
      
      <input type="submit" value="Save Configuration">
    </form>
  </div>
</body>
</html>)rawliteral";

class CaptiveRequestHandler : public AsyncWebHandler
{
public:
    CaptiveRequestHandler() {}
    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request)
    {
        return true;
    }

    void handleRequest(AsyncWebServerRequest *request)
    {
        request->send_P(200, "text/html", index_html);
    }
};

DNSServer dnsServer;
AsyncWebServer server(80);

// Biến cờ để báo hiệu Main Loop xử lý WiFi
bool wifi_creds_received = false; 
String input_ssid;
String input_password;

void sys_capserver_init()
{
    // Trang chủ hiển thị form
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      request->send_P(200, "text/html", index_html); 
      Serial.println("Client Connected to Config Page"); });

    // Xử lý khi người dùng nhấn nút Submit
    server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      String inputMessage;
      
      // 1. Xử lý Device ID và lưu EEPROM
      if (request->hasParam("device_id")) {
        inputMessage = request->getParam("device_id")->value();
        saveConfigDataToEEPROM("deviceID", inputMessage.c_str());
        Serial.println("Saved DeviceID: " + inputMessage);
      }

      // 2. Xử lý Machine Type và lưu EEPROM
      if (request->hasParam("machine_type")) {
        inputMessage = request->getParam("machine_type")->value();
        saveConfigDataToEEPROM("machineType", inputMessage.c_str());
        Serial.println("Saved MachineType: " + inputMessage);
      }

      // 3. Xử lý WiFi Credentials
      if (request->hasParam("wifi_id")) {
        input_ssid = request->getParam("wifi_id")->value();
        Serial.println("WiFi ID: " + input_ssid);
      }

      if (request->hasParam("wifi_password")) {
        input_password = request->getParam("wifi_password")->value();
        Serial.println("WiFi Password: " + input_password);
      }
      
      // Kiểm tra xem đã có đủ thông tin WiFi chưa
      if(input_ssid.length() > 0) {
        wifi_creds_received = true; 
        // Phản hồi HTML đơn giản báo thành công
        String success_html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{font-family:sans-serif;text-align:center;padding:50px;}</style></head><body><h1 style='color:green'>Configuration Saved!</h1><p>Device is restarting and connecting to WiFi...</p></body></html>";
        request->send(200, "text/html", success_html);
      } else {
        request->send(400, "text/html", "Error: WiFi SSID is missing.");
      } });

    dnsServer.start(53, "*", WiFi.softAPIP());
    server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);
    server.begin();
}

void sys_capserver_proc()
{
    if (wifiState == WIFI_NOT_CONFIGURED) {
        dnsServer.processNextRequest();
    }
    
    // Nếu nhận được thông tin từ Web
    if (wifi_creds_received)
    {
        wifi_creds_received = false;

        Serial.println("[CapServer] Received credentials. Starting connection attempt...");
        
        // Thử kết nối WiFi
        connectToWiFi(input_ssid.c_str(), input_password.c_str());
        
        if(wifiState == WIFI_CONNECTED){
            Serial.println("[CapServer] WiFi Connected. Checking Internet...");
            
            if (checkNetworkConnectivity()) { 
                // Nếu có mạng -> Lưu WiFi vào Flash (NVS)
                saveWiFiCredentials(input_ssid.c_str(), input_password.c_str());
                Serial.println("[CapServer] WiFi Saved & Verified. RESTARTING ESP32...");
                
                delay(1000);
                ESP.restart(); 
            } else {
                Serial.println("[CapServer] No Internet. State -> WIFI_CONFIGURED_NOT_CONNECTED.");
                wifiState = WIFI_CONFIGURED_NOT_CONNECTED; 
                WiFi.softAPdisconnect(true);
            }
        } 
        else if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED) {
            Serial.println("[CapServer] Connection Failed. Back to AP Mode.");
            WiFi.disconnect(true);
            wifiState = WIFI_NOT_CONFIGURED;
        }
    }
}