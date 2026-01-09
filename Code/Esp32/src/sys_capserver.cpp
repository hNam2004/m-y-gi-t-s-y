#include "sys_capserver.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"
#include "sys_eeprom.hpp"
#include "sys_wifi.hpp"

// --- KHAI BÁO BIẾN DÙNG CHUNG ---
extern char deviceSerial[32]; 
extern String ID;

// --- GIAO DIỆN HTML (ĐÃ SỬA CSS ĐỂ KHÔNG DÙNG KÝ TỰ %) ---
// Giải pháp: Dùng đơn vị 'vw' (viewport width) hoặc 'px' thay vì '%'
// Điều này giúp WebServer không bị nhầm lẫn đây là biến thay thế.
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>Kdev Config</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { 
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; 
        background-color: #eef2f5; 
        margin: 0; padding: 0; 
        display: flex; justify-content: center; align-items: center; 
        min-height: 100vh; 
    }
    .container { 
        background-color: #fff; 
        width: 90vw; /* Thay 100% bằng 90vw (90% chiều rộng màn hình) */
        max-width: 380px; 
        border-radius: 12px; 
        box-shadow: 0 10px 25px rgba(0,0,0,0.1); 
        overflow: hidden; 
        margin: 20px; 
    }
    .header { background: linear-gradient(135deg, #007bff, #0056b3); padding: 25px; text-align: center; color: white; }
    .header h2 { margin: 0; font-size: 24px; font-weight: 600; }
    .header p { margin: 5px 0 0; font-size: 14px; opacity: 0.8; }
    form { padding: 25px; }
    
    .form-group { margin-bottom: 20px; }
    label { font-weight: 600; color: #333; display: block; margin-bottom: 8px; font-size: 14px; }
    
    input[type="text"], input[type="password"], select { 
      width: 100%; /* Trình duyệt hiểu cái này, nhưng an toàn hơn ta để box-sizing */
      width: -webkit-fill-available; /* Hack cho mobile */
      padding: 12px; border: 1px solid #ddd; border-radius: 8px; box-sizing: border-box; font-size: 16px; transition: border-color 0.3s;
      background-color: #fafafa;
      display: block;
    }
    /* Ghi đè lại width bằng pixel ảo hoặc vw để tránh lỗi parser ESP */
    input[type="text"], input[type="password"], select {
        width: 80vw;
        max-width: 330px; 
    }

    input:focus, select:focus { border-color: #007bff; outline: none; background-color: #fff; }
    
    input[readonly] { background-color: #e9ecef; color: #6c757d; cursor: not-allowed; }

    .divider { height: 1px; background-color: #eee; margin: 25px 0; }
    .section-title { color: #007bff; font-size: 12px; text-transform: uppercase; letter-spacing: 1px; font-weight: bold; margin-bottom: 15px; display: block; }

    input[type="submit"] { 
      background-color: #28a745; color: white; border: none; cursor: pointer; 
      width: 80vw; max-width: 330px; /* Thay 100% bằng vw */
      padding: 15px; border-radius: 8px; font-size: 16px; font-weight: bold; text-transform: uppercase; transition: background 0.3s; box-shadow: 0 4px 6px rgba(40, 167, 69, 0.2);
    }
    input[type="submit"]:hover { background-color: #218838; transform: translateY(-1px); }
    
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
        <label>Device Serial</label>
        <input type="text" value="%DEVICE_SERIAL%" readonly>
      </div>

      <div class="form-group">
        <label for="user_selected_id">Select Device ID</label>
        <select name="user_selected_id" id="user_selected_id">
          <option value="01" %SEL_01%>ID: 01 (Default)</option>
          <option value="02" %SEL_02%>ID: 02</option>
        </select>
        <div class="note">* This ID will be saved for system use.</div>
      </div>

      <div class="form-group">
        <label for="machine_type">Machine Type</label>
        <select name="machine_type" id="machine_type">
          <option value="1">Type 1 - Modbus System</option>
          <option value="2">Type 2 - Relay System</option>
        </select>
      </div>
      
      <input type="submit" value="Save Configuration">
    </form>
  </div>
</body>
</html>
)rawliteral";

DNSServer dnsServer;
AsyncWebServer server(80);
bool wifi_creds_received = false; 
String input_ssid, input_password;

// --- HÀM PROCESSOR ---
String processor(const String& var){
  // [QUAN TRỌNG] Đã xóa logic xử lý PER để tránh xung đột
  
  if(var == "DEVICE_SERIAL"){
     // Thêm cơ chế an toàn: Đảm bảo chuỗi kết thúc null
     char safeSerial[33];
     memset(safeSerial, 0, 33);
     strncpy(safeSerial, deviceSerial, 32);
     
     String s = String(safeSerial);
     if(s.length() == 0 || s.length() > 32) return "UNKNOWN";
     return s;
  }
  
  if(var == "SEL_01"){
    if(ID == "01" || ID == "") return "selected"; 
    return "";
  }
  
  if(var == "SEL_02"){
    if(ID == "02") return "selected";
    return "";
  }
  return String();
}

class CaptiveRequestHandler : public AsyncWebHandler {
public:
    CaptiveRequestHandler() {}
    virtual ~CaptiveRequestHandler() {}
    bool canHandle(AsyncWebServerRequest *request) { return true; }
    void handleRequest(AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", index_html, processor);
    }
};

void sys_capserver_init() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html, processor); 
    });

    server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request){
        String val;

        if (request->hasParam("user_selected_id")) {
            val = request->getParam("user_selected_id")->value();
            ID = val; 
            saveConfigDataToEEPROM("ID", val.c_str());
            Serial.println(">> User selected new ID: " + ID);
        }

        if (request->hasParam("machine_type")) {
            val = request->getParam("machine_type")->value();
            saveConfigDataToEEPROM("machineType", val.c_str());
        }

        if (request->hasParam("wifi_id")) input_ssid = request->getParam("wifi_id")->value();
        if (request->hasParam("wifi_password")) input_password = request->getParam("wifi_password")->value();
        
        if(input_ssid.length() > 0) {
            wifi_creds_received = true; 
            request->send(200, "text/html", "<h1>Saved! Restarting...</h1>");
        } else {
            request->send(400, "text/html", "Missing WiFi SSID");
        } 
    });

    dnsServer.start(53, "*", WiFi.softAPIP());
    server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);
    server.begin();
}

void sys_capserver_proc() {
    if (wifiState == WIFI_NOT_CONFIGURED) dnsServer.processNextRequest();
    if (wifi_creds_received) {
        wifi_creds_received = false;
        connectToWiFi(input_ssid.c_str(), input_password.c_str());
        if(wifiState == WIFI_CONNECTED){
            if (checkNetworkConnectivity()) { 
                saveWiFiCredentials(input_ssid.c_str(), input_password.c_str());
                delay(1000); ESP.restart(); 
            } else {
                wifiState = WIFI_CONFIGURED_NOT_CONNECTED; WiFi.softAPdisconnect(true);
            }
        } else if (wifiState == WIFI_CONFIGURED_NOT_CONNECTED) {
            WiFi.disconnect(true); wifiState = WIFI_NOT_CONFIGURED;
        }
    }
}