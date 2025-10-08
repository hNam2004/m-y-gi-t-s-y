#include "sys_capserver.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"
#include "sys_eeprom.hpp"
#include "sys_wifi.hpp"

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>WiFi Setup</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #f2f2f2;
      margin: 0;
      padding: 0;
    }
    h3 {
      color: #333;
    }
    form {
      background-color: #fff;
      max-width: 400px;
      margin: 20px auto;
      padding: 20px;
      border-radius: 8px;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.1);
    }
    input[type="text"], input[type="password"], input[type="submit"] {
      width: 100%;
      padding: 10px;
      margin: 5px 0;
      border: 1px solid #ccc;
      border-radius: 4px;
      box-sizing: border-box;
    }
    input[type="submit"] {
      background-color: #4CAF50;
      color: white;
      border: none;
      cursor: pointer;
    }
    input[type="submit"]:hover {
      background-color: #45a049;
    }
  </style>
</head>
<body>
  <h3>WiFi Setup</h3>
  <form action="/get">
    <br>
    <label for="wifi_id">WiFi ID:</label><br>
    <input type="text" id="wifi_id" name="wifi_id">
    <br>
    <label for="wifi_password">WiFi Password:</label><br>
    <input type="password" id="wifi_password" name="wifi_password">
    <br>
    <input type="submit" value="Connect">
  </form>
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
bool wifi_id_received = false;
bool wifi_password_received = false;
String wifi_id;
String wifi_password;

void sys_capserver_init()
{
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      request->send_P(200, "text/html", index_html); 
      Serial.println("Client Connected"); });

    server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
              {
      String inputMessage;
      String inputParam;
  
      if (request->hasParam("wifi_id")) {
        inputMessage = request->getParam("wifi_id")->value();
        wifi_id = inputMessage;
        wifi_id_received = true;
        Serial.println("WiFi ID: " + wifi_id);
      }

      if (request->hasParam("wifi_password")) {
        inputMessage = request->getParam("wifi_password")->value();
        wifi_password = inputMessage;
        wifi_password_received = true;
        Serial.println("WiFi Password: " + wifi_password);
      }
      
      if(wifi_id_received && wifi_password_received) {
        request->send(200, "text/html", "WiFi ID and Password received. Connecting to WiFi...");
      } else {
        request->send(400, "text/html", "Error: WiFi ID or Password missing.");
      } });
    dnsServer.start(53, "*", WiFi.softAPIP());
    server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);
    server.begin();
}

void sys_capserver_proc()
{
    dnsServer.processNextRequest();
    if (wifi_id_received && wifi_password_received)
    {

        connectToWiFi(wifi_id.c_str(),wifi_password.c_str());
        if(wifiState == WIFI_CONNECTED){
            saveWiFiCredentials(wifi_id.c_str(),wifi_password.c_str());
            Serial.println("Saved");
        }
        wifi_id_received = false;
        wifi_password_received = false;
    }
    
}