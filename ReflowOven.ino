#include <WebSocketsServer.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <ESP8266SSDP.h>
#include <ESP8266mDNS.h>

#include "globals.h"
#include "ProfileController.h"

#define PIN_RELAY_TOP       D1
#define PIN_RELAY_BOTTOM    D2
#define PIN_RELAY_BOOST     D3
#define PIN_RELAY_AUX       D4

#define PIN_MAX31856_CS     D8
#define PIN_MAX31856_DRDY   D0


WiFiManager wifiManager;
ESP8266WebServer server(80);
WebSocketsServer webSocket(81);

ProfileController profileController;

char cBuffer[256];

void resetBuffer() {
  memset(cBuffer, 0, sizeof(cBuffer));
}

void saveConfigCallback () {
  // Meh...
}

int8_t connectionCount = 0;
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      connectionCount--;
      break;
    case WStype_CONNECTED:
        connectionCount++;
        webSocket.sendTXT(num, "HELLO");
      break;
    case WStype_TEXT:
      handleTextRequest(num, payload, length);
      break;
    case WStype_BIN:
      hexdump(payload, length);
      break;
  }
}

void wsBroadcastText(char *str) {
  webSocket.broadcastTXT(str);
}

void wsBroadcastText(char *str1, char *str2) {
  char msg[256+20];
  memset(msg, '\0', sizeof(msg));
  
  strcpy(msg, str1);
  strcat(msg, ":");
  strcat(msg, str2);

  wsBroadcastText(msg);
}

void wsBroadcastText(char *str1, float str2) {
  char floatBuff[7];
  dtostrf(str2, 6, 2, floatBuff);
  wsBroadcastText(str1, floatBuff);
}

void wsSendText(uint8_t num, char *str) {
  webSocket.sendTXT(num, str);
}

void wsSendText(uint8_t num, char *str1, char *str2) {
  char msg[256+20];
  memset(msg, '\0', sizeof(msg));
  
  strcpy(msg, str1);
  strcat(msg, ":");
  strcat(msg, str2);

  wsSendText(num, msg);
}

void wsSendText(uint8_t num, char *str1, float str2) {
  char floatBuff[7];
  dtostrf(str2, 6, 2, floatBuff);
  wsSendText(num, str1, floatBuff);
}

void wsSendText(uint8_t num, char *str1, uint16_t str2) {
  char str2b[17];
  
  wsSendText(num, str1, itoa(str2, str2b, 10));
}

void handleTextRequest(uint8_t num, uint8_t * payload, size_t length) {
  const char s[2] = ":";
  char *token = strtok((char*)payload, ":");
  if (token == NULL)
      token = (char*)payload;
  
  if (strcmp(token, "getTemp") == 0) {
    wsSendText(num, "temp", profileController.getTemperature());
  }
  else if (strcmp(token, "setTemp") == 0) {
    token = strtok(NULL, s);
    if (token != NULL)
      profileController.setTemperature(atof(token));
  }
  else if (strcmp(token, "getProList") == 0) {
    resetBuffer();
    wsSendText(num, "proList", profileController.getProfileList(cBuffer));
  }
  else if (strcmp(token, "getProOverview") == 0) {
    resetBuffer();
    wsSendText(num, "proOverview", profileController.getProfileOverview(cBuffer));
  }
  if (strcmp(token, "getConOverview") == 0) {
    resetBuffer();
    wsSendText(num, "conOverview", profileController.getConnectionOverview(cBuffer));
  }
  else if (strcmp(token, "getHeatOverview") == 0) {
    resetBuffer();
    wsSendText(num, "heatOverview", profileController.getHeaterOverview(cBuffer));
  }
  else if (strcmp(token, "getGraphData") == 0) {
    while (profileController.graphDataAvailable())
      wsSendText(num, "graphPoint", profileController.getGraphData());
  }
  else if (strcmp(token, "clearGraph") == 0) {
    profileController.clearGraph();
  }
  else if (strcmp(token, "startJob") == 0) {
    token = strtok(NULL, s);
    if (token != NULL)
      profileController.startJob(token);
  }
  else if (strcmp(token, "pauseJob") == 0) {
    profileController.pauseJob();
  }
  else if (strcmp(token, "resumeJob") == 0) {
    profileController.resumeJob();
  }
  else if (strcmp(token, "stopJob") == 0) {
    profileController.stopJob();
  }
  else if (strcmp(token, "resetWiFi") == 0) {
    wifiManager.resetSettings();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  profileController.init();

  const float learnedPower = 10.5f;
  const float learnedInertia = 42.0f;
  const float learnedInsulation = 157.0f;
  profileController.setLearnedConstants(learnedPower, learnedInertia, learnedInsulation);

  profileController.addThermocouple( PIN_MAX31856_CS, PIN_MAX31856_DRDY );

  // Unfortunately, for now, the heaters must be added in order: TOP, BOTTOM, BOOST
  profileController.addHeatingElement(  PIN_RELAY_TOP,    false, 75.0f, 0.8f );
  profileController.addHeatingElement(  PIN_RELAY_BOTTOM, false, 100.0f, 1.0f );

  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  if (!SPIFFS.begin()) {
    Serial.println("ERROR: Failed to mount file system!");
  }

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  if (!wifiManager.autoConnect()) {
    Serial.println("ERROR: Failed to connecto to WiFi!");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  server.serveStatic("/", SPIFFS, "/index.html");
  server.serveStatic("/style.css", SPIFFS, "/style.css");

  server.on("/description.xml", HTTP_GET, []() {
    SSDP.schema(server.client());
    server.client().stop();
  });

  server.begin();
  MDNS.begin("reflowOven");

  // SSDP
  Serial.printf("Starting SSDP\n");
  SSDP.setSchemaURL("description.xml");
  SSDP.setHTTPPort(80);
  SSDP.setName("Reflow Oven");
  SSDP.setSerialNumber(ESP.getChipId());
  SSDP.setURL("");
  SSDP.setModelName("Reflow Oven");
  SSDP.setModelNumber("Reflow-1");
  SSDP.setModelURL("http://www.robertfeddeler.com/");
  SSDP.setManufacturer("Robert Feddeler");
  SSDP.setManufacturerURL("https://github.com/rjfeddeler");
  SSDP.setDeviceType("urn:schemas-upnp-org:device:ReflowOven:1");
  SSDP.begin();

  ArduinoOTA.setHostname("ReflowOven");
  ArduinoOTA.setPassword("esp8266");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else {
      type = "filesystem";
      SPIFFS.end();
    }

    Serial.println("OTA Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  Serial.println();
  Serial.print("WiFi Hostname: ");
  Serial.println(WiFi.hostname());
  Serial.print("WiFi IP addr: ");
  Serial.println(WiFi.localIP());
  Serial.print("WiFi MAC addr: ");
  Serial.println(WiFi.macAddress());
  Serial.print("WiFi SSID: ");
  Serial.println(WiFi.SSID());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  webSocket.loop();

  profileController.update();
  
  if (profileController.graphDataAvailable() && connectionCount > 0)
    wsBroadcastText("graphPoint", profileController.getGraphData());
    
  if (profileController.proOverviewUpdateNeeded()) {
    resetBuffer(); // Probably don't need this
    wsBroadcastText("proOverview", profileController.getProfileOverview(cBuffer));
  }
  else if (profileController.heatOverviewUpdateNeeded()) {
    resetBuffer(); // Probably don't need this
    wsBroadcastText("heatOverview", profileController.getHeaterOverview(cBuffer));
  }
  else if (profileController.conOverviewUpdateNeeded()) {
    resetBuffer(); // Probably don't need this
    wsBroadcastText("conOverview", profileController.getConnectionOverview(cBuffer));
  }
}

void saveConfig() {
  /*
  Serial.println("saving config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["googleApiKey"] = googleApiKey;
  json["ipstackApiKey"] = ipstackApiKey;
  JsonObject& location = json.createNestedObject("location");
  location["overrideLatitude"] = overrideLatitude;
  location["overrideLongitude"] = overrideLongitude;
  JsonObject& color = json.createNestedObject("color");
  
  color["h"] = (int)(toColor.H * 360.0f);
  color["s"] = (int)(toColor.S * 100.0f);
  color["v"] = (int)(toColor.L * 100.0f);
  
  json["clock"] = isTwelveHour ? 12 : 24;
  json["dim"] = autoDimValue;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
    Serial.println("failed to open config file for writing");

  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();

  savedColor = HslColor(toColor);
  */
}
