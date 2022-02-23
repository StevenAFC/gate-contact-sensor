#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

#include "Config.h"

int state;

WiFiManager wifiManager;
WiFiClient wifiClient;

WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", Config::mqtt_server, sizeof(Config::mqtt_server));
WiFiManagerParameter custom_mqtt_user("user", "MQTT User", Config::mqtt_username, sizeof(Config::mqtt_username));
WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", Config::mqtt_password, sizeof(Config::mqtt_password));

uint32_t lastMqttConnectionAttempt = 0;
const uint16_t mqttConnectionInterval = 60000; // 1 minute = 60 seconds = 60000 milliseconds

PubSubClient mqttClient;

char identifier[24];
#define FIRMWARE_PREFIX "esp8266-gate-contact-sensor"
#define AVAILABILITY_ONLINE "online"
#define AVAILABILITY_OFFLINE "offline"
char MQTT_TOPIC_AVAILABILITY[128];
char MQTT_TOPIC_STATE[128];
char MQTT_TOPIC_COMMAND[128];

char MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[128];
char MQTT_TOPIC_AUTOCONF_GATE_CONTACT_SENSOR[128];

void mqttReconnect();
void handleInterrupt();
void mqttReconnect();
void publishAutoConfig();
void publishState();
void mqttCallback(char *topic, uint8_t *payload, unsigned int length) {}
int getState();

uint32_t currentMillis;
uint32_t previousMillis = 0;
const long interval = 5000;
int gateOpenPin;
int gateClosedPin;

bool shouldSaveConfig = false;

void saveConfigCallback()
{
  shouldSaveConfig = true;
}

void setupWifi()
{
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);

  WiFi.hostname(identifier);
  wifiManager.autoConnect(identifier);
  mqttClient.setClient(wifiClient);

  strcpy(Config::mqtt_server, custom_mqtt_server.getValue());
  strcpy(Config::mqtt_username, custom_mqtt_user.getValue());
  strcpy(Config::mqtt_password, custom_mqtt_pass.getValue());

  if (shouldSaveConfig)
  {
    Config::save();
  }
  else
  {
    // For some reason, the read values get overwritten in this function
    // To combat this, we just reload the config
    // This is most likely a logic error which could be fixed otherwise
    Config::load();
  }
}

void setupMQTT()
{
  mqttClient.setServer(Config::mqtt_server, 1883);
  mqttClient.setKeepAlive(10);
  mqttClient.setBufferSize(2048);
  mqttClient.setCallback(mqttCallback);

  mqttReconnect();
}

void setupOTA()
{
  ArduinoOTA.onStart([]()
                     { Serial.println("Start"); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        } });

  ArduinoOTA.setHostname(identifier);

  // This is less of a security measure and more a accidential flash prevention
  ArduinoOTA.setPassword(identifier);
  ArduinoOTA.begin();
}

void setup()
{

  Serial.begin(115200);

  Serial.println("\n");
  Serial.printf("Core Version: %s\n", ESP.getCoreVersion().c_str());
  Serial.printf("Boot Version: %u\n", ESP.getBootVersion());
  Serial.printf("Boot Mode: %u\n", ESP.getBootMode());
  Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Reset reason: %s\n", ESP.getResetReason().c_str());

  snprintf(identifier, sizeof(identifier), "GATE-SENSOR-%u", ESP.getChipId());
  snprintf(MQTT_TOPIC_AVAILABILITY, 127, "%s/%s/status", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_STATE, 127, "%s/%s/state", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_COMMAND, 127, "%s/%s/command", FIRMWARE_PREFIX, identifier);

  snprintf(MQTT_TOPIC_AUTOCONF_GATE_CONTACT_SENSOR, 127, "homeassistant/sensor/%s/%s_gate_contact_sensor/config", FIRMWARE_PREFIX, identifier);
  snprintf(MQTT_TOPIC_AUTOCONF_WIFI_SENSOR, 127, "homeassistant/sensor/%s/%s_wifi/config", FIRMWARE_PREFIX, identifier);

  Serial.println("Loading Configuration");
  Config::load();
  Serial.println("Configuration Loaded (Server: " + String(Config::mqtt_server) + ")");

  setupWifi();
  setupMQTT();
  setupOTA();

  gateOpenPin = 12;
  gateClosedPin = 13;

  pinMode(gateOpenPin, INPUT);
  pinMode(gateClosedPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(gateOpenPin), handleInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(gateClosedPin), handleInterrupt, CHANGE);
}

void loop()
{
  ArduinoOTA.handle();

  mqttClient.loop();

  currentMillis = millis();

  if (!mqttClient.connected() && currentMillis - lastMqttConnectionAttempt >= mqttConnectionInterval)
  {
    lastMqttConnectionAttempt = currentMillis;
    printf("Reconnecting to mqtt\n");
    mqttReconnect();
  }

  if (currentMillis - previousMillis >= interval) {
    // spam the state over MQTT each interval
    previousMillis = currentMillis;

    int reading = getState();

    state = reading;

    Serial.println("Loop value: " + String(state));

    publishState();
  }
}

volatile long lastDebounceTime = 0;
const int debounceDelay = 50;

ICACHE_RAM_ATTR void handleInterrupt()
{

  int reading = getState();

  if (reading == state)
    return;

  boolean debounce = false;

  if ((millis() - lastDebounceTime) <= debounceDelay)
  {
    debounce = true;
  }

  lastDebounceTime = millis();

  if (debounce)
    return;

  state = reading;

  Serial.println("Interrupt value: " + String(state));

  publishState();
}

int getState()
{
  int gateOpen = digitalRead(gateOpenPin);
  int gateClosed = digitalRead(gateClosedPin);

  if (!gateOpen && !gateClosed)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Gate Partially Open");
    return 1;
  }
  else if (!gateOpen && gateClosed)
  {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Gate Closed");
    return 0;
  }
  else 
  {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Gate Open");
    return 2;
  }
}

void publishState()
{
  DynamicJsonDocument wifiJson(192);
  DynamicJsonDocument stateJson(604);
  char payload[256];

  wifiJson["ssid"] = WiFi.SSID();
  wifiJson["ip"] = WiFi.localIP().toString();
  wifiJson["rssi"] = WiFi.RSSI();

  stateJson["gate_sensor"] = state;

  stateJson["wifi"] = wifiJson.as<JsonObject>();

  serializeJson(stateJson, payload);

  boolean success = mqttClient.publish(&MQTT_TOPIC_STATE[0], &payload[0], true);

  Serial.println("MQTT Client Connected: " + String(mqttClient.connected()));
  Serial.println("MQTT Message Publish Success: " + String(success));
}

void mqttReconnect()
{

  Serial.println("Attempting to reconnect (Server: " + String(Config::mqtt_server) + ")");

  for (uint8_t attempt = 0; attempt < 3; ++attempt)
  {
    if (mqttClient.connect(identifier, Config::mqtt_username, Config::mqtt_password, MQTT_TOPIC_AVAILABILITY, 1, true, AVAILABILITY_OFFLINE))
    {
      mqttClient.publish(MQTT_TOPIC_AVAILABILITY, AVAILABILITY_ONLINE, true);
      publishAutoConfig();

      mqttClient.subscribe(MQTT_TOPIC_COMMAND);
      break;
    }
    else
    {
      Serial.println("MQTT Client failed to connect." + String(mqttClient.state()));
    }

    delay(500);
  }
}

bool isMqttConnected()
{
  return mqttClient.connected();
}

void publishAutoConfig()
{
  char mqttPayload[2048];
  DynamicJsonDocument device(256);
  DynamicJsonDocument autoconfPayload(1024);
  StaticJsonDocument<64> identifiersDoc;
  JsonArray identifiers = identifiersDoc.to<JsonArray>();

  device["identifiers"] = identifiers;
  device["manufacturer"] = "Came";
  device["modek"] = "BK 1200";
  device["name"] = identifiers;
  device["sw_version"] = "2022-01-03";

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = identifier + String(" WiFi");
  autoconfPayload["value_template"] = "{{value_json.wifi.rssi}}";
  autoconfPayload["unique_id"] = identifier + String(" WiFi");
  autoconfPayload["unit_of_measurement"] = "dBm";
  autoconfPayload["json_attributes_topic"] = "MQTT_TOPIC_STATE";
  autoconfPayload["json_attributes_template"] = "{\"ssid\": \"{{value_json.wifi.ssid}}\", \"ip\": \"{{value_json.wifi.ip}}\"}";
  autoconfPayload["icon"] = "mdi:wifi";

  serializeJson(autoconfPayload, mqttPayload);
  mqttClient.publish(&MQTT_TOPIC_AUTOCONF_WIFI_SENSOR[0], &mqttPayload[0], true);

  autoconfPayload.clear();

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = identifier + String(" Contact");
  autoconfPayload["value_template"] = "{{value_json.contact}}";
  autoconfPayload["unique_id"] = identifier + String("_contact_sensor");
  autoconfPayload["icon"] = "mdi:gate";

  serializeJson(autoconfPayload, mqttPayload);
  mqttClient.publish(&MQTT_TOPIC_AUTOCONF_GATE_CONTACT_SENSOR[0], &mqttPayload[0], true);

  autoconfPayload.clear();
}