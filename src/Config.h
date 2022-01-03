#pragma once

#include <ArduinoJson.h>
#include <LittleFS.h>

namespace Config
{
    char mqtt_server[80] = "example.tld";

    char mqtt_username[24] = "";
    char mqtt_password[24] = "";

    void save()
    {
        DynamicJsonDocument json(512);
        json["mqtt_server"] = mqtt_server;
        json["mqtt_username"] = mqtt_username;
        json["mqtt_password"] = mqtt_password;

        File configFile = LittleFS.open("/config.json", "w");
        if (!configFile)
        {
            return;
        }

        serializeJson(json, configFile);
        configFile.close();
    }

    void load()
    {
        if (LittleFS.begin())
        {
            if (LittleFS.exists("config.json"))
            {
                File configFile = LittleFS.open("config.json", "r");

                if (configFile)
                {
                    const size_t size = configFile.size();
                    std::unique_ptr<char[]> buf(new char[size]);

                    configFile.readBytes(buf.get(), size);
                    DynamicJsonDocument json(512);

                    if (DeserializationError::Ok == deserializeJson(json, buf.get()))
                    {
                        strcpy(mqtt_server, json["mqtt_server"]);
                        strcpy(mqtt_username, json["mqtt_username"]);
                        strcpy(mqtt_password, json["mqtt_password"]);
                    }
                }
            }
            else
            {
                Serial.println("config.json file does not exist.");
            }
        }
        else
        {
            Serial.println("Config: Failed to mount file system");
        }
    }
}