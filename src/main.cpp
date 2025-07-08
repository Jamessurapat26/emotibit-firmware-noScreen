#include <Arduino.h>
#include "EmotiBit.h"
#include "time.h"
#include <ArduinoJson.h>

// Move MQTT_MAX_PACKET_SIZE before PubSubClient include
#define MQTT_MAX_PACKET_SIZE 1024
#include <PubSubClient.h>

// NTP server configuration
const char *ntpServer = "pool.ntp.org";
const char *timezone = "<+07>-7"; // Bangkok timezone

// Serial configuration
#define SerialUSB SERIAL_PORT_USBVIRTUAL
const uint32_t SERIAL_BAUD = 2000000;

// EmotiBit instance
EmotiBit emotibit;
WiFiClient wifiClient;               // WiFi client for MQTT
PubSubClient mqttClient(wifiClient); // Initialize with WiFi client

// Global variables
unsigned long epochTime;
struct tm timeinfo;
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000; // Reconnect interval in ms

// Publication control variables
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 1000; // Publish every 1 second

// MQTT configuration
const char *mqtt_server = "broker.emqx.io"; // e.g. "broker.hivemq.com"
const int mqtt_port = 1883;
const char *mqtt_client_id = "EmotiBit-Client";

// MQTT topic prefix
const char *mqtt_topic_prefix = "Emotibit/";

// Data buffers
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float eda[dataSize];
float ppg[dataSize];
float skinTempData[dataSize];
float batteryData;
float batteryVoltData;
String deviceID;

// Function prototypes
void configureNTP();
unsigned long getCurrentEpochTime();
bool connectMQTT();
void handleShortButtonPress();
void handleLongButtonPress();
String packjson(const String &deviceId, unsigned long timestamp,
                float *edaData, size_t edaSize,
                float *ppgData, size_t ppgSize,
                float skinTemp, float batteryPercent, float batteryVolt);
bool publishToMQTT(const String &topic, const String &payload);

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(2000);

    // EmotiBit setup
    String inoFilename = __FILE__;
    inoFilename.replace("/", "\\");
    inoFilename = inoFilename.substring(inoFilename.lastIndexOf("\\") + 1, inoFilename.indexOf("."));
    emotibit.setup(inoFilename);

    emotibit.attachShortButtonPress(&handleShortButtonPress);
    emotibit.attachLongButtonPress(&handleLongButtonPress);

    // Setup MQTT
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setBufferSize(MQTT_MAX_PACKET_SIZE); // Explicitly set buffer size

    // Configure NTP
    configureNTP();

    // Print device ID
    deviceID = emotibit.emotibitDeviceId;
    Serial.print("Device ID: ");
    Serial.println(deviceID);
}

void loop()
{
    // Update EmotiBit data
    emotibit.update();

    // Maintain MQTT connection
    if (!mqttClient.connected())
    {
        unsigned long now = millis();
        if (now - lastMqttReconnectAttempt > reconnectInterval)
        {
            lastMqttReconnectAttempt = now;
            if (connectMQTT())
            {
                lastMqttReconnectAttempt = 0;
            }
        }
    }
    else
    {
        mqttClient.loop();
    }

    // Only read and publish data once per second
    unsigned long currentMillis = millis();
    if (currentMillis - lastPublishTime >= publishInterval)
    {
        lastPublishTime = currentMillis;

        // Read data
        size_t dataEDA = emotibit.readData(EmotiBit::DataType::EDA, eda, dataSize);
        size_t dataPPG = emotibit.readData(EmotiBit::DataType::PPG_GREEN, ppg, dataSize);

        emotibit.readData(EmotiBit::DataType::THERMOPILE, skinTempData, dataSize);
        batteryVoltData = emotibit.readBatteryVoltage();
        batteryData = emotibit.getBatteryPercent(batteryVoltData);

        // Get current time
        epochTime = getCurrentEpochTime();

        if (dataEDA > 0)
        {
            // Pack sensor data into JSON
            String jsonData = packjson(deviceID, epochTime, eda, dataEDA, ppg, dataPPG, skinTempData[0], batteryData, batteryVoltData);

            Serial.println(jsonData);

            // Publish to MQTT if connected
            String topic = String(mqtt_topic_prefix) + deviceID;
            if (mqttClient.connected())
            {
                if (publishToMQTT(topic, jsonData))
                {
                    Serial.println("Published to MQTT successfully");
                }
                else
                {
                    Serial.println("Failed to publish to MQTT");
                }
            }
        }
    }
    delay(1000); // Short delay for responsiveness
}

void configureNTP()
{
    configTime(0, 0, ntpServer);
    setenv("TZ", timezone, 1);
    tzset();
}

unsigned long getCurrentEpochTime()
{
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time.");
        return 0;
    }
    time_t now;
    time(&now);
    return now;
}

void handleShortButtonPress()
{
    if (emotibit.getPowerMode() == EmotiBit::PowerMode::NORMAL_POWER)
    {
        emotibit.setPowerMode(EmotiBit::PowerMode::WIRELESS_OFF);
    }
    else
    {
        emotibit.setPowerMode(EmotiBit::PowerMode::NORMAL_POWER);
    }
}

void handleLongButtonPress()
{
    emotibit.sleep();
}

String packjson(const String &deviceId, unsigned long timestamp,
                float *edaData, size_t edaSize,
                float *ppgData, size_t ppgSize,
                float skinTemp, float batteryPercent, float batteryVolt)
{
    const size_t buffer = JSON_OBJECT_SIZE(4) +
                          JSON_OBJECT_SIZE(4) +
                          JSON_ARRAY_SIZE(15) +
                          JSON_ARRAY_SIZE(100) +
                          500;

    DynamicJsonDocument json(buffer);

    json["device_id"] = deviceId;
    json["timestamp"] = timestamp;
    JsonObject sensors = json.createNestedObject("sensors");
    sensors["skintemp"] = skinTemp;

    JsonArray edaArray = sensors.createNestedArray("eda");
    for (size_t i = 0; i < 15 && i < edaSize; i++)
    {
        // Convert float to integer (multiply by 1000 to preserve decimal precision)
        float edaValue = edaData[i];
        edaArray.add(edaValue);
    }

    // Add PPG array (limit to 100 values or available data)
    JsonArray ppgArray = sensors.createNestedArray("ppg");
    for (size_t i = 0; i < 100 && i < ppgSize; i++)
    {
        // Convert float to integer (multiply by 1000 to preserve decimal precision)
        float ppgValue = ppgData[i];
        ppgArray.add(ppgValue);
    }

    String jsonOutput;
    serializeJson(json, jsonOutput);

    return jsonOutput;
}
bool connectMQTT()
{
    Serial.println("Attempting MQTT connection...");

    bool connected;

    connected = mqttClient.connect(mqtt_client_id);

    if (connected)
    {
        Serial.println("Connected to MQTT broker");
    }
    else
    {
        Serial.print("MQTT connection failed, rc=");
        Serial.println(mqttClient.state());
    }

    return connected;
}

bool publishToMQTT(const String &topic, const String &payload)
{
    Serial.println("Publishing to topic: " + topic);
    Serial.print("Payload size: ");
    Serial.println(payload.length());

    // Check if payload is too large for PubSubClient's default buffer
    if (payload.length() > 128)
    {
        Serial.println("Warning: Payload size exceeds default PubSubClient buffer (128 bytes)");
        Serial.println("Make sure you've defined MQTT_MAX_PACKET_SIZE before including PubSubClient.h");
    }

    // Try to publish
    bool success = mqttClient.publish(topic.c_str(), payload.c_str(), false);

    if (!success)
    {
        Serial.print("MQTT publish error, state: ");
        Serial.println(mqttClient.state());

        // Print specific error based on state
        switch (mqttClient.state())
        {
        case -4:
            Serial.println("Connection timeout");
            break;
        case -3:
            Serial.println("Connection lost");
            break;
        case -2:
            Serial.println("Connection failed");
            break;
        case -1:
            Serial.println("Disconnected");
            break;
            // Add other error codes as needed
        }
    }

    return success;
}
