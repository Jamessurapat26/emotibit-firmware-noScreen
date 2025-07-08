#include <Arduino.h>
#include "EmotiBit.h"
#include "time.h"
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// Move MQTT_MAX_PACKET_SIZE before PubSubClient include
#define MQTT_MAX_PACKET_SIZE 1024
#include <PubSubClient.h>

// NTP server configuration
const char *ntpServer = "pool.ntp.org";
const char *timezone = "<+07>-7"; // Bangkok timezone

// Serial configuration
#define SerialUSB SERIAL_PORT_USBVIRTUAL
const uint32_t SERIAL_BAUD = 2000000;

// กำหนดขา SPI
#define TFT_CS 16   // ขา CS
#define TFT_RST -1  // ใช้ -1 ถ้าไม่ได้ใช้ขา RST
#define TFT_DC 17   // ขา D/C
#define TFT_SCLK 5  // ขา CLK
#define TFT_MOSI 19 // ขา DIN (MOSI)

// EmotiBit instance
EmotiBit emotibit;
WiFiClient wifiClient;               // WiFi client for MQTT
PubSubClient mqttClient(wifiClient); // Initialize with WiFi client
// สร้างอ็อบเจ็กต์จอ TFT
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

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

bool firstDraw = true;

// Data buffers
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float eda[dataSize];
float ppg[dataSize];
float skinTempData[dataSize];
float batteryData;
float batteryVoltData;
String deviceID;

// Battery
float BatteryVoltage = 0.0;
float BatteryPercentage = 0.0;
float prevBatteryPercentage = -1.0;

// อุณหภูมิ
float prevTemp = -100.0;
unsigned long lastTempTime = 0;

// เวลา
String prevTime = "";

// Function prototypes
void configureNTP();
unsigned long getCurrentEpochTime();
bool connectMQTT();
void handleShortButtonPress();
void handleLongButtonPress();
float calculateHeartRate(float *ppgData, size_t dataSize, float sampleRate);
// วาดแบตเตอรี่
void drawBattery(int x, int y, float percent);
float readAndSmoothData(EmotiBit::DataType dataType, float &lastValue, unsigned long &lastTime, unsigned long timeout = 5000);
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

    // Initialize TFT display
    tft.initR(INITR_BLACKTAB);
    tft.setRotation(1);           // Set to landscape mode
    tft.fillScreen(ST77XX_BLACK); // Black background
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);

    // Display startup message
    tft.setCursor(10, 10);
    tft.println("EmotiBit Monitor");
    tft.setCursor(10, 30);
    tft.println("Connecting...");

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

    // เวลา
    String currentTime = "--:--:--";
    if (WiFi.status() == WL_CONNECTED)
    {
        char timeString[10];
        strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);
        currentTime = String(timeString);
    }

    // แบตเตอรี่
    BatteryVoltage = emotibit.readBatteryVoltage();
    BatteryPercentage = emotibit.getBatteryPercent(BatteryVoltage);

    // อุณหภูมิ
    // float temperature = readAndSmoothData(EmotiBit::DataType::THERMOPILE, prevTemp, lastTempTime);
    float temperature = skinTempData[0];
    float heartRate = calculateHeartRate(ppg, dataSize, 100.0); // Use your existing calculateHeartRate function

    if (firstDraw)
    {
        tft.fillScreen(ST77XX_BLACK);
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(1);

        tft.fillRect(0, 0, 160, 16, ST77XX_BLUE);
        tft.setCursor(5, 5);
        tft.print("Time:");

        tft.setCursor(5, 25);
        tft.print("Skin Temp:");

        tft.setCursor(5, 45);
        tft.print("Battery:");

        tft.setCursor(5, 65); // Add a new line for heart rate
        tft.print("Heart Rate:");

        firstDraw = false;
    }

    // เวลา
    if (currentTime != prevTime)
    {
        tft.fillRect(40, 5, 120, 8, ST77XX_BLUE);
        tft.setCursor(40, 5);
        tft.print(currentTime);
        prevTime = currentTime;
    }

    // อุณหภูมิ
    if (abs(temperature) >= 0.1)
    {
        tft.fillRect(75, 25, 60, 8, ST77XX_BLACK);
        tft.setCursor(75, 25);

        tft.print(temperature, 1);
        tft.print(" C");
    }

    // แบตเตอรี่
    if (abs(BatteryPercentage - prevBatteryPercentage) >= 1.0)
    {
        tft.fillRect(75, 45, 50, 8, ST77XX_BLACK);
        tft.setCursor(75, 45);
        tft.print(BatteryPercentage, 0);
        tft.print("%");
        drawBattery(130, 45, BatteryPercentage);
        prevBatteryPercentage = BatteryPercentage;
    }

    // อัตราการเต้นของหัวใจ
    if (abs(heartRate) >= 1.0)
    {
        tft.fillRect(75, 65, 50, 8, ST77XX_BLACK);
        tft.setCursor(75, 65);
        tft.print(heartRate, 0);
        tft.print(" bpm");
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

// Function to read and smooth data, handling timeouts
float readAndSmoothData(EmotiBit::DataType dataType, float &lastValue, unsigned long &lastTime, unsigned long timeout)
{
    float currentData[dataSize];
    size_t dataAvailable = emotibit.readData(dataType, &currentData[0], dataSize);

    if (dataAvailable > 0)
    {
        // Apply smoothing
        float smoothData = 0;
        float smoother = 0.8; // Smoothing factor (0.0 to 1.0)

        for (size_t i = 0; i < dataAvailable && i < dataSize; i++)
        {
            if (i == 0)
            {
                smoothData = currentData[i];
            }
            else
            {
                smoothData = smoothData * smoother + currentData[i] * (1.0 - smoother);
            }
        }

        lastValue = smoothData;
        lastTime = millis();
        return smoothData;
    }
    else if (millis() - lastTime > timeout)
    {
        // Data is stale, return 0 or some indicator value
        return 0;
    }

    return lastValue; // Return the last valid value if within timeout
}

// วาดแบตเตอรี่
void drawBattery(int x, int y, float percent)
{
    int w = 20, h = 8;
    int level = map(percent, 0, 100, 0, w - 4);
    tft.drawRect(x, y, w, h, ST77XX_WHITE);
    tft.drawRect(x + w, y + 2, 2, h - 4, ST77XX_WHITE);
    tft.fillRect(x + 2, y + 2, level, h - 4, ST77XX_GREEN);
}

float calculateHeartRate(float *ppgData, size_t dataSize, float sampleRate)
{
    const float threshold = 100.0; // กำหนดค่า threshold สำหรับการหาจุดสูงสุด
    int peakCount = 0;
    bool aboveThreshold = false;

    for (size_t i = 1; i < dataSize; i++)
    {
        if (ppgData[i] > threshold && !aboveThreshold)
        {
            peakCount++;
            aboveThreshold = true;
        }
        else if (ppgData[i] < threshold)
        {
            aboveThreshold = false;
        }
    }

    // คำนวณอัตราการเต้นของหัวใจ (BPM)
    float heartRate = (peakCount / (dataSize / sampleRate)) * 60.0;

    return heartRate;
}