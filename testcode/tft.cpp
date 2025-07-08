#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "EmotiBit.h"
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "heartRate.h"

#define SerialUSB SERIAL_PORT_USBVIRTUAL
const uint32_t SERIAL_BAUD = 2000000;

EmotiBit emotibit;
const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
float data[dataSize];

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600); // UTC+7 for Bangkok

float BatteryVoltage = 0.0f;
float BatteryPercentage = 0.0f;

// Track last successful data readings
float lastHR = 0;
float lastTemp = 0;
float lastPPG = 0;
unsigned long lastHRTime = 0;
unsigned long lastTempTime = 0;
unsigned long lastPPGTime = 0;

// Add these variables to track previous values
float prevHR = 0;
float prevTemp = 0;
float prevPPG = 0;
float prevBatteryPercentage = 0;
String prevTime = "";
bool heartFilled = false;
bool firstDraw = true;

#define HR_BUFFER_SIZE 10
float hrBuffer[HR_BUFFER_SIZE] = {0};
int hrBufferIndex = 0;
bool hrBufferFilled = false;

// กำหนดขา SPI
#define TFT_CS 16   // ขา CS
#define TFT_RST -1  // ใช้ -1 ถ้าไม่ได้ใช้ขา RST
#define TFT_DC 17   // ขา D/C
#define TFT_SCLK 5  // ขา CLK
#define TFT_MOSI 19 // ขา DIN (MOSI)

// สร้างอ็อบเจ็กต์จอ TFT
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Function prototypes
void drawHeart(int16_t x, int16_t y, uint16_t color, uint8_t size);
void drawBattery(int16_t x, int16_t y, float percentage);
float readAndSmoothData(EmotiBit::DataType dataType, float &lastValue, unsigned long &lastTime, unsigned long timeout = 5000);

float calculateHeartRate(float *ppgData, size_t dataSize, float sampleRate);
void onShortButtonPress()
{
    // Toggle wifi on/off
    if (emotibit.getPowerMode() == EmotiBit::PowerMode::NORMAL_POWER)
    {
        emotibit.setPowerMode(EmotiBit::PowerMode::WIRELESS_OFF);
    }
    else
    {
        emotibit.setPowerMode(EmotiBit::PowerMode::NORMAL_POWER);
    }
}

void onLongButtonPress()
{
    emotibit.sleep();
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    Serial.println("Serial started");

    // Initialize EmotiBit - attach button press handlers
    String inoFilename = __FILE__;
    inoFilename.replace("/", "\\");
    if (inoFilename.lastIndexOf("\\") != -1)
    {
        inoFilename = inoFilename.substring((inoFilename.lastIndexOf("\\")) + 1, (inoFilename.indexOf(".")));
    }
    emotibit.setup(inoFilename);
    emotibit.attachShortButtonPress(&onShortButtonPress);
    emotibit.attachLongButtonPress(&onLongButtonPress);

    // Initialize TFT
    tft.initR(INITR_BLACKTAB);
    tft.setRotation(1);           // Set to landscape mode (1 or 3 for landscape)
    tft.fillScreen(ST77XX_BLACK); // Black background
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);

    // Display startup screen
    tft.setCursor(10, 10);
    tft.println("EmotiBit Monitor");
    tft.setCursor(10, 30);
    tft.println("Connecting to WiFi...");

    delay(2000);
}

void loop()
{
    // Update EmotiBit data
    emotibit.update();

    // Update time if WiFi is connected
    String currentTime = "--:--:--";
    if (WiFi.status() == WL_CONNECTED)
    {
        timeClient.update();
        currentTime = timeClient.getFormattedTime();
    }

    // Read battery data
    BatteryVoltage = emotibit.readBatteryVoltage();
    BatteryPercentage = emotibit.getBatteryPercent(BatteryVoltage);

    // Read sensor data with smoothing and timeout handling
    float temperature = readAndSmoothData(EmotiBit::DataType::THERMOPILE, lastTemp, lastTempTime);
    float ppgValue = readAndSmoothData(EmotiBit::DataType::PPG_GREEN, lastPPG, lastPPGTime);

    // For heart rate, use your existing method
    // อ่านข้อมูล PPG
    float ppgData[dataSize];
    size_t ppgDataSize = emotibit.readData(EmotiBit::DataType::PPG_RED, &ppgData[0], dataSize);

    // คำนวณอัตราการเต้นของหัวใจ
    float heartRate = 0;
    if (ppgDataSize > 0)
    {
        heartRate = calculateHeartRate(ppgData, ppgDataSize, 100.0);
        lastHR = heartRate;
        lastHRTime = millis();
    }
    bool hasValidHR = (millis() - lastHRTime < 5000);

    // On first run, draw everything
    if (firstDraw)
    {
        tft.fillScreen(ST77XX_BLACK);

        // Draw header with blue background
        tft.fillRect(0, 0, 160, 16, ST77XX_BLUE);

        // Draw labels (these don't change)
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(1);

        tft.setCursor(5, 5);
        tft.print("Time: ");

        tft.setCursor(5, 25);
        tft.print("Heart Rate: ");

        tft.setCursor(5, 45);
        tft.print("Skin Temp: ");

        tft.setCursor(5, 65);
        tft.print("PPG: ");

        tft.setCursor(5, 85);
        tft.print("Battery: ");

        firstDraw = false;
    }

    // Only update time if it changed
    if (currentTime != prevTime)
    {
        // Clear previous time
        tft.fillRect(40, 5, 120, 8, ST77XX_BLUE);
        tft.setTextColor(ST77XX_WHITE);
        tft.setTextSize(1);
        tft.setCursor(40, 5);
        tft.print(currentTime);
        prevTime = currentTime;
    }

    // Update heart rate only if changed
    if (hasValidHR != (prevHR > 0) || (hasValidHR && abs(heartRate - prevHR) >= 1))
    {
        tft.fillRect(75, 25, 50, 8, ST77XX_BLACK); // Clear value area
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(75, 25);

        if (hasValidHR)
        {
            tft.print(heartRate, 0);
            tft.print(" BPM");
        }
        else
        {
            tft.print("--");
        }
        prevHR = hasValidHR ? heartRate : 0;
    }

    // Always update heart animation
    bool currentHeartFilled = (millis() % 1000) < 500;
    if (currentHeartFilled != heartFilled)
    {
        drawHeart(140, 25, ST77XX_RED, 8);
        heartFilled = currentHeartFilled;
    }

    // Update temperature only if changed
    if (abs(temperature - prevTemp) >= 0.1)
    {
        tft.fillRect(75, 45, 50, 8, ST77XX_BLACK); // Clear value area
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(75, 45);

        if (millis() - lastTempTime < 5000)
        {
            tft.print(temperature, 1);
            tft.print(" C");
        }
        else
        {
            tft.print("--");
        }
        prevTemp = temperature;
    }

    // Update PPG only if changed
    if (abs(ppgValue - prevPPG) >= 1.0)
    {
        tft.fillRect(40, 65, 85, 8, ST77XX_BLACK); // Clear value area
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(40, 65);

        if (millis() - lastPPGTime < 5000)
        {
            tft.print(ppgValue, 0);
        }
        else
        {
            tft.print("--");
        }
        prevPPG = ppgValue;
    }

    // Update battery percentage only if changed
    if (abs(BatteryPercentage - prevBatteryPercentage) >= 1.0)
    {
        tft.fillRect(65, 85, 30, 8, ST77XX_BLACK); // Clear value area
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(65, 85);
        tft.print(BatteryPercentage, 0);
        tft.print("%");

        // Redraw battery icon
        drawBattery(110, 85, BatteryPercentage);

        prevBatteryPercentage = BatteryPercentage;
    }

    delay(50); // Shorter delay for responsiveness
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

// Function to draw a heart symbol
void drawHeart(int16_t x, int16_t y, uint16_t color, uint8_t size)
{
    // Determine if heart should be filled based on timing (creates pulsing effect)
    bool filled = (millis() % 1000) < 500;

    if (filled)
    {
        // Draw filled heart
        tft.fillCircle(x - size / 2, y, size / 2, color);
        tft.fillCircle(x + size / 2, y, size / 2, color);
        tft.fillTriangle(
            x - size, y + size / 4,
            x + size, y + size / 4,
            x, y + size,
            color);
    }
    else
    {
        // Draw outline heart
        tft.drawCircle(x - size / 2, y, size / 2, color);
        tft.drawCircle(x + size / 2, y, size / 2, color);
        tft.drawTriangle(
            x - size, y + size / 4,
            x + size, y + size / 4,
            x, y + size,
            color);
    }
}

// Function to draw a battery icon
void drawBattery(int16_t x, int16_t y, float percentage)
{
    uint16_t battColor;

    // Choose color based on percentage
    if (percentage > 60)
    {
        battColor = ST77XX_GREEN;
    }
    else if (percentage > 30)
    {
        battColor = ST77XX_YELLOW;
    }
    else
    {
        battColor = ST77XX_RED;
    }

    // Draw battery outline
    tft.drawRect(x, y, 20, 10, ST77XX_WHITE);
    tft.drawRect(x + 20, y + 2, 2, 6, ST77XX_WHITE);

    // Fill battery based on percentage
    int fillWidth = (int)(percentage * 18 / 100);
    if (fillWidth > 0)
    {
        tft.fillRect(x + 1, y + 1, fillWidth, 8, battColor);
    }
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