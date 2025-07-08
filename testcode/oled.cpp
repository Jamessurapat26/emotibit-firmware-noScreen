#include "EmotiBit.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "heartRate.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32 // Change this to 64 for 128x64 OLEDs 1.3"
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

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

void onShortButtonPress()
{
    if (emotibit.getPowerMode() == EmotiBit::PowerMode::NORMAL_POWER)
    {
        emotibit.setPowerMode(EmotiBit::PowerMode::WIRELESS_OFF);
        Serial.println("PowerMode::WIRELESS_OFF");
    }
    else
    {
        emotibit.setPowerMode(EmotiBit::PowerMode::NORMAL_POWER);
        Serial.println("PowerMode::NORMAL_POWER");
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
    Serial.println("EmotiBit Sensor Data Display");

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    display.clearDisplay();
    display.display();
    delay(2000);

    String inoFilename = __FILE__;
    inoFilename = inoFilename.substring(inoFilename.indexOf("."), inoFilename.lastIndexOf("\\") + 1);

    emotibit.setup(inoFilename);
    emotibit.attachShortButtonPress(&onShortButtonPress);
    emotibit.attachLongButtonPress(&onLongButtonPress);

    // Initialize NTPClient
    timeClient.begin();
    timeClient.update();
}

void readAndDisplayData(EmotiBit::DataType dataType, const char *label, int yPosition)
{
    size_t dataAvailable = emotibit.readData(dataType, &data[0], dataSize);
    if (dataAvailable > 0)
    {
        float smoothData = -1;
        float smoother = 0.95f;
        for (size_t i = 0; i < dataAvailable && i < dataSize; i++)
        {
            if (smoothData < 0)
            {
                smoothData = data[i];
            }
            else
            {
                smoothData = smoothData * smoother + data[i] * (1 - smoother);
            }
        }

        Serial.print(label);
        Serial.print(": ");
        Serial.println(smoothData);

        // Display on OLED
        display.setCursor(0, yPosition);
        display.print(label);
        display.print(": ");
        display.print(smoothData);
    }
}

void loop()
{
    emotibit.update();

    BatteryVoltage = emotibit.readBatteryVoltage();
    BatteryPercentage = emotibit.getBatteryPercent(BatteryVoltage);

    // Serial.print("Battery Percentage: ");
    // Serial.print(BatteryPercentage);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    // Read & display temperature
    readAndDisplayData(EmotiBit::DataType::THERMOPILE, "Temp", 0);

    // Read & display PPG (Green)
    readAndDisplayData(EmotiBit::DataType::PPG_GREEN, "PPG", 8);

    // Read & display EDA
    // readAndDisplayData(EmotiBit::DataType::EDA, "EDA", 16);
    display.setCursor(0, 16);
    display.print("Battery: ");
    display.print(BatteryPercentage);

    // Update and display the current time in Bangkok
    timeClient.update();
    String formattedTime = timeClient.getFormattedTime();
    display.setCursor(0, 24);
    display.print("Time: ");
    display.print(formattedTime);

    display.display();
    delay(1000); // Update every 1 second
}