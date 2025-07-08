// #include <Arduino.h>
// #include "EmotiBit.h"
// #include <Firebase_ESP_Client.h>
// #include "time.h"
// #include "addons/TokenHelper.h"
// #include "addons/RTDBHelper.h"

// // Firebase configuration
// #define API_KEY "AIzaSyBQMCCbXNfV4-AwZiH1z40SmTfgOwuLJgU"
// #define DATABASE_URL "https://emotibit-32581-default-rtdb.asia-southeast1.firebasedatabase.app/"

// // NTP server configuration
// const char *ntpServer = "pool.ntp.org";
// const char *timezone = "<+07>-7"; // Bangkok timezone

// // Serial configuration
// #define SerialUSB SERIAL_PORT_USBVIRTUAL
// const uint32_t SERIAL_BAUD = 2000000;

// // EmotiBit instance
// EmotiBit emotibit;

// // Firebase instances
// FirebaseData fbdo;
// FirebaseAuth auth;
// FirebaseConfig config;

// // Global variables
// bool firebaseReady = false;
// unsigned long epochTime;
// struct tm timeinfo;

// // Data buffers
// const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
// float eda[dataSize];
// float ppg[dataSize];
// float skinTempData[dataSize];
// float batteryData;
// float batteryVoltData;

// // Function prototypes
// void configureFirebase();
// void configureNTP();
// unsigned long getCurrentEpochTime();
// void sendToFirebase(const String &deviceID);
// void handleShortButtonPress();
// void handleLongButtonPress();

// void setup()
// {
//     Serial.begin(SERIAL_BAUD);
//     delay(2000);

//     // EmotiBit setup
//     String inoFilename = __FILE__;
//     inoFilename.replace("/", "\\");
//     inoFilename = inoFilename.substring(inoFilename.lastIndexOf("\\") + 1, inoFilename.indexOf("."));
//     emotibit.setup(inoFilename);

//     emotibit.attachShortButtonPress(&handleShortButtonPress);
//     emotibit.attachLongButtonPress(&handleLongButtonPress);

//     // Configure Firebase
//     configureFirebase();

//     // Configure NTP
//     configureNTP();

//     // Print device ID
//     String deviceID = emotibit.getFeatherMacAddress();
//     Serial.print("Device ID: ");
//     Serial.println(deviceID);
// }

// void loop()
// {
//     // Update EmotiBit data
//     emotibit.update();

//     size_t dataEDA = emotibit.readData(EmotiBit::DataType::EDA, eda, dataSize);
//     size_t dataPPG = emotibit.readData(EmotiBit::DataType::PPG_GREEN, ppg, dataSize);

//     emotibit.readData(EmotiBit::DataType::THERMOPILE, skinTempData, dataSize);
//     batteryVoltData = emotibit.readBatteryVoltage();
//     batteryData = emotibit.getBatteryPercent(batteryVoltData);

//     // Get current time
//     epochTime = getCurrentEpochTime();

//     if (dataEDA > 0 && firebaseReady)
//     {
//         String deviceID = emotibit.emotibitDeviceId;
//         sendToFirebase(deviceID);
//     }
// }

// void configureFirebase()
// {
//     config.api_key = API_KEY;
//     config.database_url = DATABASE_URL;

//     if (Firebase.signUp(&config, &auth, "", ""))
//     {
//         Serial.println("Firebase signup successful.");
//         firebaseReady = true;
//     }
//     else
//     {
//         Serial.printf("Firebase signup error: %s\n", config.signer.signupError.message.c_str());
//     }

//     config.token_status_callback = tokenStatusCallback;
//     Firebase.begin(&config, &auth);
//     Firebase.reconnectWiFi(true);
// }

// void configureNTP()
// {
//     configTime(0, 0, ntpServer);
//     setenv("TZ", timezone, 1);
//     tzset();
// }

// unsigned long getCurrentEpochTime()
// {
//     if (!getLocalTime(&timeinfo))
//     {
//         Serial.println("Failed to obtain time.");
//         return 0;
//     }
//     time_t now;
//     time(&now);
//     return now;
// }

// void sendToFirebase(const String &deviceID)
// {
//     FirebaseJson jsonData;

//     char date[15], time[30];
//     strftime(date, sizeof(date), "%d-%b-%Y", &timeinfo);
//     strftime(time, sizeof(time), "%H:%M:%S", &timeinfo);

//     String path = "/Emotibit/" + deviceID + "/" + date + "/" + time;

//     jsonData.set("Timestamp", static_cast<int>(epochTime));
//     for (size_t i = 0; i < 100 && i < dataSize; i++)
//     {
//         if (i < 15)
//         {
//             jsonData.set("EDA_array/" + String(i), eda[i]);
//         }
//         jsonData.set("PPG_array/" + String(i), ppg[i]);
//     }
//     jsonData.set("SkinTemp", skinTempData[0]);
//     jsonData.set("BatteryPercent", batteryData);
//     jsonData.set("BatteryVolt", batteryVoltData);

//     Serial.println("Sending data to Firebase...");
//     if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &jsonData))
//     {
//         Serial.println("Data sent successfully!");
//     }
//     else
//     {
//         Serial.printf("Failed to send data: %s\n", fbdo.errorReason().c_str());
//     }
// }

// void handleShortButtonPress()
// {
//     if (emotibit.getPowerMode() == EmotiBit::PowerMode::NORMAL_POWER)
//     {
//         emotibit.setPowerMode(EmotiBit::PowerMode::WIRELESS_OFF);
//     }
//     else
//     {
//         emotibit.setPowerMode(EmotiBit::PowerMode::NORMAL_POWER);
//     }
// }

// void handleLongButtonPress()
// {
//     emotibit.sleep();
// }
