// #include <Arduino.h>
// #include "EmotiBit.h"
// #include <Firebase_ESP_Client.h>
// #include "time.h"

// /* Provide the token generation process info. */
// #include "addons/TokenHelper.h"
// /* Provide the RTDB payload printing info and other helper functions. */
// #include "addons/RTDBHelper.h"

// /* Insert Firebase project API Key */
// #define API_KEY "AIzaSyBQMCCbXNfV4-AwZiH1z40SmTfgOwuLJgU"
// /* Insert RTDB URL */
// #define DATABASE_URL "https://emotibit-32581-default-rtdb.asia-southeast1.firebasedatabase.app/"

// const char *ntpServer = "pool.ntp.org";
// struct tm timeinfo;
// unsigned long epochTime;
// unsigned long getTime()
// {
//     time_t now;
//     struct tm timeinfo;
//     if (!getLocalTime(&timeinfo))
//     {
//         Serial.println("Failed to obtain time");
//         return (0);
//     }
//     time(&now);
//     return now;
// }

// #define SerialUSB SERIAL_PORT_USBVIRTUAL // Required to work in Visual Micro / Visual Studio IDE
// const uint32_t SERIAL_BAUD = 2000000;    // 115200

// EmotiBit emotibit;
// /* Declare variables for handling data from Emotibit */
// const size_t dataSize = EmotiBit::MAX_DATA_BUFFER_SIZE;
// float eda[dataSize];
// size_t dataEDA;
// size_t dataPPG;
// float skinTempData[dataSize];
// float heartRateData[dataSize];
// float batteryData[dataSize];
// float batteryVoltData[dataSize];
// float ppg[dataSize];

// /* Declare variables for connecting and sending data to Realtime Database */
// FirebaseData fbdo;
// FirebaseAuth Auth;
// FirebaseConfig config;
// bool signupOK = false;

// void onShortButtonPress()
// {
//     /* Toggle wifi on/off */
//     if (emotibit.getPowerMode() == EmotiBit::PowerMode::NORMAL_POWER)
//     {
//         emotibit.setPowerMode(EmotiBit::PowerMode::WIRELESS_OFF);
//     }
//     else
//     {
//         emotibit.setPowerMode(EmotiBit::PowerMode::NORMAL_POWER);
//     }
// }

// void onLongButtonPress()
// {
//     emotibit.sleep();
// }

// void setup()
// {
//     Serial.begin(SERIAL_BAUD);
//     delay(2000);

//     /* Capture the calling ino into firmware_variant information */
//     String inoFilename = __FILE__;
//     inoFilename.replace("/", "\\");
//     if (inoFilename.lastIndexOf("\\") != -1)
//     {
//         inoFilename = inoFilename.substring((inoFilename.lastIndexOf("\\")) + 1, (inoFilename.indexOf(".")));
//     }
//     emotibit.setup(inoFilename);

//     /* Attach callback functions */
//     emotibit.attachShortButtonPress(&onShortButtonPress);
//     emotibit.attachLongButtonPress(&onLongButtonPress);

//     /* Assign the api key (required) */
//     config.api_key = API_KEY;

//     /* Assign the RTDB URL (required) */
//     config.database_url = DATABASE_URL;

//     /* Sign up */
//     if (Firebase.signUp(&config, &Auth, "", ""))
//     {
//         Serial.println("Sign up successful");
//         signupOK = true;
//     }
//     else
//     {
//         Serial.printf("Sign up error: %s\n", config.signer.signupError.message.c_str());
//     }

//     /* Assign the callback function for the long running token generation task */
//     config.token_status_callback = tokenStatusCallback;

//     /* Connect to Firebase */
//     Firebase.begin(&config, &Auth);
//     Firebase.reconnectWiFi(true);

//     /* Connect to NTP pool server */
//     configTime(0, 0, ntpServer);

//     /* Set timezone as Bangkok - GMT+7 */
//     setenv("TZ", "<+07>-7", 1);
//     tzset();

//     /* Retrieve the device ID */
//     String deviceID = emotibit.getFeatherMacAddress();
//     Serial.print("Device ID: ");
//     Serial.println(deviceID);
// }

// void loop()
// {
//     /* Read physiological signals from Emotibit */
//     emotibit.update();
//     dataEDA = emotibit.readData(EmotiBit::DataType::EDA, &eda[0], dataSize);
//     dataPPG = emotibit.readData(EmotiBit::DataType::PPG_GREEN, &ppg[0], dataSize);

//     size_t skinTempAvailable = emotibit.readData(EmotiBit::DataType::THERMOPILE, &skinTempData[0], dataSize);
//     batteryVoltData[0] = emotibit.readBatteryVoltage();
//     batteryData[0] = emotibit.getBatteryPercent(batteryVoltData[0]);

//     /* Get timestamp */
//     epochTime = getTime();
//     getLocalTime(&timeinfo);

//     if (dataEDA > 0)
//     {
//         if (Firebase.ready() && signupOK)
//         {
//             /* Create JSON data for Realtime Database */
//             FirebaseJson jsonData;

//             char date[15];
//             char times[15];
//             strftime(date, 15, "%d-%B-%Y", &timeinfo);
//             strftime(times, 15, "%H:%M:%S", &timeinfo);

//             /* Retrieve the device ID */
//             String deviceID = emotibit.emotibitDeviceId;

//             /* Create path: /Emotibit/{deviceID}/bed01/{date}/{time} */
//             String path = "/Emotibit/" + deviceID + "/" + String(date) + "/" + String(times);

//             /* Populate JSON data */
//             jsonData.set("Timestamp", int(epochTime));
//             for (size_t i = 0; i < 100 && i < dataSize; i++)
//             {
//                 if (i < 15)
//                 {
//                     jsonData.set("EDA_array/" + String(i), eda[i]);
//                 }
//                 jsonData.set("PPG_array/" + String(i), ppg[i]);
//             }
//             jsonData.set("SkinTemp", skinTempData[0]);
//             jsonData.set("BatteryPercent", batteryData[0]);
//             jsonData.set("BatteryVolt", batteryVoltData[0]);

//             /* Send data to Realtime Database */
//             Serial.println("Sending data to Realtime Database...");
//             if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &jsonData))
//             {
//                 Serial.println("Data sent successfully!");
//             }
//             else
//             {
//                 Serial.print("Failed to send data: ");
//                 Serial.println(fbdo.errorReason());
//             }
//         }
//     }
// }