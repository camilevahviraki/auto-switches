#include <SPI.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <Firebase_ESP_Client.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

SoftwareSerial softSerial(D6, D5);

#define FIREBASE_HOST "auto-switch-full.firebaseio.com"
#define API_KEY "AIzaSyARXsRvmUzoJBvs16XxWt2CXQ_1blQ5HHk"
// Insert Authorized Email and Corresponding Password
#define USER_EMAIL "usertest@gmail.com"
#define USER_PASSWORD "qwerty243"
#define DATABASE_URL "https://auto-switch-full-default-rtdb.firebaseio.com/"

const char* ssid = "JulienWifi";
const char* password = "123456789";

String tempPath = "/temperature";
String oxygenPath = "/oxygen";
String hearRatePath = "/heartRate";
String uid;
String databasePath;
String chartPath;
String parentPath;
int timestamp;
int hideRate = 1;
int wifiAvailable;

FirebaseJson json;
FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 2000;

void setup() {

  Serial.begin(9600);
  timeClient.begin();
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  
   softSerial.begin(9600);

  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  Firebase.reconnectWiFi(true);
  firebaseData.setResponseSize(4096);
  config.token_status_callback = tokenStatusCallback;
  config.max_token_generation_retry = 5;
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  Firebase.begin(&config, &auth);
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  // Print user UID
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);
    databasePath = "/UsersData/" + uid ;
  delay(2000);

}


void loop() {
 

 
    if (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0){
      sendDataPrevMillis = millis();
      updateData();
    }
//
//    if(millis() - chartPrevMillis > updateChartTime || chartPrevMillis == 0){
//      chartPrevMillis = millis();
//      updateChart();
//    }
  
}

void updateData() {

  parentPath= databasePath + String("/currentData");
// ////////////////////////////////////////////////////////////////
  if (softSerial.available() > 0) {
    // Read incoming data until a newline character is received
    String json_string = softSerial.readStringUntil('\n');
    DynamicJsonDocument json_doc(256); // Adjust the size based on your JSON
    DeserializationError error = deserializeJson(json_doc, json_string);

    if (error) {
      Serial.print("JSON parsing failed: ");
      Serial.println(error.c_str());
    } else {
      // send JSON data
       Firebase.RTDB.setFloat(&firebaseData, "data/realtime/current", 67);
       Firebase.RTDB.setFloat(&firebaseData, "data/realtime/voltage", 456);
       Firebase.RTDB.setFloat(&firebaseData, "data/realtime/power", 45); 
//        if (Firebase.RTDB.setJSON(&firebaseData, parentPath.c_str(), &json)) {
//         Serial.println("JSON data set successfully!");
//        }
    }
  }

//  /////////////////////////////////////////////////////////////
}

void updateChart(float temp, float heartBeat, float oxygen) {
     timestamp = getTime();
   json.set(tempPath.c_str(), String(temp));
    json.set(hearRatePath.c_str(), String(heartBeat));
    json.set(oxygenPath.c_str(), String(oxygen));
    String path2 = databasePath + "/" + String("chartData") + "/" + String(timestamp);
  
  if (Firebase.RTDB.setJSON(&firebaseData, path2.c_str(), &json)) {
    Serial.println("JSON data set successfully!");
    wifiAvailable = 1;
  } else {
    wifiAvailable = 0;
    Serial.println("Failed to set JSON data.");
    Serial.println("Error: " + firebaseData.errorReason());
  }
 // Send data every minute (adjust as needed)

}

unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}
