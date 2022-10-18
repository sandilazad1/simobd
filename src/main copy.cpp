// const char apn[]      = "internet"; // APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
// const char gprsUser[] = ""; // GPRS User
// const char gprsPass[] = ""; // GPRS Password
// const char simPIN[]   = ""; 

// //const char server[] = "example.com"; // domain name: example.com, maker.ifttt.com, etc
// //const char resource[] = "/post-data.php";         // resource path, for example: /post-data.php
// //const int  port = 80;                             // server port number


// #define MODEM_RST            5
// #define MODEM_PWKEY          4
// #define MODEM_POWER_ON       23





// #define GSM_AUTOBAUD_MIN 9600
// #define GSM_AUTOBAUD_MAX 115200


// #define SerialMon Serial
// #define  SerialAT Serial1

// #define TINY_GSM_MODEM_SIM800      // Modem is SIM800
// #define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// const char* broker = "54.174.12.126";
// const char* obdtopicSend       = "obd/866262037106043";


// #include <Wire.h>
// #include <TinyGsmClient.h>
// #include <PubSubClient.h>
// #include <ArduinoJson.h>
// int lastReconnectAttempt =  0;
//   float h = 5.6 ;
//   float t = 90.0;
// TinyGsm        modem(SerialAT);

// TinyGsmClient client(modem);
// PubSubClient  mqtt(client);



// void messageHandler(char* topic, byte* payload, unsigned int length)
// {
//   Serial.print("incoming: ");
//   Serial.println(topic);
 
//   StaticJsonDocument<200> doc;
//   deserializeJson(doc, payload);
//   const char* message = doc["message"];
//   Serial.println(message);
// }



// boolean connectAWS()
// {
 
//   // Create a message handler 
//   Serial.println("Connecting to AWS IOT");
 
//   while (!mqtt.connect("01"))
//   {
//     Serial.print(".");
//     delay(100);
//   }
 
//   if (!mqtt.connected())
//   {
//     Serial.println("AWS IoT Timeout!");
//     return false;
//   }
 
//   mqtt.subscribe("test");
//   Serial.println("AWS IoT Connected!");
//   mqtt.publish(obdtopicSend, "device started publish");
//   return mqtt.connected();
// }







// void publishMessage()
// {
//   StaticJsonDocument<200> doc;
//   doc["humidity"] = h;
//   doc["temperature"] = t;
//   char jsonBuffer[512];
//   serializeJson(doc, jsonBuffer); // print to client
 
//   mqtt.publish(obdtopicSend, jsonBuffer);
// }




// void setup() {
//   SerialMon.begin(9600);

//   SerialMon.println("Wait...");

//   // Set GSM module baud rate
//   TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
//   // SerialAT.begin(9600);
//   delay(6000);


//   SerialMon.println("Initializing modem...");
//   //modem.restart();
//   // modem.init();

//   String modemInfo = modem.getModemInfo();
//   SerialMon.print("Modem Info: ");
//   SerialMon.println(modemInfo);

// #if TINY_GSM_USE_GPRS
//   // Unlock your SIM card with a PIN if needed
//   if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }
// #endif

//   SerialMon.print("Waiting for network...");
//   if (!modem.waitForNetwork()) {
//     SerialMon.println(" fail");
//     delay(10000);
//     return;
//   }
//   SerialMon.println(" success");

//   if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }

//   SerialMon.print(F("Connecting to "));
//   SerialMon.print(apn);
//   if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
//     SerialMon.println(" fail");
//     delay(10000);
//     return;
//   }
//   SerialMon.println(" success");

//   if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }

//   mqtt.setServer(broker, 1883);
//   mqtt.setCallback(messageHandler);

// }

// void loop() {
//   h = 5.6 ;
//   t = 90.0;


//   if (!modem.isNetworkConnected()) {
//     SerialMon.println("Network disconnected");
//     if (!modem.waitForNetwork(180000L, true)) {
//       SerialMon.println(" fail");
//       delay(10000);
//       return;
//     }
//     if (modem.isNetworkConnected()) {
//       SerialMon.println("Network re-connected");
//     }

//     // and make sure GPRS/EPS is still connected
//     if (!modem.isGprsConnected()) {
//       SerialMon.println("GPRS disconnected!");
//       SerialMon.print(F("Connecting to "));
//       SerialMon.print(apn);
//       if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
//         SerialMon.println(" fail");
//         delay(10000);
//         return;
//       }
//       if (modem.isGprsConnected()) { SerialMon.println("GPRS reconnected"); }
//     }

//   }

//     if (!mqtt.connected()) {
//     SerialMon.println("=== MQTT NOT CONNECTED ===");
//     //Reconnect every 10 seconds
//     uint32_t t = millis();
//     if (t - lastReconnectAttempt > 5000L) {
//       lastReconnectAttempt = t;
//       if (connectAWS()) { lastReconnectAttempt = 0; }
//     }
//     delay(1000);
//     return;
//   }
 
//   if (isnan(h) || isnan(t) )  // Check if any reads failed and exit early (to try again).
//   {
//     Serial.println(F("Failed to read from DHT sensor!"));
//     return;
//   }
 
//   Serial.print(F("Humidity: "));
//   Serial.print(h);
//   Serial.print(F("%  Temperature: "));
//   Serial.print(t);
//   Serial.println(F("°C "));
//   publishMessage();
//   mqtt.loop();
//   delay(1000);
// }