// //*
// bool connected_ = false;
// #define ARRAY_SIZE 20
// double distance_array_5[ARRAY_SIZE] = {0};
// double distance_array_5_installation[ARRAY_SIZE] = {0};

// // Your GPRS credentials (leave empty, if missing)
// const char apn[] = "iot.1nce.net"; // Access point name. Leave empty if it is not needed.
// const char gprsUser[] = ""; // GPRS User
// const char gprsPass[] = ""; // GPRS Password
// char simPIN[] = ""; // SIM pin number. Leave empty if it is not needed.

// // ******Required for Wakeup***************
// #include "driver/rtc_io.h"

// // ******Required for Wakeup***************
// #include <esp_task_wdt.h>
// //3 seconds WDT
// #define WDT_TIMEOUT 60

// // *****Setting IP5306 (battery management IC) registers and address
// #define IP5306_ADDR 0x75
// #define IP5306_REG_SYS_CTL0 0x00
// #define IP5306_REG_SYS_CTL1 0x01

// // TTGO T-Call pin definitions
// #define MODEM_RST            5
// #define MODEM_PWKEY          4
// #define MODEM_POWER_ON       23
// #define MODEM_TX             27
// #define MODEM_RX             26
// #define I2C_SDA              21
// #define I2C_SCL              22

// // Set serial for debug console (to the Serial Monitor, default speed 115200)
// #define SerialMon Serial
// // Set serial for AT commands (to the module)
// #define SerialAT  Serial1

// // BME280 pins
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BME280.h>
// Adafruit_BME280 bme;
// #define SEALEVELPRESSURE_HPA (1013.25)
// //*************************************

// // Pin assignment for analog sensors (Maxbotix US sensor)
// const int ANALOG_PIN_0 = 35; // ADC1_0 GPIO36
// const int enable_5 = 34;
// int analog_value, distance_mm, distance_cm;
// int distance;
// //******************************************

// // Configure TinyGSM library
// #define TINY_GSM_MODEM_SIM800      // Modem is SIM800
// #define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
// #include <CayenneMQTTGSM.h>
// char username[] = "xx";
// char password[] = "xx";
// char clientID[] = "xx";

// // Define the serial console for debug prints, if needed
// //#define TINY_GSM_DEBUG SerialMon
// //#define DUMP_AT_COMMANDS

// #include <Wire.h>
// #include <TinyGsmClient.h>
// #include <Update.h>

// #define SIM800L_IP5306_VERSION_20190610
// #include "utilities.h"

// TinyGsm modem(SerialAT);
// #define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
// #define TIME_TO_SLEEP  250        /* Time ESP32 will go to sleep (in seconds) 250 corresponds pretty well to 5min */

// // Server details
// const char server[] = "vsh.pp.ua";
// const char resource[] = "/TinyGSM/logo.txt";

// const int  port = 80;

// void setup() {
//   // Set console baud rate
//   SerialMon.begin(115200);
//   delay(20);
//   SerialMon.println("Configuring WDT...");
//   esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
//   esp_task_wdt_add(NULL); //add current thread to WDT watch
//   Wire.begin(I2C_SDA, I2C_SCL);
//   pinMode(ANALOG_PIN_0, INPUT);
//   pinMode(enable_5, OUTPUT);

//     // Start power management
//     if (setupPMU() == false) {
//         Serial.println("Setting power error");
//     }

//   // Set-up modem reset, enable, power pins
//   pinMode(MODEM_PWKEY, OUTPUT);
//   pinMode(MODEM_RST, OUTPUT);
//   pinMode(MODEM_POWER_ON, OUTPUT);
//   digitalWrite(MODEM_PWKEY, LOW);
//   digitalWrite(MODEM_RST, HIGH);
//   digitalWrite(MODEM_POWER_ON, HIGH);

//   // Set GSM module baud rate and UART pins
//   SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
//   delay(3000);

//   esp_sleep_enable_ext0_wakeup(GPIO_NUM_27,0); //enable ext0 interrupt when going from high to low : https://lastminuteengineers.com/esp32-deep-sleep-wakeup-sources/
//   rtc_gpio_pullup_en(GPIO_NUM_27); // pulling 27 normally high so when interrupting it can be pulled low

//   // Restart takes quite some time
//   // To skip it, call init() instead of restart()
//   SerialMon.println("Initializing modem...");
//   modem.restart();
//   // Or, use modem.init() if you don't need the complete restart

// // *************Not sure if this function is necessary but it sets the boost on mode of the IP5306 again
// Wire.beginTransmission(IP5306_ADDR);
// Wire.write(IP5306_REG_SYS_CTL0);
// Wire.write(0b110111);
// // 0x37 = 0b110111 TCALL example
// /*
// [1] Boost EN (default 1) [EXM note: if 0 ESP32 will not boot from battery]
// [1] Charger EN (1) [EXM note: did not observe difference]
// [1] Reserved (1) [EXM note: did not observe difference]
// [1] Insert load auto power EN (1) [EXM note: did not observe difference]
// [1] BOOST output normally open ( 1) [EXM note: if 0 will shutdown when
// ESP32 sleeps after 32s]
// [1] Key off EN: (0) [EXM note: could not detect difference]
// */
// Wire.write(IP5306_REG_SYS_CTL1);
// Wire.write(0x1D); // Set HEX:1D DEC:29 BIN:11101
// /*
// [1] Turn off boost control signal selection: short press twice
// [1] Switch WLED flashlight control signal selection: short press twice
// [1] Short press switch boost: disabled
// [0] Whether to turn on Boost after VIN is pulled out: opened
// [1] Batlow 3.0V Low Power Shutdown EN: enabled
// */
// //***********************************************************************

//   String modemInfo = modem.getModemInfo();
//   SerialMon.print("Modem: ");
//   SerialMon.println(modemInfo);

//   // Unlock your SIM card with a PIN if needed
//   if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
//     modem.simUnlock(simPIN);
//   }
//     SerialMon.println("Still in setup mode and calling Cayenne.begin");
//     Cayenne.begin(username, password, clientID, SerialAT, apn, gprsUser, gprsPass, simPIN);

// }

// void loop() {
//   SerialMon.println("Entering loop and calling Cayenne.loop");
//   Cayenne.loop();
// }

// // Default function for sending sensor data at intervals to Cayenne.
// // You can also use functions for specific channels, e.g CAYENNE_OUT(1) for sending channel 1 data.
// CAYENNE_OUT_DEFAULT()
// {
// int signal_str = signal_strength();
// bme.begin();
// delay(100);

// double temperature = 0;
// double tempsum = 0;
// for (int i = 0; i < 10; i++)
// {
//     tempsum = bme.readTemperature();
//     delay(10);
//     temperature = tempsum + temperature;
// }
// temperature = temperature / 10;

// double humidity = 0;
// double humiditysum = 0;
// for (int i = 0; i < 10; i++)
// {
//     humiditysum = bme.readHumidity();
//     humidity = humiditysum + humidity;
//     delay(10);
// }
// humidity = humidity / 10;

// double pressure = 0;
// double pressuresum = 0;
//   for (int i = 0; i < 10; i++)
//   {
//     pressuresum = bme.readPressure() / 100.0F;
//     pressure = pressuresum + pressure;
//     delay(10);
//   }
// pressure = pressure / 10;
// double distance = determine_distance_5(temperature);
// double batterylvl = getBatteryLevel();
// SerialMon.print("Temperature: ");
// SerialMon.print(temperature);
// SerialMon.print("°C");
// SerialMon.print(", Humidity: ");
// SerialMon.print(humidity);
// SerialMon.print("%RH");
// SerialMon.print(", Pressure: ");
// SerialMon.print(pressure);
// SerialMon.print("hPa");
// SerialMon.print(", Distance: ");
// SerialMon.print(distance);
// SerialMon.print("cm");
// SerialMon.print(", Signal strength: ");
// SerialMon.print(signal_str);
// SerialMon.print(", Battery level: ");
// SerialMon.print(batterylvl);
// SerialMon.println("%");
// SerialMon.println("Transmitting...");
//   // Write data to Cayenne here. This example just sends the current uptime in milliseconds on virtual channel 0.
// Cayenne.virtualWrite(0, temperature, "temp", "c");
// Cayenne.virtualWrite(1, signal_str, "signal", "null");
// Cayenne.virtualWrite(2, distance, "analog_sensor", "null");
// Cayenne.virtualWrite(3, humidity, "rel_hum", "p");
// Cayenne.virtualWrite(4, (double)(pressure), "bp", "hpa");
// Cayenne.virtualWrite(5, (double)(batterylvl), "batt", "p");
// delay(3000);
// modem.gprsDisconnect();
// SerialMon.println(F("GPRS disconnected"));
// //After all off
// modem.poweroff();
// SerialMon.println(F("Poweroff"));
// esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
// esp_deep_sleep_start();
// }

// CAYENNE_CONNECTED() {
//   connected_ = true;
//   Serial.println("I am connected to Cayenne");
// }

// CAYENNE_DISCONNECTED() {
//   connected_ = false;
//   Serial.println("I am DISconnected from Cayenne");
// }

// // Default function for processing actuator commands from the Cayenne Dashboard.
// // You can also use functions for specific channels, e.g CAYENNE_IN(1) for channel 1 commands.
// CAYENNE_IN_DEFAULT()
// {
//   CAYENNE_LOG("Channel %u, value %s", request.channel, getValue.asString());
//   //Process message here. If there is an error set an error message using getValue.setError(), e.g getValue.setError("Error message");
// }

// // Reading the network signal
// int signal_strength() {
//   int csq = modem.getSignalQuality();
//   delay(500);
//   Serial.print("Signal quality:");
//   Serial.println(csq);
//   return (int) (csq);
// }

// double getBatteryLevel()
// {
//   Wire.beginTransmission(0x75);
//   Wire.write(0x78);
//   if (Wire.endTransmission(false) == 0
//    && Wire.requestFrom(0x75, 1)) {
//     switch (Wire.read() & 0xF0) {
//     case 0xE0: return 25;
//     case 0xC0: return 50;
//     case 0x80: return 75;
//     case 0x00: return 100;
//     default: return 0;
//     }
//   }
//   return -1;
// }

// double determine_distance_5(double temperature) {
//   calculate_distance_5(temperature);
//   int ok_index = 0;
//   double deviation = 0.5;
//   double val_ = 0;
//   float average_ = getAverage(distance_array_5, ARRAY_SIZE);
//   average_ = 11.93 + 1.0819 * average_;
//   float std_dev = getStdDev(distance_array_5, ARRAY_SIZE, average_);
//   std_dev = 11.93 + 1.0819 * std_dev;
//   Serial.print("AVG distance: "); Serial.print(average_); Serial.println(" cm");
//   Serial.print("STD distance: "); Serial.print(std_dev); Serial.println(" cm");
//   return average_;
// }

// float getAverage(double * val, int arrayCount) {
//   long total = 0;
//   for (int i = 0; i < arrayCount; i++) {
//     total += val[i];
//   }
//   return total / float(arrayCount);
// }

// float getStdDev(double * val, int arrayCount, double avrg) {
//   float avg = avrg;
//   long total = 0;
//   for (int i = 0; i < arrayCount; i++) {
//     total = total + (val[i] - avg) * (val[i] - avg);
//   }

//   float variance = total / (float)arrayCount;
//   float stdDev = sqrt(variance);
//   return stdDev;
// }

// double calculate_distance_5(double temperature) {
//   Serial.println("Starting with the US sensor measurement ");
//   for (int i = 0; i < ARRAY_SIZE - 1; i++) {
//     distance_array_5[i] = calculateDistanceUS_5(temperature);
//     delay(50);
//   }
// }

// double calculateDistanceUS_5(double temperature) {
//   digitalWrite(enable_5, HIGH);
//   delay(10);
//   float adc = (analogRead(ANALOG_PIN_0) / 4096.0) * 3385.0;
//   float distance_20 = (adc / 3385.0) * 500; //distance in cm
//   float coeff = (sqrt(temperature + 273.15) / 17.12);
//   float distance_compansated = distance_20 * coeff;
//   digitalWrite(enable_5, LOW);
//   return (double)(distance_20);
// }