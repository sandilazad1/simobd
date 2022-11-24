/**************************************************************
 *
 * For this example, you need to install PubSubClient library:
 *   https://github.com/knolleary/pubsubclient
 *   or from http://librarymanager/all#PubSubClient
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 * For more MQTT examples, see PubSubClient library
 *
 **************************************************************
 * This example connects to HiveMQ's showcase broker.
 *
 * You can quickly test sending and receiving messages from the HiveMQ webclient
 * available at http://www.hivemq.com/demos/websocket-client/.
 *
 * Subscribe to the topic GsmClientTest/ledStatus
 * Publish "toggle" to the topic GsmClientTest/led and the LED on your board
 * should toggle and you should see a new message published to
 * GsmClientTest/ledStatus with the newest LED status.
 *
 **************************************************************/

// Select your modem:
#define TINY_GSM_MODEM_SIM800
// #define TINY_GSM_MODEM_SIM808
// #define TINY_GSM_MODEM_SIM868
// #define TINY_GSM_MODEM_SIM900
// #define TINY_GSM_MODEM_SIM7000
// #define TINY_GSM_MODEM_SIM7000SSL
// #define TINY_GSM_MODEM_SIM7080
// #define TINY_GSM_MODEM_SIM5360
// #define TINY_GSM_MODEM_SIM7600
// #define TINY_GSM_MODEM_UBLOX
// #define TINY_GSM_MODEM_SARAR4
// #define TINY_GSM_MODEM_M95
// #define TINY_GSM_MODEM_BG96
// #define TINY_GSM_MODEM_A6
// #define TINY_GSM_MODEM_A7
// #define TINY_GSM_MODEM_M590
// #define TINY_GSM_MODEM_MC60
// #define TINY_GSM_MODEM_MC60E
// #define TINY_GSM_MODEM_ESP8266
// #define TINY_GSM_MODEM_XBEE
// #define TINY_GSM_MODEM_SEQUANS_MONARCH

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1
#define SerialGps Serial2

// or Software Serial on Uno, Nano

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

#define MYPORT_TX 23
#define MYPORT_RX 22

// Range to attempt to autobaud
// NOTE:  DO NOT AUTOBAUD in production code.  Once you've established
// communication, set a fixed baud rate using modem.setBaud(#).
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet.
// This is only needed for this example, not in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
const char wifiSSID[] = "YourSSID";
const char wifiPass[] = "YourWiFiPass";

// MQTT details
const char *broker = "50.19.50.166";
const char *obdDash = "eraobd/obddata/obddash/866262037106043";
const char *miniObdDash = "eraobd/obddata/miniobddash/866262037106043";
const char *obdGps = "eraobd/obddata/gps/866262037106043";
const char *obdDiag = "eraobd/obddata/obddiag/866262037106043";
const char *obdDiagSub = "eraobd/obddata/obddiag/sub/866262037106043";

const char server[] = "52.73.212.48";
const int port = 80;

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "ELMduino.h"
#include <ArduinoJson.h>
#include "obd.h"
#include <TinyGPSPlus.h>
#include <ArduinoHttpClient.h>
#include <CRC32.h>
#include "FS.h"
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <Update.h>

#include <esp_task_wdt.h>
#define WDT_TIMEOUT 3600

const char *version = "3.0";
void updateFw(void);
void fwupdatecheck(void);
void obdDiagpublishMessage();

static const uint32_t GPSBaud = 9600;

// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else

// #if !defined(TINY_GSM_RX_BUFFER)
// #define TINY_GSM_RX_BUFFER 650
// #endif

#define TINY_GSM_RX_BUFFER 1030

TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem, 0);
PubSubClient mqtt(client);

TinyGsmClient clientfoupdate(modem, 1);
HttpClient http(clientfoupdate, server, port);

float batteryvoltage = 0.0;
int8_t get_vin_blocking = 0;
uint32_t supportedpids_1_20 = 0;
uint32_t monitorstatus = 0;
uint16_t freezedtc = 0;
uint16_t fuelsystemstatus = 0;
float engineload = 0.0;
float enginecoolanttemp = 0.0;
float shorttermfueltrimbank_1 = 0.0;
float longtermfueltrimbank_1 = 0.0;
float shorttermfueltrimbank_2 = 0.0;
float longtermfueltrimbank_2 = 0.0;
float fuelpressure = 0.0;
uint8_t manifoldpressure = 0;
float rpm = 0.0;
int32_t kph = 0;
float mph = 0.0;
float timingadvance = 0.0;
float intakeairtemp = 0.0;
float mafrate = 0.0;
float throttle = 0.0;
uint8_t commandedsecairstatus = 0;
uint8_t oxygensensorspresent_2banks = 0;
uint8_t obdstandards = 0;
uint8_t oxygensensorspresent_4banks = 0;
bool auxinputstatus = false;
uint16_t runtime = 0;
uint32_t supportedpids_21_40 = 0;
uint16_t disttravelwithmil = 0;
float fuelrailpressure = 0.0;
float fuelrailguagepressure = 0.0;
float commandedegr = 0.0;
float egrerror = 0.0;
float commandedevappurge = 0.0;
float fuellevel = 0.0;
uint8_t warmupssincecodescleared = 0;
uint16_t distsincecodescleared = 0;
float evapsysvappressure = 0.0;
uint8_t absbaropressure = 0;
float cattempb1s1 = 0.0;
float cattempb2s1 = 0.0;
float cattempb1s2 = 0.0;
float cattempb2s2 = 0.0;
uint32_t supportedpids_41_60 = 0;
uint32_t monitordrivecyclestatus = 0;
float ctrlmodvoltage = 0.0;
float absload = 0.0;
float commandedairfuelratio = 0.0;
float relativethrottle = 0.0;
float ambientairtemp = 0.0;
float absthrottleposb = 0.0;
float absthrottleposc = 0.0;
float absthrottleposd = 0.0;
float absthrottlepose = 0.0;
float absthrottleposf = 0.0;
float commandedthrottleactuator = 0.0;
uint16_t timerunwithmil = 0;
uint16_t timesincecodescleared = 0;
float maxmafrate = 0.0;
uint8_t fueltype = 0;
float ethonolpercent = 0.0;
float absevapsysvappressure = 0.0;
float evapsysvappressure2 = 0.0;
float absfuelrailpressure = 0.0;
float relativepedalpos = 0.0;
float hybridbatlife = 0.0;
float oiltemp = 0.0;
float fuelinjecttiming = 0.0;
float fuelrate = 0.0;
uint8_t emissionrqmts = 0.0;
uint32_t supportedpids_61_80 = 0;
float demandedtorque = 0.0;
float torque = 0.0;
String DTCCODE = "";
uint16_t referencetorque = 0;
uint16_t auxsupported = 0;
int check = 5;
int check2 = 6;
int check3 = 7;
String read_dtc_();
uint32_t knownCRC32 = 0x6f50d767;
uint32_t knownFileSize = 1024; // In case server does not send it

uint32_t previousMills = 0;
const long intervalPUB = 5000;
uint32_t gpsPreviousMills = 0;
const long intervalGps = 30000;
uint32_t updatePreviousMills = 0;
const long intervalUpdate = 1200000;
uint32_t resetPreviousMills = 0;
const long intervalreset = 30000; // 3600000;

int iCheckReset = 0;

float lat = 0.0;
float lng = 0.0;
float speed = 0.0;
const char *imei = "866262037106043";
uint32_t lastReconnectAttempt = 0;

const bool DEBUG = true;
const int TIMEOUT = 2000;
const bool HALT_ON_FAIL = false;

#define OBD_READY 0
#define OBD_SEND 1
#define OBD_READ 2

#define READ_DTC__ 0
#define FOUND_4_ 1
#define FOUND_1 2
#define FOUND_SPACE_ 3
#define FOUND_0_ 4
#define FOUND_1_ 5
const String conversion_[16] = {"P0", "P1", "P2", "P3",
                                "C0", "C1", "C2", "C3",
                                "B0", "B1", "B2", "B3",
                                "U0", "U1", "U2", "U3"};
int OBD_state = 0;
char inChar;
String resp = "";
unsigned long timeout_read = 0;
unsigned long timeout_talk = 0;
uint8_t num_ecu = 0;

#define OBD_READ_TIMEOUT 1000 // ms
#define OBD_TALK_TIMEOUT 2000 // ms

SoftwareSerial obdSerial;

ELM327 myELM327;

TinyGPSPlus gps;
//################################################miniObdDash##############################################
obd_pid_states mini_bd_Dash = BATTERYVOLTAGE;
boolean miniobdDashBoad()
{

    switch (mini_bd_Dash)
    {
        //########################################BATTERYVOLTAGE###########################

    case BATTERYVOLTAGE:
    {
        batteryvoltage = myELM327.batteryVoltage();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("batteryvoltage: ");
            Serial.println(batteryvoltage);
            mini_bd_Dash = ENGINELOAD;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = ENGINELOAD;
        }

        break;
    }

    case ENGINELOAD:
    {
        engineload = myELM327.engineLoad();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("engineload: ");
            Serial.println(engineload);
            mini_bd_Dash = ENGINECOOLANTTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = ENGINECOOLANTTEMP;
        }

        break;
    } //########################################ENGINECOOLANTTEMP###########################

    case ENGINECOOLANTTEMP:
    {
        enginecoolanttemp = myELM327.engineCoolantTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("enginecoolanttemp: ");
            Serial.println(enginecoolanttemp);
            mini_bd_Dash = RPM;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = RPM;
        }

        break;
    }
        //########################################RPM###########################
    case RPM:
    {
        rpm = myELM327.rpm();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("rpm: ");
            Serial.println(rpm);
            mini_bd_Dash = KPH;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = KPH;
        }

        break;
    } //########################################KPH###########################

    case KPH:
    {
        kph = myELM327.kph();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("kph: ");
            Serial.println(kph);
            mini_bd_Dash = TIMINGADVANCE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = TIMINGADVANCE;
        }

        break;
    } //########################################MPH###########################

    case TIMINGADVANCE:
    {
        timingadvance = myELM327.timingAdvance();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("timingadvance: ");
            Serial.println(timingadvance);
            mini_bd_Dash = INTAKEAIRTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = INTAKEAIRTEMP;
        }

        break;
    } //########################################INTAKEAIRTEMP###########################

    case INTAKEAIRTEMP:
    {
        intakeairtemp = myELM327.intakeAirTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("intakeairtemp: ");
            Serial.println(intakeairtemp);
            mini_bd_Dash = THROTTLE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = THROTTLE;
        }

        break;
    }

        //########################################THROTTLE###########################

    case THROTTLE:
    {
        throttle = myELM327.throttle();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("throttle: ");
            Serial.println(throttle);
            mini_bd_Dash = AUXINPUTSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = AUXINPUTSTATUS;
        }

        break;

    case AUXINPUTSTATUS:
    {
        auxinputstatus = myELM327.auxInputStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("auxinputstatus: ");
            Serial.println(auxinputstatus);
            mini_bd_Dash = RUNTIME;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = RUNTIME;
        }

        break;
    } //########################################RUNTIME###########################

    case RUNTIME:
    {
        runtime = myELM327.runTime();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("runtime: ");
            Serial.println(runtime);
            mini_bd_Dash = AMBIENTAIRTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = AMBIENTAIRTEMP;
        }

        break;
    }

    case AMBIENTAIRTEMP:
    {
        ambientairtemp = myELM327.ambientAirTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("ambientairtemp: ");
            Serial.println(ambientairtemp);
            mini_bd_Dash = HYBRIDBATLIFE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = HYBRIDBATLIFE;
        }

        break;
    }

        //########################################HYBRIDBATLIFE###########################

    case HYBRIDBATLIFE:
    {
        hybridbatlife = myELM327.hybridBatLife();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("hybridbatlife: ");
            Serial.println(hybridbatlife);
            mini_bd_Dash = TORQUE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            mini_bd_Dash = TORQUE;
        }

        break;
    } //########################################OILTEMP###########################

    case TORQUE:
    {
        DTCCODE = read_dtc_();

        Serial.print("DTCCODE: ");
        Serial.println(DTCCODE);
        mini_bd_Dash = BATTERYVOLTAGE;

        check3 = 0;
        delay(2000);
        break;
    }
    }
    }

    return 0;
}

void miniobdDashpublishMessage()
{
    StaticJsonDocument<1500> doc;
    doc["imei"] = imei;
    doc["auxInputStatus"] = auxinputstatus;
    doc["batteryVoltage"] = batteryvoltage;
    doc["engineLoad"] = engineload;
    doc["intakeAirTemp"] = intakeairtemp;
    doc["kph"] = kph;
    doc["rpm"] = rpm;
    doc["runTime"] = runtime;
    doc["Temp"] = enginecoolanttemp;
    doc["throttle"] = throttle;
    doc["ambientairTemp"] = ambientairtemp;
    doc["hybridBatlife"] = hybridbatlife;
    doc["dtc"] = DTCCODE;
    doc["version"] = version;

    char jsonBuffer[1900];
    serializeJson(doc, jsonBuffer);
    mqtt.publish(miniObdDash, jsonBuffer);
}

//################################################obdDash##############################################
obd_pid_states obd_Dash = BATTERYVOLTAGE;
boolean obdDashBoad()
{

    switch (obd_Dash)
    {
        //########################################BATTERYVOLTAGE###########################

    case BATTERYVOLTAGE:
    {
        batteryvoltage = myELM327.batteryVoltage();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("batteryvoltage: ");
            Serial.println(batteryvoltage);
            obd_Dash = MONITORSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = MONITORSTATUS;
        }

        break;
    }

        //########################################MONITORSTATUS###########################

    case MONITORSTATUS:
    {
        monitorstatus = myELM327.monitorStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("monitorstatus");
            Serial.println(monitorstatus);
            obd_Dash = FREEZEDTC;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = FREEZEDTC;
        }

        break;
    } //########################################FREEZEDTC###########################

    case FREEZEDTC:
    {
        freezedtc = myELM327.freezeDTC();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("freezedtc: ");
            Serial.println(freezedtc);
            obd_Dash = FUELSYSTEMSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = FUELSYSTEMSTATUS;
        }

        break;
    } //########################################FUELSYSTEMSTATUS###########################

    case FUELSYSTEMSTATUS:
    {
        fuelsystemstatus = myELM327.fuelSystemStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelsystemstatus: ");
            Serial.println(fuelsystemstatus);
            obd_Dash = ENGINELOAD;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = ENGINELOAD;
        }

        break;
    } //########################################ENGINELOAD###########################

    case ENGINELOAD:
    {
        engineload = myELM327.engineLoad();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("engineload: ");
            Serial.println(engineload);
            obd_Dash = ENGINECOOLANTTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = ENGINECOOLANTTEMP;
        }

        break;
    } //########################################ENGINECOOLANTTEMP###########################

    case ENGINECOOLANTTEMP:
    {
        enginecoolanttemp = myELM327.engineCoolantTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("enginecoolanttemp: ");
            Serial.println(enginecoolanttemp);
            obd_Dash = SHORTTERMFUELTRIMBANK_1;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = SHORTTERMFUELTRIMBANK_1;
        }

        break;
    } //########################################SHORTTERMFUELTRIMBANK_1###########################

    case SHORTTERMFUELTRIMBANK_1:
    {
        shorttermfueltrimbank_1 = myELM327.shortTermFuelTrimBank_1();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("shorttermfueltrimbank_1: ");
            Serial.println(shorttermfueltrimbank_1);
            obd_Dash = LONGTERMFUELTRIMBANK_1;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = LONGTERMFUELTRIMBANK_1;
        }

        break;
    } //########################################LONGTERMFUELTRIMBANK_1###########################

    case LONGTERMFUELTRIMBANK_1:
    {
        longtermfueltrimbank_1 = myELM327.longTermFuelTrimBank_1();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_1_20: ");
            Serial.println(supportedpids_1_20);
            obd_Dash = SHORTTERMFUELTRIMBANK_2;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = SHORTTERMFUELTRIMBANK_2;
        }

        break;
    } //########################################SHORTTERMFUELTRIMBANK_2###########################

    case SHORTTERMFUELTRIMBANK_2:
    {
        shorttermfueltrimbank_2 = myELM327.shortTermFuelTrimBank_2();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_1_20: ");
            Serial.println(supportedpids_1_20);
            obd_Dash = LONGTERMFUELTRIMBANK_2;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = LONGTERMFUELTRIMBANK_2;
        }

        break;
    } //########################################LONGTERMFUELTRIMBANK_2###########################

    case LONGTERMFUELTRIMBANK_2:
    {
        longtermfueltrimbank_2 = myELM327.longTermFuelTrimBank_2();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("longtermfueltrimbank_2: ");
            Serial.println(longtermfueltrimbank_2);
            obd_Dash = FUELPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = FUELPRESSURE;
        }

        break;
    } //########################################FUELPRESSURE###########################

    case FUELPRESSURE:
    {
        fuelpressure = myELM327.fuelPressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelpressure: ");
            Serial.println(fuelpressure);
            obd_Dash = MANIFOLDPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = MANIFOLDPRESSURE;
        }

        break;
    } //########################################MANIFOLDPRESSURE###########################

    case MANIFOLDPRESSURE:
    {
        manifoldpressure = myELM327.manifoldPressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("manifoldpressure: ");
            Serial.println(manifoldpressure);
            obd_Dash = RPM;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = RPM;
        }

        break;
    } //########################################RPM###########################

    case RPM:
    {
        rpm = myELM327.rpm();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("rpm: ");
            Serial.println(rpm);
            obd_Dash = KPH;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = KPH;
        }

        break;
    } //########################################KPH###########################

    case KPH:
    {
        kph = myELM327.kph();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("kph: ");
            Serial.println(kph);
            obd_Dash = MPH;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = MPH;
        }

        break;
    } //########################################MPH###########################

    case MPH:
    {
        mph = myELM327.mph();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("mph: ");
            Serial.println(mph);
            obd_Dash = TIMINGADVANCE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = TIMINGADVANCE;
        }

        break;
    } //########################################TIMINGADVANCE###########################

    case TIMINGADVANCE:
    {
        timingadvance = myELM327.timingAdvance();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("timingadvance: ");
            Serial.println(timingadvance);
            obd_Dash = INTAKEAIRTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = INTAKEAIRTEMP;
        }

        break;
    } //########################################INTAKEAIRTEMP###########################

    case INTAKEAIRTEMP:
    {
        intakeairtemp = myELM327.intakeAirTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("intakeairtemp: ");
            Serial.println(intakeairtemp);
            obd_Dash = MAFRATE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = MAFRATE;
        }

        break;
    } //########################################MAFRATE###########################

    case MAFRATE:
    {
        mafrate = myELM327.mafRate();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("mafrate: ");
            Serial.println(mafrate);
            obd_Dash = THROTTLE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = THROTTLE;
        }

        break;
    }

        //########################################THROTTLE###########################

    case THROTTLE:
    {
        throttle = myELM327.throttle();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("throttle: ");
            Serial.println(throttle);
            obd_Dash = UCOMMANDEDSECAIRSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = UCOMMANDEDSECAIRSTATUS;
        }

        break;

        //########################################UCOMMANDEDSECAIRSTATUS###########################

    case UCOMMANDEDSECAIRSTATUS:
    {
        commandedsecairstatus = myELM327.commandedSecAirStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("commandedsecairstatus: ");
            Serial.println(commandedsecairstatus);
            obd_Dash = UOXYGENSENSORSPRESENT_2BANKS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = UOXYGENSENSORSPRESENT_2BANKS;
        }

        break;
    } //########################################UOXYGENSENSORSPRESENT_2BANKS###########################

    case UOXYGENSENSORSPRESENT_2BANKS:
    {
        oxygensensorspresent_2banks = myELM327.oxygenSensorsPresent_2banks();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("oxygensensorspresent_2banks: ");
            Serial.println(oxygensensorspresent_2banks);
            obd_Dash = UOBDSTANDARDS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = UOBDSTANDARDS;
        }

        break;
    } //########################################UOBDSTANDARDS###########################

    case UOBDSTANDARDS:
    {
        obdstandards = myELM327.obdStandards();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("obdstandards: ");
            Serial.println(obdstandards);
            obd_Dash = UOXYGENSENSORSPRESENT_4BANKS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = UOXYGENSENSORSPRESENT_4BANKS;
        }

        break;
    } //########################################UOXYGENSENSORSPRESENT_4BANKS###########################

    case UOXYGENSENSORSPRESENT_4BANKS:
    {
        oxygensensorspresent_4banks = myELM327.oxygenSensorsPresent_4banks();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("oxygensensorspresent_4banks: ");
            Serial.println(oxygensensorspresent_4banks);
            obd_Dash = AUXINPUTSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = AUXINPUTSTATUS;
        }

        break;
    } //########################################AUXINPUTSTATUS###########################

    case AUXINPUTSTATUS:
    {
        auxinputstatus = myELM327.auxInputStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("auxinputstatus: ");
            Serial.println(auxinputstatus);
            obd_Dash = RUNTIME;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = RUNTIME;
        }

        break;
    } //########################################RUNTIME###########################

    case RUNTIME:
    {
        runtime = myELM327.runTime();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("runtime: ");
            Serial.println(runtime);
            obd_Dash = DISTTRAVELWITHMIL;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = DISTTRAVELWITHMIL;
        }

        break;
    }

        //########################################DISTTRAVELWITHMIL###########################

    case DISTTRAVELWITHMIL:
    {
        disttravelwithmil = myELM327.distTravelWithMIL();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("disttravelwithmil: ");
            Serial.println(disttravelwithmil);
            obd_Dash = FUELLEVEL;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = FUELLEVEL;
        }

        break;
    }

        //########################################FUELLEVEL###########################

    case FUELLEVEL:
    {
        fuellevel = myELM327.fuelLevel();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuellevel: ");
            Serial.println(fuellevel);
            obd_Dash = MONITORDRIVECYCLESTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = MONITORDRIVECYCLESTATUS;
        }

        break;
    }
        //########################################MONITORDRIVECYCLESTATUS###########################

    case MONITORDRIVECYCLESTATUS:
    {
        monitordrivecyclestatus = myELM327.monitorDriveCycleStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("monitordrivecyclestatus: ");
            Serial.println(monitordrivecyclestatus);
            obd_Dash = ABSLOAD;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = ABSLOAD;
        }

        break;
    }
        //########################################ABSLOAD###########################

    case ABSLOAD:
    {
        absload = myELM327.absLoad();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absload: ");
            Serial.println(absload);
            obd_Dash = AMBIENTAIRTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = AMBIENTAIRTEMP;
        }

        break;
    }
        //########################################AMBIENTAIRTEMP###########################

    case AMBIENTAIRTEMP:
    {
        ambientairtemp = myELM327.ambientAirTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("ambientairtemp: ");
            Serial.println(ambientairtemp);
            obd_Dash = ETHONOLPERCENT;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = ETHONOLPERCENT;
        }

        break;
    }
        //########################################ETHONOLPERCENT###########################

    case ETHONOLPERCENT:
    {
        ethonolpercent = myELM327.ethonolPercent();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("ethonolpercent: ");
            Serial.println(ethonolpercent);
            obd_Dash = HYBRIDBATLIFE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = HYBRIDBATLIFE;
        }

        break;
    }

        //########################################HYBRIDBATLIFE###########################

    case HYBRIDBATLIFE:
    {
        hybridbatlife = myELM327.hybridBatLife();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("hybridbatlife: ");
            Serial.println(hybridbatlife);
            obd_Dash = OILTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = OILTEMP;
        }

        break;
    } //########################################OILTEMP###########################

    case OILTEMP:
    {
        oiltemp = myELM327.oilTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("oiltemp: ");
            Serial.println(oiltemp);
            obd_Dash = FUELINJECTTIMING;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = FUELINJECTTIMING;
        }

        break;
    }

        //########################################UABSBAROPRESSURE###########################

    case FUELINJECTTIMING:
    {
        fuelinjecttiming = myELM327.fuelInjectTiming();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelinjecttiming: ");
            Serial.println(fuelinjecttiming);
            obd_Dash = FUELRATE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = FUELRATE;
        }

        break;
    }

        //########################################FUELRATE###########################

    case FUELRATE:
    {
        fuelrate = myELM327.fuelRate();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelrate: ");
            Serial.println(fuelrate);
            obd_Dash = TORQUE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = TORQUE;
        }

        break;
    }

        //########################################TORQUE###########################

    case TORQUE:
    {
        torque = myELM327.torque();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("torque: ");
            Serial.println(torque);
            obd_Dash = BATTERYVOLTAGE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_Dash = BATTERYVOLTAGE;
        }
        check3 = 0;
        break;
    }
    }
    }

    return 0;
}

void obdDashpublishMessage()
{
    StaticJsonDocument<1500> doc;
    doc["imei"] = imei;
    doc["auxInputStatus"] = auxinputstatus;
    doc["batteryVoltage"] = batteryvoltage;
    doc["commandedSecAirStatus"] = commandedsecairstatus;
    doc["engineLoad"] = engineload;
    doc["freezeDTC"] = freezedtc;
    doc["fuelLevel"] = fuellevel;
    doc["fuelPressure"] = fuelpressure;
    doc["fuelSystemStatus"] = fuelsystemstatus;
    doc["intakeAirTemp"] = intakeairtemp;
    doc["kph"] = kph;
    doc["longTermFuelTrimBank_1"] = longtermfueltrimbank_1;
    doc["longTermFuelTrimBank_2"] = longtermfueltrimbank_2;
    doc["mafRate"] = mafrate;
    doc["manifoldPressure"] = fuelrailpressure;
    doc["monitorStatus"] = monitorstatus;
    doc["mph"] = mph;
    doc["obdStandards"] = obdstandards;
    doc["oxygenSensorsPresent_2banks"] = oxygensensorspresent_2banks;
    doc["oxygenSensorsPresent_4banks"] = oxygensensorspresent_4banks;
    doc["rpm"] = rpm;
    doc["runTime"] = runtime;
    doc["shorttermfueltrimbank_1"] = shorttermfueltrimbank_1;
    doc["shorttermfueltrimbank_2"] = shorttermfueltrimbank_2;
    doc["Temp"] = enginecoolanttemp;
    doc["throttle"] = throttle;
    doc["timingAdvance"] = timingadvance;
    doc["distTravelwithmil"] = disttravelwithmil;
    doc["absLoad"] = absload;
    doc["ambientairTemp"] = ambientairtemp;
    doc["ethonolPercent"] = ethonolpercent;
    doc["hybridBatlife"] = hybridbatlife;
    doc["oilTemp"] = oiltemp;
    doc["fuelinJecttiming"] = fuelinjecttiming;
    doc["fuelRate"] = fuelrate;
    char jsonBuffer[1900];
    serializeJson(doc, jsonBuffer);
    mqtt.publish(obdDash, jsonBuffer);
}

//################################################obdDiag##############################################
obd_pid_states obd_state = BATTERYVOLTAGE;
boolean obdDiagBoad()
{
    switch (obd_state)
    {
        //########################################BATTERYVOLTAGE###########################

    case BATTERYVOLTAGE:
    {
        batteryvoltage = myELM327.batteryVoltage();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("batteryvoltage: ");
            Serial.println(batteryvoltage);
            obd_state = SUPPORTEDPIDS_1_20;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = SUPPORTEDPIDS_1_20;
        }

        break;
    }

        //########################################SUPPORTEDPIDS_1_20###########################

    case SUPPORTEDPIDS_1_20:
    {
        supportedpids_1_20 = myELM327.supportedPIDs_1_20();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_1_20: ");
            Serial.println(supportedpids_1_20);
            obd_state = MONITORSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = MONITORSTATUS;
        }

        break;
    }

        //########################################MONITORSTATUS###########################

    case MONITORSTATUS:
    {
        monitorstatus = myELM327.monitorStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("monitorstatus");
            Serial.println(monitorstatus);
            obd_state = FREEZEDTC;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = FREEZEDTC;
        }

        break;
    } //########################################FREEZEDTC###########################

    case FREEZEDTC:
    {
        freezedtc = myELM327.freezeDTC();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("freezedtc: ");
            Serial.println(freezedtc);
            obd_state = FUELSYSTEMSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = FUELSYSTEMSTATUS;
        }

        break;
    } //########################################FUELSYSTEMSTATUS###########################

    case FUELSYSTEMSTATUS:
    {
        fuelsystemstatus = myELM327.fuelSystemStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelsystemstatus: ");
            Serial.println(fuelsystemstatus);
            obd_state = ENGINELOAD;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ENGINELOAD;
        }

        break;
    } //########################################ENGINELOAD###########################

    case ENGINELOAD:
    {
        engineload = myELM327.engineLoad();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("engineload: ");
            Serial.println(engineload);
            obd_state = ENGINECOOLANTTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ENGINECOOLANTTEMP;
        }

        break;
    } //########################################ENGINECOOLANTTEMP###########################

    case ENGINECOOLANTTEMP:
    {
        enginecoolanttemp = myELM327.engineCoolantTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("enginecoolanttemp: ");
            Serial.println(enginecoolanttemp);
            obd_state = SHORTTERMFUELTRIMBANK_1;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = SHORTTERMFUELTRIMBANK_1;
        }

        break;
    } //########################################SHORTTERMFUELTRIMBANK_1###########################

    case SHORTTERMFUELTRIMBANK_1:
    {
        shorttermfueltrimbank_1 = myELM327.shortTermFuelTrimBank_1();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("shorttermfueltrimbank_1: ");
            Serial.println(shorttermfueltrimbank_1);
            obd_state = LONGTERMFUELTRIMBANK_1;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = LONGTERMFUELTRIMBANK_1;
        }

        break;
    } //########################################LONGTERMFUELTRIMBANK_1###########################

    case LONGTERMFUELTRIMBANK_1:
    {
        longtermfueltrimbank_1 = myELM327.longTermFuelTrimBank_1();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_1_20: ");
            Serial.println(supportedpids_1_20);
            obd_state = SHORTTERMFUELTRIMBANK_2;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = SHORTTERMFUELTRIMBANK_2;
        }

        break;
    } //########################################SHORTTERMFUELTRIMBANK_2###########################

    case SHORTTERMFUELTRIMBANK_2:
    {
        shorttermfueltrimbank_2 = myELM327.shortTermFuelTrimBank_2();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_1_20: ");
            Serial.println(supportedpids_1_20);
            obd_state = LONGTERMFUELTRIMBANK_2;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = LONGTERMFUELTRIMBANK_2;
        }

        break;
    } //########################################LONGTERMFUELTRIMBANK_2###########################

    case LONGTERMFUELTRIMBANK_2:
    {
        longtermfueltrimbank_2 = myELM327.longTermFuelTrimBank_2();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("longtermfueltrimbank_2: ");
            Serial.println(longtermfueltrimbank_2);
            obd_state = FUELPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = FUELPRESSURE;
        }

        break;
    } //########################################FUELPRESSURE###########################

    case FUELPRESSURE:
    {
        fuelpressure = myELM327.fuelPressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelpressure: ");
            Serial.println(fuelpressure);
            obd_state = MANIFOLDPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = MANIFOLDPRESSURE;
        }

        break;
    } //########################################MANIFOLDPRESSURE###########################

    case MANIFOLDPRESSURE:
    {
        manifoldpressure = myELM327.manifoldPressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("manifoldpressure: ");
            Serial.println(manifoldpressure);
            obd_state = RPM;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = RPM;
        }

        break;
    } //########################################RPM###########################

    case RPM:
    {
        rpm = myELM327.rpm();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("rpm: ");
            Serial.println(rpm);
            obd_state = KPH;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = KPH;
        }

        break;
    } //########################################KPH###########################

    case KPH:
    {
        kph = myELM327.kph();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("kph: ");
            Serial.println(kph);
            obd_state = MPH;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = MPH;
        }

        break;
    } //########################################MPH###########################

    case MPH:
    {
        mph = myELM327.mph();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("mph: ");
            Serial.println(mph);
            obd_state = TIMINGADVANCE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = TIMINGADVANCE;
        }

        break;
    } //########################################TIMINGADVANCE###########################

    case TIMINGADVANCE:
    {
        timingadvance = myELM327.timingAdvance();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("timingadvance: ");
            Serial.println(timingadvance);
            obd_state = INTAKEAIRTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = INTAKEAIRTEMP;
        }

        break;
    } //########################################INTAKEAIRTEMP###########################

    case INTAKEAIRTEMP:
    {
        intakeairtemp = myELM327.intakeAirTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("intakeairtemp: ");
            Serial.println(intakeairtemp);
            obd_state = MAFRATE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = MAFRATE;
        }

        break;
    } //########################################MAFRATE###########################

    case MAFRATE:
    {
        mafrate = myELM327.mafRate();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("mafrate: ");
            Serial.println(mafrate);
            obd_state = THROTTLE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = THROTTLE;
        }

        break;
    }

        //########################################THROTTLE###########################

    case THROTTLE:
    {
        throttle = myELM327.throttle();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("throttle: ");
            Serial.println(throttle);
            obd_state = UCOMMANDEDSECAIRSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = UCOMMANDEDSECAIRSTATUS;
        }

        break;

        //########################################UCOMMANDEDSECAIRSTATUS###########################

    case UCOMMANDEDSECAIRSTATUS:
    {
        commandedsecairstatus = myELM327.commandedSecAirStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("commandedsecairstatus: ");
            Serial.println(commandedsecairstatus);
            obd_state = UOXYGENSENSORSPRESENT_2BANKS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = UOXYGENSENSORSPRESENT_2BANKS;
        }

        break;
    } //########################################UOXYGENSENSORSPRESENT_2BANKS###########################

    case UOXYGENSENSORSPRESENT_2BANKS:
    {
        oxygensensorspresent_2banks = myELM327.oxygenSensorsPresent_2banks();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("oxygensensorspresent_2banks: ");
            Serial.println(oxygensensorspresent_2banks);
            obd_state = UOBDSTANDARDS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = UOBDSTANDARDS;
        }

        break;
    } //########################################UOBDSTANDARDS###########################

    case UOBDSTANDARDS:
    {
        obdstandards = myELM327.obdStandards();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("obdstandards: ");
            Serial.println(obdstandards);
            obd_state = UOXYGENSENSORSPRESENT_4BANKS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = UOXYGENSENSORSPRESENT_4BANKS;
        }

        break;
    } //########################################UOXYGENSENSORSPRESENT_4BANKS###########################

    case UOXYGENSENSORSPRESENT_4BANKS:
    {
        oxygensensorspresent_4banks = myELM327.oxygenSensorsPresent_4banks();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("oxygensensorspresent_4banks: ");
            Serial.println(oxygensensorspresent_4banks);
            obd_state = AUXINPUTSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = AUXINPUTSTATUS;
        }

        break;
    } //########################################AUXINPUTSTATUS###########################

    case AUXINPUTSTATUS:
    {
        auxinputstatus = myELM327.auxInputStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("auxinputstatus: ");
            Serial.println(auxinputstatus);
            obd_state = RUNTIME;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = RUNTIME;
        }

        break;
    } //########################################RUNTIME###########################

    case RUNTIME:
    {
        runtime = myELM327.runTime();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("runtime: ");
            Serial.println(runtime);
            obd_state = SUPPORTEDPIDS_21_40;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = SUPPORTEDPIDS_21_40;
        }

        break;
    } //########################################SUPPORTEDPIDS_21_40###########################

    case SUPPORTEDPIDS_21_40:
    {
        supportedpids_21_40 = myELM327.supportedPIDs_21_40();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_21_40: ");
            Serial.println(supportedpids_21_40);
            obd_state = DISTTRAVELWITHMIL;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = DISTTRAVELWITHMIL;
        }

        break;
    } //########################################DISTTRAVELWITHMIL###########################

    case DISTTRAVELWITHMIL:
    {
        disttravelwithmil = myELM327.distTravelWithMIL();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("disttravelwithmil: ");
            Serial.println(disttravelwithmil);
            obd_state = FUELRAILPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = FUELRAILPRESSURE;
        }

        break;
    } //########################################FUELRAILPRESSURE###########################

    case FUELRAILPRESSURE:
    {
        fuelrailpressure = myELM327.fuelRailPressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelrailpressure: ");
            Serial.println(fuelrailpressure);
            obd_state = FUELRAILGUAGEPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = FUELRAILGUAGEPRESSURE;
        }

        break;
    } //########################################FUELRAILGUAGEPRESSURE###########################

    case FUELRAILGUAGEPRESSURE:
    {
        fuelrailguagepressure = myELM327.fuelRailGuagePressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_1_20: ");
            Serial.println(supportedpids_1_20);
            obd_state = COMMANDEDEGR;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = COMMANDEDEGR;
        }

        break;
    }

        //########################################COMMANDEDEGR###########################

    case COMMANDEDEGR:
    {
        commandedegr = myELM327.commandedEGR();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("commandedegr: ");
            Serial.println(commandedegr);
            obd_state = EGRERROR;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = EGRERROR;
        }

        break;
    }

        //########################################EGRERROR###########################

    case EGRERROR:
    {
        egrerror = myELM327.egrError();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("egrerror: ");
            Serial.println(egrerror);
            obd_state = COMMANDEDEVAPPURGE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = COMMANDEDEVAPPURGE;
        }

        break;
    } //########################################COMMANDEDEVAPPURGE###########################

    case COMMANDEDEVAPPURGE:
    {
        commandedevappurge = myELM327.commandedEvapPurge();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("commandedevappurge: ");
            Serial.println(commandedevappurge);
            obd_state = FUELLEVEL;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = FUELLEVEL;
        }

        break;
    } //########################################FUELLEVEL###########################

    case FUELLEVEL:
    {
        fuellevel = myELM327.fuelLevel();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuellevel: ");
            Serial.println(fuellevel);
            obd_state = UWARMUPSSINCECODESCLEARED;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = UWARMUPSSINCECODESCLEARED;
        }

        break;
    } //########################################UWARMUPSSINCECODESCLEARED###########################

    case UWARMUPSSINCECODESCLEARED:
    {
        warmupssincecodescleared = myELM327.warmUpsSinceCodesCleared();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("warmupssincecodescleared: ");
            Serial.println(warmupssincecodescleared);
            obd_state = DISTSINCECODESCLEARED;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = DISTSINCECODESCLEARED;
        }

        break;
    } //########################################DISTSINCECODESCLEARED###########################

    case DISTSINCECODESCLEARED:
    {
        distsincecodescleared = myELM327.distSinceCodesCleared();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("distsincecodescleared: ");
            Serial.println(distsincecodescleared);
            obd_state = EVAPSYSVAPPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = EVAPSYSVAPPRESSURE;
        }

        break;
    } //########################################EVAPSYSVAPPRESSURE###########################

    case EVAPSYSVAPPRESSURE:
    {
        evapsysvappressure = myELM327.evapSysVapPressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("evapsysvappressure: ");
            Serial.println(evapsysvappressure);
            obd_state = UABSBAROPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = UABSBAROPRESSURE;
        }

        break;
    }

        //########################################UABSBAROPRESSURE###########################

    case UABSBAROPRESSURE:
    {
        absbaropressure = myELM327.absBaroPressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absbaropressure: ");
            Serial.println(absbaropressure);
            obd_state = CATTEMPB1S1;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = CATTEMPB1S1;
        }

        break;
    }

        //########################################CATTEMPB1S1###########################

    case CATTEMPB1S1:
    {
        cattempb1s1 = myELM327.catTempB1S1();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("cattempb1s1: ");
            Serial.println(cattempb1s1);
            obd_state = CATTEMPB2S1;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = CATTEMPB2S1;
        }

        break;
    } //########################################CATTEMPB2S1###########################

    case CATTEMPB2S1:
    {
        cattempb2s1 = myELM327.catTempB2S1();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("cattempb2s1: ");
            Serial.println(cattempb2s1);
            obd_state = CATTEMPB1S2;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = CATTEMPB1S2;
        }

        break;
    } //########################################CATTEMPB1S2###########################

    case CATTEMPB1S2:
    {
        cattempb1s2 = myELM327.catTempB1S2();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("cattempb1s2: ");
            Serial.println(cattempb1s2);
            obd_state = CATTEMPB2S2;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = CATTEMPB2S2;
        }

        break;
    } //########################################CATTEMPB2S2,###########################

    case CATTEMPB2S2:
    {
        cattempb2s2 = myELM327.catTempB2S2();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("cattempb2s2: ");
            Serial.println(cattempb2s2);
            obd_state = SUPPORTEDPIDS_41_60;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = SUPPORTEDPIDS_41_60;
        }

        break;
    }

        //########################################SUPPORTEDPIDS_41_60###########################

    case SUPPORTEDPIDS_41_60:
    {
        supportedpids_41_60 = myELM327.supportedPIDs_41_60();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_41_60: ");
            Serial.println(supportedpids_41_60);
            obd_state = MONITORDRIVECYCLESTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = MONITORDRIVECYCLESTATUS;
        }

        break;
    }

        //########################################MONITORDRIVECYCLESTATUS###########################

    case MONITORDRIVECYCLESTATUS:
    {
        monitordrivecyclestatus = myELM327.monitorDriveCycleStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("monitordrivecyclestatus: ");
            Serial.println(monitordrivecyclestatus);
            obd_state = CTRLMODVOLTAGE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = CTRLMODVOLTAGE;
        }

        break;
    } //########################################CTRLMODVOLTAGE###########################

    case CTRLMODVOLTAGE:
    {
        ctrlmodvoltage = myELM327.ctrlModVoltage();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("ctrlmodvoltage: ");
            Serial.println(ctrlmodvoltage);
            obd_state = ABSLOAD;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ABSLOAD;
        }

        break;
    } //########################################ABSLOAD###########################

    case ABSLOAD:
    {
        absload = myELM327.absLoad();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absload: ");
            Serial.println(absload);
            obd_state = COMMANDEDAIRFUELRATIO;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = COMMANDEDAIRFUELRATIO;
        }

        break;
    } //########################################COMMANDEDAIRFUELRATIO###########################

    case COMMANDEDAIRFUELRATIO:
    {
        commandedairfuelratio = myELM327.commandedAirFuelRatio();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("commandedairfuelratio: ");
            Serial.println(commandedairfuelratio);
            obd_state = RELATIVETHROTTLE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = RELATIVETHROTTLE;
        }

        break;
    } //########################################RELATIVETHROTTLE###########################

    case RELATIVETHROTTLE:
    {
        relativethrottle = myELM327.relativeThrottle();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("relativethrottle: ");
            Serial.println(relativethrottle);
            obd_state = AMBIENTAIRTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = AMBIENTAIRTEMP;
        }

        break;
    } //########################################AMBIENTAIRTEMP###########################

    case AMBIENTAIRTEMP:
    {
        ambientairtemp = myELM327.ambientAirTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("ambientairtemp: ");
            Serial.println(ambientairtemp);
            obd_state = ABSTHROTTLEPOSB;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ABSTHROTTLEPOSB;
        }

        break;
    } //########################################ABSTHROTTLEPOSB###########################
    case ABSTHROTTLEPOSB:
    {
        absthrottleposb = myELM327.absThrottlePosB();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absthrottleposb: ");
            Serial.println(absthrottleposb);
            obd_state = ABSTHROTTLEPOSC;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ABSTHROTTLEPOSC;
        }

        break;
    } //########################################SUPPORTEDPIDS_1_20###########################

    case ABSTHROTTLEPOSC:
    {
        absthrottleposc = myELM327.absThrottlePosC();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absthrottleposc: ");
            Serial.println(absthrottleposc);
            obd_state = ABSTHROTTLEPOSD;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ABSTHROTTLEPOSD;
        }

        break;
    } //########################################ABSTHROTTLEPOSD###########################

    case ABSTHROTTLEPOSD:
    {
        absthrottleposd = myELM327.absThrottlePosD();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absthrottleposd: ");
            Serial.println(absthrottleposd);
            obd_state = ABSTHROTTLEPOSE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ABSTHROTTLEPOSE;
        }

        break;
    } //########################################ABSTHROTTLEPOSE###########################

    case ABSTHROTTLEPOSE:
    {
        absthrottlepose = myELM327.absThrottlePosE();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absthrottlepose: ");
            Serial.println(absthrottlepose);
            obd_state = ABSTHROTTLEPOSF;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ABSTHROTTLEPOSF;
        }

        break;
    } //########################################ABSTHROTTLEPOSF###########################

    case ABSTHROTTLEPOSF:
    {
        absthrottleposf = myELM327.absThrottlePosF();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absthrottleposf: ");
            Serial.println(absthrottleposf);
            obd_state = COMMANDEDTHROTTLEACTUATOR;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = COMMANDEDTHROTTLEACTUATOR;
        }

        break;
    } //########################################COMMANDEDTHROTTLEACTUATOR###########################

    case COMMANDEDTHROTTLEACTUATOR:
    {
        commandedthrottleactuator = myELM327.commandedThrottleActuator();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("commandedthrottleactuator: ");
            Serial.println(commandedthrottleactuator);
            obd_state = TIMERUNWITHMIL;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = TIMERUNWITHMIL;
        }

        break;
    } //########################################TIMERUNWITHMIL###########################

    case TIMERUNWITHMIL:
    {
        timerunwithmil = myELM327.timeRunWithMIL();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("timerunwithmil: ");
            Serial.println(timerunwithmil);
            obd_state = TIMESINCECODESCLEARED;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = TIMESINCECODESCLEARED;
        }

        break;
    } //########################################TIMESINCECODESCLEARED###########################

    case TIMESINCECODESCLEARED:
    {
        timesincecodescleared = myELM327.timeSinceCodesCleared();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("timesincecodescleared: ");
            Serial.println(timesincecodescleared);
            obd_state = MAXMAFRATE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = MAXMAFRATE;
        }

        break;
    } //########################################SUPPORTEDPIDS_1_20###########################

    case MAXMAFRATE:
    {
        maxmafrate = myELM327.maxMafRate();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("maxmafrate: ");
            Serial.println(maxmafrate);
            obd_state = UFUELTYPE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = UFUELTYPE;
        }

        break;
    } //########################################UFUELTYPE###########################

    case UFUELTYPE:
    {
        fueltype = myELM327.fuelType();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fueltype: ");
            Serial.println(fueltype);
            obd_state = ETHONOLPERCENT;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ETHONOLPERCENT;
        }

        break;
    } //########################################ETHONOLPERCENT###########################

    case ETHONOLPERCENT:
    {
        ethonolpercent = myELM327.ethonolPercent();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("ethonolpercent: ");
            Serial.println(ethonolpercent);
            obd_state = ABSEVAPSYSVAPPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ABSEVAPSYSVAPPRESSURE;
        }

        break;
    } //########################################SUPPORTEDPIDS_1_20###########################

    case ABSEVAPSYSVAPPRESSURE:
    {
        absevapsysvappressure = myELM327.absEvapSysVapPressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absevapsysvappressure: ");
            Serial.println(absevapsysvappressure);
            obd_state = EVAPSYSVAPPRESSURE2;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = EVAPSYSVAPPRESSURE2;
        }

        break;
    } //########################################EVAPSYSVAPPRESSURE2###########################

    case EVAPSYSVAPPRESSURE2:
    {
        evapsysvappressure2 = myELM327.evapSysVapPressure2();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("evapsysvappressure2: ");
            Serial.println(evapsysvappressure2);
            obd_state = ABSFUELRAILPRESSURE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = ABSFUELRAILPRESSURE;
        }

        break;
    } //########################################ABSFUELRAILPRESSURE###########################

    case ABSFUELRAILPRESSURE:
    {
        absfuelrailpressure = myELM327.absFuelRailPressure();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("absfuelrailpressure: ");
            Serial.println(absfuelrailpressure);
            obd_state = RELATIVEPEDALPOS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = RELATIVEPEDALPOS;
        }

        break;
    } //########################################RELATIVEPEDALPOS,###########################

    case RELATIVEPEDALPOS:
    {
        relativepedalpos = myELM327.relativePedalPos();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("relativepedalpos: ");
            Serial.println(relativepedalpos);
            obd_state = HYBRIDBATLIFE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = HYBRIDBATLIFE;
        }

        break;
    } //########################################HYBRIDBATLIFE###########################

    case HYBRIDBATLIFE:
    {
        hybridbatlife = myELM327.hybridBatLife();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("hybridbatlife: ");
            Serial.println(hybridbatlife);
            obd_state = OILTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = OILTEMP;
        }

        break;
    } //########################################OILTEMP###########################

    case OILTEMP:
    {
        oiltemp = myELM327.oilTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("oiltemp: ");
            Serial.println(oiltemp);
            obd_state = FUELINJECTTIMING;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = FUELINJECTTIMING;
        }

        break;
    }

        //########################################UABSBAROPRESSURE###########################

    case FUELINJECTTIMING:
    {
        fuelinjecttiming = myELM327.fuelInjectTiming();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelinjecttiming: ");
            Serial.println(fuelinjecttiming);
            obd_state = FUELRATE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = FUELRATE;
        }

        break;
    }

        //########################################FUELRATE###########################

    case FUELRATE:
    {
        fuelrate = myELM327.fuelRate();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelrate: ");
            Serial.println(fuelrate);
            obd_state = UEMISSIONRQMTS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = UEMISSIONRQMTS;
        }

        break;
    } //########################################SUPPORTEDPIDS_1_20###########################

    case UEMISSIONRQMTS:
    {
        emissionrqmts = myELM327.emissionRqmts();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("emissionrqmts: ");
            Serial.println(emissionrqmts);
            obd_state = SUPPORTEDPIDS_61_80;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = SUPPORTEDPIDS_61_80;
        }

        break;
    } //########################################SUPPORTEDPIDS_61_80###########################

    case SUPPORTEDPIDS_61_80:
    {
        supportedpids_61_80 = myELM327.supportedPIDs_61_80();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_61_80: ");
            Serial.println(supportedpids_61_80);
            obd_state = DEMANDEDTORQUE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = DEMANDEDTORQUE;
        }

        break;
    } //########################################SUPPORTEDPIDS_1_20###########################

    case DEMANDEDTORQUE:
    {
        demandedtorque = myELM327.demandedTorque();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("demandedtorque: ");
            Serial.println(demandedtorque);
            obd_state = TORQUE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = TORQUE;
        }

        break;
    } //########################################TORQUE###########################

    case TORQUE:
    {
        torque = myELM327.torque();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("torque: ");
            Serial.println(torque);
            obd_state = REFERENCETORQUE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = REFERENCETORQUE;
        }

        break;
    } //########################################REFERENCETORQUE###########################

    case REFERENCETORQUE:
    {
        referencetorque = myELM327.referenceTorque();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("referenceTorque: ");
            Serial.println(referencetorque);
            obd_state = AUXSUPPORTEDF;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd_state = AUXSUPPORTEDF;
        }

        break;
    } //########################################AUXSUPPORTEDF###########################

    case AUXSUPPORTEDF:
    {
        auxsupported = myELM327.auxSupported();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("auxsupported: ");
            Serial.println(auxsupported);
            obd_state = BATTERYVOLTAGE;
            obdDiagpublishMessage();

            check2 = 0;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            obdDiagpublishMessage();
            obd_state = BATTERYVOLTAGE;
        }
        check2 = 0;
        break;
    }
    }
    }

    return 0;
}

void obdDiagpublishMessage()

{

    StaticJsonDocument<1500> doc;

    doc["get_vin_blocking"] = 0;
    doc["usupportedpids_1_20"] = 0;
    doc["umonitorstatus"] = 0;
    doc["freezedtc"] = 0;
    doc["fuelsystemstatus"] = 0;
    doc["engineload"] = 0;
    doc["enginecoolanttemp"] = 0;
    doc["shorttermfueltrimbank_1"] = 0;
    doc["longtermfueltrimbank_1"] = 0;
    doc["shorttermfueltrimbank_2"] = 0;
    doc["longtermfueltrimbank_2"] = 0;
    doc["fuelpressure"] = 0;
    doc["umanifoldpressure"] = 0;
    doc["rpm"] = 0;
    doc["kph"] = 0;
    doc["mph"] = 0;
    doc["timingadvance"] = 0;
    doc["intakeairtemp"] = 0;
    doc["mafrate"] = 0;
    doc["throttle"] = 0;
    doc["ucommandedsecairstatus"] = 0;
    doc["uoxygensensorspresent_2banks"] = 0;
    doc["uobdstandards"] = 0;
    doc["uoxygensensorspresent_4banks"] = 0;
    doc["auxinputstatus"] = false;
    doc["runtime"] = 0;
    doc["usupportedpids_21_40"] = 0;
    doc["disttravelwithmil"] = 0;
    doc["fuelrailpressure"] = 0;
    doc["fuelrailguagepressure"] = 0;
    doc["commandedegr"] = 0;
    doc["egrerror"] = 0;
    doc["commandedevappurge"] = 0;
    doc["fuellevel"] = 0;
    doc["uwarmupssincecodescleared"] = 0;
    doc["distsincecodescleared"] = 0;
    doc["evapsysvappressure"] = 0;
    doc["uabsbaropressure"] = 0;
    doc["cattempb1s1"] = 0;
    doc["cattempb2s1"] = 0;
    doc["cattempb1s2"] = 0;
    doc["cattempb2s2"] = 0;
    doc["usupportedpids_41_60"] = 0;
    doc["umonitordrivecyclestatus"] = 0;
    doc["ctrlmodvoltage"] = 0;
    doc["absload"] = 0;
    doc["commandedairfuelratio"] = 0;
    doc["relativethrottle"] = 0;
    doc["ambientairtemp"] = 0;
    doc["absthrottleposb"] = 0;
    doc["absthrottleposc"] = 0;
    doc["absthrottleposd"] = 0;
    doc["absthrottlepose"] = 0;
    doc["absthrottleposf"] = 0;
    doc["commandedthrottleactuator"] = 0;
    doc["timerunwithmil"] = 0;
    doc["timesincecodescleared"] = 0;
    doc["maxmafrate"] = 0;
    doc["ufueltype"] = 0;
    doc["ethonolpercent"] = 0;
    doc["absevapsysvappressure"] = 0;
    doc["evapsysvappressure2"] = 0;
    doc["absfuelrailpressure"] = 0;
    doc["relativepedalpos"] = 0;
    doc["hybridbatlife"] = 0;
    doc["oiltemp"] = 0;
    doc["fuelinjecttiming"] = 0;
    doc["fuelrate"] = 0;
    doc["uemissionrqmts"] = 0;
    char jsonBuffer[1900];
    serializeJson(doc, jsonBuffer);
    mqtt.publish(obdDiagSub, jsonBuffer);

    check2 = 6;
}

obd_pid_states obd = BATTERYVOLTAGE;
bool dtc()
{

    switch (obd)
    {
        //########################################BATTERYVOLTAGE###########################

    case BATTERYVOLTAGE:
    {
        batteryvoltage = myELM327.batteryVoltage();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("batteryvoltage: ");
            Serial.println(batteryvoltage);
            obd = SUPPORTEDPIDS_1_20;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd = SUPPORTEDPIDS_1_20;
        }

        break;
    }

        //########################################SUPPORTEDPIDS_1_20###########################

    case SUPPORTEDPIDS_1_20:
    {
        supportedpids_1_20 = myELM327.supportedPIDs_1_20();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("supportedpids_1_20: ");
            Serial.println(supportedpids_1_20);
            obd = MONITORSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd = MONITORSTATUS;
        }

        break;
    }

        //########################################MONITORSTATUS###########################

    case MONITORSTATUS:
    {
        monitorstatus = myELM327.monitorStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("monitorstatus");
            Serial.println(monitorstatus);
            obd = FREEZEDTC;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd = FREEZEDTC;
        }

        break;
    } //########################################FREEZEDTC###########################

    case FREEZEDTC:
    {
        freezedtc = myELM327.freezeDTC();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("freezedtc: ");
            Serial.println(freezedtc);
            obd = FUELSYSTEMSTATUS;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd = FUELSYSTEMSTATUS;
        }

        break;
    } //########################################FUELSYSTEMSTATUS###########################

    case FUELSYSTEMSTATUS:
    {
        fuelsystemstatus = myELM327.fuelSystemStatus();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("fuelsystemstatus: ");
            Serial.println(fuelsystemstatus);
            obd = ENGINELOAD;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd = ENGINELOAD;
        }

        break;
    } //########################################ENGINELOAD###########################

    case ENGINELOAD:
    {
        engineload = myELM327.engineLoad();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            Serial.print("engineload: ");
            Serial.println(engineload);
            obd = ENGINECOOLANTTEMP;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            myELM327.printError();
            obd = ENGINECOOLANTTEMP;
        }

        break;
    } //########################################ENGINECOOLANTTEMP###########################

    case ENGINECOOLANTTEMP:
    {
        enginecoolanttemp = myELM327.engineCoolantTemp();

        if (myELM327.nb_rx_state == ELM_SUCCESS)
        {
            check = 0;
            Serial.print("enginecoolanttemp: ");
            Serial.println(enginecoolanttemp);
            obd = BATTERYVOLTAGE;
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            check = 0;
            myELM327.printError();
            obd = BATTERYVOLTAGE;
        }

        break;

    } //########################################SHORTTERMFUELTRIMBANK_1###########################
    }

    return false;
}

void dtcmyfun()
{
    while (!dtc())
    {
        if (check == 0)
        {
            check = 5;
            break;
        }
    }
}

void obdDiagBoadfun()
{
    while (!obdDiagBoad())
    {
        Serial.println(check2);

        if (check2 == 0)
        {
            Serial.println(check2);

            check2 = 75;
            break;
        }
    }
}

const char *dtcCheck = "dtc";
void mqttCallback(char *topic, byte *payload, unsigned int len)
{
    SerialMon.print("Message arrived");
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    const char *message = doc["message"];
    Serial.println(message);
    if (doc["message"] == "dtc")
    {
        // dtcmyfun();
        // SerialMon.println(dtc());
        // mqtt.publish(obdtopicSend, "done");

        myELM327.sendCommand_Blocking(SERVICE_03);

        delay(2000);
        // getResponse();
        // getResponse();
    }
}

const char *obdDiagPub = "obddiag";
void obdMqttCallback(char *topic, byte *payload, unsigned int len)
{
    SerialMon.print("obddiag arrived");
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    const char *message = doc["message"];
    Serial.println(message);
    if (doc["message"] == "obddiag")
    {
        obdDiagBoadfun();
        mqtt.publish(obdDiagSub, "done");
        delay(2000);
    }
}

boolean mqttConnect()
{
    SerialMon.print("Connecting to ");
    SerialMon.print(broker);

    // Connect to MQTT Broker
    // boolean status = mqtt.connect("obd");

    // Or, if you want to authenticate MQTT:
    boolean status = mqtt.connect("obd1", "eraobd", "EraObd@#@#");

    if (status == false)
    {
        SerialMon.println(" fail");
        return false;
    }
    SerialMon.println(" success");
    mqtt.publish(obdDash, "odb started");
    mqtt.subscribe("dtc");
    mqtt.subscribe(obdDiagSub);
    return mqtt.connected();
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void performUpdate(Stream &updateSource, size_t updateSize)
{
    if (Update.begin(updateSize))
    {
        size_t written = Update.writeStream(updateSource);
        if (written == updateSize)
        {
            Serial.println("Writes : " + String(written) + " successfully");
        }
        else
        {
            Serial.println("Written only : " + String(written) + "/" + String(updateSize) + ". Retry?");
        }
        if (Update.end())
        {
            Serial.println("OTA finished!");
            if (Update.isFinished())
            {
                Serial.println("Restart ESP device!");
                ESP.restart();
            }
            else
            {
                Serial.println("OTA not fiished");
            }
        }
        else
        {
            Serial.println("Error occured #: " + String(Update.getError()));
            ESP.restart();
        }
    }
    else
    {
        Serial.println("Cannot beggin update");
    }
}

void updateFromFS()
{
    File updateBin = SPIFFS.open("/update.bin");
    if (updateBin)
    {
        if (updateBin.isDirectory())
        {
            Serial.println("Directory error");
            updateBin.close();
            return;
        }

        size_t updateSize = updateBin.size();

        if (updateSize > 0)
        {
            Serial.println("Starting update");
            performUpdate(updateBin, updateSize);
        }
        else
        {
            Serial.println("Error, archivo vacío");
        }

        updateBin.close();

        // whe finished remove the binary from sd card to indicate end of the process
        // fs.remove("/update.bin");
    }
    else
    {
        Serial.println("no such binary");
    }
}

void printPercent(uint32_t readLength, uint32_t contentLength)
{
    // If we know the total length
    if (contentLength != -1)
    {
        Serial.print("\r ");
        Serial.print((100.0 * readLength) / contentLength);
        Serial.print('%');
    }
    else
    {
        Serial.println(readLength);
    }
}

void fwupdatecheck(void)
{
    mqtt.disconnect();
    mqtt.flush();
    SerialMon.print(F("Performing HTTP GET request... "));
    int err = http.get("/obdfw/version.json");
    if (err != 0)
    {
        SerialMon.println(F("failed to connect"));
        delay(10000);
        return;
    }

    int status = http.responseStatusCode();
    SerialMon.print(F("Response status code: "));
    SerialMon.println(status);
    if (!status)
    {
        delay(10000);
        return;
    }

    SerialMon.println(F("Response Headers:"));
    while (http.headerAvailable())
    {
        String headerName = http.readHeaderName();
        String headerValue = http.readHeaderValue();
        SerialMon.println("    " + headerName + " : " + headerValue);
    }

    int length = http.contentLength();
    if (length >= 0)
    {
        SerialMon.print(F("Content length is: "));
        SerialMon.println(length);
    }
    if (http.isResponseChunked())
    {
        SerialMon.println(F("The response is chunked"));
    }

    String body = http.responseBody();
    SerialMon.println(F("Response:"));
    SerialMon.println(body);

    SerialMon.print(F("Body length is: "));
    SerialMon.println(body.length());

    StaticJsonDocument<48> doc;
    DeserializationError error = deserializeJson(doc, body);
    if (error)
    {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }

    http.stop();
    SerialMon.println(F("Server disconnected"));
    const char *versionWeb = doc["version"];
    Serial.println(versionWeb);
    if (!(strcmp(versionWeb, version) == 0))
    {

        // if you get a connection, report back via serial:
        if (!clientfoupdate.connect(server, port))
        {
            Serial.println(" fail");
            delay(10000);
            return;
        }
        Serial.println(" OK");

        // Make a HTTP request:
        clientfoupdate.print(String("GET ") + "/obdfw/fw.bin" + " HTTP/1.0\r\n");
        clientfoupdate.print(String("Host: ") + server + "\r\n");
        clientfoupdate.print("Connection: close\r\n\r\n");

        long timeout = millis();
        while (clientfoupdate.available() == 0)
        {
            if (millis() - timeout > 60000L)
            {
                Serial.println(">>> Client Timeout !");
                clientfoupdate.stop();
                delay(10000L);
                return;
            }
        }

        Serial.println("Reading header");
        uint32_t contentLength = knownFileSize;

        File file = SPIFFS.open("/update.bin", FILE_APPEND);

        while (clientfoupdate.available())
        {
            String line = clientfoupdate.readStringUntil('\n');
            line.trim();
            // Serial.println(line);    // Uncomment this to show response header
            line.toLowerCase();
            if (line.startsWith("content-length:"))
            {
                contentLength = line.substring(line.lastIndexOf(':') + 1).toInt();
            }
            else if (line.length() == 0)
            {
                break;
            }
        }

        timeout = millis();
        uint32_t readLength = 0;
        CRC32 crc;

        unsigned long timeElapsed = millis();
        printPercent(readLength, contentLength);

        while (readLength < contentLength && clientfoupdate.connected() && millis() - timeout < 10000L)
        {
            int i = 0;
            while (clientfoupdate.available())
            {
                // read file data to spiffs
                if (!file.print(char(clientfoupdate.read())))
                {
                    Serial.println("Appending file");
                }
                // Serial.print((char)c);       // Uncomment this to show data
                // crc.update(c);
                readLength++;

                if (readLength % (contentLength / 13) == 0)
                {
                    printPercent(readLength, contentLength);
                }
                timeout = millis();
            }
        }

        file.close();

        printPercent(readLength, contentLength);
        timeElapsed = millis() - timeElapsed;
        Serial.println();
        delay(5000);
        clientfoupdate.stop();
        Serial.println("stop client");

        modem.gprsDisconnect();
        Serial.println("gprs disconnect");
        Serial.println();

        float duration = float(timeElapsed) / 1000;

        updateFromFS();
        // Do nothing forevermore
        while (true)
        {
            delay(1000);
        }
    }
}

void appendFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("APOK");
    }
    else
    {
        Serial.println("APX");
    }
}

void readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while (file.available())
    {
        Serial.write(file.read());
        delayMicroseconds(100);
    }
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
}

void deleteFile(fs::FS &fs, const char *path)
{
    Serial.printf("Deleting file: %s\n", path);
    if (fs.remove(path))
    {
        Serial.println("File deleted");
    }
    else
    {
        Serial.println("Delete failed");
    }
}

void setup()
{
    // Set console baud rate
    SerialMon.begin(9600);
    delay(10);
    Serial.begin(9600);
    Serial.println("Configuring WDT...");
    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);               // add current thread to WDT watch

    SerialGps.begin(GPSBaud);
    obdSerial.begin(38400, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);

    Serial.println("Attempting to connect to ELM327...");

    if (!myELM327.begin(obdSerial, DEBUG, TIMEOUT))
    {
        Serial.println("Couldn't connect to OBD scanner");

        if (HALT_ON_FAIL)
            while (1)
                ;
    }

    Serial.println("Connected to ELM327");
    SerialMon.println("Wait...");

    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    SPIFFS.format();
    listDir(SPIFFS, "/", 0);

    // Set GSM module baud rate
    TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
    // SerialAT.begin(9600);
    delay(6000);

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    // modem.restart();
    modem.init();

    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem Info: ");
    SerialMon.println(modemInfo);

#if TINY_GSM_USE_GPRS
    // Unlock your SIM card with a PIN if needed
    if (GSM_PIN && modem.getSimStatus() != 3)
    {
        modem.simUnlock(GSM_PIN);
    }
#endif

#if TINY_GSM_USE_WIFI
    // Wifi connection parameters must be set before waiting for the network
    SerialMon.print(F("Setting SSID/password..."));
    if (!modem.networkConnect(wifiSSID, wifiPass))
    {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");
#endif

#if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
    // The XBee must run the gprsConnect function BEFORE waiting for network!
    modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork())
    {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");

    if (modem.isNetworkConnected())
    {
        SerialMon.println("Network connected");
    }

#if TINY_GSM_USE_GPRS
    // GPRS connection parameters are usually set after network registration
    SerialMon.print(F("Connecting to "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass))
    {
        SerialMon.println(" fail");
        delay(10000);
        return;
    }
    SerialMon.println(" success");

    if (modem.isGprsConnected())
    {
        SerialMon.println("GPRS connected");
    }
#endif

    // MQTT Broker setup
    mqtt.setServer(broker, 1883);
    mqtt.setCallback(mqttCallback);
    mqtt.setCallback(obdMqttCallback);
    mqtt.setBufferSize(2048);
}
void obdFunCall()
{
    do
    {
        obdDashBoad();
    }

    while (1 == obdDashBoad());
}

void obdminidash()
{
    do
    {
        miniobdDashBoad();
    }

    while (1 == miniobdDashBoad());
}

void Gps()
{
    bool checkGps = false;
    StaticJsonDocument<200> doc;
    for (int i = 0; i < 200; i++)
    {

        if (SerialGps.available() > 0)
        {
            gps.encode(SerialGps.read());
            // Serial.print("LAT=");
            Serial.println(gps.location.lat(), 6);
            // Serial.print("LONG=");
            Serial.println(gps.location.lng(), 6);
            // Serial.print("ALT=");
            // Serial.println(gps.altitude.meters());
            lat = gps.location.lat();
            lng = gps.location.lng();
            // checkGps = gps.location.isUpdated();
        }
    }

    if (lat && lat)
    {

        doc["lat"] = lat;
        doc["lng"] = lng;
        doc["imei"] = imei;
        char jsonBuffer[512];
        serializeJson(doc, jsonBuffer);
        mqtt.publish(obdGps, jsonBuffer);
    }
}

void OBD_talk(String cmd)
{

    bool data_received = false;
    timeout_talk = millis();
    resp = "";
    do
    {
        switch (OBD_state)
        {
        case OBD_READY:
            OBD_state = OBD_SEND;
            break;
        case OBD_SEND:
            obdSerial.print(cmd + '\r');
            timeout_read = millis();
            OBD_state = OBD_READ;
            break;
        case OBD_READ:
            do
            {
                if (obdSerial.available() > 0)
                {
                    inChar = char(obdSerial.read()); // Convert the byte into a char
                    resp += inChar;                  // Add the read char to the response string
                }
                // The ELM327 finishes all its responses by the '>' char
            } while (inChar != '>' && ((millis() - timeout_read) < OBD_READ_TIMEOUT));
            if (inChar == '>')
            {
                OBD_state = OBD_READY;
                data_received = true;
                break;
            }
            else
            {
                OBD_state = OBD_SEND;
            }
            break;
        default:
            break;
        }
    } while (!data_received && ((millis() - timeout_talk) < OBD_TALK_TIMEOUT));
}

long *get_dtc_number()
{
    static long n_dtc[5] = {0, 0, 0, 0, 0};
    uint8_t dtc_num_state = READ_DTC__;
    uint16_t s_idx = 0;
    num_ecu = 0;
    do
    {
        switch (dtc_num_state)
        {
        case READ_DTC__:
        {
            if (resp[s_idx++] == '4')
                dtc_num_state = FOUND_4_;
            break;
        }
        case FOUND_4_:
        {
            if (resp[s_idx++] == '1')
                dtc_num_state = FOUND_1;
            else
                dtc_num_state = READ_DTC__;
            break;
        }
        case FOUND_1:
        {
            if (resp[s_idx++] == ' ')
                dtc_num_state = FOUND_SPACE_;
            else
                dtc_num_state = READ_DTC__;
            break;
        }
        case FOUND_SPACE_:
        {
            if (resp[s_idx++] == '0')
                dtc_num_state = FOUND_0_;
            else
                dtc_num_state = READ_DTC__;
            break;
        }
        case FOUND_0_:
        {
            if (resp[s_idx++] == '1')
                dtc_num_state = FOUND_1_;
            else
                dtc_num_state = READ_DTC__;
            break;
        }
        case FOUND_1_:
        {
            String s = resp.substring(s_idx + 1, s_idx + 3);
            if ((s[0] - '0') >= 8)
                n_dtc[num_ecu++] = strtol(s.c_str(), NULL, 16) - 128;
            else
                n_dtc[num_ecu++] = strtol(s.c_str(), NULL, 16);
            dtc_num_state = READ_DTC__;
            break;
        }
        }
    } while (num_ecu < 5 && s_idx < resp.length());
    return n_dtc;
}

void get_dtc_s(String &data, long n_dtc, uint8_t ecu_idx)
{
    uint8_t ecus = 0;
    for (uint16_t i = 0; i < resp.length() - 1; i++)
    { // Search for responses
        if (resp[i] == '4' && resp[i + 1] == '3' && ecus++ == ecu_idx)
        { // found desired response
            for (long j = 0; j < n_dtc; j++)
            { // Read all DTCs available for that specific ECU
                // resp :: "43 01 33 00 00 00 00"
                String sub = resp.substring(i + 3 + j * 6, i + 8 + j * 6);
                // sub  :: "01 33"
                if (isDigit(sub[0]))
                    data += conversion_[sub[0] - '0'];
                else
                    data += conversion_[10 + (sub[0] - 'A')]; // If sub[0] == 'F' --> (sub[0]-'F')+10 == 15
                data += sub[1];
                data += sub[3];
                data += sub[4];
                data += ',';
                // data :: "0P133,"
            }
        }
    }
}

String read_dtc_()
{
    OBD_talk("0101"); // Request number of DTCs (Diagnostic Trouble Codes)
    long *num_dtcs = get_dtc_number();
    delay(50);              // OBD is slow and will need some time between commands
    OBD_talk("03");         // Request actual DTCs
    String dtc_string = ""; // String that'll be filled with DTCs in the format
                            // "P0133,C2122, ..."
    for (uint8_t i = 0; i < num_ecu; i++)
    {
        if (*(num_dtcs + i))
        { // There exists TCs
            get_dtc_s(dtc_string, *(num_dtcs + i), i);
        }
    }
    return dtc_string;
}

void loop()
{

    uint32_t currentMills = millis();
    uint32_t gpsCurrentMills = millis();
    uint32_t updateCurrentMills = millis();
    uint32_t resetCurrentMills = millis();

    // obdFunCall();

    obdminidash();

    if (!modem.isNetworkConnected())
    {
        SerialMon.println("Network disconnected");
        if (!modem.waitForNetwork(180000L, true))
        {
            SerialMon.println(" fail");
            delay(10000);
            return;
        }
        if (modem.isNetworkConnected())
        {
            SerialMon.println("Network re-connected");
        }

#if TINY_GSM_USE_GPRS
        // and make sure GPRS/EPS is still connected
        if (!modem.isGprsConnected())
        {
            SerialMon.println("GPRS disconnected!");
            SerialMon.print(F("Connecting to "));
            SerialMon.print(apn);
            if (!modem.gprsConnect(apn, gprsUser, gprsPass))
            {
                SerialMon.println(" fail");
                delay(10000);
                return;
            }
            if (modem.isGprsConnected())
            {
                SerialMon.println("GPRS reconnected");
            }
        }
#endif
    }

    if (!mqtt.connected())
    {
        SerialMon.println("=== MQTT NOT CONNECTED ===");
        obd_state = BATTERYVOLTAGE;

        // Reconnect every 10 seconds
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L && iCheckReset < 10)
        {

            if (iCheckReset == 9)
            {
                Serial.println("Resetting WDT...");
                esp_task_wdt_reset();
                delay(10000);
                modem.restart();
                delay(1000);
                ESP.restart();
            }

            lastReconnectAttempt = t;
            if (mqttConnect())
            {
                lastReconnectAttempt = 0;
                iCheckReset = 0;
            }
            Serial.print(iCheckReset);
            iCheckReset++;
        }
        delay(100);
        return;
    }

    if (mqtt.connected())

    {
        if (check2 == 75)
        {
            Serial.println("check2check2check2check2check2check2check2check2");
            Serial.println(check2);
            obdDiagpublishMessage();
            check2 = 6;
        }
    }

    if (currentMills - previousMills >= intervalPUB)
    {

        previousMills = currentMills;
        // obdDashpublishMessage();
        miniobdDashpublishMessage();
    }

    if (gpsCurrentMills - gpsPreviousMills >= intervalGps)
    {

        gpsPreviousMills = gpsCurrentMills;
        Gps();
    }

    if (updateCurrentMills - updatePreviousMills >= intervalUpdate)
    {

        updatePreviousMills = updateCurrentMills;
        fwupdatecheck();
    }

    // if (resetCurrentMills - resetPreviousMills >= intervalreset)
    // {
    //     mqtt.publish(obdDiagSub, "restart");
    //     delay(10000);
    //     modem.restart();
    //     delay(1000);
    //     ESP.restart();

    //     resetPreviousMills = resetCurrentMills;
    // }

    mqtt.loop();
}
