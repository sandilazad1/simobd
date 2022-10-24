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
const char *broker = "184.72.193.102";
const char *obdtopicSend = "obd/866262037106043";
const char *obdgpsSend = "gps/866262037106043";

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "ELMduino.h"
#include <ArduinoJson.h>
#include "obd.h"
#include <TinyGPSPlus.h>
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
TinyGsm modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient mqtt(client);

#define LED_PIN 13
int ledStatus = LOW;

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
uint16_t referencetorque = 0;
uint16_t auxsupported = 0;
int check = 5;
int check2 = 6;

uint32_t previousMills = 0;
const long intervalPUB = 20000;

uint32_t gpsPreviousMills = 0;
const long intervalGps = 10000;

float lat = 0.0;
float lng = 0.0;

uint32_t lastReconnectAttempt = 0;

const bool DEBUG = true;
const int TIMEOUT = 2000;
const bool HALT_ON_FAIL = false;

SoftwareSerial obdSerial;

ELM327 myELM327;

TinyGPSPlus gps;

obd_pid_states obd_state = BATTERYVOLTAGE;

boolean obdLoop()
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
        }
        else if (myELM327.nb_rx_state != ELM_GETTING_MSG)
        {
            obd_state = BATTERYVOLTAGE;
        }
        check2 = 0;
        break;
    }
    }
    }

    return 0;
}

void obdpublishMessage()
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
    //  doc["usupportedpids_61_80"] = 0;
    //  doc["demandedtorque"] = 0;
    //  doc["torque"] = 0;
    //  doc["referencetorque"] = 0;
    //  doc["auxsupported"] = 0;
    char jsonBuffer[1900];
    serializeJson(doc, jsonBuffer);
    mqtt.publish(obdtopicSend, jsonBuffer);

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

char rxData[20];
char rxIndex = 0;
void getResponse(void)
{
    char inChar = 0;
    // Keep reading characters until we get a carriage return
    while (inChar != '\r')
    {
        // If a character comes in on the serial port, we need to act on it.
        if (obdSerial.available() > 0)
        {
            // Start by checking if we've received the end of message character ('\r').
            if (obdSerial.peek() == '\r')
            {
                // Clear the Serial buffer
                inChar = obdSerial.read();
                // Put the end of string character on our data string
                rxData[rxIndex] = '\0';
                // Reset the buffer index so that the next character goes back at the beginning of the string.
                rxIndex = 0;
            }
            // If we didn't get the end of message character, just add the new character to the string.
            else
            {
                // Get the new character from the Serial port.
                inChar = obdSerial.read();
                // Add the new character to the string, and increment the index variable.
                rxData[rxIndex++] = inChar;
            }
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

boolean mqttConnect()
{
    SerialMon.print("Connecting to ");
    SerialMon.print(broker);

    // Connect to MQTT Broker
    boolean status = mqtt.connect("obd");

    // Or, if you want to authenticate MQTT:
    // boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

    if (status == false)
    {
        SerialMon.println(" fail");
        return false;
    }
    SerialMon.println(" success");
    mqtt.publish(obdtopicSend, "odb started");
    mqtt.subscribe("dtc");
    return mqtt.connected();
}

void setup()
{
    // Set console baud rate
    SerialMon.begin(9600);
    delay(10);
    Serial.begin(9600);

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

    pinMode(LED_PIN, OUTPUT);

    // !!!!!!!!!!!
    // Set your reset, enable, power pins here
    // !!!!!!!!!!!

    SerialMon.println("Wait...");

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
    mqtt.setBufferSize(2048);
}
void obdFunCall()
{
    do
    {
        obdLoop();
    } while (1 == obdLoop());
}

void Gps()
{

    if (SerialGps.available() > 0)
    {
        gps.encode(SerialGps.read());
        Serial.print("LAT=");
        Serial.println(gps.location.lat(), 6);
        Serial.print("LONG=");
        Serial.println(gps.location.lng(), 6);
        Serial.print("ALT=");
        Serial.println(gps.altitude.meters());
        lat = (float)gps.location.lat();
        lng = (float)gps.location.lng();

        StaticJsonDocument<200> doc;

        doc["lat"] = lat;
        doc["lng"] = lng;

        char jsonBuffer[512];
        serializeJson(doc, jsonBuffer);
        mqtt.publish(obdgpsSend, jsonBuffer);
    }
}

void loop()
{

    uint32_t currentMills = millis();

    uint32_t gpsCurrentMills = millis();

    // while (!obdLoop())
    // {
    //     if (check2 == 0)
    //     {
    //         break;
    //     }
    // }
    obdFunCall();

    // Make sure we're still registered on the network
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
        // Reconnect every 10 seconds
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L)
        {
            lastReconnectAttempt = t;
            if (mqttConnect())
            {
                lastReconnectAttempt = 0;
            }
        }
        delay(100);
        return;
    }

    if (currentMills - previousMills >= intervalPUB)
    {

        previousMills = currentMills;
        obdpublishMessage();
    }

    if (gpsCurrentMills - gpsPreviousMills >= intervalGps)
    {

        gpsPreviousMills = gpsCurrentMills;
        Gps();
    }

    // mqtt.publish(obdtopicSend, "hhhhhhhhhhhhh");
    // obdpublishMessage();
    // delay(100);
    mqtt.loop();
}
