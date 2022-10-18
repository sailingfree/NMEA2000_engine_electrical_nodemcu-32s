/*
  This code is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Version 1.3, 04.08.2020, AK-Homberger
// Version 1.0  20.may.2021 P Martin

#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5 (Caution!!! Pin 2 before)
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <freertos/FreeRTOS.h>
#include <N2kMessages.h>

#include <N2kMsg.h>
#include <NMEA2000.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <Preferences.h>
#include <Seasmart.h>
#include <Time.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

#include "BoatData.h"
#include "ESPmDNS.h"
#include "N2kDeviceList.h"
#include <nmea2000Handlers.h>
#include <GwShell.h>
#include <map>
#include <StringStream.h>
#include <SysInfo.h>
#include <EngineRpm.h>
#include <html_header.h>
#include <html_footer.h>
#include <GwPrefs.h>
#include <Idle.h>
#include <list>
#include <Naiad_INA219.h>

template <class T>
using LinkedList = std::list<T>;

#define ADC_Calibration_Value  22.525 // 24.20  // 34.3 The real value depends on the true resistor values for the ADC input (100K / 22 K)

#define WLAN_CLIENT 0  // Set to 1 to enable client network. 0 to act as AP only

// Data wire for teperature (Dallas DS18B20) is plugged into GPIO 13 on the ESP32
#define ONE_WIRE_BUS 13

#define MiscSendOffset 120
#define VerySlowDataUpdatePeriod 10000  // temperature etc
#define SlowDataUpdatePeriod 1000       // Time between CAN Messages sent
#define FastDataUpdatePeriod 100       // Fast data period, engine rpm etc

#define USE_ARDUINO_OTA true
#define USE_MDNS true

#define GWMODE "Engine"
#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500 

// Global objects and variables
String host_name;
String Model = "Naiad N2K Electrical";

Stream *OutputStream = NULL; //&Serial;

// List of n2k devices for the device scanner
tN2kDeviceList *pN2kDeviceList;

// Map for the wifi access points
typedef struct {
    String ssid;
    String pass;
} WiFiCreds;

static const uint16_t MaxAP = 2;
WiFiCreds wifiCreds [MaxAP];

// Wifi cofiguration Client and Access Point
String AP_password; // AP password  read from preferences
String AP_ssid;     // SSID for the AP constructed from the hostname

// Put IP address details here
const IPAddress AP_local_ip(192, 168, 15, 1);  // Static address for AP
const IPAddress AP_gateway(192, 168, 15, 1);
const IPAddress AP_subnet(255, 255, 255, 0);

int wifiType = 0;  // 0= Client 1= AP

// Define the console to output to serial at startup.
// this can get changed later, eg in the gwshell.
Stream * Console = & Serial;

IPAddress UnitIP;   // The address of this device. Could be client or AP

// UPD broadcast for Navionics, OpenCPN, etc.
const int YDudpPort = 4444;                   // port 4444 is for the Yacht devices interface

// Struct to update BoatData. See BoatData.h for content
tBoatData BoatData;

int NodeAddress = 32;  // To store last NMEA2000 Node Address

const size_t MaxClients = 10;

// Define the network servers
// The web server on port 80
WebServer webserver(80);

// The telnet server for the shell.
WiFiServer telnet(23);

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = {  127489L,  // Engine dynamic
                                                    127488L,  // Engine fast dynamic
                                                    127508L,    // Bat status
                                                    127513L,     // Bat conf
                                                  0};
const unsigned long ReceiveMessages[] PROGMEM = {/*126992L,*/  // System time
                                                 127250L,      // Heading
                                                 127258L,      // Magnetic variation
                                                 128259UL,     // Boat speed
                                                 128267UL,     // Depth
                                                 129025UL,     // Position
                                                 129026L,      // COG and SOG
                                                 129029L,      // GNSS
                                                 130306L,      // Wind
                                                 128275UL,     // Log
                                                 127245UL,     // Rudder
                                                 0};


String other_data;

void debug_log(char *str) {
#if ENABLE_DEBUG_LOG == 1
    Console->println(str);
#endif
}

// Battery volatge and current sensors
Naiad_INA219 ina219_house(0x41, "House", BAT_HOUSE);   
Naiad_INA219 ina219_engine(0x40, "Engine", BAT_ENGINE);  

/////// Variables
using namespace std;

// Time

uint32_t mTimeToSec = 0;
uint32_t mTimeSeconds = 0;

// Forward declarations
void HandleNMEA2000Msg(const tN2kMsg &);
void GetTemperature(void *parameter);
void loadTimerFunc(TimerHandle_t xTimer);
void handleIndex();
void handleNotFound();
void clickedIt();

// Initialize the Arduino OTA
void initializeOTA() {
    // TODO: option to authentication (password)
    Console->println("OTA Started");

    // ArduinoOTA
    ArduinoOTA.onStart([]() {
                  String type;
                  if (ArduinoOTA.getCommand() == U_FLASH)
                      type = "sketch";
                  else  // U_SPIFFS
                      type = "filesystem";
                  Console->println("Start updating " + type);
              })
        .onEnd([]() {
            Console->println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Console->printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Console->printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                Console->println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                Console->println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                Console->println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                Console->println("Receive Failed");
            else if (error == OTA_END_ERROR)
                Console->println("End Failed");
        });

    // Begin
    ArduinoOTA.begin();
}

String WifiMode = "Unknown";
String WifiSSID = "Unknown";
String WifiIP   = "Unknown";

// Connect to a wifi AP
// Try all the configured APs
bool connectWifi() {
    int wifi_retry = 0;

    Serial.printf("There are %d APs to try\n", MaxAP);

    for(int i = 0; i < MaxAP; i++) {
        if(wifiCreds[i].ssid != "---") {
        Serial.printf("\nTrying %s\n", wifiCreds[i].ssid.c_str());
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        WiFi.mode(WIFI_STA);
        WiFi.begin(wifiCreds[i].ssid.c_str(), wifiCreds[i].pass.c_str());
        wifi_retry = 0;

        while (WiFi.status() != WL_CONNECTED && wifi_retry < 20) {  // Check connection, try 5 seconds
            wifi_retry++;
            delay(500);
            Console->print(".");
        }
        Console->println("");
        if(WiFi.status() == WL_CONNECTED) {
            WifiMode = "Client";
            WifiSSID = wifiCreds[i].ssid;
            WifiIP = WiFi.localIP().toString();
            Console->printf("Connected to %s\n", wifiCreds[i].ssid.c_str());
            return true;
        } else {
            Console->printf("Can't connect to %s\n", wifiCreds[i].ssid.c_str());
        }
        }
    }
    return false;
}

void disconnectWifi() {
    Console = &Serial;
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    WifiMode = "Not connected";
}

// Map for received n2k messages. Logs the PGN and the count
std::map<int, int> N2kMsgMap;

uint8_t chipid[6];
String macAddress;

// HTML handlers
String html_start = HTML_start; //Read HTML contents
String html_end = HTML_end;
void handleRoot() {
 webserver.send(200, "text/html", html_start + html_end); //Send web page
}

void handleBoat() {
    StringStream boatData;
    boatData.printf("<pre>");
    boatData.printf("<h1>Boat Data</h1>");
    boatData.printf("<div class='info'>");
    displayBoat(boatData);
    boatData.printf("</div>");

    boatData.printf("<h1>NMEA2000 Devices</h1>");
    boatData.printf("<div class='info'>");
    ListDevices(boatData, true);
    boatData.printf("</div>");

    boatData.printf("<h1>Network</h1>");
    boatData.printf("<div class='info'>");
    getNetInfo(boatData);
    boatData.printf("</div>");

    boatData.printf("<h1>System</h1>");
    boatData.printf("<div class='info'>");
    getSysInfo(boatData);
    boatData.printf("</div>");

    webserver.send(200, "text/html", html_start + boatData.data.c_str() + html_end);  //Send web page 
}

/**
 * @name: N2kToYD_Can
 */
void N2kToYD_Can(const tN2kMsg &msg, char *MsgBuf) {
    unsigned long DaysSince1970 = BoatData.DaysSince1970;
    double SecondsSinceMidnight = BoatData.GPSTime;
    int i, len;
    uint32_t canId = 0;
    char time_str[20];
    char Byte[5];
    unsigned int PF;
    time_t rawtime;
    struct tm ts;
    len = msg.DataLen;
    if (len > 134) {
        len = 134;
        Console->printf("Truncated from %d to 134\n", len);
    }

    // Set CanID

    canId = msg.Source & 0xff;
    PF = (msg.PGN >> 8) & 0xff;

    if (PF < 240) {
        canId = (canId | ((msg.Destination & 0xff) << 8));
        canId = (canId | (msg.PGN << 8));
    } else {
        canId = (canId | (msg.PGN << 8));
    }

    canId = (canId | (msg.Priority << 26));

    rawtime = (DaysSince1970 * 3600 * 24) + SecondsSinceMidnight;  // Create time from GNSS time;
    ts = *localtime(&rawtime);
    strftime(time_str, sizeof(time_str), "%T.000", &ts);  // Create time string

    snprintf(MsgBuf, 25, "%s R %0.8x", time_str, canId);  // Set time and canID

    for (i = 0; i < len; i++) {
        snprintf(Byte, 4, " %0.2x", msg.Data[i]);  // Add data fields
        strcat(MsgBuf, Byte);
    }
}

#define Max_YD_Message_Size 500
char YD_msg[Max_YD_Message_Size] = "";

// Create UDP instance
WiFiUDP udp;

// Send to Yacht device clients over udp using the broadcast address
void GwSendYD(const tN2kMsg &N2kMsg) {
    IPAddress udpAddress = WiFi.broadcastIP();
    N2kToYD_Can(N2kMsg, YD_msg);  // Create YD message from PGN
    udp.beginPacket(udpAddress, YDudpPort);  // Send to UDP
    udp.printf("%s\r\n", YD_msg);
    udp.endPacket();

  char buf[MAX_NMEA2000_MESSAGE_SEASMART_SIZE];
  if ( N2kToSeasmart(N2kMsg,millis(),buf,MAX_NMEA2000_MESSAGE_SEASMART_SIZE)==0 ) return;
  
    udp.beginPacket(udpAddress, 4445);
    udp.println(buf);
    udp.endPacket();
}


// Main setup
void setup() {
    uint8_t chipid[6];
    uint32_t id = 0;
    int i = 0;

    // Init USB serial port
    Serial.begin(115200);
    Console = &Serial;
    sleep(2);
    Serial.printf("Starting setup...\n");

    // Set the on board LED off
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Initialise the preferences
    GwPrefsInit();

    // get the MAC address
    esp_err_t fuse_error = esp_efuse_mac_get_default(chipid);
    if(fuse_error) {
        Serial.printf("efuse error: %d\n", fuse_error);
    }

    for (i = 0; i < 6; i++)  {
        if(i != 0) {
            macAddress += ":";
        }
        id += (chipid[i] << (7 * i));
        macAddress += String(chipid[i], HEX);
    }

    // Generate the hostname by appending the last two octets of the mac address to make it unique
    String hname = GwGetVal(GWHOST, "n2kgw");
    host_name = hname + String(chipid[4], HEX) + String(chipid[5], HEX);

    Serial.printf("Chipid %x %x %x %x %x %x id 0x%x MAC %s Hostname %s\n",
        chipid[0], chipid[1], chipid[2], chipid[3], chipid[4], chipid[5], 
        id, macAddress.c_str(), host_name.c_str());

      // get CPU calibration timing
    calibrateCpu();


   // setup the WiFI map from the preferences
    wifiCreds[0].ssid = GwGetVal(SSID1);
    wifiCreds[0].pass = GwGetVal(SSPW1);
    wifiCreds[1].ssid = GwGetVal(SSID2);
    wifiCreds[1].pass = GwGetVal(SSPW2);

    // Setup params if we are to be an AP
    AP_password =       GwGetVal(GWPASS);

    if (WLAN_CLIENT == 1) {
        Console->println("Start WLAN Client");  // WiFi Mode Client
        delay(100);
        WiFi.setHostname(host_name.c_str());
        connectWifi();
    }
    
    if (WiFi.status() != WL_CONNECTED) {  // No client connection start AP
        // Init wifi connection

        int channel = 11;
        int max_clients = 8;

        Console->println("Start WLAN AP");  // WiFi Mode AP
        WiFi.mode(WIFI_AP);
        AP_ssid = host_name;
        WiFi.softAP(AP_ssid.c_str(), AP_password.c_str(), channel, 0, max_clients);
        delay(100);
        WiFi.softAPConfig(AP_local_ip, AP_gateway, AP_subnet);
        UnitIP = WiFi.softAPIP();
        Console->println("");
        Console->print("AP IP address: ");
        Console->println(UnitIP);
        wifiType = 1;
        WifiMode = "AP";
        WifiIP = UnitIP.toString();
        WifiSSID = AP_ssid;

    } else {  // Wifi Client connection was sucessfull

        Console->println("");
        Console->println("WiFi client connected");
        Console->println("IP client address: ");
        Console->println(WiFi.localIP());
        UnitIP = WiFi.localIP();
    }

    sleep(10);

 
#ifdef USE_ARDUINO_OTA
    // Update over air (OTA)
    initializeOTA();
#endif

    // Register host name in mDNS
#if defined USE_MDNS

    if (MDNS.begin(host_name.c_str())) {
        Console->print("* MDNS responder started. Hostname -> ");
        Console->println(host_name);
    }

    // Register the services

#ifdef WEB_SERVER_ENABLED
    MDNS.addService("http", "tcp", 80);  // Web server
#endif

#ifndef DEBUG_DISABLED
    Console->println("Adding telnet");
    MDNS.addService("telnet", "tcp", 23);  // Telnet server of RemoteDebug, register as telnet
#endif

#endif  // MDNS

    // Start the telnet server
    telnet.begin();

    // Start Web Server
    webserver.on("/", handleRoot);
    webserver.on("/boat", handleBoat);

    webserver.onNotFound(handleNotFound);

    webserver.begin();
    Console->println("HTTP server started");

    // Init the shell
    initGwShell();

    // Init the INA219 voltage and current sensors
    if (!ina219_house.begin()) {
        Serial.println("Failed to find INA219 chip HOUSE");
        while (1) {
            delay(10);
        }
    }
    if (!ina219_engine.begin()) {
        Serial.println("Failed to find INA219 chip ENGINE");
        while (1) {
            delay(10);
        }
    }

    /// Calibrate for 16V 30A with a 0.0025 Ohm shunt
    ina219_engine.setCalibration_16V_30A();
    ina219_house.setCalibration_16V_30A();

    // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  
   NMEA2000.SetN2kCANMsgBufSize(8);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);
    NMEA2000.SetN2kCANSendFrameBufSize(250);

    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 50);
    pN2kDeviceList = new tN2kDeviceList(&NMEA2000);

    // Set product information
    Model += GWMODE;

    NMEA2000.SetProductInformation(host_name.c_str(),               // Manufacturer's Model serial code
                                   100,                             // Manufacturer's product code
                                   Model.c_str(),  // Manufacturer's Model ID
                                   "1.0.0 (2021-06-11)",         // Manufacturer's Software version code
                                   "1.0.0 (2021-06-11)"           // Manufacturer's Model version
    );
    // Set device information
    NMEA2000.SetDeviceInformation(id,   // Unique number. Use e.g. Serial number. Id is generated from MAC-Address
                                  130,  // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  25,   // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                  2046  // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
    );


    
    NMEA2000.SetConfigurationInformation("Naiad ",
                                         "Must be installed internally, not water or dust proof.");

    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below

    NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);  // Show in clear text. Leave uncommented for default Actisense format.

   NodeAddress = GwGetVal(LASTNODEADDRESS, "32").toInt();

    Console->printf("NodeAddress=%d\n", NodeAddress);

    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);

    NMEA2000.ExtendTransmitMessages(TransmitMessages);
    NMEA2000.ExtendReceiveMessages(ReceiveMessages);
    NMEA2000.Open();

    IdleInit();

    // Get the RPM calibration values and setup the timer
    InitRPM();   
  
    delay(200);
    Serial.println("Finished setup");
}

//*****************************************************************************

void handleNotFound() {
    webserver.send(404, "text/plain", "File Not Found\n\n");
}

#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500

bool IsTimeToUpdate(unsigned long NextUpdate) {
    return (NextUpdate < millis());
}
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset = 0) {
    return millis() + Period + Offset;
}

void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period) {
    while (NextUpdate < millis()) NextUpdate += Period;
}


// Construct and send a battery status message for the given battery.
void SendN2kBatteryDetails(BatteryStat battery) {
    tN2kMsg N2kMsg;

    SetN2kDCBatStatus(N2kMsg,
        battery.instance,
        battery.voltage,
        battery.current,
        battery.temperature,
        0 );
            
        NMEA2000.SendMsg(N2kMsg);
        GwSendYD(N2kMsg);
 
    SetN2kBatConf(N2kMsg,
        battery.instance,
        N2kDCbt_AGM,
        N2kDCES_No,
        N2kDCbnv_12v        ,
        N2kDCbc_LeadAcid,
        80.0,
        1,
        1.0,
        80);
    NMEA2000.SendMsg(N2kMsg);
    GwSendYD(N2kMsg);     
}

// Read and send the status for both batteries
void SendN2kBattery() {
    static unsigned long SlowDataUpdated = InitNextUpdate(SlowDataUpdatePeriod, MiscSendOffset);

    if (IsTimeToUpdate(SlowDataUpdated)) {
        SetNextUpdate(SlowDataUpdated, SlowDataUpdatePeriod);
        BatteryStat battery;

        battery = read_ina219(ina219_house);
        SendN2kBatteryDetails(battery);
        battery = read_ina219(ina219_engine);
        SendN2kBatteryDetails(battery);
    }
}

void SendN2kEngineFast() {
    static unsigned long FastDataUpdated = InitNextUpdate(FastDataUpdatePeriod, MiscSendOffset);
    tN2kMsg N2kMsg;
    double rpm = ReadRPM();

    //Console->printf("RPM %f\n", rpm);
    if (IsTimeToUpdate(FastDataUpdated)) {
        SetNextUpdate(FastDataUpdated, FastDataUpdatePeriod);
        if (rpm > 0.0) {
            SetN2kEngineParamRapid(N2kMsg, 0,
                                   rpm,
                                   N2kDoubleNA,
                                   0);

            NMEA2000.SendMsg(N2kMsg);
            GwSendYD(N2kMsg);
        }
    }
}

static WiFiClient telnetClient;

void disconnect() {
    telnetClient.stop();
}

void handleTelnet() {

    if(telnetClient && telnetClient.connected()) {
        // Got a connected client so use it
    } else {
        // See if there is a new connection and assign the new client
        telnetClient = telnet.available();
        if(telnetClient) {
            // Set up the client
            //telnetClient.setNoDelay(true); // More faster
            telnetClient.flush(); // clear input buffer, else you get strange characters
            setShellSource(&telnetClient);
        }
    }

    if(!telnetClient) {
        setShellSource(&Serial);
        return;
    }

    /*
    // read and process data one char at a time to avoid blocking main loop
    if(telnetClient.available()) {
        unsigned char r = telnetClient.read();

        // End of line checks "\r\n"
        if(last == '\r' && r == '\n') {
            // End of line
            // Do something with it
            Console->print(command);
 

            if(command == "exit") {
                Console = &Serial;
                telnetClient.stop();
            } 
            command = "";
            last = ' ';
        } else {
            last = r;
            if(isprint(r)) {
                command += char(r);
            } else {
                Console->printf("TNX: 0x%x %d (0x%x %d)\n", r, r, last, last);
            }
        }
    }
    */
}

void displayBoat(Stream & stream) {
    stream.printf("Latitude      %f\n", BoatData.Latitude);
    stream.printf("Longitude     %f\n",BoatData.Longitude);
    stream.printf("Heading       %f\n",BoatData.Heading);
    stream.printf("COG           %f\n",BoatData.COG);
    stream.printf("SOG           %f\n",BoatData.SOG);
    stream.printf("STW           %f\n",BoatData.STW);
    stream.printf("AWS           %f\n",BoatData.AWS);
    stream.printf("TWS           %f\n",BoatData.TWS);
    stream.printf("MaxAws        %f\n",BoatData.MaxAws);
    stream.printf("MaxTws        %f\n",BoatData.MaxTws);
    stream.printf("AWA           %f\n",BoatData.AWA);
    stream.printf("TWA           %f\n",BoatData.TWA);
    stream.printf("TWD           %f\n",BoatData.TWD);
    stream.printf("TripLog       %f\n",BoatData.TripLog);
    stream.printf("Log           %f\n",BoatData.Log);
    stream.printf("WaterTemp     %f\n",BoatData.WaterTemperature);
    stream.printf("WaterDepth    %f\n",BoatData.WaterDepth);
    stream.printf("Variation     %f\n",BoatData.Variation);
    stream.printf("Altitude      %f\n",BoatData.Altitude);
    stream.printf("GPSTime       %f\n",BoatData.GPSTime);
    stream.printf("DaysSince1970 %lu\n",BoatData.DaysSince1970);
}


//*****************************************************************************
void PrintUlongList(const char *prefix, const unsigned long *List, Stream & stream) {
    uint8_t i;
    if (List != 0) {
        stream.printf(prefix);
        for (i = 0; List[i] != 0; i++) {
            if (i > 0) stream.print(", ");
            stream.printf("%lud", List[i]);
        }
        stream.println();
    }
}

//*****************************************************************************
void PrintText(const char *Text, bool AddLineFeed, Stream & stream) {
    if (Text != 0) 
        stream.print(Text);
    if (AddLineFeed) 
        stream.println();
}

//*****************************************************************************
void PrintDevice(const tNMEA2000::tDevice *pDevice, Stream & stream) {
    if (pDevice == 0) return;

    stream.printf("----------------------------------------------------------------------\n");
    stream.printf("%s\n", pDevice->GetModelID());
    stream.printf("  Source: %d\n", pDevice->GetSource());
    stream.printf("  Manufacturer code:        %d\n",pDevice->GetManufacturerCode());
    stream.printf("  Serial Code:              %s\n", pDevice->GetModelSerialCode());
    stream.printf("  Unique number:            %d\n", pDevice->GetUniqueNumber());
    stream.printf("  Software version:         %s\n", pDevice->GetSwCode());
    stream.printf("  Model version:            %s\n", pDevice->GetModelVersion());
    stream.printf("  Manufacturer Information: ");
    PrintText(pDevice->GetManufacturerInformation(), true, stream);
    stream.printf("  Installation description1: ");
    PrintText(pDevice->GetInstallationDescription1(), true, stream);
    stream.printf("  Installation description2: ");
    PrintText(pDevice->GetInstallationDescription2(), true, stream);
    PrintUlongList("  Transmit PGNs :", pDevice->GetTransmitPGNs(), stream);
    PrintUlongList("  Receive PGNs  :", pDevice->GetReceivePGNs(), stream);
    stream.printf("\n");
}

#define START_DELAY_IN_S 8
//*****************************************************************************
void ListDevices(Stream & stream, bool force = false) {
    static bool StartDelayDone = false;
    static int StartDelayCount = 0;
    static unsigned long NextStartDelay = 0;
  
    if (!StartDelayDone) {  // We let system first collect data to avoid printing all changes
        if (millis() > NextStartDelay) {
            if (StartDelayCount == 0) {
                stream.print("Reading device information from bus ");
                NextStartDelay = millis();
            }
            stream.print(".");
            NextStartDelay += 1000;
            StartDelayCount++;
            if (StartDelayCount > START_DELAY_IN_S) {
                StartDelayDone = true;
                stream.println();
            }
        }
        return;
    }
    if (!force && !pN2kDeviceList->ReadResetIsListUpdated()) return;

    stream.println();
    stream.println("**********************************************************************");
    for (uint8_t i = 0; i < N2kMaxBusDevices; i++) {
        PrintDevice(pN2kDeviceList->FindDeviceBySource(i), stream);
    }
}

// Main application loop.
void loop() {
    int wifi_retry;
    static time_t last = 0;
    static time_t last2 = 0;
    time_t now = time(NULL);;

    // Process any n2k messages
    NMEA2000.ParseMessages();
 
    StringStream s;
    if (now > last + 30) {
        ListDevices(s, true);
        last = now;
    } else {
        ListDevices(s, false);
    }
    Console->print(s.data);
 
    ArduinoOTA.handle();

    // every few hundred msecs

    if (millis() >= mTimeToSec) {
        // Time

        mTimeToSec = millis() + 250;

        mTimeSeconds++;

        // Blink the led
        //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
 
    // Handle any web server requests
    webserver.handleClient();
 
    // handle the telnet session
    handleTelnet();
 
    // And run and shell commands
    handleShell();
    SendN2kEngineFast();
    SendN2kBattery();
    NMEA2000.ParseMessages();
 
    int SourceAddress = NMEA2000.GetN2kSource();
    if (SourceAddress != NodeAddress) {  // Save potentially changed Source Address to NVS memory
        NodeAddress = SourceAddress;     // Set new Node Address (to save only once)
        GwSetVal(LASTNODEADDRESS, String(SourceAddress));
        Console->printf("Address Change: New Address=%d\n", SourceAddress);
    }

    // Dummy to empty input buffer to avoid board to stuck with e.g. NMEA Reader
    if (Console->available()) {
        Console->read();
    }
 
    if (wifiType == 0) {  // Check connection if working as client
        wifi_retry = 0;
        while (WiFi.status() != WL_CONNECTED && wifi_retry < 5) {  // Connection lost, 5 tries to reconnect
            wifi_retry++;
            Console->println("WiFi not connected. Try to reconnect");
            disconnectWifi();
            connectWifi();
        }
    }
}