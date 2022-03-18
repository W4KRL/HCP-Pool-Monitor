const int    FW_VERSION  = 1019;
const String FW_FILENAME = "HCP-WeatherBitIo2021clock";

// uncomment this line for serial output
#define DEBUG
// select the appropriate Thingspeak channel
// use the correct FW_VERSION

//***********************************
//* Put WiFi credentials in setup() *
//* Use ESP8266 core version 2.7.4  * 3.0.2
//***********************************

// Weather Service:
// https://www.weatherbit.io/
// FREE 500 calls/day (minimum interval 173 s)

// 1019 2021-07-06 moved instantiations, revised udp timings
// 1018 2021-07-04 change serial print to DEBUG, clean up clock adjust
// 1017 2021-06-30 new calculation of sleep time
// 1016 2021-06-30 clean up
// 1015 2021-06-30 added timeAwake to clock adjust
// 1014 2021-06-29 fix station ID, add OTA every nighttime cycle
// 1013 2021-06-29 Combined NTP subs. Changed Serial.print to + (string)
// 1012 2021-06-25 reduced CPU to 80MHz, increased NTP retries, added midnight adjust
// 1011 2021-06-22 fixed clockAdjust
// 1010 2021-06-21 removed last interval before daytime
// 1009 2021-06-15 removed postToThingspeak in checkOTA()
// 1008 2021-06-15 improve error handling in ntpEpoch()
// 1007 2021-06-14 add setSleepInterval, changed all times to seconds
// 1006 2021-06-12 revise daytime limits
// 1005 2021-06-12 change nighttime sleep to 60 minutes, space after time string
//                 initialize sleep to SLEEP_SHORT
// 1004 2021-06-11 final hardware on test
// 1003 2021-06-11 display version
// 1002 2021-06-11 clean up
// 1001 2021-06-10 change short sleep to 20 minutes
// 1000 2021-06-10 add NTP, add D1 to power DS18 sensor

/*
   Hunters Creek Swim & Racquet Club
   417 Queens Row Street, Herndon, VA 20170
   P.O. Box 197, Herndon, VA 20172
   (703) 437-9866 (Summer only)
   Summer hours
   SUN:       12pm - 8pm
   MON-THU:   11am - 8pm
   FRI & SAT: 11am - 9pm
*/

/*_____________________________________________________________________________
   Copyright(c) 2018-2021 Karl W. Berger dba IoT Kits https://w4krl.com/iot-kits

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   _____________________________________________________________________________
*/

// *******************************************************
// ******************* INCLUDES **************************
// *******************************************************

// For WiFi
#include <ESP8266WiFi.h>              // [builtin] 
#include <ESP8266WiFiMulti.h>         // [builtin] connect to multiple SSIDs

// For NTP
#include <WiFiUdp.h>                  // [builtin] User Datagram Protocol

// For HTTP OTA
#include <ESP8266HTTPClient.h>        // [builtin] http
#include <WiFiClientSecureBearSSL.h>  // [builtin] https
#include <ESP8266httpUpdate.h>        // [builtin] OTA

// For DS18B20 temperature sensor
#include <OneWire.h>                  // [manager] v2.3.4 Jim Studt https://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DallasTemperature.h>        // [manager] v3.9.0 Miles Burton https://github.com/milesburton/Arduino-Temperature-Control-Library

// Parse weather feed
#include <ArduinoJson.h>              // [manager] v6.18.0 Benoit Blanchon https://github.com/bblanchon/ArduinoJson

// *******************************************************
// ******************* DEFAULTS **************************
// *******************************************************
// !!!!!!!!!!!!!!   CHANGE WITH CARE    !!!!!!!!!!!!!!!!!!

// Hunters Creek Pool location
const String WX_LATITUDE       = "38.976866";
const String WX_LONGITUDE      = "-77.368354";
const String WX_ZIPCODE        = "20172";
const unsigned long TZ_OFFSET = -4 * 3600;     // from UTC
const unsigned long DAY_START = 10 * 3600;     // 10 am
const unsigned long DAY_END   = 21 * 3600;     //  9 pm

// WeatherBit.io
const String WX_SERVER        = "http://api.weatherbit.io/v2.0";
const String WX_API_KEY       = "441bc12bd08748e9895a59f354c68ff6";
const String WX_TYPE          = "current";

// ThingSpeak
const char   TS_SERVER[]      = "api.thingspeak.com";

// HPC Test
//https://thingspeak.com/channels/1415937
//const String TS_WRITE_KEY     = "K9MZK9781JVSUTN0"; // TEST CHANNEL!!!!!!

// Hunters Creek Pool
// https://thingspeak.com/channels/503316
const String TS_WRITE_KEY     = "N3Q2GDC3Q9HX5BR9";

// Device parameters
const float LOW_VBAT          = 3.3;     // 1.1 v/cell
const int   LOW_DBM           = -85;
const long  SLEEP_SHORT       = 20 * 60; // in seconds
const long  SLEEP_LONG        = 60 * 60; // in seconds
const long  OTA_SPAN          = 3;       // daytime OTA interval span
const int   ADC_PIN           = A0;      // scaled to 0-6.5V with 330K resistor
const int   VDD_PIN           = D1;      // Vdd for sensor power
const int   ONE_WIRE_PIN      = D4;      // data from sensor
const int   OTA_OFFSET        = 32;      // OTA uses first 32 blocks of rtc memory (128 bytes)

// NTP parameters
//const char* NTP_SERVER_NAME = "time.nist.gov";
const char* NTP_SERVER_NAME   = "us.pool.ntp.org";

// *******************************************************
// ********** WIRING CONNECTIONS *************************
// *******************************************************
// For Wemos D1 Mini:
// RST - connect to D0 for normal operation
// A0  - connect to 5V with 330 k resistor for 6.5 V scaling
// D0  - open connection to RST when programming via USB
// D5  - not used
// D6  - not used
// D7  - not used
// D8  - pull down, not used
// 3V3 - not used

// TX  - not used
// RX  - not used
// D1  - OneWire source
// D2  - not used
// D3  - pull up, used for boot
// D4  - builtin LED/internal pull up - OneWire data
// GND - battery supply (-) & OneWire ground
// 5V  - battery supply (+)

// *******************************************************
// ******************* GLOBALS ***************************
// *******************************************************
String unitStatus = "";          // holds device status messages
long startTime  = millis();      // record time at start of sketch
// alarm flags
bool lowVbat    = false;
bool lowRSSI    = false;
bool sensorFail = false;
bool rtcValid   = false;         // not used
bool dayTime    = false;

// The ESP8266 Real Time Clock memory is arranged into blocks of 4 bytes.
// The RTC data structure MUST be padded to a 4-byte multiple.
// Maximum 512 bytes (128 blocks) available.
// https://arduino-esp8266.readthedocs.io/en/latest/libraries.html#esp-specific-apis
// Use fixed width types to avoid variations between devices, for example,
// int is two bytes in Arduino UNO and four bytes in ESP8266.
struct
{
  uint32_t  crc32;               // 4 bytes   4 total
  uint8_t   sequence;            // 1 byte,   5 total
  uint8_t   sensorFail;          // 1 byte,   6 total
  uint8_t   lowVbat;             // 1 byte,   7 total
  uint8_t   lowRSSI;             // 1 byte,   8 total
  float     timeAwake;           // 4 bytes, 12 total
  uint32_t  ntpMark;             // 4 bytes  16 total
  uint32_t  lastSleep;           // 4 bytes  20 total
  float     clockAdjust;         // 4 byte   24 total
} rtcData;

// structure to hold sensor measurements & calculations
struct {
  float     airTemp;              // air temperature (F)
  float     poolTemp;             // pool temperature (F)
  float     feelsLike;            // apparent temperature (F)
  float     uvIndex;              // UV Index
  float     humidity;             // relative humidity (%)
  float     vBat;                 // battery voltage
  int       dBm;                  // WiFi signal strength
} monitorData;                    // declare struct

#ifdef DEBUG
#define DEBUG_PRINT(x)    Serial.print (x)
#define DEBUG_PRINTLN(x)  Serial.println (x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// *******************************************************
// ******************** SETUP ****************************
// *******************************************************
void setup()
{
  unsigned long sleep = SLEEP_SHORT;
#ifdef DEBUG
  Serial.begin( 115200 );
#endif
  DEBUG_PRINTLN("\n");

  // this IO line powers the temperature sensor
  // so that it does not draw power during sleep
  pinMode( VDD_PIN, OUTPUT );
  digitalWrite( VDD_PIN, HIGH );

  readRTCmemory();
  DEBUG_PRINT(F(" Sequence: ")); DEBUG_PRINTLN(rtcData.sequence);
  DEBUG_PRINT(F(" Version: ")); DEBUG_PRINTLN( FW_VERSION );

  logonToRouter();

  sleep = setSleepInterval();

  if ( (dayTime  && ( rtcData.sequence % OTA_SPAN == 0 )) || !dayTime)
  {
    checkOTAupdate();                 // check for OTA update
  }
  readSensors();                      // read device sensors
  checkAlarms();                      // see if new or cleared alarms
  getWeatherBit();                    // read current local weather
  postToThingSpeak();                 // send data to ThingSpeak
  printToSerial();                    // print it to the serial port
  writeRTCmemory();                   // save unit parameters
  enterSleep( sleep, rtcData.clockAdjust );  // All done - go to low power sleep
} //setup()

// *******************************************************
// ******************** LOOP *****************************
// *******************************************************
void loop()
{
  // everything is done in setup()
} // loop()

// *******************************************************
// ************** Logon to Wi-Fi router ******************
// *******************************************************
void logonToRouter()
{
  ESP8266WiFiMulti wifiMulti;                // Multiple WiFi networks

  wifiMulti.addAP("DCMNET", "0F1A2D3E4D5G6L7O8R9Y");
  wifiMulti.addAP("KarlsGalaxy", "Compound#1");
  wifiMulti.addAP("hc-iot-net", "smartpool");

  WiFi.forceSleepWake();       // Bakke
  //  WiFi.persistent( false );    // prevent it from writing logon to flash memory
  WiFi.mode( WIFI_STA );

  int count = 100;
  while ( wifiMulti.run() != WL_CONNECTED )
  {
    count--;
    // give up if more than 100 tries (10 seconds)
    if ( count < 0)
    {
      DEBUG_PRINT(F("\nWiFi failed. Restart after sleep. "));
      enterSleep( SLEEP_SHORT, rtcData.clockAdjust );      // retry after sleep
      // **************************************************
      // * PROCESSING ENDS HERE IF IT FAILS TO CONNECT!!! *
      // **************************************************
    }
    delay( 100 );                     // ms delay between checks
    DEBUG_PRINT(".");
  } // loop while not connected
  // WiFi is sucesfully connected
  DEBUG_PRINT(F("\nWiFi SSID: "));
  DEBUG_PRINTLN( WiFi.SSID() );
} // logonToRouter()

// *******************************************************
// ************** Read Pool Sensor ***********************
// *******************************************************
void readSensors()
{
  OneWire oneWire( ONE_WIRE_PIN );           // D4 = GPIO0 has on-board 10K pullup
  DallasTemperature poolSensor( &oneWire );  // DS18B20 sensor

  poolSensor.begin();                 // DS18B20 temperature sensor
  poolSensor.setResolution( 10 );     // 10-bit 0.25C, 188 ms
  DEBUG_PRINTLN("Read pool temp");
  poolSensor.requestTemperatures();
  float poolTempC = poolSensor.getTempCByIndex( 0 );  // read temperature
  float poolTempF = c2f( poolTempC );           // convert C to F
  if ( poolTempF < 0 ) // a missing sensor returns -198 C
  {
    poolTempF = 0;
    sensorFail = true;
  }

  // read battery voltage with external 330K scaling resistor
  // calibrated with linear fit
  float vBat = ( analogRead( ADC_PIN ) - 4.8616 ) / 159.05;
  if ( vBat < LOW_VBAT)
  {
    lowVbat = true;
  }

  int dBm = WiFi.RSSI();                       // RSSI is a negative number
  if ( dBm < LOW_DBM )
  {
    lowRSSI = true;
  }

  // stuff data into sensor struct       TS Field #
  monitorData.poolTemp = poolTempF;            // 5
  monitorData.vBat     = vBat;                 // 6
  monitorData.dBm      = dBm;                  // 7

  DEBUG_PRINTLN("Done with sensor");
} // readSensors()

// *******************************************************
// ********** Get WeatherAPI Weather *********************
// *******************************************************
// https://www.weatherbit.io/api/weather-current
void getWeatherBit()
{
  WiFiClient client;                         // OTA, Weatherbit & ThingSpeak
  HTTPClient http;

  String getQuery = WX_SERVER;
  getQuery += "/" + WX_TYPE;
  getQuery += "?lat=" + WX_LATITUDE + "&lon=" + WX_LONGITUDE;
  getQuery += "&key=" + WX_API_KEY;

  if ( http.begin( client, getQuery ) )
  {
    DEBUG_PRINTLN( getQuery );
    // start connection and send HTTP header
    int httpCode = http.GET();

    // httpCode will be negative on error
    if ( httpCode > 0 )
    {
      DEBUG_PRINT("[HTTP] GET code: "); DEBUG_PRINTLN(httpCode);
      if ( httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY)
      {
        String payload = http.getString();
        parseCurrentWX( payload );
        DEBUG_PRINTLN("Payload: ");
        DEBUG_PRINTLN( payload );
      }
    }
    else
    {
      DEBUG_PRINT("[HTTP] GET error: "); DEBUG_PRINTLN(http.errorToString( httpCode ).c_str());
    }
    http.end();
  }
  else
  {
    DEBUG_PRINTLN("[HTTP} Unable to connect");
  }
} // getWeatherBit()

// *******************************************************
// ************* Parse WeatherBit.io *********************
// *******************************************************
void parseCurrentWX( String json )
{
  DynamicJsonDocument doc( 1536 );  // https://arduinojson.org/v6/assistant/

  deserializeJson( doc, json );

  JsonObject data_0 = doc["data"][0];
  monitorData.humidity  = data_0["rh"];
  monitorData.uvIndex   = data_0["uv"];

  JsonObject data_0_weather = data_0["weather"];
  monitorData.airTemp   = c2f( data_0["temp"] );
  monitorData.feelsLike = c2f( data_0["app_temp"] );

} // parseCurrentWX()

// *******************************************************
// ********** Post data to ThingSpeak ********************
// *******************************************************
void postToThingSpeak()
{
  WiFiClient client;

  // assemble and post the data
  DEBUG_PRINTLN("Connecting to ThingSpeak");
  if ( client.connect( TS_SERVER, 80 ) == true )
  {
    DEBUG_PRINTLN("ThingSpeak Server connected.");
    // declare dataString as a String and initialize with the API_WRITE_KEY
    String dataString = TS_WRITE_KEY;
    // cocatenate each field onto the end of dataString
    dataString += "&field1=" + String( monitorData.feelsLike );
    dataString += "&field2=" + String( monitorData.airTemp );
    dataString += "&field3=" + String( rtcData.timeAwake );  // from previous cycle
    dataString += "&field4=" + String( monitorData.uvIndex );
    dataString += "&field5=" + String( monitorData.poolTemp );
    dataString += "&field6=" + String( monitorData.vBat );
    dataString += "&field7=" + String( monitorData.dBm );
    dataString += "&field8=" + String( monitorData.humidity );

    if ( unitStatus != "" )
    {
      dataString += "&status=" + unitStatus;
    }

    // post the data to ThingSpeak
    client.println("POST /update HTTP/1.1");
    client.println("Host: api.thingspeak.com");
    client.println("Connection: close");
    client.println("X-THINGSPEAKAPIKEY: " + TS_WRITE_KEY);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Content-Length: " + String( dataString.length() ) );
    client.println("");
    client.println( dataString );
  }
  client.stop();
  DEBUG_PRINTLN("ThingSpeak disconnected.");
} // postToThingSpeak()

// *******************************************************
// *********** Display data on serial port ***************
// *******************************************************
void printToSerial()
{
  DEBUG_PRINTLN(F("---------------"));
  DEBUG_PRINT(F("Pool Temperature: \t"));
  DEBUG_PRINTLN( monitorData.poolTemp );
  DEBUG_PRINT(F("Air Temperature: \t"));
  DEBUG_PRINTLN( monitorData.airTemp );
  DEBUG_PRINT(F("Feels like: \t\t"));
  DEBUG_PRINTLN( monitorData.feelsLike );
  DEBUG_PRINT(F("Humidity: \t\t"));
  DEBUG_PRINTLN( monitorData.humidity );
  DEBUG_PRINT(F("UV Index: \t\t"));
  DEBUG_PRINTLN( monitorData.uvIndex );
  DEBUG_PRINT(F("Battery Voltage: \t"));
  DEBUG_PRINTLN( monitorData.vBat );
  DEBUG_PRINT(F("Signal Strength: \t"));
  DEBUG_PRINTLN( monitorData.dBm );
  DEBUG_PRINT(F("Time awake: \t\t"));
  DEBUG_PRINTLN( rtcData.timeAwake );
  DEBUG_PRINT(F("Clock Adjust: \t\t"));
  //  DEBUG_PRINTLN( rtcData.clockAdjust, 3 );
  DEBUG_PRINTLN( rtcData.clockAdjust );
  DEBUG_PRINT(F("Unit Status: "));
#ifdef DEBUG
  if ( unitStatus == "" )
  {
    DEBUG_PRINTLN(F("\tOK"));
  }
  else
  {
    DEBUG_PRINTLN( unitStatus );
  }
#endif
  DEBUG_PRINTLN(F("---------------"));
} // printToSerial()

// *******************************************************
// *************** Celsius to Fahrenheit *****************
// *******************************************************
float c2f( float c )
{
  return 1.8 * c + 32.0;
} // c2f()

// *******************************************************
// ****************** Check Alarm status *****************
// *******************************************************
// if there is an alarm now but not in the past it is a new alarm
// if there is no alarm now but there was one in the past it is cleared
void checkAlarms() {
  if ( lowVbat && !rtcData.lowVbat ) {
    unitStatus += "Low battery Voltage. ";
  }
  if ( !lowVbat && rtcData.lowVbat ) {
    unitStatus += "Low Battery Voltage cleared. ";
  }

  if ( sensorFail && !rtcData.sensorFail ) {
    unitStatus += "Sensor Failed. ";
  }
  if ( !sensorFail && rtcData.sensorFail ) {
    unitStatus += "Sensor Failure cleared. ";
  }

  if ( lowRSSI && !rtcData.lowRSSI )  {
    unitStatus += "Low WiFi. ";
  }
  if ( !lowRSSI && rtcData.lowRSSI ) {
    unitStatus += "Low WiFi cleared. ";
  }
} // checkAlarms()

// *******************************************************
// **************** Check for OTA Updates ****************
// *******************************************************
void checkOTAupdate() {
  const String FW_VERSION_EXT = ".version";
  const String FW_IMAGE_EXT = ".ino.d1_mini.bin";
  const String FW_URL_BASE = "https://w4krl.com/fota/";
  const String FW_PATH = FW_FILENAME + "/";
  const String FW_VERSION_URL = FW_URL_BASE + FW_PATH + FW_FILENAME + FW_VERSION_EXT;
  const String FW_IMAGE_URL   = FW_URL_BASE + FW_PATH + FW_FILENAME + FW_IMAGE_EXT;

  //  DEBUG_PRINT("bin: "); DEBUG_PRINTLN( FW_IMAGE_URL );

  std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
  client->setInsecure();  // doesn't need fingerprint!!!!

  HTTPClient https;

  if ( https.begin( *client, FW_VERSION_URL) )
  {
    //    unitStatus += "OTA check @ seq. " + String(rtcData.sequence);
    // start connection and send HTTP header
    int httpCode = https.GET();
    DEBUG_PRINT("[HTTPS] GET code: "); DEBUG_PRINTLN(httpCode);
    if ( httpCode > 0 )
    {
      // HTTP header has been sent and Server response header has been handled
      if ( httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY )
      {
        String newFWVersion = https.getString();
        int newVersion = newFWVersion.toInt();
        DEBUG_PRINT(F("Using version: ")); DEBUG_PRINTLN( FW_VERSION );
        DEBUG_PRINT(F("Found version: ")); DEBUG_PRINTLN( newVersion );
        if ( newVersion > FW_VERSION )
        {
          unitStatus += "Update to FW:" + newFWVersion + ". ";
          DEBUG_PRINTLN(F("Updating bin: "));
          DEBUG_PRINTLN( FW_IMAGE_URL );

          // this performs the update and reboots
          t_httpUpdate_return ret = ESPhttpUpdate.update( *client, FW_IMAGE_URL );  // must be *client
          switch ( ret )
          {
            case HTTP_UPDATE_FAILED:
              DEBUG_PRINTLN("[update] Update failed.");
              break;
            case HTTP_UPDATE_NO_UPDATES:
              DEBUG_PRINTLN("[update] Update no Update.");
              break;
          }
        }
        else
        {
          unitStatus += "FW:";
          unitStatus += FW_VERSION;
          unitStatus += ". ";
          DEBUG_PRINTLN(F( "On latest FW version." ));
        }
      }
    }
    else
    {
      DEBUG_PRINT(F("[HTTPS] GET failed, error: ")); DEBUG_PRINTLN( https.errorToString( httpCode ).c_str( ));
    }
    https.end();
  }
  else
  {
    DEBUG_PRINTLN(F("[HTTPS] Unable to connect"));
  }
}

// RTC Memory Functions: The ESP8266 internal Real Time Clock has unused memory
// that remains active during the Deep Sleep mode. This sketch stores WiFi connection
// information in RTC memory to speed up connection time.
// *******************************************************
// ******************* Read RTC Memory *******************
// *******************************************************
void readRTCmemory()
{
  rtcValid = false;
  // offset data 32 bytes to avoid OTA area
  if ( ESP.rtcUserMemoryRead( OTA_OFFSET, (uint32_t*)&rtcData, sizeof( rtcData ) ) )
  {
    // Calculate the CRC of what we just read from RTC memory,
    // but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32(((uint8_t*)&rtcData ) + 4, sizeof( rtcData ) - 4 );
    if ( crc == rtcData.crc32 )
    {
      rtcValid = true;
    }
    rtcData.timeAwake = constrain( rtcData.timeAwake, 0, 30 );
    rtcData.clockAdjust = constrain( rtcData.clockAdjust, 0.9, 1.1 );
  }
} // readRTCmemory()

// *******************************************************
// ****************** Write RTC Memory *******************
// *******************************************************
void writeRTCmemory()
{
  //  rtcData.wifiChannel = WiFi.channel();         // WiFi channel
  //  // memcpy explanation: http://arduino.land/FAQ/content/6/30/en/how-to-copy-an-array.html
  //  memcpy( rtcData.bssid, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
  rtcData.lowVbat    = lowVbat;
  rtcData.lowRSSI    = lowRSSI;
  rtcData.sensorFail = sensorFail;
  rtcData.timeAwake  = constrain(( millis() - startTime ) / 1000.0, 0, 30);  // total awake time in seconds
  rtcData.clockAdjust = constrain( rtcData.clockAdjust, 0.9, 1.1 );
  //  DEBUG_PRINT("RTC write CA: "); DEBUG_PRINTLN(rtcData.clockAdjust, 3);
  DEBUG_PRINT("RTC write CA: "); DEBUG_PRINTLN(rtcData.clockAdjust);
  rtcData.crc32      = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );

  // offset data 32 bytes to avoid OTA area
  ESP.rtcUserMemoryWrite( OTA_OFFSET, (uint32_t*)&rtcData, sizeof( rtcData ) );
} // writeRTCmemory()

// *******************************************************
// ******************** Calculate CRC32 ******************
// *******************************************************
// Cribbed from Bakke. Originated by others.
uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while ( length-- ) {
    uint8_t c = *data++;
    for ( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if ( c & i ) {
        bit = !bit;
      }
      crc <<= 1;
      if ( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
} // calculateCRC32()

// *******************************************************
// ***************** Enter Sleep Mode ********************
// *******************************************************
void enterSleep( unsigned long sleep, float clockRatio )
{
  DEBUG_PRINT("Sleeping for "); DEBUG_PRINT(sleep); DEBUG_PRINTLN(" seconds.");
  sleep = sleep * clockRatio;
  // WAKE_RF_DEFAULT wakes the ESP8266 with Wi-Fi enabled
  ESP.deepSleep( sleep * 1000000UL, WAKE_RF_DEFAULT );
} // enterSleep()
