/*******************************************************************************
* Example of Vehicle Tracking and Logging using TraLog
* Version: 1.00
* Date: 04-10-2012
* Company: Rocket Scream Electronics
* Author: Lim Phang Moh
* Website: www.rocketscream.com
*
* This is an example on how to use all 3 feature (GPRS-GPS-SD) of the TraLog 
* shield. We will update a Cosm feed using HTTP PUT request over GPRS, writing 
* to a text file on the microSD card, and receiving GPS NMEA data. Due to the 
* limitation on the RAM size on ATMega328, all the task must be executed in a 
* modest approach by allowing all task to run one at a time rather than doing 
* everything at one go. Avoid using excessive amount of RAM 
* (example: Serial.print("Hello") should be avoided and store the string in 
* Flash memory like for example: Serial.print(F("Hello))).
*
* ============
* Requirements
* ============
* 1. UART selection switch to SW position (uses pin D5 (RX) & D6 (TX)).
* 2. Jumper J14 is closed to allow usage of pin A2 to control on-off state of 
*    WISMO228 module. This is the default factory setting.
* 3. You need to know your service provider APN name, username, and password. 
*    If they don't specify the username and password, you can use " ". Notice 
*    the space in between the quote mark. 
* 4. The Cosm server name & port.
* 5. Cosm Api Key.
* 6. Cosm data stream names.
* 7. A microSD card pre-formatted in FAT32 format.
* 8. Both GSM & GPS antenna attached to the TraLog shield.
*
* ============
* Instructions
* ============
* 1. Use the RTC example (on WISMO228 library) to set the GSM RTC to your 
*    desired time prior to using this example. Or you can uncomment the RTC_SET
*    setting section in the setup code but time will be set to the value 
*    specified in this firmware every time it power up.
* 2. Ensure a CR1220 coin cell is used to provide battery backup to the 
*    WISMO228 internal RTC in the even of power inavailability.
* 3. Please create your own Cosm/Pachube datastream and replace necessary
*    parameter(s) to suit your setup. 
*
* This example is licensed under Creative Commons Attribution-ShareAlike 3.0 
* Unported License. 
*
* Revision  Description
* ========  ===========
* 1.00      Initial public release. Tested with L22 & L23 of WISMO228 firmware.
*           Only works with Arduino IDE 1.0 & 1.0.1. Uses v1.10 of the WISMO228 
*           library.
*******************************************************************************/
// ***** COMPILE OPTIONS *****
// Uncomment to allow serial debug messages
#define DEBUG 
// Uncomment to set the GSM RTC using the time & date specified in newClock[]
#define RTC_SET

// ***** INCLUDES *****
#include <SD.h>
#include "SoftwareSerial.h"
#include <WISMO228.h>
#include <TinyGPS.h>

// ***** PIN ASSIGNMENT *****
const  uint8_t  gsmRxPin = 5;
const  uint8_t  gsmTxPin = 6;
const  uint8_t  gsmOnOffPin = A2;
const  uint8_t  gpsRxPin = 7;
const  uint8_t  gpsTxPin = 8;
const  uint8_t  sdCsPin = 4;

// ***** CONSTANTS *****
// ***** GPRS PARAMATERS *****
const  char  apn[] = "apn";
const  char  username[] = "username";
const  char  password[] = "password";
// ***** COSM PARAMATERS *****
const  char server[] = "api.cosm.com";
const  char path[] = "/v2/feeds/InsertYourFeedNumberHere.csv";
const  char port[] = "80";
const  char host[] = "api.cosm.com";
const  char controlKey[] = "X-PachubeApiKey: InsertYourPachubeApiKeyHere";
const  char contentType[] = "text/csv";
const  char stream1[] = "0"; // Cosm feeds data stream name
const  char stream2[] = "1"; // Cosm feeds data stream name
// ***** RTC *****
// Clock in YY/MM/DD,HH:MM:SS*ZZ format
// ZZ represent time difference between local & GMT time expressed in quarters
// of an hour. *ZZ has a range of -48 to +48. 
// Example: 12/10/16,17:30:00+32 (GMT+8 local time).
const  char  newClock[CLOCK_COUNT_MAX + 1] = "12/10/16,17:30:00+32";
#define LOG_INTERVAL 60000

// ***** CLASSES *****
// File class
File logFile;
// GPS Parser Class
TinyGPS gps;
// Software serial for the GPS
SoftwareSerial ss(gpsRxPin, gpsTxPin);
// WISMO228 class
WISMO228  wismo(gsmRxPin, gsmTxPin, gsmOnOffPin);

// ***** VARIABLES *****
char  clock[CLOCK_COUNT_MAX + 1];
char  data[32];
unsigned long scheduler;
boolean newGpsData = false;

void setup()  
{
  // Use hardware serial to track the progress of task execution
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println(F("===Vehicle Tracking & Logging using TraLog==="));
  #endif

  // Initialize the SD card
  if (!SD.begin(sdCsPin)) 
  {
    #ifdef DEBUG
      Serial.println(F("SD card init failed!"));
    #endif
    return;
  }
  Serial.println(F("SD card init OK."));

  // Initialize WISMO228
  wismo.init();

  // Perform WISMO228 power up sequence
  if (wismo.powerUp())
  {
    #ifdef DEBUG
      Serial.println(F("GSM is awake."));
    #endif
    
		#ifdef RTC_SET
      if (wismo.setClock(newClock))
      {
        #ifdef DEBUG
          Serial.println(F("New clock set."));
        #endif
        // Let's check the time & date
        if (wismo.getClock(clock))
        {
          #ifdef DEBUG    
            Serial.print(F("RTC: "));
            Serial.println(clock);
          #endif
        }  
      }
		#endif
  }
  // Running 2 software serial at 1 time makes the application unpredictable
  // due to excessive RAM usage and multiple fast edge interrupt
  wismo.shutdown();
  
  #ifdef DEBUG
    Serial.println(F("GSM back to sleep."));	
  #endif
  
  // Set tracking, feed update & logging interval
  scheduler = millis() + LOG_INTERVAL;
}

void loop() 
{ 
  if (millis() > scheduler)
  {
    scheduler = millis() + LOG_INTERVAL;

    // Get GPS data for current location
    processGps();

    // New valid GPS data available
    if (newGpsData)
    {
      // Feed current location to COSM datastream
      processGsm();

      // Log current location into microSD card
      processLog();
      
      // Latest GPS data logged
      newGpsData = false;
    }	
  }
}

void processGps(void)
{
  ss.begin(4800);

  #ifdef DEBUG
    Serial.println(F("GPS gathering data..."));
  #endif

  // Parse GPS data for 2 second
  for (unsigned long start = millis(); millis() - start < 2000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      // New valid NMEA data available
      if (gps.encode(c)) 
      {
        newGpsData = true;
      }
    }
  }  
  ss.end();
  #ifdef DEBUG
    if (newGpsData)
    {
      Serial.println(F("New GPS data available."));
    }
    else
    {
      Serial.println(F("No GPS fix available.")); 
    }
  #endif
}

void processGsm(void)
{
  if (wismo.powerUp())
  {
    #ifdef DEBUG
      Serial.println(F("GSM wakes up."));
    #endif
    // Retrieve time & date
    if (wismo.getClock(clock))
    {
      #ifdef DEBUG
        Serial.print("Current clock: ");
        Serial.println(clock);
      #endif
    }
    
    float flat, flon;
    unsigned long age;
    char  coordinate[8];
    gps.f_get_position(&flat, &flon, &age);

    // Prepare data for Cosm feed
    strcpy(data, stream1);
    strcat(data, ",");
    dtostrf(flat, 2, 4, coordinate); 
    strcat(data, coordinate);
    strcat(data, "\r\n");
    strcat(data, stream2);
    strcat(data, ",");
    dtostrf(flon, 2, 4, coordinate);
    strcat(data, coordinate);
    strcat(data, "\r\n");
    // Terminate the string
    strcat(data, "\0");
    
    // Connect to GPRS network
    if (wismo.openGPRS(apn, username, password))
    {
      #ifdef DEBUG
        Serial.println(F("GPRS OK."));
        Serial.println(F("Sending feed to Cosm."));      
      #endif
      
      // Send current coordinate as Cosm feed through HTTP PUT request
      if (wismo.putHttp(server, path, port, host, data, controlKey, 
			                  contentType))			
      {
        #ifdef DEBUG
          Serial.println(F("Feed sent!"));
        #endif
      }
      else
      {
        #ifdef DEBUG
          Serial.println(F("Unable to send feed."));
        #endif
      }

      // Close the GPRS connection
      if (wismo.closeGPRS())
      {
        #ifdef DEBUG
          Serial.println(F("GPRS closed."));
        #endif
      }
    }
    else
    {
      #ifdef DEBUG
        Serial.println(F("GPRS failed."));
      #endif
    }
  }
  wismo.shutdown();
}

void processLog()
{
  // Open the file
  logFile = SD.open("log.txt", FILE_WRITE);

  // File is successfully opened
  if (logFile) 
  {
    #ifdef DEBUG
      Serial.println(F("Logging..."));
    #endif	

    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    // Log current time
    logFile.print(clock);
    logFile.print(",");
    // Log latitude
    logFile.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 2);
    logFile.print(",");
    // Log longitude
    logFile.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 2);
    // Close file
    logFile.close();
    
    #ifdef DEBUG
      Serial.println(F("Logging complete."));
    #endif
  } 
  // File open failed
  else 
  {
    #ifdef DEBUG
      Serial.println(F("Logging error."));
    #endif
  }
}
