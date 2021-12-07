#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_wifi.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <TimeLib.h>
#include <Ticker.h>
#include <TinyGPSPlus.h>
#include <RTClib.h>
#include <HardwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ArduinoJson.h"
#include "timecore.h"
#include "datastore.h"

#include "ntp_server.h"
#include "network.h"

#define  GPSSerial Serial1
#define  GPSBAUD (9600)

#define MAX_SRV_CLIENTS (5)

#define BUTTON_A 15
#define BUTTON_B 32
#define BUTTON_C 14

WiFiServer TelnetServer(23);
WiFiClient serverClients[MAX_SRV_CLIENTS];

Timecore timec;
RTC_DS3231 rtc;
Ticker TimeKeeper;
TinyGPSPlus gps;
NTP_Server NTPServer;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// Information variables for OLED Display
String currInfo = "";
String oldInfo = " ";
int currButton = 0;

/* 63 Char max and 17 missign for the mac */
TaskHandle_t GPSTaskHandle;
SemaphoreHandle_t xSemaphore = NULL;
SemaphoreHandle_t xi2cmtx = NULL;

//Used for the PPS interrupt 
const byte interruptPin = 25;

volatile uint32_t UptimeCounter=0;
volatile uint32_t GPS_Timeout=0;
volatile uint32_t pps_counter=0;
bool pps_active = false;
gps_settings_t gps_config;
uint32_t RTC_ReadUnixTimeStamp(bool* delayed_result);
void RTC_WriteUnixTimestamp(uint32_t ts);



/**************************************************************************************************
 *    Function      : GetUTCTime
 *    Description   : Reads the UTCTime
 *    Input         : none 
 *    Output        : uint32_t 
 *    Remarks       : none
 **************************************************************************************************/
 uint32_t GetUTCTime(void);

/**************************************************************************************************
 *    Function      : handlePPSInterrupt
 *    Description   : Interrupt from the GPS module
 *    Input         : none 
 *    Output        : none
 *    Remarks       : needs to be placed in RAM ans is only allowed to call functions also in RAM
 **************************************************************************************************/
void IRAM_ATTR handlePPSInterrupt() {
  pps_counter++;
  UptimeCounter++;
  timec.RTC_Tick();
  decGPSTimeout();
  pps_active = true; 
  xSemaphoreGiveFromISR(xSemaphore, NULL);       
}

/**************************************************************************************************
 *    Function      : setup
 *    Description   : Get all components in ready state
 *    Input         : none 
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void setup() {
  
  // First we setup the serial console with 115k2 8N1
  Serial.begin (57600);

  // The next is to initilaize the datastore, here the eeprom emulation
  datastoresetup();

  // This is for the flash file system to access the webcontent
  SPIFFS.begin(true);
  
  // We setup the xSemaphore to sync the screen update
  xSemaphore = xSemaphoreCreateBinary();
  
  // We setup the i2c semaphore to sync i2c access
  xi2cmtx = xSemaphoreCreateBinary();
  
  // Make access to the i2c possible
  xSemaphoreGive(xi2cmtx); 
  
  // We read the Config from flash
  Serial.println(F("Read Timecore Config"));
  timecoreconf_t cfg = read_timecoreconf();
  timec.SetConfig(cfg);
  
  // This delay the boot for a few seconds and will erase all config if the boot btn is pressed
  Serial.println(F("Booting...."));
  
  for(uint32_t i = 0; i < 25; i++){
    
    if(digitalRead(0) == false){
     
      Serial.println(F("Erase EEPROM"));
      erase_eeprom();  
      break;
      
    } else {
      vTaskDelay(100/portTICK_PERIOD_MS); 
    }
  }

  // Start OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();
  
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text sizing and color
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  
  // We start to configure the WiFi
  Serial.println(F("Init WiFi"));     
  initWiFi();

  // Next is to read how the GPS is configured here, if we use it for sync or not
  TelnetServer.begin();
  TelnetServer.setNoDelay(true);
  gps_config = read_gps_config();
    
  if(! rtc.begin()){
    // We can run without rtc..
    Serial.println("RTC device is missing.");
    Serial.flush();
  } else {
    if (rtc.lostPower()) {

      Serial.println("RTC lost power.  Setting the time.");
      
      // When time needs to be set on a new device or after power loss, the
      // following line sets the RTC to the date & time this sketch was compiled.
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      Serial.println(F("I2C RTC"));
      
      // We now register the clock in the time core component      
      rtc_source_t I2C_DS3231;
      I2C_DS3231.SecondTick = NULL;
      I2C_DS3231.type = RTC_CLOCK;
      I2C_DS3231.ReadTime = RTC_ReadUnixTimeStamp;
      I2C_DS3231.WriteTime = RTC_WriteUnixTimestamp;
      timec.RegisterTimeSource(I2C_DS3231);
      
      // Force a snyc to the clock 
      DateTime now = rtc.now();
      timec.SetUTC(now.unixtime(), RTC_CLOCK);
      
      // Next is to output the time we have form the clock to the user 
      Serial.print(F("Read RTC Time:"));
      Serial.println(now.unixtime());

     
      
    } else {
      
      // Clock is found
      Serial.println("Found the clock!");
      Serial.println(F("I2C RTC"));
      
      // We now register the clock in the time core component 
      rtc_source_t I2C_DS3231;
  
      I2C_DS3231.SecondTick = NULL;
      I2C_DS3231.type = RTC_CLOCK;
      I2C_DS3231.ReadTime = RTC_ReadUnixTimeStamp;
      I2C_DS3231.WriteTime = RTC_WriteUnixTimestamp;
      timec.RegisterTimeSource(I2C_DS3231);
  
      // Force a snyc to the clock 
      DateTime now = rtc.now();
      timec.SetUTC(now.unixtime(), RTC_CLOCK);
      
    }
  }
 
  // Last step is to get the NTP running
  NTPServer.begin(123 , GetUTCTime);
  
  // Now we start with the config for the Timekeeping and sync 
  TimeKeeper.attach_ms(200, _200mSecondTick);
 
  //hws.begin(GPSBAUD ,SERIAL_8N1,17,16); // 17->RX , 16->TX
  GPSSerial.begin(9600);

}
/**************************************************************************************************
 *    Function      : GetUTCTime
 *    Description   : Returns the UTC Timestamp
 *    Input         : none 
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
uint32_t GetUTCTime( void ){
  uint32_t timest = 0;
  timest = timec.GetUTC();
  Serial.printf("Timestamp is %i\n\r",timest);
  return timest;
}

/**************************************************************************************************
 *    Function      : _200mSecondTick
 *    Description   : Runs all functions inside once a second
 *    Input         : none 
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void _200mSecondTick( void ){
   static uint8_t callcount=0;
   static uint8_t pps_check_cnt=0;
   static uint32_t pps_count_last = 0;
   static bool last_pps_state = false;

   if( (last_pps_state != pps_active) && ( true == pps_active ) ){
     Serial.println("Switch to PPS");
   }

   // As long as the PPS is active we set the internal counter to zero
   if(true == pps_active){
     callcount=0;
   } else {
     callcount++;
   }

   // If we have no new PPS interrupt we increment the timout value
   if(pps_count_last != pps_counter){
      pps_check_cnt=0;
   } else {
      pps_check_cnt++;
   }

   /*  If we are running on internal clock
    *  we increment every  second the time 
    *  also we give to the xSemaphore to inform
    *  that the display needs to update
    */
   if( callcount >=5 ){
     if(false == pps_active ){
       timec.RTC_Tick();
       GPS_Timeout=0;
       UptimeCounter++; 
       xSemaphoreGive(  xSemaphore );   
     } 
     callcount=0;
        
   } 

   /*
    * if the pps is active and over 1200ms since the last time
    * we switch back to the internal clock and also we 
    * compensate the overdue by setting the next second 
    * to be fired 600ms after this
    * 
    */
   if( ( pps_active== true) && (pps_check_cnt>=7) ){ /*1400ms*/
    /* We assume that the pps is gone now and switch back to the internal timsource */
    /* Also we need to increment time here */
    pps_active=false;
    callcount=2;
    timec.RTC_Tick();
    GPS_Timeout=0;
    UptimeCounter++; 
    xSemaphoreGive(  xSemaphore );       
    /* The callcount shall now be 2 and we need  to set it to keep time*/
    Serial.println("Switch to internal clock");
   }
   
   pps_count_last=pps_counter;
   last_pps_state = pps_active;   
}

/**************************************************************************************************
 *    Function      : decGPSTimeout
 *    Description   : decements the timputvaue for GPS Time Update
 *    Input         : none 
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void decGPSTimeout( void ){
  if(GPS_Timeout>0){
    GPS_Timeout--;
  }
}


/**************************************************************************************************
 *    Function      : TelnetDebugService
 *    Description   : Cares for new and old connections to keep them alive
 *    Input         : none 
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void TelnetDebugService( void )//This keeps the connections alive 
{
   uint8_t con =0;
   if (TelnetServer.hasClient()){
      for(con = 0; con < MAX_SRV_CLIENTS; con++){
        //find free/disconnected spot
        if (!serverClients[con] || !serverClients[con].connected()){
          if(serverClients[con]) serverClients[con].stop();
          serverClients[con] = TelnetServer.available();
          if (!serverClients[con]) Serial.println("available broken");
          Serial.print("New client: ");
          Serial.print(con); Serial.print(' ');
          Serial.println(serverClients[con].remoteIP());
          break;
        }
      }
      if ( con >= MAX_SRV_CLIENTS) {
        //no free/disconnected spot so reject
        TelnetServer.available().stop();
      }
    }

    for(uint8_t i = 0; i < MAX_SRV_CLIENTS; i++){
      if (serverClients[i] && serverClients[i].connected()){
        if(serverClients[i].available()){
          //get data from the telnet client and flush it
          serverClients[i].read();
        }
      }
      else {
        if (serverClients[i]) {
          serverClients[i].stop();
        }
      }
    }


}


/**************************************************************************************************
 *    Function      : TelenetDebugServerTx
 *    Description   : Will send one byte to teh conncted clients
 *    Input         : none 
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void TelenetDebugServerTx( int16_t Data ){
      if(Data<0){
        return;
      }
      //This is really inefficent but shall work 
      uint8_t sbuf[1];
      sbuf[0] = Data;
      //push UART data to all connected telnet clients
      for(uint8_t i = 0; i < MAX_SRV_CLIENTS; i++){
        if (serverClients[i] && serverClients[i].connected()){
          serverClients[i].write(sbuf, 1);
          delay(1); //?
        }
      }
}


/**************************************************************************************************
 *    Function      : loop
 *    Description   : Superloop
 *    Input         : none 
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void loop()
{   
  // Process all networkservices
  NetworkTask();
  TelnetDebugService();
 
  // timeupdate done here is here
  while (GPSSerial.available()){
      //int16_t Data = hws.read();
      char Data = GPSSerial.read();
      gps.encode(Data);
      // We check here if we have a new timestamp, and a valid GPS position
      if( (gps.date.isValid()==true) && ( gps.time.isValid()==true) && ( gps.location.isValid() == true ) ) {
        if(gps.time.isUpdated()==true){   
          uint32_t ts_age = gps.time.age(); //this are milliseconds and should be lower than 1 second 
          if(ts_age> 1000 ){
            Serial.println("Warning GPS Timestamp older than a second");      
          }
          /* We need to feed the gps task */
          datum_t newtime;
          newtime.year=gps.date.year();
          if(newtime.year>=2000){
            newtime.year-=2000;
          }
          newtime.month=gps.date.month();
          newtime.day=gps.date.day();
          newtime.dow=0;
          newtime.hour=gps.time.hour();
          newtime.minute=gps.time.minute();
          newtime.second=gps.time.second();
          
          // Added by T. Godau DL9SEC 30.05.2020 
          // Fix for older GPS with the week number rollover problem (see https://en.wikipedia.org/wiki/GPS_Week_Number_Rollover)
          uint32_t newtimestamp= timec.TimeStructToTimeStamp(newtime);

          if(gps_config.rollover_cnt>0){

           newtimestamp += (  SECS_PER_WEEK * 1024 *gps_config.rollover_cnt );
          
          }
          
          /* We print the time for debug */
          if( (true == gps_config.sync_on_gps) && (GPS_Timeout<=0) ){
            //Serial.println("Update Time from GPS");
            //Serial.printf("Date is: %i/%i/%i at %i:%i:%i \r\n",newtime.year,newtime.month,newtime.day,newtime.hour,newtime.minute,newtime.second);
            //This function is overloaded and takes timestamps as time_t structs
            timec.SetUTC(newtimestamp,GPS_CLOCK);
            GPS_Timeout= 600; //10 Minute timeout
          } else {
            // Do nothing           
          }
      } 
    }
    TelenetDebugServerTx(Data);

  }

  currInfo = "";

  // Buttons on the display are only up for one loop and are then powered down
  // Set a global variable so that we can work on it as soon as the button is found to be pushed
  // IP info display
  if(!digitalRead(BUTTON_A)) {
    currButton = 1;
  }
  // GPS lock check display
  if(!digitalRead(BUTTON_B)) {
    currButton = 2;
  }
  // RTC info display
  if(!digitalRead(BUTTON_C)) {
      currButton = 3;
  }

  if (currButton == 1) {
    String currIP = WiFi.localIP().toString();
    if (currIP != "192.168.4.1") {
      currInfo = "Server Mode\n" + currIP;
    } else {
      currInfo = "Soft AP Mode\n" + currIP;
    }
  }
  
  if (currButton == 2) {
    if( (gps.date.isValid()==true) && ( gps.time.isValid()==true) && ( gps.location.isValid() == true ) ) {
      currInfo = "Latitude: ";
      currInfo.concat(gps.location.lat());
      currInfo = currInfo + "\nLongitude:  ";
      currInfo.concat(gps.location.lng());
    } else {
      currInfo = "GPS does not have a valid lock.";
    }
  }

  if (currButton == 3) {
    DateTime time = rtc.now();
    currInfo = String("")+time.timestamp(DateTime::TIMESTAMP_FULL) + "\n";
    float temp = rtc.getTemperature();
    currInfo.concat(temp);
    currInfo = currInfo + " C";
  }

  // if currInfo is new and not empty, update the display
  if (currInfo != "" && currInfo != oldInfo) {
      infoDisplay(currInfo, currButton);
      if (currInfo == " ") {
          currInfo = "";
      }
      oldInfo = currInfo;
  }
}


/**************************************************************************************************
 *    Function      : RTC_ReadUnixTimeStamp
 *    Description   : Writes a UTC Timestamp to the RTC
 *    Input         : bool*  
 *    Output        : uint32_t
 *    Remarks       : Requiered to do some conversation
 **************************************************************************************************/
uint32_t RTC_ReadUnixTimeStamp(bool* delayed_result){
  DateTime now = (0,0,0,0,0,0);
  if( true == xSemaphoreTake(xi2cmtx,(100 / portTICK_PERIOD_MS) ) ){
   now = rtc.now();
   xSemaphoreGive(xi2cmtx);
  }
   *delayed_result=false;
   return now.unixtime();
}


/**************************************************************************************************
 *    Function      : RTC_WriteUnixTimestamp
 *    Description   : Writes a UTC Timestamp to the RTC
 *    Input         : uint32_t 
 *    Output        : none
 *    Remarks       : Requiered to do some conversation
 **************************************************************************************************/
void RTC_WriteUnixTimestamp( uint32_t ts){
   uint32_t start_wait = millis();
   if( true == xSemaphoreTake(xi2cmtx,(40 / portTICK_PERIOD_MS) ) ){  
    ts = ts + ( ( millis()-start_wait)/1000);
    rtc.adjust(DateTime( ts)); 
    DateTime now = rtc.now();
    //Serial.println("Update RTC");
    if( ts != now.unixtime() ){
      Serial.println(F("I2C-RTC W-Fault"));
    }
     xSemaphoreGive(xi2cmtx);
   }
}

/**************************************************************************************************
 *    Function      : infoDisplay
 *    Description   : Write value thrown from loop to the display
 *    Input         : string, integer
 *    Output        : none
 *    Remarks       : Handles the screen refresh, delay, and yield
 **************************************************************************************************/
void infoDisplay(String info, int button) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(info);
    delay(10);
    yield();
    display.display();
}


 
