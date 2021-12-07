# Burger-ESP32-NTPServer
NTP Server using ESP32 hardware


Hello!  This was a project from a request from a colleague.  This was built by my wife and I for fun.

First and foremost, we do not intend to maintain this software.  When the hardware and/or supporting software changes over time, we will not update this code.  Use at your own risk!

The project is rather simple in scope.  A stratum-1 NTP server that uses GPS, RTC, and with a relatively easy to use web interface.

---HARDWARE---

1.  Adafruit HUZZAH32 – ESP32 Feather Board (https://www.adafruit.com/product/3405)
2.  DS3231 Precision RTC FeatherWing (https://www.adafruit.com/product/3028)
3.  Adafruit Ultimate GPS FeatherWing (https://www.adafruit.com/product/3133)
4.  Adafruit FeatherWing OLED - 128x32 OLED (https://www.adafruit.com/product/2900)
5.  Lithium Ion Polymer Battery - 3.7V 400mAh (https://www.adafruit.com/product/3898)
6.  SMA to uFL/u.FL/IPX/IPEX RF Adapter Cable (https://www.adafruit.com/product/851)
7.  Right-angle Mini GSM/Cellular Quad-Band Antenna - 2dBi SMA Plug (https://www.adafruit.com/product/1858)**
8.  (2) 3V Lithium Coin Cell Battery - CR1220

Currently, the NTP server is in a stacked configuration (top-down):

-  |--LED--|
-  |--GPS--|
-  |--ESP--|
-  |--RTC--|

You could also use a feather breadboard, but the stacked setup has less soldering requirements.  We will eventually upload a .STL file for a printable box that holds all the hardware components.  

**We have been using the right-angle mini gsm/cellular antenna, but we would recommend to try out adafruit's external active antenna (https://www.adafruit.com/product/960) - 10 watt draw for 5 meters of cable length + magnetic antenna seems like it would be better suited for the project.  We have not tested this other antenna.  In theory, it should be a substantial benefit.



---SOFTWARE---

Before we begin, we would like to say we are not C++ developers by trade.  My wife and I do work within the technology space together, but this was my first arduino projects and we learned quite a bit along the way.  If you see improvements that can be made, feel free to share for others if you wish!

Let's break this down into two sections - first on getting the code prepped to install.  Second, some specific parts of this that my wife and I worked through.


---WORKSPACE

Setup the Arduino IDE documented (https://www.arduino.cc/en/software)(https://www.arduino.cc/en/Guide).  Arduino IDE used for this build was 1.8.16.

Confirm and/or install the following libraries within the IDE:
- Wifi by Adafruit (minimum 1.2.7)
- DS3231 by Andrew Wickert (minimum 1.0.7)
- Adafruit GPS Library by Adafruit (minimum 1.5.4)
- RTClib by Adafruit (minimum 1.14.1)
- Time by Paul Stoffregen (minimum 1.6.1)
- TinyGPSPlus by Mikal Hart (minimum 13.0.0)
- Adafruit SSD1306 by Adafruit (minimum 2.4.7)
- Adafruit GFX Library (minimum 1.10.12)
- ArduinoJson by Benoit Blanchon (minimum 6.18.4)
- CRC32 by Christopher Baker (minimum 2.0.0)

Install ESP32 Sketch Uploader (https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/)
- Download the latest arduino-esp32fs-plugin (https://github.com/me-no-dev/arduino-esp32fs-plugin/releases/) and unzip.
- Go to the Arduino IDE directory and opn the tools folder
- Move the unzipped release into the tools folder
- Restart Arduino IDE and it should be under the tools section.

Install the CP2104 USB Driver (https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)

Install the Board in Arduino IDE
- Go to Tools > Boards Manager
- Install ESP32 by Espressif Systems
- Set your Arduino IDE to the correct board.  Under tools > ESP32 Arduino >  "Adafruit ESP32 Feather"

---SETUP---

Connect just the ESP32 board to the computer.  Select the correct COM port that connects to the ESP32 board.

Open Serial Montior and set the baud rate to 921600.

Hard reset the ESP32 by holding down the reset button for at least 5 seconds.  

Under tools, click "ESP32 Sketch Data Upload".  This should initiate the upload of content within the "data" directory into the ESP32's SPIFFS

Click Upload button (right arrow) to compile and upload the software.  Expect some warnings from the compiler.

The board should reboot and you should see it initiate the Wifi configuration and broadcast its own SSID.  Connect to that SSID with a device and configure the wifi settings.  Once restarted, the board should automatically connect to the network.

Install the CR1220 batteries to GPS and DS3231 modules

Disconnect the ESP32 board from the computer and any external power source (i.e. battery) and connect all remaining components.

Reconnect the ESP32 board to the computer and you should see that it finds the RTC and sets it to a time that is likely not correct (initial time is of the compiliation of the software)

Disconnect the board from the computer again and connect the antenna to the gps module.  The wire's connection is finnicky, so support it in some manner.

Connect the battery to the device and set it outside on a clear day and about 5 - 10 feet away from any buildings.  Leave the device for about 3 - 5 minutes.

You should be able to press the buttons on the screen to cycle to the GPS information.  This will tell you if a lock has been found.

After a lock has been found, you can connect to the webpage on the LAN to see the advertised time that is as expected and it should be ready to service NTP requests on the LAN.

---CHANGES---

For those that want to see more specifics we've worked through, keep reading, otherwise, skip this.

The initial code we got together from Elektor and other sources did not match the hardware that we wanted to use.  ESP8266 is similar to ESP32, but not the same.  Additionally, the code from Elektor is relatively old and some functionality can be written a bit more simply as methods/libraries were merged into the Arduino base (e.g. Ticker librar).

Let's cover this chronologically as we worked through them:

Changed:

	uint32_t RTC_ReadUnixTimeStamp(bool* delayed_result){
	  DateTime now = 0;

To:

	uint32_t RTC_ReadUnixTimeStamp(bool* delayed_result){
	  DateTime now = (0,0,0,0,0,0);

Reason:

Wouldn't compile otherwise.  Initialize a new instance of the DateTime structure to zeroed year, month, day, hour, minute, and second.
	
Removed:

	 xTaskCreatePinnedToCore(
	   Display_Task,
	   "Display_Task",
	   10000,
	   NULL,
	   1,
	   NULL,
	   1);

Reason:

This caused issues with DS3231 from being connected to and read by the ESP32 board.  Additionally, we do not plan on using their methodology for displaying data on screen.
	
Changed:

  	/* We reassign the I2C Pins to 4 and 5 with 100kHz */
	Wire.begin(5,4,100000);

  	/* This will check if the RTC is on the I2C */
  	Wire.beginTransmission(0x68);
  	if(Wire.endTransmission() == 0 ){

To:

  	if(! rtc.begin()){
  
Reason:

We do not need to call a hardware serial to reach the DS3231 in this manner.  The current RTC library allws one to just call it with rtc.begin().
	
Removed:

	/* We setup the PPS Pin as interrupt source */
	pinMode(interruptPin, INPUT_PULLUP);
  	attachInterrupt(digitalPinToInterrupt(interruptPin), handlePPSInterrupt, RISING);
  
Reason:

This is unnecessary, even the PPS configuration in general i think is unnecessary.  RTC library + Adafruit board design handles the PPS interrupt for you if you need it.

Changed:

	int16_t Data = hws.read();
	  
To:

	char Data = GPSSerial.read();
	
Reason:

Should be declared as a char because that's what the GPS device is giving us.
	
Chnaged:

	//U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled_left(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
	//U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled_right(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
	
	/* As we use a pointer to the oled we need to make sure it's the same type as out displays */
	U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled_left(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 4, /* data=*/ 5);
	U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled_right(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 4, /* data=*/ 5);
	U8G2_SSD1306_128X64_NONAME_F_HW_I2C* oled_ptr=NULL;

To:

	Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

Reason:

Using the Adafruit GFX library greatly simplifies are code.  Just call in via the SSD1306 library.  Note that if you want to use the 128x64 OLED, this library and this call needs to change.
	

Added:

Basically anything with buttons and OLED display was added by us.  Something of note, I had a heck of a time wrapping my mind around buttons and arduino, but following the examples for the Adafruit display library made things easier to get straight.


---RECOGNITIONS---

Initial starting point was firwmare found at https://www.elektormagazine.com/labs/mini-ntp-server-with-gps.

Big thank you to my wife to getting me started and doing the soldering and figuring out a number of software issues between our intial starting point above and our hardware and arduino IDE.  Definitely my better half :)
