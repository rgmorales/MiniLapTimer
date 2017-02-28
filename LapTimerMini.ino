#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <TinyGPS++.h>
#include <Time.h>
#include <SoftwareSerial.h>

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

SSD1306AsciiAvrI2c oled;

#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" 

//pins used by software serial
static const int RXPin = 4, TXPin = 3;

// The TinyGPS++ object
TinyGPSPlus gps;


// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//if we have an active session going on, we must do the laps tracking
bool sessionActive = false;

unsigned long sessionTime = 0;
unsigned long bestSessionTime = 0;

//previous position measured
double previousLat = 0.0;
double previousLon = 0.0;
//From the track, the finishing line
double finishLineLat1 = 33.044556; 
double finishiLineLon1 = -96.682413;
double finishLineLat2 = 33.044540;
double finishLineLon2 = -96.682239;

unsigned long lapComplete = 0;

void setup()
{
	Serial.begin(115200);	

	oled.begin(&Adafruit128x64, I2C_ADDRESS);
	oled.setFont(Adafruit5x7);
	//oled.setFont(Stang5x7);	

	initializeGPS();

	waitForFix();
	
	oled.clear();
	updateScreen(0, 0, 0, 0);

}

void loop()
{
	// This sketch displays information every time a new sentence is correctly encoded.
	while (ss.available() > 0) {
		//the GPS is configured to 10Hz, this if will happen 10 times per second
		if (gps.encode(ss.read())) {			

			if ((previousLat != gps.location.lat()) || (previousLon != gps.location.lng())) {
				//new lap, should log the current lap, check for best lap, and give the lap timer 5 seconds to move to next position
				if (segmentsIntersect(gps.location.lat(), gps.location.lng(), previousLat, previousLon, finishLineLat1, finishiLineLon1, finishLineLat2, finishLineLon2) && (lapComplete > 50)) {					
					//reset lap complete, for the next lap
					lapComplete = 0;
					
					if (!sessionActive)  sessionActive = true;

					if (bestSessionTime == 0) bestSessionTime = millis() - sessionTime;

					if (bestSessionTime > millis() - sessionTime) bestSessionTime = millis() - sessionTime;
					
					//reset the sessionTime
					sessionTime = millis();
				}
			}

			previousLat = gps.location.lat();
			previousLat = gps.location.lng();

			if (sessionActive) updateScreen(gps.satellites.value(), gps.speed.mph(), millis()-sessionTime, bestSessionTime);
			else updateScreen(gps.satellites.value(), gps.speed.mph(), 0,0);

			//used to control 5 seconds interval every time a lap is completed
			lapComplete += 1;
		}
	}
			
}



void initializeGPS() {
	oled.clear();
	oled.setContrast(255);
	oled.println(F("Initializing GPS..."));
	oled.println(F(""));
	oled.println(F("Config baud rate"));
	// If GPS has the default settings, we need to change the BAUD rate to 57600
	ss.begin(9600);
	ss.println(F(PMTK_SET_BAUD_57600));
	delay(500);//wait for the command to take effect  
	//we need to close and open the GPS with the new baud rate
	ss.end();
	delay(500);
	ss.begin(57600);
	delay(500);
	oled.println(F("Config NMEA"));
	//Turn on RMC (recommended minimum) and GGA (fix data) including altitude
	ss.println(F(PMTK_SET_NMEA_OUTPUT_RMCGGA));
	delay(500);
	// uncomment this line to turn on only the "minimum recommended" data
	// Set the update rate
	oled.println(F("Config 10hz"));
	ss.println(F(PMTK_SET_NMEA_UPDATE_10HZ));   // 10 Hz update rate
	delay(500);
	// Request updates on antenna status, comment out to keep quiet
	oled.println(F("Config no antenna "));
	ss.println(F(PGCMD_NOANTENNA));//No antena
	delay(500);	
	oled.println(F("Config finished!!!"));
	delay(2000);
}

void updateScreen(uint32_t numSats,double speed,unsigned long currentSession, unsigned long bestSession) {
	oled.home();
	oled.set1X();
	if (numSats != 0) oled.println("Sats:"+ String(numSats,0)+" - " +String(speed,0) + " mph     ");
	else waitForFix();
	oled.println();		
	oled.println(F("Best:"));
	//oled.println();
	oled.set2X();
	oled.println(formatSessionTime(bestSession));
	oled.set1X();
	//oled.println();
	oled.println(F("Current:"));
	oled.set2X();
	oled.println(formatSessionTime(currentSession)+ "        ");	
}

//Formats session time in the following format:
// min:seconds:tenth of a second 
// 00:00:0
String formatSessionTime(unsigned long sessionTime) {
	unsigned long minutes = sessionTime / 60000;
	unsigned long seconds = (sessionTime / 1000) - ((sessionTime / 60000) * 60);
	unsigned long tenths = (sessionTime / 100) % 10;
	if (seconds <10) return String(minutes) + ":0" + String(seconds) + ":" + String(tenths);
	else return String(minutes) + ":" + String(seconds) + ":" + String(tenths);
}

//Calculates if finish line was crossed
boolean segmentsIntersect(float lat1, float  lon1, float  lat2, float  lon2, float  finishLineLat1, float  finishLineLon1, float  finishLineLat2, float  finishLineLon2) {
	// does line(p1, p2) intersect line(p3, p4)
	float fDeltaX = lat2 - lat1;
	float fDeltaY = lon2 - lon1;
	float da = finishLineLat2 - finishLineLat1;
	float db = finishLineLon2 - finishLineLon1;
	if ((da * fDeltaY - db * fDeltaX) == 0) {
		//The segments are parallel
		return false;

	}
	float s = (fDeltaX * (finishLineLon1 - lon1) + fDeltaY * (lat1 - finishLineLat1)) / (da * fDeltaY - db * fDeltaX);
	float t = (da * (lon1 - finishLineLon1) + db * (finishLineLat1 - lat1)) / (db * fDeltaX - da * fDeltaY);

	return (s >= 0) && (s <= 1) && (t >= 0) && (t <= 1);
}

void waitForFix() {
	int count = 1;
	oled.clear();			
	// This sketch displays information every time a new sentence is correctly encoded.
	do{
		oled.home();
		oled.set1X();
		oled.println(F("Waiting for Sats Fix."));
		oled.println(F(""));
		oled.println(F("3 Sats needed."));
		oled.println(F("Current Sats:"));
		oled.println(F(""));
		oled.set2X();
		oled.print("    " + String(gps.satellites.value()));
		oled.set1X();
		oled.print(" (" + String(count) + "s)");
		oled.set1X();
		if (gps.charsProcessed()>0)	count += 1;
		smartDelay(1000);
	} while ((gps.satellites.value() < 3)|| (count<3));
	oled.clear();
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
	unsigned long start = millis();
	do
	{
		while (ss.available())
			gps.encode(ss.read());
	} while (millis() - start < ms);
}
