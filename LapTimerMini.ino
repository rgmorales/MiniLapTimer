#include <TinyGPS++.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"


// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

SSD1306AsciiAvrI2c oled;


#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D" 

// The TinyGPS++ object
TinyGPSPlus gps;

#define serial1 Serial1

//if we have an active session going on, we must do the laps tracking
bool sessionActive = false;

unsigned long sessionTime = 0;
unsigned long bestSessionTime = 0;

unsigned long currentLap = 0;
unsigned long bestLap = 0;

//previous position measured
float previousLat = 0.0;
float previousLng = 0.0;
//From the track, the finishing line
float finishLineLat1 = 33.044556;
float finishiLineLon1 = -96.682413;
float finishLineLat2 = 33.044540;
float finishLineLon2 = -96.682239;

unsigned long lapComplete = 0;



void setup()
{

	while (!Serial);
	while (!Serial1);

	Serial.begin(57600);	

	oled.begin(&Adafruit128x64, I2C_ADDRESS);
	oled.setFont(Adafruit5x7);
	oled.setFont(Stang5x7);	

	initializeGPS();

	waitForFix();
	
	oled.clear();
	

}

void loop()
{
	
	while (serial1.available() > 0) {
		//the GPS is configured to 10Hz, this if will happen 10 times per second
		if (gps.encode(serial1.read())) {

			//the GPS is configured to 10Hz, this if will happen 10 times per second

			if ((previousLat != gps.location.lat()) || (previousLng != gps.location.lng())) {
				//new lap, should log the current lap, check for best lap, and give the lap timer 5 seconds to move to next position
				if (segmentsIntersect(gps.location.lat(), gps.location.lng(), previousLat, previousLng, finishLineLat1, finishiLineLon1, finishLineLat2, finishLineLon2)) {			

					//Session starting, this is the first lap
					if (!sessionActive) {
						sessionActive = true;
						sessionTime = millis();
					}
					else {						
						currentLap += 1;
						//this is the best or first lap
						if ((bestSessionTime > millis() - sessionTime) || (bestSessionTime==0)) {//test for session time and also for the very first lap, when bestSesstionTime is 0
							bestSessionTime = millis() - sessionTime;
							bestLap = currentLap;
						}						
					}
					//reset the sessionTime
					sessionTime = millis();
				}

			}
				
				

			previousLat = gps.location.lat();
			previousLng = gps.location.lng();

			if (gps.speed.isUpdated()) {
				if (sessionActive) updateScreen((int)gps.satellites.value(), gps.speed.mph(), millis() - sessionTime, bestSessionTime, bestLap);
				else updateScreen((int)gps.satellites.value(), gps.speed.mph(), 0, 0, 0);
			}				
		}
	}
			
}

void initializeGPS() {
	oled.clear();
	oled.setContrast(255);
	oled.println(F("Initializing GPS..."));
	oled.set2X();
	oled.println(F(""));
	oled.print(F("*"));
	// If GPS has the default settings, we need to change the BAUD rate to 57600
	serial1.begin(9600);
	serial1.println(F(PMTK_SET_BAUD_57600));
	delay(500);//wait for the command to take effect  
			   //we need to close and open the GPS with the new baud rate
	serial1.end();
	delay(500);
	serial1.begin(57600);
	delay(500);
	oled.print(F("*"));
	//Turn on RMC (recommended minimum) and GGA (fix data) including altitude
	serial1.println(F(PMTK_SET_NMEA_OUTPUT_RMCGGA));
	delay(500);
	// uncomment this line to turn on only the "minimum recommended" data
	// Set the update rate
	oled.print(F("*"));
	serial1.println(F(PMTK_SET_NMEA_UPDATE_10HZ));   // 10 Hz update rate
	delay(500);
	// Request updates on antenna status, comment out to keep quiet
	oled.print(F("*"));
	serial1.println(F(PGCMD_NOANTENNA));//No antena
	delay(500);
	oled.print(F("*"));
	delay(2000);
}

void updateScreen(uint8_t numSats,double speed,unsigned long currentSession, unsigned long bestSession, unsigned long bestLap) {
	if (numSats < 3){
		waitForFix();
		return;
	}
	oled.home();
	oled.set1X();
	oled.println("Sats:" + String((int)numSats) + " - " + String(speed,0) + " mph     ");	
	oled.println((((millis() / 100) % 10) % 2==0)?"       ***          ":"                  ");
	oled.println("Best:" + String(bestLap));	
	oled.set2X();
	oled.println(formatSessionTime(bestSession) +"         ");
	oled.set1X();	
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
	oled.clear();			
	// This sketch displays information every time a new sentence is correctly encoded.
	do{	
		while (serial1.available() > 0) 			
			if (gps.encode(serial1.read())) {
				oled.home();
				oled.set1X();
				oled.println(F("Waiting for Sats Fix."));
				oled.println(F(""));
				oled.println(F("3 Sats needed."));
				oled.println(F("Current Sats:"));
				oled.println(F(""));
				oled.set2X();
				oled.print("    " + String((int)gps.satellites.value()));
				oled.set1X();
				oled.set1X();
				oled.println((((millis() / 100) % 10) % 2 == 0) ? "  ***          " : "                  ");
			}
	} while ((int)gps.satellites.value() < 3);
	delay(3000);
	oled.clear();
}


