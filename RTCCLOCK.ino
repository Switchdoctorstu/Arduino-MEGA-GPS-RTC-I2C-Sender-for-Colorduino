/*

// Stuarts code to drive Colourduino and TM1638 from GPS and DS1307 RTC
*	This code is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *	
 *	This library is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *	Lesser General Public License for more details.
 
 
// Reads GPS on serial 3
// if GPS invalid:Reads the RTC 
// Displays time on TM1638
// Handles reset of RTC
// Handles button updates to RTC  < needs fixing
// Sends display to array of 4 Colorduino displays on I2C

Gets time from RTC or GPS receiver on Serial3
Sends I2C Character & Raster to Colorduino Receiver

All code attributed to original authors, i've put this together from 
so many sources that i can't track them all.


Currently working on:
OK: make packets fixed length = 16 bytes
OK: stx,type,index,data*11,chk,cr
OK: sending char packets
XX: BST offset
XX: RF handler
XX: Temperature
OK: startup time from rtc 

*/
// Added debug mode to clear serial noise
// Added gps handler
// Added I2C display module handler


//****************************************************
// define the modes that we can work as
#define RTCMODE 0
#define GPSMODE 1
#define NUMMODES 2
#define DEBUGI2C true
#define DEBUGMATRIX true
#define DEBUGGPS false
#define DEBUGREPORT true
#define DEBUGTEMPERATURE true
#define DEBUGTIME true
// GPS Definitions
#define GPSBUFFERLEN 128
#define ARGCOUNT 40
#define MAXI2CDEVICES 20
#define MAGNOADDRESS 0x1e
#define MAXERRORCOUNT 5
#define MAXMOUNTMSGLEN 128
#define GPSPASSTHRU false  // enable pass-thru of GPS data so PC sees GPS

#define FIRSTMODULE 4  // i2c address of 1st module
#define LASTMODULE 7	// i2c address of last module

// some pins
#define INBOXTEMPPIN A0


// Include the right libraries
#include <Wire.h>
#include <Time.h>
#include <DS1307RTC.h>  // RTC
#include <TM1638.h>  // LED Display 
#include <String.h>
#include <Adafruit_BMP085.h>

// RF includes
#include <SPI.h>
#include "nRF24L01.h"  // Radio Libraries
#include "RF24.h" // Radio


// TM1638(byte dataPin, byte clockPin, byte strobePin, boolean activateDisplay = true, byte intensity = 7);
TM1638 dm (5, 6, 7);
tmElements_t tm;   	// storage for time components
time_t time_now;    // storage for local clock

// Declare all the global vars
unsigned long waitcheckTime=0; // timer for time checking
unsigned long waitcheckButtons=0; // timer for buttons
unsigned long intervalcheckTime=1000;
unsigned long intervalcheckButtons=500;
unsigned long reportTime=1000;
unsigned long reportInterval=1000;
unsigned long gapSecond=0;
unsigned long interrupts=0;			// interrupt counter
unsigned long lasttime=0;
unsigned long thistime=0;	// Time handles
unsigned long timeNow;
unsigned long RTCNextUpdateTime=0;
unsigned long RTCUpdateInterval=60000; // 60 seconds
unsigned long displayNextUpdateTime=0;
unsigned long displayUpdateInterval=5000;
boolean dots=0;
boolean moduleOff=0;
// setup mode 
int currentmode=RTCMODE;
int displaymode=1; // major display mode 0=char 1=raster
boolean rtcokFlag=false;
boolean gpstimeokFlag=false;
// array of month names
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
// global GPS vars

int i2cdevicecount=0;  // count of attached I2C devices
int i2caddresses[MAXI2CDEVICES];
int index=0;
char msgfrommount[MAXMOUNTMSGLEN];
int msgfrommountindex=0;
String gpsstring="                                                                               ";
String frommountstring="";
char gpsmsg[GPSBUFFERLEN];
unsigned int gpsmsgindex=0;
char msgtomount[MAXMOUNTMSGLEN];
int msgtomountindex=0;
char debugging='N';
int trigger=0;
int i2cmagno=0;
int fontheight=7;
int fontwidth=5;
char timearray[6]; // string for current valid time

int inboxtemperature=0;
long temperaturechecktime=0;
long temperaturecheckinterval=5000;

// gps handling
String gpsfixvalid="N";
String gpsfixtime="000000";
String gpsfixdate="000000";
String gpslat="0.000000";
String gpslatort = "N";
String gpslong="0.000000"; // gps longitude
String gpslongort = "E";  // gps longitude compass
String Arg[ARGCOUNT];
String GPShhmmssdss="000000.00";
String GPSdd="01";
String GPSmm="01";
String GPSyy="01";
String GPSzh="00";
String GPSzm="00";

int argcount=0;  // number of arguments


// buffer to hold raster 4 modules *8*8
unsigned char matrix[256][3];
const unsigned char font_5x7[][5] = { 
        { 0x00, 0x00, 0x00, 0x00, 0x00 },               /*   - 0x20 - 32 */
        { 0x00, 0x00, 0x5f, 0x00, 0x00 },               /* ! - 0x21 - 33 */ 
        { 0x00, 0x07, 0x00, 0x07, 0x00 },               /* " - 0x22 - 34 */ 
        { 0x14, 0x7f, 0x14, 0x7f, 0x14 },               /* # - 0x23 - 35 */
        { 0x24, 0x2a, 0x7f, 0x2a, 0x12 },               /* $ - 0x24 - 36 */
        { 0x23, 0x13, 0x08, 0x64, 0x62 },               /* % - 0x25 - 37 */
        { 0x36, 0x49, 0x55, 0x22, 0x50 },               /* & - 0x26 - 38 */
        { 0x00, 0x05, 0x03, 0x00, 0x00 },               /* ' - 0x27 - 39 */
        { 0x00, 0x1c, 0x22, 0x41, 0x00 },               /* ( - 0x28 - 40 */
        { 0x00, 0x41, 0x22, 0x1c, 0x00 },               /* ) - 0x29 - 41 */
        { 0x14, 0x08, 0x3e, 0x08, 0x14 },               /* * - 0x2a - 42 */
        { 0x08, 0x08, 0x3e, 0x08, 0x08 },               /* + - 0x2b - 43 */
        { 0x00, 0x50, 0x30, 0x00, 0x00 },               /* , - 0x2c - 44 */
        { 0x08, 0x08, 0x08, 0x08, 0x08 },               /* - - 0x2d - 45 */
        { 0x00, 0x60, 0x60, 0x00, 0x00 },               /* . - 0x2e - 46 */
        { 0x20, 0x10, 0x08, 0x04, 0x02 },               /* / - 0x2f - 47 */
        { 0x3e, 0x51, 0x49, 0x45, 0x3e },               /* 0 - 0x30 - 48 */
        { 0x00, 0x42, 0x7f, 0x40, 0x00 },               /* 1 - 0x31 - 49 */
        { 0x42, 0x61, 0x51, 0x49, 0x46 },               /* 2 - 0x32 - 50 */
        { 0x21, 0x41, 0x45, 0x4b, 0x31 },               /* 3 - 0x33 - 51 */
        { 0x18, 0x14, 0x12, 0x7f, 0x10 },               /* 4 - 0x34 - 52 */
        { 0x27, 0x45, 0x45, 0x45, 0x39 },               /* 5 - 0x35 - 53 */
        { 0x3c, 0x4a, 0x49, 0x49, 0x30 },               /* 6 - 0x36 - 54 */
        { 0x01, 0x71, 0x09, 0x05, 0x03 },               /* 7 - 0x37 - 55 */
        { 0x36, 0x49, 0x49, 0x49, 0x36 },               /* 8 - 0x38 - 56 */
        { 0x06, 0x49, 0x49, 0x29, 0x1e },               /* 9 - 0x39 - 57 */
        { 0x00, 0x36, 0x36, 0x00, 0x00 },               /* : - 0x3a - 58 */
        { 0x00, 0x56, 0x36, 0x00, 0x00 },               /* ; - 0x3b - 59 */
        { 0x08, 0x14, 0x22, 0x41, 0x00 },               /* < - 0x3c - 60 */
        { 0x14, 0x14, 0x14, 0x14, 0x14 },               /* = - 0x3d - 61 */
        { 0x00, 0x41, 0x22, 0x14, 0x08 },               /* > - 0x3e - 62 */
        { 0x02, 0x01, 0x51, 0x09, 0x06 },               /* ? - 0x3f - 63 */
        { 0x32, 0x49, 0x79, 0x41, 0x3e },               /* @ - 0x40 - 64 */
        { 0x7e, 0x11, 0x11, 0x11, 0x7e },               /* A - 0x41 - 65 */
        { 0x7f, 0x49, 0x49, 0x49, 0x36 },               /* B - 0x42 - 66 */
        { 0x3e, 0x41, 0x41, 0x41, 0x22 },               /* C - 0x43 - 67 */
        { 0x7f, 0x41, 0x41, 0x22, 0x1c },               /* D - 0x44 - 68 */
        { 0x7f, 0x49, 0x49, 0x49, 0x41 },               /* E - 0x45 - 69 */
        { 0x7f, 0x09, 0x09, 0x09, 0x01 },               /* F - 0x46 - 70 */
        { 0x3e, 0x41, 0x49, 0x49, 0x7a },               /* G - 0x47 - 71 */
        { 0x7f, 0x08, 0x08, 0x08, 0x7f },               /* H - 0x48 - 72 */
        { 0x00, 0x41, 0x7f, 0x41, 0x00 },               /* I - 0x49 - 73 */
        { 0x20, 0x40, 0x41, 0x3f, 0x01 },               /* J - 0x4a - 74 */
        { 0x7f, 0x08, 0x14, 0x22, 0x41 },               /* K - 0x4b - 75 */
        { 0x7f, 0x40, 0x40, 0x40, 0x40 },               /* L - 0x4c - 76 */
        { 0x7f, 0x02, 0x0c, 0x02, 0x7f },               /* M - 0x4d - 77 */
        { 0x7f, 0x04, 0x08, 0x10, 0x7f },               /* N - 0x4e - 78 */
        { 0x3e, 0x41, 0x41, 0x41, 0x3e },               /* O - 0x4f - 79 */
        { 0x7f, 0x09, 0x09, 0x09, 0x06 },               /* P - 0x50 - 80 */
        { 0x3e, 0x41, 0x51, 0x21, 0x5e },               /* Q - 0x51 - 81 */
        { 0x7f, 0x09, 0x19, 0x29, 0x46 },               /* R - 0x52 - 82 */
        { 0x46, 0x49, 0x49, 0x49, 0x31 },               /* S - 0x53 - 83 */
        { 0x01, 0x01, 0x7f, 0x01, 0x01 },               /* T - 0x54 - 84 */
        { 0x3f, 0x40, 0x40, 0x40, 0x3f },               /* U - 0x55 - 85 */
        { 0x1f, 0x20, 0x40, 0x20, 0x1f },               /* V - 0x56 - 86 */
        { 0x3f, 0x40, 0x38, 0x40, 0x3f },               /* W - 0x57 - 87 */
        { 0x63, 0x14, 0x08, 0x14, 0x63 },               /* X - 0x58 - 88 */
        { 0x07, 0x08, 0x70, 0x08, 0x07 },               /* Y - 0x59 - 89 */
        { 0x61, 0x51, 0x49, 0x45, 0x43 },               /* Z - 0x5a - 90 */
        { 0x00, 0x7f, 0x41, 0x41, 0x00 },               /* [ - 0x5b - 91 */
        { 0x02, 0x04, 0x08, 0x10, 0x20 },               /* \ - 0x5c - 92 */
        { 0x00, 0x41, 0x41, 0x7f, 0x00 },               /* ] - 0x5d - 93 */
        { 0x04, 0x02, 0x01, 0x02, 0x04 },               /* ^ - 0x5e - 94 */
        { 0x40, 0x40, 0x40, 0x40, 0x40 },               /* _ - 0x5f - 95 */
        { 0x00, 0x01, 0x02, 0x04, 0x00 },               /* ` - 0x60 - 96 */
        { 0x20, 0x54, 0x54, 0x54, 0x78 },               /* a - 0x61 - 97 */
        { 0x7f, 0x48, 0x44, 0x44, 0x38 },               /* b - 0x62 - 98 */
        { 0x38, 0x44, 0x44, 0x44, 0x20 },               /* c - 0x63 - 99 */
        { 0x38, 0x44, 0x44, 0x48, 0x7f },               /* d - 0x64 - 100 */
        { 0x38, 0x54, 0x54, 0x54, 0x18 },               /* e - 0x65 - 101 */
        { 0x08, 0x7e, 0x09, 0x01, 0x02 },               /* f - 0x66 - 102 */
        { 0x38, 0x44, 0x44, 0x54, 0x34 },               /* g - 0x67 - 103 */
        { 0x7f, 0x08, 0x04, 0x04, 0x78 },               /* h - 0x68 - 104 */
        { 0x00, 0x44, 0x7d, 0x40, 0x00 },               /* i - 0x69 - 105 */
        { 0x20, 0x40, 0x44, 0x3d, 0x00 },               /* j - 0x6a - 106 */
        { 0x7f, 0x10, 0x28, 0x44, 0x00 },               /* k - 0x6b - 107 */
        { 0x00, 0x41, 0x7f, 0x40, 0x00 },               /* l - 0x6c - 108 */
        { 0x7c, 0x04, 0x18, 0x04, 0x78 },               /* m - 0x6d - 109 */
        { 0x7c, 0x08, 0x04, 0x04, 0x78 },               /* n - 0x6e - 110 */
        { 0x38, 0x44, 0x44, 0x44, 0x38 },               /* o - 0x6f - 111 */
        { 0x7c, 0x14, 0x14, 0x14, 0x08 },               /* p - 0x70 - 112 */
        { 0x08, 0x14, 0x14, 0x18, 0x7c },               /* q - 0x71 - 113 */
        { 0x7c, 0x08, 0x04, 0x04, 0x08 },               /* r - 0x72 - 114 */
        { 0x48, 0x54, 0x54, 0x54, 0x20 },               /* s - 0x73 - 115 */
        { 0x04, 0x3f, 0x44, 0x40, 0x20 },               /* t - 0x74 - 116 */
        { 0x3c, 0x40, 0x40, 0x20, 0x7c },               /* u - 0x75 - 117 */
        { 0x1c, 0x20, 0x40, 0x20, 0x1c },               /* v - 0x76 - 118 */
        { 0x3c, 0x40, 0x30, 0x40, 0x3c },               /* w - 0x77 - 119 */
        { 0x44, 0x28, 0x10, 0x28, 0x44 },               /* x - 0x78 - 120 */
        { 0x0c, 0x50, 0x50, 0x50, 0x3c },               /* y - 0x79 - 121 */
        { 0x44, 0x64, 0x54, 0x4c, 0x44 },               /* z - 0x7a - 122 */
        { 0x00, 0x08, 0x36, 0x41, 0x00 },               /* { - 0x7b - 123 */
        { 0x00, 0x00, 0x7f, 0x00, 0x00 },               /* | - 0x7c - 124 */
        { 0x00, 0x41, 0x36, 0x08, 0x00 },               /* } - 0x7d - 125 */
        { 0x10, 0x08, 0x08, 0x10, 0x08 },               /* ~ - 0x7e - 126 */
};
String marquee="Initialising";
void handleInterrupt() {
	interrupts++;
}
void checktemperature(){
	if(timeNow>=temperaturechecktime){
		temperaturechecktime+=temperaturecheckinterval;
		inboxtemperature=analogRead(INBOXTEMPPIN);
		if(DEBUGTEMPERATURE){
			Serial.print("Temperature V:");
			Serial.println(inboxtemperature,DEC);
		}
	}
}
void gettime(){
	timeNow=millis(); // timer for local loop triggers
	
	String rtctimestring="000000";
  if (RTC.read(tm)) {
		rtcokFlag=true;
	}	
	else 
	{
		rtcokFlag=false;
    	if (RTC.chipPresent()) {
		  Serial.println("The DS1307 RTC is stopped.  Attempting to initialise");
		  setupRTC();
		  
		} else {
		Serial.println("DS1307 read error!  Please check the circuitry.");
		Serial.println();
		}
  }
  // get hrs and mins in usable format
	if(gpsfixvalid="A"){  // check for valid time from GPS
		gpsfixtime.toCharArray(timearray,6);
		currentmode=GPSMODE;
	}	
	else{
		// get rtc time instead
		rtctimestring=getrtctime();
		rtctimestring.toCharArray(timearray,6);
			currentmode=RTCMODE;
	}
	time_now=now(); // set the current time stamp
}

String twochars(int number){
	if (number >= 0 && number < 10)	{
		return "0"+String(number);
	}
	else	{
	return String(number);
	}
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

void setup() {
	Serial.begin(19200);
	Serial3.begin(9600); // GPS on Mega
	// while (!Serial) ; // wait for serial
	delay(200);
	if(debugging=='Y'){
		Serial.println("Stuart's LED RTC - GPS and DS1307RTC V0.1");
		Serial.println("-----------------------------------------");
	}
	// get time
	time_now=now();
	
	// Start I2C
	Wire.begin();
	
	// setup the frequency counter interrupt
	// attachInterrupt(0, handleInterrupt, FALLING);
	// reset interval timers
	unsigned long t=millis();
	
	waitcheckTime = t + intervalcheckTime;  
	waitcheckButtons = t + intervalcheckButtons;
	RTCNextUpdateTime=t + RTCUpdateInterval;
	displayNextUpdateTime=t+displayUpdateInterval;
	scani2c();
}

void loop(){

// main code here - runs repeatedly
		
		gettime();   // read the time from the RTC or GPS 
		checkDisplayTimer();
		checkButtons();
		if(DEBUGREPORT)checkReport(); // see if we need to report
		// checki2c(); // Check for i2c communications - magno / temp
	  
		// check for GPS data
		if(Serial3.available()>0)
			{
			GetGPSData();
		}
		checktimesync(); // keep RTC in sync with GPS if available
		checktemperature();
	}

void clearMatrix(){
	for(int p=0;p<256;p++){
		for(int c=0;c<4;c++){
		matrix[p][c]=0;
		}
	}
}	

void buildMatrix(){
	// finally, if we build a matrix display here it should be sent to the display
	// if we set displaymode to 0x01
	// matrix =ColourRGB[256]
	clearMatrix();  // clear the dot matrix buffer
	
	
	//int character = (millis()/displayUpdateInterval % 64); // rotate character
// start by just copying time chars to buffer
int intensity = 128;
char backred=0x00;
char backgreen=0x04;
char backblue=0x04;
char red=0xa0;
char green = 0x00;
char blue=0xFF;
char fontbyte;
char mybyte;
unsigned int z,m,l,w,y;
int x;
	// time should be in timearray
	if(gpsfixvalid=="V"){
		backgreen=0x00;
	backblue=0x00;
		backred=0x04;
		
	}
	if(gpsfixvalid=="A"){
		backgreen=inboxtemperature&0x3f;
		backred=inboxtemperature&0x1f;
		backblue=0x00;
	}
// build the raster
	for(m=0;m<4;m++){  // Module Loop
		for(x=7;x>-1;x--){
			if(x<fontwidth){
				fontbyte=font_5x7[timearray[m]-32][x];
			}
			else {fontbyte=0;
			}
			
			if(DEBUGMATRIX){
				Serial.print("M:");
				Serial.print(m,DEC);
			}
			for(y=0;y<8;y++){ // cycle thru bits
			z=(m*6)+(y*32)+x;
				if(fontbyte&128>>y){ // bit is set
					if(DEBUGMATRIX)Serial.print("1");
					matrix[z][0]=red;
					matrix[z][1]=green;
					matrix[z][2]=blue;
				} 
				else { // bit is not set
					if(DEBUGMATRIX)Serial.print("0");
					matrix[z][0]=backred;
					matrix[z][1]=backgreen;
					matrix[z][2]=backblue;
				
				}	
			}
			if(DEBUGMATRIX)Serial.println();
		}
	}
}

void sendToMatrix(){  	// draw something out to LED grid displays
	char c;
	int returncode=0;
	// get hrs and mins in usable format
	if(gpsfixvalid=="A"){  // check for valid time from GPS
		//reload tm and write it to the RTC
		long int t=gpsfixtime.toInt();	
		gpsfixtime.toCharArray(timearray,6);
	}	
	
	if(displaymode==0){
		
		// wants 
		//  '#' 
		// 		type = 0x01
		//    	character in ASCII
		// red,blue,green
		// ... padding to make whole packet 16 bytes
		// etx = 0x0d
		// cycle thru modules
		int red=128; int blue=128; int green=128;
		for(int a=FIRSTMODULE;a<LASTMODULE+1;a++){
			Wire.beginTransmission(a); // start talking to Module
			Wire.write(0x23);
			Wire.write(0x01);
			c=timearray[a-FIRSTMODULE];
			Wire.write(c); // send the character
			Wire.write(red);
			Wire.write(green);
			Wire.write(blue);
			// pad out to 16 bytes
			
			for(int n=0;n<9;n++){
				Wire.write(0x00);
			}
			Wire.write(0x0d);
			returncode=Wire.endTransmission();
			delay(16);
			if(DEBUGI2C){
				Serial.println("sent "+ String(c) + " To Address:" +String(a)+" Return:"+String(returncode));
			}
		}
	}
	
	if(displaymode==1){
		// we're in raster mode so send some chars
		// protocol goes:
		//  0 	stx
		//  1 	0x02   = type 2 = raster
		//  2 	0xLC = Line(0-7) Colour(0-2) 0=red 1=blue 2=green
		// 	3-10 	byte x 8    intensity data for that line
		// 	11	padding to 16 bytes
		// 	15 	etx = 0x0d
			
		for(int colour=0;colour<3;colour++){
			for(int line=0;line<8;line++){
				for(int m=FIRSTMODULE;m<LASTMODULE+1;m++){	
					// red first
					Wire.beginTransmission(m);
					Wire.write(0x23);  // stx= '#'
					Wire.write(0x02); // type = 0x02 = raster
					char t=line*16+colour;
					Wire.write(t);
					if(DEBUGMATRIX) Serial.print("A:"+String(m)+" ");
					int z=((m-FIRSTMODULE)*8)+(line*32);
					for(int p=z;p<(z+8);p++){
						Wire.write(matrix[p][colour]);
						if(DEBUGMATRIX) {
							Serial.print(matrix[p][colour],HEX);
							Serial.print(",");
						}
					}
					if(DEBUGMATRIX) Serial.println();
					// Pad out to 16 bytes
					Wire.write(0x00);
					Wire.write(0x00);
					Wire.write(0x00);
					Wire.write(0x00); // should be checksum.. later
					Wire.write(0x0d); // send the ETX
					returncode=Wire.endTransmission();
					if(DEBUGI2C){Serial.println(returncode,HEX);}
					delay(16);
				}
			}
		}
		// send flip to modules
			for(int m=FIRSTMODULE;m<LASTMODULE+1;m++){	
				sendflip(m);
			}
	}
}
	
void sendflip(int m){
	Wire.beginTransmission(m);
					Wire.write(0x23);  // stx= '#'
					Wire.write(0x03); // type = 0x03 = flip buffer
					for(int n=0;n<13;n++) Wire.write(0x00); // fill buffer
					// Wire.beginTransmission(m);
					Wire.write(0x0d);  // etx= CR
					Wire.endTransmission();
					delay(10);
					
}
	
void sendclear(int m, int r,int g, int b){ // send clear screen to matrix
// clear screen to 
			// protocol goes:
			//  0 	stx
			//  1 	0x04   = type 4 = Fill
			//  2 	red
			// 	3	green
			//	4  	blue
			// etx in 15
					Wire.beginTransmission(m);
					Wire.write(0x23);  // stx= '#'
					Wire.write(0x04); // type = 0x04 = fill screen
					Wire.write(r);
					Wire.write(g);
					Wire.write(b);
					for(int n=0;n<10;n++) Wire.write(0x00); // fill buffer
					Wire.write(0x0d);  // etx= CR
					Wire.endTransmission();
					delay(10);
}	

void checktimesync(){
	if(timeNow>RTCNextUpdateTime){
		RTCNextUpdateTime=timeNow+RTCUpdateInterval;
		// Time precedence
		// GPS if Valid
		// RTC if valid
		// system clock
		if(gpsfixvalid="A"){ //reload tm and write it to the RTC
			long int t=gpsfixtime.toInt();
			Serial.println("GPS Fix Time" + gpsfixtime  + "  ToInt:" + String(t));
			tm.Hour=t/10000;
			t=t % 10000; // mod 10000
			tm.Minute=t/100;
			t=t % 100;
			tm.Second=t;
			t=gpsfixdate.toInt();
			tm.Day = t/10000;
			t=t % 10000;
			tm.Month = t/100;
			t=t % 100;
			t=t+2000-1970;
			tm.Year =	t;
			// now write it to RTC
			if(DEBUGTIME){
				Serial.println("Writing:"+ String(tm.Hour)+":"+String(tm.Minute) +":"+String(tm.Second)+":"+String(tm.Day)+":"+String(tm.Month)+":"+String(tm.Year));
			}
			if (RTC.write(tm))			{
				Serial.println("RTC reset from GPS");
			}
			else			{
				Serial.println("Error Writing to RTC");		
			}
			//
		}
		// set system time to RTC
		setTime(tm.Hour,tm.Minute,tm.Second,tm.Day,tm.Month,tm.Year-30);
	}
}
	
void checkDisplayTimer(){
  if (timeNow >= waitcheckTime) {
    dots = !dots;
    drawToModule();
    waitcheckTime = timeNow+ intervalcheckTime;
  }
  if(timeNow>=displayNextUpdateTime){
	  displayNextUpdateTime=timeNow+displayUpdateInterval; // reset LED display timer
	  buildmarqee();
	  buildMatrix(); // build the raster to display on the matrix
	  sendToMatrix();
  }
}

void buildmarqee(){
	
	marquee=String(hour(time_now))+":"+ String(minute(time_now)) +":"+ String(second(time_now));  // Print system time
			if(isAM(time_now)) marquee=marquee+" AM ";
			if(isPM(time_now)) marquee=marquee+" PM ";
			marquee=marquee+String(dayShortStr(weekday(time_now)))+" ";
			marquee=marquee+String(day(time_now))+" "+String(monthShortStr(month(time_now)))+" "+String(year(time_now));
}
void checkReport() {
		if( timeNow>= reportTime){ // if we're due
			reportTime=timeNow+reportInterval; // reset interval
			Serial.println("*******************************");
			Serial.println(marquee);
			Serial.println("Onboard Clock:"+String(hour(time_now))+":"+ String(minute(time_now))) +":"+ String(second(time_now));  // Print system time
			if(isAM(time_now)) Serial.println("AM");
			if(isPM(time_now)) Serial.println("PM");
			// now() time in seconds since 1/1/70
			Serial.println(String(day(time_now))+":"+String(month(time_now))+":"+String(year(time_now)));             // The day now (1-31)
			Serial.println("DOTW:"+String(weekday(time_now)));         // Day of the week, Sunday is day 1
 			Serial.println(String(dayStr(weekday(time_now)))+" "+String(day(time_now))+String(monthStr(month(time_now))));
			Serial.println(String( monthShortStr(month(time_now)))+" "+String(dayShortStr(weekday(time_now))));
			Serial.println("Time Status:"+String(timeStatus()));
			if(rtcokFlag=true){
				Serial.print("RTC Ok, Time = ");
				print2digits(tm.Hour);
				Serial.write(':');
				print2digits(tm.Minute);
				Serial.write(':');
				print2digits(tm.Second);
				Serial.print(", Date(D/M/Y) ");
				Serial.print(tm.Day);
				Serial.write('/');
				Serial.print(tm.Month);
				Serial.write('/');
				Serial.print(tmYearToCalendar(tm.Year));
				Serial.println();
			}
			else
			{
				Serial.println("RTC not valid.");
			}
		
			Serial.println("GPS fix status:" + gpsfixvalid);
			if(gpsfixvalid=="A"){
				Serial.println("GPS fix time:" + gpsfixtime + " Date:"+gpsfixdate);
				Serial.print("GPS latitude:" + gpslat + " " + gpslatort);
				Serial.println(" longitude:" + gpslong + " " + gpslongort);  // gps longitude compass
				
			}
			else{
				Serial.println("GPS fix Invalid");
			}			
		}
	}

void checkButtons(){
  if (millis() >= waitcheckButtons) {
    // dm.setLEDs(0);
    byte buttons=dm.getButtons();
    if(buttons>0){
      for (byte i=0;i<8;i++){
        if ((buttons & 1<<i) != 0) {
          Serial.print("Button Event:");
		  Serial.println(i);
		  buttonEvent(i);
          waitcheckButtons +=(intervalcheckButtons*5);
          // drawToModule();
        }
      }
    }
    waitcheckButtons +=intervalcheckButtons;
  }
}

void drawToModule(){
  if (!moduleOff){
    unsigned long elapSecond = round(millis() / 1000);
    unsigned long totalSecond = gapSecond + elapSecond;
    byte pos = 2; // totalSecond % 4;
    if (pos>2) pos=1;
	// get hrs and mins in usable format
	dm.clearDisplay();
	if(gpsfixvalid="A"){  // check for valid time from GPS
		// gpsfixtime.toCharArray(timearray,6);
		dm.setDisplayToString(gpsfixtime,(dots * 80),pos);
	}
	else{	// use rtc time instead
	dm.setDisplayToString(getrtctime(),(dots * 80),pos);
	}
   }
}

void buttonEvent(byte inp){
  dm.setLED( 1,inp);
  switch (inp) {
  case 0: { // inc hours
    if (tm.Hour != 23)
    {  
		tm.Hour++ ;
	}
    else
	{
      tm.Hour=0;
    }
	// now write it to RTC
	if (RTC.write(tm))
	{
		Serial.println("hour set");
	}
	else
	{
		Serial.println("Error Writing to RTC");		
    }
    break;
  }
  case 1:  { // decrement hours
  if (tm.Hour != 0)
    {  
		tm.Hour-- ;
	}
    else
	{
      tm.Hour=23;
    }
	// now write it to RTC
	if (RTC.write(tm)) {
			Serial.println("hour set");
		}
	else
	{
	Serial.println("Error Writing to RTC");		
    }
    break;
  }
  case 2: { // increment minutes
    if (tm.Minute != 59)
    {  
		tm.Minute++ ;
	}
    else
	{
      tm.Minute=0;
    }
	// now write it to RTC
	if (RTC.write(tm)) {
			Serial.println("minute set");
	}
	else
	{
		Serial.println("Error Writing to RTC");		
    }
	break;
  }
  case 3:  { // decrement Minutes
    if (tm.Minute != 0)
    {  
		tm.Minute-- ;
	}
    else
	{
      tm.Minute=59;
    }
	// now write it to RTC
	if (RTC.write(tm)) {
			Serial.println("minute set");
	}
	else
	{
		Serial.println("Error Writing to RTC");		
    }
	break;
  }
  case 4: { // reset seconds
    tm.Second=0;
	if (RTC.write(tm)) {
			Serial.println("seconds reset");
	}
	else
	{
		Serial.println("Error Writing to RTC");		
    }
	
	break;
  }
  case 5:{  // led on/off
    moduleOff = !moduleOff;
    if (moduleOff) dm.clearDisplay();
    break;
  }
  case 6:
  Serial.println("Button 6 Pressed !");
    break;
  case 7:{
	// cycle thru the modes
	Serial.println("Button 7 Pressed !");
	Serial.print("Mode:");
	Serial.println(currentmode);
	currentmode++;
	// send clearscreen
	for(int m=4;m<8;m++){
		sendclear(m,0,0,0x20);
	}
	if(displaymode==0)
	{
		displaymode=1;
	}
	else{
		displaymode=0;
	}
    break;
  }
  }
}

// getrtctime
// returns time as a 6 character string with zero padding on seconds and minutes
String getrtctime()   {
return twochars(tm.Hour)+twochars(tm.Minute)+twochars(tm.Second);
}

void setupRTC() {// reset the RTC
	bool parse=false;
	bool config=false;
	timeNow=now(); // get current system time
	tm.Hour=hour(timeNow);
	tm.Minute=minute(timeNow);
	tm.Second=second(time_now);
	tm.Year=year(timeNow)-30;
  tm.Month=month(timeNow);
	tm.Day=day(timeNow);
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  
  delay(200);
  if(DEBUGTIME){
	  if (config) {
		Serial.print("setupRTC: DS1307 configured Time=");
		Serial.println(String(tm.Hour)+":"+String(tm.Minute)+":"+String(tm.Second));
		Serial.print(", Date=");
		Serial.println(String(tm.Day)+"/"+String(tm.Month)+"/"+String(tm.Year));
	  } else {
		Serial.println("DS1307 Communication Error :-{");
		Serial.println("Please check your circuitry");
	  } 
	}
}

void scani2c(){
	int error;
  int address;
  Serial.println("Scanning I2C...");
  i2cdevicecount = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
		i2caddresses[i2cdevicecount]=address;  // store the found device address

		i2cdevicecount++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (i2cdevicecount == 0)
    Serial.println("No valid I2C devices found\n");
  else
    Serial.println("I2C Scan complete \n");

}

void GetGPSData(){
		int lf = 10;
		char inchar;
		
		inchar = Serial3.read();
		// Serial.println("GPS:"+String(inchar));
		// strip out nasties
		if(inchar>32){
			if(inchar<128){
				gpsmsg[gpsmsgindex]=inchar;
				gpsmsgindex++;
			}
		}
		if(inchar=='$')  // first char of gps message is a $
		{
			gpsmsg[0]=inchar;
			gpsmsgindex=1;
			trigger=1;
		}

      // gpsmsgindex=gpsstring.length();
		if (gpsmsgindex>(GPSBUFFERLEN-1)){
			// gps message buffer over-run (no CR seen)
			Serial.print("GPS Message Buffer Overrun");
			if(DEBUGGPS)PrintGPSMessage();
			gpsmsgindex=0;
			exit;
		}
	
		if(inchar==lf){
			//check for valid message
			if(trigger==1){
				// end of message
				// copy to String
				gpsmsg[gpsmsgindex]='\0';  // null terminate
				gpsstring=gpsmsg;
				// Process GPS Message 
				if(debugging=='Y'){
					Serial.print("GPS:");
					Serial.println(gpsstring);
				}
				// pass GPS data if pass-thru enabled
				if(GPSPASSTHRU)Serial.println(gpsstring);
				GPSParser();
				GPSProcess();
			}
			gpsmsgindex=0; // reset the buffer
			trigger=0;
			exit;
		}
	
	}

void GPSParser(){
	// parse GPS message into string array
	int i;
	int pc; // parm count
	int gpsmsglength;
	int cp1; // comma pointer1
	int cp2; // comma pointer2
	int lastcomma=0;
    char c1;
	String s1=gpsstring;
	int mlen=s1.length();
	if(debugging=='Y'){
		Serial.print("Parsing:");
        Serial.print(gpsmsgindex,DEC);
		Serial.print(" Characters: ");
    	 
		Serial.print(mlen);
		Serial.println(s1);
	}
	pc=0; // reset parm counter
	cp2=0; 
	int lastcommapos=s1.lastIndexOf(',');
	// do we have commas?
	if(lastcommapos>0){
		while(lastcomma==0)
		{
			cp1=s1.indexOf(',',cp2);
			if(cp1>0){
				cp2=s1.indexOf(',',cp1+1);
				if(cp2>0){
					// Serial.println(cp1,DEC);
					Arg[pc]=s1.substring(cp1+1,cp2);
				pc++;
				} else {
				lastcomma=1;
				}
			}
			else
			{
				lastcomma=1;
			}
			if(pc>(ARGCOUNT-1)){
				lastcomma=1;
			}
		}
		if(debugging=='Y'){
			Serial.print("Found ");
			Serial.print(pc,DEC);
			Serial.println(" parameters");
			for(i=0;i<pc;i++)
			{
				Serial.println(String(String(i)+':'+Arg[i]));
			}
		}
	
	}
	argcount = pc;
}

void GPSProcess(){
	// GSP messages
	String gpscmd = gpsstring.substring(1,6);
	if(DEBUGGPS)	Serial.println("Processing "+gpscmd);

	if(gpscmd=="GPGLL"){ 
		if(DEBUGGPS){
		Serial.println("LL - Latitude:"+Arg[0]+" "+ Arg[1]+ "Longitude:"+Arg[2]+" "+Arg[3]);
		}
		/* 
		$GPGLL,4916.45,N,12311.12,W,225444,A,*1D

		Where:
			 GLL          Geographic position, Latitude and Longitude
		0,1	 4916.46,N    Latitude 49 deg. 16.45 min. North
		2,3	 12311.12,W   Longitude 123 deg. 11.12 min. West
		4	 225444       Fix taken at 22:54:44 UTC
		5	 A            Data Active or V (void)
			 *iD          checksum data
		*/
		// PrintGPSMessage();		
		exit;
	}
	if(gpscmd=="GPRMC"){
		/*
		$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

Where:
     RMC          Recommended Minimum sentence C
    0- 123519       Fix taken at 12:35:19 UTC
    1-  A            Status A=active or V=Void.
    2-  4807.038   Latitude 48 deg 07.038'
	3-  N				orthagonal
    4-  01131.000  Longitude 11 deg 31.000' 
	5-  E				orthagonal
    6 - 022.4        Speed over the ground in knots
    7 - 084.4        Track angle in degrees True
    8 - 230394       Date - 23rd of March 1994
    9 - 003.1,W      Magnetic Variation
    10 - *6A          The checksum data, always begins with *
		*/
		gpsfixvalid=Arg[1];
		if(gpsfixvalid=="A"){
			gpsfixtime=Arg[0];
			gpslat=Arg[2];
			gpslatort = Arg[3];
			gpsfixdate=Arg[8];
			gpslong=Arg[4]; // gps longitude
			gpslongort = Arg[5];  // gps longitude compass
			
		} 
		exit;
	}
	if(gpscmd=="GPGGA"){
		// Serial.print("GA Message:");	
		/*
		$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
		string gpsfixtime="000000";
		float gpslat=0.00;
		string gpslatort = "N";
		float gpslong=0.00;
		string gpslongort = "E";
			Where:
				 GGA          Global Positioning System Fix Data
				 123519       Fix taken at 12:35:19 UTC
				 4807.038,N   Latitude 48 deg 07.038' N
				 01131.000,E  Longitude 11 deg 31.000' E
				 1            Fix quality: 0 = invalid
										   1 = GPS fix (SPS)
										   2 = DGPS fix
										   3 = PPS fix
							   4 = Real Time Kinematic
							   5 = Float RTK
										   6 = estimated (dead reckoning) (2.3 feature)
							   7 = Manual input mode
							   8 = Simulation mode
				 08           Number of satellites being tracked
				 0.9          Horizontal dilution of position
				 545.4,M      Altitude, Meters, above mean sea level
				 46.9,M       Height of geoid (mean sea level) above WGS84
								  ellipsoid
				 (empty field) time in seconds since last DGPS update
				 (empty field) DGPS station ID number
				 *47          the checksum data, always begins with *
				*/
		if(gpsstring[7]!=0x2c){  // no comma in position 7 so have fix details
			if(DEBUGGPS){
				// copy the fix time
				Serial.print("GPS Fix Time:"+Arg[0]);
				Serial.print(" Fix Status:" + gpsfixvalid);
				Serial.print(" -  Latitude:"+Arg[1]+" "+Arg[2]);
				Serial.println(" -  Longitude:"+Arg[3]+" "+Arg[4]);
			}
			gpsfixtime=gpsstring.substring(7,13);
			// Serial.println(gpsfixtime);
		} exit;
	}
	
	if(gpscmd=="GPZDA"){
		if(DEBUGGPS)Serial.print("ZDA Message:");	
		/*
		ZDA - Data and Time

  $GPZDA,hhmmss.ss,dd,mm,yyyy,xx,yy*CC
  $GPZDA,201530.00,04,07,2002,00,00*60

where:
	hhmmss    HrMinSec(UTC)
        dd,mm,yyy Day,Month,Year
        xx        local zone hours -13..13
        yy        local zone minutes 0..59
        *CC       checksum
				*/
		if(gpsstring[7]!=0x2c){  // no comma in position 7 so have time details
			gpstimeokFlag=true;
			GPShhmmssdss=Arg[0];
			GPSdd=Arg[1];
			GPSmm=Arg[2];
			GPSyy=Arg[3];
			GPSzh=Arg[4];
			GPSzm=Arg[5];
		
				
		} 
		else {
		gpstimeokFlag=false;	
		}
		exit;
	}
}	

void PrintGPSMessage(){
	// dump the current GPS buffer to the serial port 
	int i;
	for(i=0;i<gpsmsgindex;i++){
		Serial.print(gpsmsg[i]);
	}
	Serial.println();
}
