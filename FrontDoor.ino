//This program was written by Brandon Saffell for use by Kory Easterday.

//Operation
//
//
//
//
//Device
//

// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// You should have received a copy of the GNU General    
// Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************

#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/
#include <LDR10k.h>
#include <Thermistor10k.h>

//*********************************************************************************************
// *********** IMPORTANT SETTINGS - YOU MUST CHANGE/ONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NETWORKID     100  //the same on all nodes that talk to each other
#define NODEID        5  
#define GATEWAYID     1
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "16CHARACTERS1234" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Remove/comment if you have RFM69W!
//*********************************************************************************************


char transmitBuffer[15];  //Buffer to send messages to base station.
int sendSize=0;  //Used for radio transmissions.

RFM69_ATC radio;

const int SWITCHPIN=4;  //The pin the reed switch is attached to.
const int BUTTONPIN=3;  //push button
const int BUTTONINTERRUPT=1;  //Using interrupt #1
const int BATTERYSENSEPIN=A2;  //Battery voltage is read with A0
const int LEDPIN=9;  //Internal LED
const int GREENLED=6;
const int REDLED=5;
bool switchStatus=0;  
bool lastSwitchStatus=0;
volatile bool securityUpdated=0;
volatile bool securityStatus=0;
byte securityCounter=0;
unsigned long environmentalTimer=0;

const float MOTEINOVOLTAGE=3.3;  //Max ADC voltage
const float BATTERYLOWVOLTAGE=2.4;  //0.8 volts per cell. The device will probably operate below this for a time, but cell rupture is risked.
const float BATTERYLOWSHUTDOWN=2.1; //Below this voltage the system goes into a deeper sleep.
const float BATTERYLOWVOLTAGERESET=2.7; //Hystersis
int batterySenseValue=0;  //Output of ADC

float batterySenseValueFP=0.0;  //Battery sense value as a FP.
float batteryVoltageRaw=0.0;  //Actual battery voltage.
float batteryVoltageFiltered=0.0;
float batteryVoltageFilterFactor=0.1;

//Transmit variables//

const char SENSORNAME1[3]={"BV"};
const byte PRECISION1 = 3;  //Used for dtostrf() later.
char floatBuffer1[PRECISION1+4];

const char SENSORNAME2[3]={"TE"};
const byte PRECISION2 = 2;  //Used for dtostrf() later.
char floatBuffer2[PRECISION2+4];

const char SENSORNAME3[3]={"LI"};
const byte PRECISION3 = 2;  //Used for dtostrf() later.
char floatBuffer3[PRECISION3+4];

const char SENSORNAME4[3]="TL";
int sensorValue4=0;

const char SENSORNAME5[3]="SS";
int sensorValue5=0;

const char SENSORNAME6[3]="RT";
int sensorValue6=0;

//LDR Variables//

const int LDRPIN=A0;
const int LDRRESISTOR=9850;
float ldrRaw=0.0;
float ldrFiltered=0.0;
float ldrFilterFactor=0.7;

//Thermistor Variables//

const int THERMISTORPIN=A1;
const int THERMISTORRESISTOR=9810;
float thermistorRaw=0.0;
float thermistorFiltered=0.0;
float thermistorFilterFactor=0.2;

//Objects//

LDR10k ldr(LDRPIN, LDRRESISTOR);
thermistor10k thermistor(THERMISTORPIN, THERMISTORRESISTOR); 

int sleepCounter=0;
byte batteryCounter=0;

//void readBattery();  //Prototype Functions
void readSwitch();
void transmitLevel();
void signalStrength();
void enclosureTemperature();
void buttonISR();
void sleepCounterFunction();
void getSecrity();

void setup() 
{
 
	//Serial.begin(19200); //Debug info
  
  	radio.initialize(FREQUENCY,NODEID,NETWORKID); //Startup radio
  	radio.encrypt(ENCRYPTKEY);
  	radio.enableAutoPower(-60);

  	pinMode(SWITCHPIN, INPUT_PULLUP);  //Setup control pins.
  	pinMode(BUTTONPIN, INPUT_PULLUP);
  	pinMode(REDLED, OUTPUT);
  	pinMode(GREENLED, OUTPUT);
  	pinMode(LEDPIN, OUTPUT);
  
  	ldrFiltered=ldr.readLDR();
  	thermistorFiltered=thermistor.readThermistor();
	/*
  	batterySenseValue=analogRead(BATTERYSENSEPIN);  //Read ADC fort he battery
  	batterySenseValueFP=float(batterySenseValue);  //Converty to a FP
	batteryVoltageRaw=(batterySenseValueFP*MOTEINOVOLTAGE*2)/1023; 
  	batteryVoltageFiltered=batteryVoltageRaw;
	*/  
  	attachInterrupt(BUTTONINTERRUPT, buttonISR, FALLING);
  
  	getSecurity();
  
  	LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_ON);
}


void loop() 
{
	if ((millis()-environmentalTimer)>2000)
	{
	sleepCounterFunction();
	if (securityUpdated==0)
    {
    	if (securityCounter==15)
    	{
    	getSecurity();
    	securityCounter=0;
    	}
    	securityCounter++;
    }
	}
  
	if (radio.receiveDone())
    {
    	if (radio.DATA[0]=='S' && radio.DATA[1]=='C' && radio.DATA[2]=='1')
    	{
    		digitalWrite(REDLED, LOW);
    		digitalWrite(GREENLED, HIGH);
    		securityUpdated=1;
    		securityStatus=1;
    	}
    	
    	else if (radio.DATA[0]=='S' && radio.DATA[1]=='C' && radio.DATA[2]=='0')
    	{
    		digitalWrite(REDLED, HIGH);
    		digitalWrite(GREENLED, LOW);
    		securityUpdated=1;
    		securityStatus=0;
    	}
    	else
    	{
    		digitalWrite(REDLED, HIGH);
    		digitalWrite(GREENLED, HIGH);
    	}
    	
    	if (radio.ACKRequested())
    	{
      		radio.sendACK();
      	}
    	
    }
}

void getSecurity()
{
	sprintf(transmitBuffer, "$%d@GS#GS*",NODEID);
  	sendSize = strlen(transmitBuffer);
  	radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
}

void sleepCounterFunction()
{
	readSwitch();  //Reads and sends the reed switch information.
	
	//if (sleepCounter==30)
	//{
	//readBattery();  //Reads and sends the battery voltage.	
	//}
	if (sleepCounter==50)
	{
		transmitLevel();
	}
	if (sleepCounter==68 || sleepCounter==188 || sleepCounter==308 ||sleepCounter==428 || sleepCounter==548)
  	{
  		readTemperature();
  	}
  	if (sleepCounter==100)
  	{
  		enclosureTemperature();
  	}
  	if (sleepCounter==12 || sleepCounter==132 || sleepCounter==252 || sleepCounter==372 || sleepCounter==492)
  	{
  		readLight();
  	}
  	if (sleepCounter==190)
  	{
  		signalStrength();
  	}
  	if (sleepCounter==600)
  	{
  		sleepCounter=0;
  	}
  	sleepCounter++;
}

void buttonISR()
{
	//detachInterrupt(BUTTONINTERRUPT);
	LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_ON); // debounhce
	//delay(250);
	//Serial.println("ISR");
	if(securityStatus==0)
	{
		sprintf(transmitBuffer, "$%d@SE#SE*",NODEID);
		//securityStatus=1;
	}
	else if(securityStatus==1)
	{
		sprintf(transmitBuffer, "$%d@SD#SD*",NODEID);
		//securityStatus=0;
	}
  	sendSize = strlen(transmitBuffer);
  	radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
  	securityUpdated=0;
  	digitalWrite(REDLED, HIGH);
    digitalWrite(GREENLED, HIGH);
    //attachInterrupt(BUTTONINTERRUPT, buttonISR, RISING);
}

/*void readBattery()
{
	batterySenseValue=analogRead(BATTERYSENSEPIN);  //Read ADC fort he battery
	//Serial.println(batterySenseValue);
	batterySenseValueFP=float(batterySenseValue);  //Converty to a FP
	batteryVoltageRaw=(batterySenseValueFP*MOTEINOVOLTAGE*2)/1023;  //Convery to a voltage
	//Serial.println(batteryVoltageRaw);
	batteryVoltageFiltered=(batteryVoltageFilterFactor*batteryVoltageRaw)+((1-batteryVoltageFilterFactor)*batteryVoltageFiltered);
	//Serial.println(batteryVoltageFiltered);
	
	dtostrf(batteryVoltageFiltered, PRECISION1+3, PRECISION1, floatBuffer1);  //Converts battery voltage to a string to be sent out for debugging.
  	sprintf(transmitBuffer, "$%d@%s#%s*",NODEID,SENSORNAME1,floatBuffer1);
  	sendSize = strlen(transmitBuffer);
  	radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
  	
	return;
}
*/
void readSwitch()  //Reads the reed switch.
{
	LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);
	//delay(250);
	
	switchStatus=digitalRead(SWITCHPIN);  //Determines if the switch is open or closed.
	if ((sleepCounter==16)||(lastSwitchStatus!=switchStatus))
	{
		if (switchStatus==1)  //Gate is open.
		{
			sprintf(transmitBuffer, "$%d@DO#DO*", NODEID);
			//Serial.println(transmitBuffer);
			sendSize = strlen(transmitBuffer);
			radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize); //Gate Open
		}
		else  //Gate is cosed.
		{
			sprintf(transmitBuffer, "$%d@DC#DC*", NODEID);
			//Serial.println(transmitBuffer);
			sendSize = strlen(transmitBuffer);
			radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize); //Gate Closed
   		}
	}
    
    lastSwitchStatus=switchStatus;
	
	//delay(250);
	//radio.sleep();
	LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_ON);
	
	return;
}

void readLight()
{
	float ldrRaw=ldr.readLDR();
  	ldrFiltered=(ldrFilterFactor*ldrRaw)+((1-ldrFilterFactor)*ldrFiltered);
   	dtostrf(ldrFiltered, PRECISION3+3, PRECISION3, floatBuffer3);  //Converts battery voltage to a string to be sent out for debugging.
  	sprintf(transmitBuffer, "$%d@%s#%s*",NODEID,SENSORNAME3,floatBuffer3);
  	sendSize = strlen(transmitBuffer);
  	radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
  	
	return;
}

void readTemperature()
{
	float thermistorRaw=thermistor.readThermistor();
  	thermistorFiltered=(thermistorFilterFactor*thermistorRaw)+((1-thermistorFilterFactor)*thermistorFiltered);
    dtostrf(thermistorFiltered, PRECISION2+3, PRECISION2, floatBuffer2);  //Converts battery voltage to a string to be sent out for debugging.
  	sprintf(transmitBuffer, "$%d@%s#%s*",NODEID,SENSORNAME2,floatBuffer2);
  	sendSize = strlen(transmitBuffer);
  	radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
  	
  	return;
}

void transmitLevel()
{
	sensorValue4=radio._transmitLevel;
  	sprintf(transmitBuffer, "$%d@%s#%d*",NODEID,SENSORNAME4,sensorValue4);
  	sendSize = strlen(transmitBuffer);
  	radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
  	sensorValue5=radio.RSSI;
  	
  	return;
}

void signalStrength()
{
	sprintf(transmitBuffer, "$%d@%s#%d*",NODEID,SENSORNAME5,sensorValue5);
  	sendSize = strlen(transmitBuffer);
  	radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
  	
  	return;
}

void enclosureTemperature()
{
	sensorValue6=radio.readTemperature(0)*1.8+32;
  	sprintf(transmitBuffer, "$%d@%s#%d*",NODEID,SENSORNAME6,sensorValue6);
  	sendSize = strlen(transmitBuffer);
  	radio.sendWithRetry(GATEWAYID, transmitBuffer, sendSize);
  	
  	return;
}
