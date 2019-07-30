/* ////////////////////////////////////// 
 * //FLIGHT CONTROLLER
 * //////////////////////////////////////
 * 
 * ////////////////////////////////////// 
 * // HARDWARE
 * ////////////////////////////////////// 
 * 
 * MICROCONTROLLER: TEENSY 3.5 
 * 
 * HAND HELD CONTROLLER: https://www.amazon.com/FlySky-FS-i6-M2-2-4GHz-6-Channel-Transmitter/dp/B00PF160IK
 * 
 * MAKE SURE TO SET THE FAIL SAFE MODE ON THE CONTROLLER 
 * REFER TO THIS VIDEO: https://www.youtube.com/watch?v=LXTEXqR_ghI
 * (add settings here ) 
 * 
 * 
 * IMU: NXP 9DOF ADIAFRUIT 
 * https://www.adafruit.com/product/3463
 * 
 * MOTOR: Turnigy Multistar 2213-980Kv 14 Pole Multi-Rotor Outrunner V2
 * https://hobbyking.com/en_us/turnigy-multistar-2213-980kv-14-pole-multi-rotor-outrunner-v2.html
 * 
 * PROPELLERS: 10x4.5 Inch 
 * https://www.amazon.com/uxcell-Propellers-Fixed-Wing-Airplane-Adapter/dp/B07PXKKV3G/ref=sr_1_6?crid=3S7H28DITMXYT&keywords=1045+propeller&qid=1555036892&s=gateway&sprefix=1045+p%2Caps%2C129&sr=8-6
 * 
 * FRAME: 4-Axis Multi Rotor Airframe 450mm
 * https://www.amazon.com/ShareGoo-Airframe-FrameWheel-Quadcopter-Aircraft/dp/B07H3WDSX3/ref=sr_1_fkmrnull_1_sspa?keywords=4-Axis+Multi+Rotor+Airframe+450mm&qid=1555038824&s=gateway&sr=8-1-fkmrnull-spons&psc=1
 * 
 * CREATED BY:
 * JOSHUA WALLACE, KIETH NASON
 * 
 * 4-12-19 
 * 
 * ////////////////////////////////////// 
 * // WARNING!! 
 * //////////////////////////////////////
 * Don't be an idot. -JOSH
 *  
*/

/////////////////////////////////////////////////////////
//LIBARIES 
/////////////////////////////////////////////////////////
#include <Arduino.h> 							//Used for Visual Studio's code 
#include <ActualAttitude.hpp>     
#include <DesiredAttitude.hpp>
#include <AttitudeController.hpp>

/////////////////////////////////////////////////////////
//RANDOM
/////////////////////////////////////////////////////////
elapsedMicros elapsedTime;

/////////////////////////////////////////////////////////
//SWITCHS
/////////////////////////////////////////////////////////
bool debug = false; 

// Variables for debugging
int printTimer = 5000;
int lastPrint = 0;

int updateTime = 4000;
int lastUpdate = 0;

////////////////////////////////////////////////////////
//PIN DEFINITIONS
////////////////////////////////////////////////////////

// Reciever  
 int ch1 = 24;
 int ch2 = 25;
 int ch3 = 26;
 int ch4 = 27;
 int ch5 = 28;
 int ch6 = 29;

// esc
int escOut1 = 6;
int escOut2 = 10;
int escOut3 = 5;
int escOut4 = 20;

// led 
int led = 13; 

/////////////////////////////////////////////////////////
// PULSE LENGTH AND PWM 
/////////////////////////////////////////////////////////

 // Get maximum value for selected PWM resolution (100% Duty)
 int pwmMax = 256;
 // Initializing pulse for ESCs, 25% duty
 int escInit = pwmMax/4;

 int pwmFreq = 250;
 int pwmRes = 8;
 int escPulseTime = 4000;

 // Value for analog write function 
 int escPulse1PWM;
 int escPulse2PWM;
 int escPulse3PWM;
 int escPulse4PWM;

 /////////////////////////////////////////////////////////
 // INTERUPTS
 /////////////////////////////////////////////////////////
 // The interupts are used to obtain the pulse lengths from the hand held controller

 // Timing Variables for Pulse Width
 unsigned long prev1 = 0;
 volatile unsigned int roll_ratePulse = 1500;
 unsigned long prev2 = 0;
 volatile unsigned int pitch_ratePulse = 1500;
 unsigned long prev3 = 0;
 volatile unsigned int throttle_Pulse = 1500;
 unsigned long prev4 = 0;
 volatile unsigned int yaw_ratePulse = 1500;
 unsigned long prev5 = 0;
 volatile unsigned int activateMotor = 1500;

 // Get pulse length 

 // Roll 
 void ch1Int()
{
  if (digitalReadFast(ch1)){
	prev1 = elapsedTime;
  }
  else{
	roll_ratePulse = elapsedTime - prev1;
  }
}

// Pitch 
void ch2Int()
{
  if (digitalReadFast(ch2)){
	prev2 = elapsedTime;
  }
  else{
	pitch_ratePulse = elapsedTime - prev2;
  }
}

// Throttle 
void ch3Int()
{
   if (digitalReadFast(ch3)){
	prev3 = elapsedTime;
   }
   else{
	throttle_Pulse = elapsedTime - prev3;
   }  
}

// Yaw 
void ch4Int()
{
   if (digitalReadFast(ch4)){
	prev4 = micros();
   }
   else{
	yaw_ratePulse = micros() - prev4;
   }
}

// Switch 
void ch5Int()
{
   if (digitalReadFast(ch5)){
	prev5 = elapsedTime;
   }
   else{
	activateMotor = elapsedTime - prev5;
   }
}

// AttitudeController 
extern int escPulse1, escPulse2, escPulse3, escPulse4;

/////////////////////////////////////////////////////////
// CONVERT PULSE TO PWM 
/////////////////////////////////////////////////////////
// This function converts the calculate pulse to the PWM signal.
// Then the PWM signal is written to the motors

void pulsetoPWM()
{
	// Convert Mircosecond time to PWM pulse for motors
	escPulse1PWM = escPulse1*pwmMax/escPulseTime;
	escPulse2PWM = escPulse2*pwmMax/escPulseTime;
	escPulse3PWM = escPulse3*pwmMax/escPulseTime;
	escPulse4PWM = escPulse4*pwmMax/escPulseTime;

	// Send PWM pulse to motors
	analogWrite(escOut1, escPulse1PWM);
  	analogWrite(escOut2, escPulse2PWM);
  	analogWrite(escOut3, escPulse3PWM);
  	analogWrite(escOut4, escPulse4PWM);
}

/////////////////////////////////////////////////////////
// CONTROLLER CHECK 
/////////////////////////////////////////////////////////
// Make sure controller is in the right position
void controllerCheck()
{
	while(activateMotor > 1100)
	{
		Serial.println("Turn left controller nobe to 1000");
	}

	while(throttle_Pulse > 1100)
	{
		Serial.println("Lower throttle pulse to 1000");
	}
}

/////////////////////////////////////////////////////////
// ESC INTILIZATION
/////////////////////////////////////////////////////////
// Set motors so they are ready to run

void escInitialize()
{
	// All on Timer FTM0 -> pwmFreq
  	analogWriteFrequency(escOut1, pwmFreq);

  	// Set PWM resolution
  	analogWriteResolution(pwmRes);
	
	// Initialize ESCs
  	analogWrite(escOut1, escInit);
  	analogWrite(escOut2, escInit);
  	analogWrite(escOut3, escInit);
  	analogWrite(escOut4, escInit);
  	delay(5000);
	Serial.println("ESC Initialization Completed");	
}

/////////////////////////////////////////////////////////
// SETUP
/////////////////////////////////////////////////////////
// Get ready to fly

void setup() 
{
	// Do you want to debug?
	if(debug == true)  
	{
		Serial.begin(115200);
		while(!Serial);
		Serial.println("DEBUGING");
	}

	// Set pin direction(INPUT/OUTPUT)

	// Reciever 
	pinMode(ch1,INPUT);
	pinMode(ch2,INPUT);
	pinMode(ch3,INPUT);
	pinMode(ch4,INPUT);

	// Escs
	pinMode(escOut1,OUTPUT);
	pinMode(escOut2,OUTPUT);
	pinMode(escOut3,OUTPUT);
	pinMode(escOut4,OUTPUT);

	//Led 
	pinMode(led,OUTPUT);

	// Setup rx pin interrupts
  	attachInterrupt(ch1,ch1Int,CHANGE);
  	attachInterrupt(ch2,ch2Int,CHANGE);
  	attachInterrupt(ch3,ch3Int,CHANGE);
  	attachInterrupt(ch4,ch4Int,CHANGE);
	attachInterrupt(ch5,ch5Int,CHANGE);

	// Make sure controller is in the right starting position
	controllerCheck();

	// Intilizate the IMU 
	imuIntilization();

	// Initialize esc
	escInitialize();

	// Setup completed 
	digitalWrite(led,HIGH);
	Serial.println("Finished Setup");
	delay(1000);
}

/////////////////////////////////////////////////////////
// MAIN CODE LOOP 
/////////////////////////////////////////////////////////
void loop() 
{
	// Update pulse to motors every 250hz
	if ((elapsedTime - lastUpdate) > updateTime)
	{

		lastUpdate = elapsedTime;

		// Get the rates and angles
		GetActualAttitude();

		// Get the input pulse for the PID 
		getInput();

		// Get the PID corrections and calculate the motor pulses 
		getPID();

		// Make sure pulse is in correct range and kill or activate motors  
		boundPulse();

		// Convert the Pulse to PWM in order to spin the motors 
		pulsetoPWM();

	}	

	// Check output variables
	if (debug == true)
	{
		if ((elapsedTime - lastPrint) >= printTimer)
		{
			lastPrint = elapsedTime;
			Serial.println(yaw_ratePulse);
		}
	}
}