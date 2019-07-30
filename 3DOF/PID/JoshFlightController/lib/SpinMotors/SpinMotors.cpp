// This subrountine allows the motors to spin 
#include <Arduino.h> 	

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

 // esc
int escOut1 = 6;
int escOut2 = 10;
int escOut3 = 5;
int escOut4 = 20;

extern int escPulse1, escPulse2, escPulse3, escPulse4;


/////////////////////////////////////////////////////////
// ESC INTILIZATION
/////////////////////////////////////////////////////////
// Set motors so they are ready to run

void escInitialize()
{

	// Escs
	pinMode(escOut1,OUTPUT);
	pinMode(escOut2,OUTPUT);
	pinMode(escOut3,OUTPUT);
	pinMode(escOut4,OUTPUT);

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

	return;
}

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

	return;
}