/////////////////////////////////////////////////////////
// SPIN MOTORS 
/////////////////////////////////////////////////////////
// This subroutine spins the motors on the quadcopter 
#include <Arduino.h> 	

// GLOBAL VARIABLES USED FOR DEBUGGING
extern float actualPitch, actualRoll, actualYaw;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern int escPulse1, escPulse2, escPulse3, escPulse4;
extern float offsetPitchRate, offsetRollRate, offsetYawRate; 
extern volatile int R[7];

 // Get maximum value for selected PWM resolution (100% Duty)
 int pwmMax = 256;
 // Initializing pulse for ESCs, 25% duty
 int escInit = pwmMax/4;

 int pwmFreq = 250;
 int pwmRes = 8;
 int escPulseTime = 4000;

 // Value for analog write function 
 int escPulse1PWM = 0, escPulse2PWM = 0, escPulse3PWM = 0, escPulse4PWM = 0;

 // esc pins 
int escOut1 = 6, escOut2 = 10, escOut3 = 5, escOut4 = 20;

extern int escPulse1, escPulse2, escPulse3, escPulse4;

// Set motors so they are ready to run
void EscInitialize()
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

// This function converts the calculate pulse to the PWM signal.
// Then the PWM signal is written to the motors
void PulsetoPWM()
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

// idle motors 
int minPulse = 1100;

// max motors
int maxPulse = 2000;

void BoundPulse()
{
	// Upper Bound 
	if (escPulse1 > maxPulse)
	{
		escPulse1 = maxPulse;
	}

	if (escPulse2 > maxPulse)
	{
		escPulse2 = maxPulse;
	}

	if (escPulse3 > maxPulse)
	{
		escPulse3 = maxPulse;
	}

	if (escPulse4 > maxPulse)
	{
		escPulse4 = maxPulse;
	}

	// LowerBound 
	if (escPulse1 < minPulse)
	{
		escPulse1 = minPulse;
	}

	if (escPulse2 < minPulse)
	{
		escPulse2 = minPulse;
	}

	if (escPulse3 < minPulse)
	{
		escPulse3 = minPulse;
	}

	if (escPulse4 < minPulse)
	{
		escPulse4 = minPulse;
	}

	// Start/Kill MOTORS
	if( R[5] < 1100)
	{
		escPulse1 = 1000;
		escPulse2 = 1000;
		escPulse3 = 1000;
		escPulse4 = 1000;

		// Reset Integral term until takeoff
	}

	return;
}