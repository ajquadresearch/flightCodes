//---------------------------------------------------------------
//SAFETY FEATURES ARE PROVIDED BELOW TO DECREASE CHANCE OF INJURY 
//---------------------------------------------------------------
#include <Arduino.h>

const int led = 13; 
// GLOBAL VARIABLES
extern float actualPitch, actualRoll, actualYaw;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern int escPulse1, escPulse2, escPulse3, escPulse4;
extern float offsetPitchRate, offsetRollRate, offsetYawRate; 
extern volatile int R[7];

void Blink()
{
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
}

void SafteyInitialization()
{
    pinMode(led, OUTPUT);

    // Make sure quadcopter is in correct starting position
    while( R[5] > 1100 )
    {
        Serial.println("Move SWB UP");
        Blink();
    }

    while( R[3] > 1100)
    {
        Serial.println("Move throttle to low position");
        Blink();
    }

    Serial.print("Setup Finished!");
    digitalWrite(led, HIGH);
	delay(1000);
}