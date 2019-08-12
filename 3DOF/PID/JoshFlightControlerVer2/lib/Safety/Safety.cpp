//---------------------------------------------------------------
//SAFETY FEATURES ARE PROVIDED BELOW TO DECREASE CHANCE OF INJURY 
//---------------------------------------------------------------
#include <Arduino.h>

const int led = 13; 
extern volatile unsigned int R[7];

void Blink()
{
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
}

void SafteyInitialization()
{
    // set led pin direction
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

    // Make sure reciever signals are greater than 990
    while(R[1] < 990 || R[2] < 990 || R[3] < 990 || R[4] < 990)
    {
        Serial.println("Reciever signals are less than 990");
        Blink();
    }

    Serial.print("Setup Finished!");
    digitalWrite(led, HIGH);
	delay(1000);
}