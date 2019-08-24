//----------------------------------------------------------------------------------
// THIS SUBROUTINE CONTROLS THE START AND STOP OF THE MOTORS AND THE TAKEOFF ROUTINE
//----------------------------------------------------------------------------------
// Note: still need to create a landing routine
extern volatile unsigned int R[7];
extern int throttle;
extern float averageZ;
bool startMotor = false, takeOff = false;
int throttle = 1100;
int flightMode = 1;
int takeOffThrottle = 0;
#include <Arduino.h>


// Start/Kill MOTORS

void GetMode()
{
    // Determine Quadcopter state
    if( R[5] > 1500)                        // Motor state 
    {
        startMotor = true;
    }
    else 
    {
        startMotor = false;
        takeOff = false;
        takeOffThrottle = 0;
    }

    if( R[6] < 1200)                        // Manual Flight 
    {
        flightMode = 1;
    }
    else if (R[6] > 1200 && R[6] < 1700)    // Auto takeoff 
    {
        flightMode = 2;
    }
    else                                    // Auto landing 
    {
        flightMode = 3;
    }

    // Procedures for each state 
    if(startMotor == true && flightMode == 1) // Manual Flight
    {
        throttle = R[3];
    }

    if( startMotor == true && flightMode == 2 && takeOff == false) // auto take off 
    {
        if(R[3] > 1400 && R[3] < 1600)
        {
            throttle++;
            if(throttle >= 1750)
            {
                throttle = 1750;
                takeOffThrottle = 0;
            }
            if(averageZ > 10.15)
            {
                takeOff = true;
                takeOffThrottle = throttle - 1530;
            }
        }
    }

    if(startMotor == true && flightMode == 2 && takeOff == true) // Once quadcopter is in the air 
    {
        throttle = R[3] + takeOffThrottle;
    }

    if(startMotor == true && flightMode == 3 && takeOff == true) // Auto landing 
    {
        if(throttle > 1100)
        {
            throttle--;
        }
        if(throttle <= 1100)
        {
            takeOff = false;
        }
    }


    
    
}

    



