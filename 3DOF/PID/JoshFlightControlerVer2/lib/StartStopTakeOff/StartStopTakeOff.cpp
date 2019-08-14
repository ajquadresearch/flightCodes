//----------------------------------------------------------------------------------
// THIS SUBROUTINE CONTROLS THE START AND STOP OF THE MOTORS AND THE TAKEOFF ROUTINE
//----------------------------------------------------------------------------------
extern volatile unsigned int R[7];
extern int throttle;
extern float averageZ;
float startUpMagAcceleration = 9.81; // need to find a way to take intial values at startup 
int takeOff = 0, takeOffDetected = 0;
int start = 0, throttle = 1100;
int takeOffThrottle = 0;
#include <Arduino.h>


// Start/Kill MOTORS

void GetMode()
{
    // Get Ready for takeOff
    if ( R[5] < 1100)
    {
        start = 0;                                               
        takeOffDetected = 0;             // Reset take off detection 
        takeOffThrottle = 0;             // Reset throttle

    }
    else
        start = 1;                       // Idle motors
    
    if ( R[6] > 1400 && R[6] < 1700)
        takeOff = 1;                     // Manual take off will be preformed 
    else if (R[6] > 1700) 
        takeOff = 2;                     // Auto take off will be preformed
    else
        takeOff = 0;


    if( takeOff == 1 && start == 1) 
        throttle = R[3];                                    
    if( takeOff == 2 && start == 1 && takeOffDetected == 0) // autoTakeOff
    {
        if( R[3] > 1400 && R[3] < 1750) // put throttle stick to center channel
        {
            throttle++;
            if(throttle > 1600)         
            {
                takeOffThrottle = 0; // Failed to takeOff 
            }
            if( (averageZ - startUpMagAcceleration) < -1 ) // Threshold for take off detection
            {
                takeOffDetected = 1;                       
                takeOffThrottle = throttle - 1530;
                Serial.println(takeOffDetected); // Currently for debugging
            }

        }
    } 
    if( takeOff == 2 && start == 1 && takeOffDetected == 1)
    {
        throttle = R[3] + takeOffThrottle;
    }    

}

    



