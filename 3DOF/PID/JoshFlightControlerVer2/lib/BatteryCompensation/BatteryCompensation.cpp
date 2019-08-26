//-------------------------------------------------------------------------------
//THIS SUBROUTINE ACCOUNTS FOR THE DROP IN BATTERY VOLTAGE OVER TIME
// TWO RESISTORS ARE USED TO CREATE A VOLTAGE DIVIDER R1 = 6.8 kOhm R2 = 2.2 kOhm
//--------------------------------------------------------------------------------
#include <Arduino.h>

#define led 13 		   //Warning LED Pin
#define batt 23      //Battery Monitor Pin
#define N 1000
#define battComp 40.0 
#define battSlope 0.013279114088
#define battIntercept 0.091917237806
#define lowBatt 11.1

extern int escPulse1, escPulse2, escPulse3, escPulse4;

float voltage = 12.6;
int count = 0, sum = 0;

//Measure Battery Voltage//
void GetBatteryCompensation()
{
  //Battery Voltage Monitoring//
  sum += analogRead(batt);
  if (++count == N) {voltage = battSlope*sum/count + battIntercept; count = 0;  sum = 0; }//Moving Average Filter
  if (voltage < lowBatt){digitalWrite(led,HIGH);} //Battery Low -> Warning LED On
  if (voltage < 12.40 && voltage > 6.0) {   //Battery Connected?
    escPulse1 += (12.40 - voltage)*battComp; escPulse2 += (12.40 - voltage)*battComp;
    escPulse3 += (12.40 - voltage)*battComp; escPulse4 += (12.40 - voltage)*battComp;}
}