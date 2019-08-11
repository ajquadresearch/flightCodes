//-------------------------------------------------------------------
//THIS SUBROUTINE ACCOUNTS FOR THE DROP IN BATTERY VOLTAGE OVER TIME
//-------------------------------------------------------------------
#include <Ardunio.h>

#define led 13 		   //Warning LED Pin
#define batt 32      //Battery Monitor Pin
#define N 1000
#define battComp 40.0 
#define battSlope 0.013279114088
#define battIntercept 0.091917237806
#define lowBatt 11.1

float voltage = 12.6;
int count = 0, sum = 0;

//Measure Battery Voltage//
void battery(){

  //Battery Voltage Monitoring//
  sum += analogRead(batt);
  if (++count == N) {voltage = battSlope*sum/count + battIntercept; count = 0;  sum = 0;} //Moving Average Filter
  if (voltage < lowBatt) digitalWrite(led,HIGH);}        //Battery Low -> Warning LED On