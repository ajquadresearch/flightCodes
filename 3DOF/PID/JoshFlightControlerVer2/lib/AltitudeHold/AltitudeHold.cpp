#include <Arduino.h> 
#include <Wire.h>

extern volatile int R[7];
extern volatile unsigned int throttle, base_throttle;
extern bool altitudeHold;

 /////////////////////////////////////////////////////////
 //Global Variables for Barometer
 /////////////////////////////////////////////////////////
uint8_t MS5611_address = 0x00000077;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.
uint8_t start;
uint8_t error;

int16_t count_var;

uint32_t loop_timer;

float dummy_float;
int16_t manual_throttle;
uint8_t manual_altitude_change;
float pid_error_temp;

//Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;
double altitude, referenceAltitude;
double seaLevelPressure = 101325;                     //Needed for altitude calculations, taken from google (same value as used in JoopBarometerTest)
//Altitude PID variables
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;

float pid_p_gain_altitude = 5.0;             //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0.1;             //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 0.95;           //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).



//////////////////////////////////////////////////////////
// Barometer
//////////////////////////////////////////////////////////

void GetAltitude() {
  barometer_counter ++;
  
  //Every time this function is called the barometer_counter variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  if (barometer_counter == 1) {                                                 //When the barometer_counter variable is 1.
    if (temperature_counter == 0) {                                             //And the temperature counter is 0.
      //Get temperature data from MS-5611
      Wire.beginTransmission(MS5611_address);                                   //Open a connection with the MS5611
      Wire.write((u_int8_t)0x00);                                                         //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                   //End the transmission with the MS5611.

      Wire.requestFrom(MS5611_address, 3);                                       //Poll 3 data bytes from the MS5611.
      delayMicroseconds(100);
      // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      
      raw_temperature_rotating_memory[average_temperature_mem_location] = (Wire.read() << 16 | Wire.read() << 8 | Wire.read());

      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
    }
    else {
      //Get pressure data from MS-5611
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write((u_int8_t)0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
 
      Wire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      delayMicroseconds(100);
      raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();     //Shift the individual bytes in the correct position and add them to the raw_pressure variable.

    }

    temperature_counter ++;                                                     //Increase the temperature_counter variable.
    if (temperature_counter == 20) {                                            //When the temperature counter equals 20.
      temperature_counter = 0;                                                  //Reset the temperature_counter variable.
      //Request temperature data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write((u_int8_t)0x58);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    }
    else {                                                                      //If the temperature_counter variable does not equal 20.
      //Request pressure data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      Wire.write((u_int8_t)0x48);                                                        //Send a 0x48 to indicate that we want to request the pressure data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    }
  }
  if (barometer_counter == 2) {                                                 //If the barometer_counter variable equals 2.
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
    altitude = (44330.0f * (1.0f - pow((double)actual_pressure / (double)seaLevelPressure, 0.1902949f)));
  }

  if (barometer_counter == 3) {                                                                               //When the barometer counter is 3

    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

    if (altitudeHold && (R[6] > 1000)) {                                                          //If the quadcopter is in altitude mode and flying.
      if (pid_altitude_setpoint == 0){
		  pid_altitude_setpoint = actual_pressure;                                  //If not yet set, set the PID altitude setpoint.
		  base_throttle = R[3];											//When altitude hold is first enabled the current throttle value is stored for calculating throttle -Jaiden
	  }
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.

      if (R[3] > 1600) {                                                   //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (R[3] - 1600) / 3;                               //To prevent very fast changes in hight limit the function of the throttle.
	  }

      if (R[3] < 1400) {                                                   //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = ((int)R[3] - 1400) / 5;                          //To prevent very fast changes in hight limit the function of the throttle. | Need to cast to actual integer to prevent loss of control -Jaiden
      }

      //Serial.println(pid_altitude_setpoint);

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pid_error_gain_altitude = ((abs(pid_error_temp) - 10) / 20.0)/2;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
	  }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude) pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      //In the following line the PID-output is calculated.
      //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
      //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
      //D = pid_d_gain_altitude * parachute_throttle.
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      //To prevent extreme PID-output the output must be limited.
      if (pid_output_altitude > pid_max_altitude) pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;

	  throttle = base_throttle + pid_output_altitude + manual_throttle;					//Caclulate throttle output that is passed to the ESCs -Jaiden
    }
 
    
    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    else if (R[6]<=1000 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
      pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }

	else throttle = R[3];													//Let throttle function normally when altitude hold is disabled -Jaiden
  }
  
}

//************************************************************************************************************************************
// AltHold Add-In

/////////////////////////////////////////////////////////
//Barometer Initialization
/////////////////////////////////////////////////////////

void barometerInitialize(){
  //Check if the MS5611 barometer is responding.                        
  Wire.begin();
  Wire.beginTransmission(MS5611_address);                      //Start communication with the MS5611.
  error = Wire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MS5611 did not responde.
    Wire.beginTransmission(MS5611_address);
    error = Wire.endTransmission();
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }

  //Create a 5 second delay before calibration.
  for (count_var = 0; count_var < 1250; count_var++) {          //1250 loops of 4 microseconds = 5 seconds.
    if (count_var % 125 == 0) {                                 //Every 125 loops (500ms).
      //digitalWrite(PB4, !digitalRead(PB4));                     //Change the led status.
    }
    delay(4);                                                   //Simulate a 250Hz refresch rate as like the main loop.
  }
  count_var = 0;                                                //Set start back to 0.

  
  //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
  //These 2 byte values are stored in the memory location 0xA2 and up.
  for (start = 1; start <= 6; start++) {
    Wire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
    Wire.write(0xA0 + start * 2);                              //Send the address that we want to read.
    Wire.endTransmission();                                    //End the transmission.

    Wire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
    C[start] = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the C[x] calibration variable.
  }

  OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
  SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.

  //The MS5611 needs a few readings to stabilize.
  for (start = 0; start < 100; start++) {                       //This loop runs 100 times.
    GetAltitude();                                           //Read and calculate the barometer data.
    delay(4);                                                   //The main program loop also runs 250Hz (4ms per loop).
  }

  actual_pressure = 0;                                          //Reset the pressure calculations.
}

//********************************************************************************************************************************************