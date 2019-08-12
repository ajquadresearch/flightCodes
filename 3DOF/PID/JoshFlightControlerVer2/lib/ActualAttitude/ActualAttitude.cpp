//------------------------------------------------
// DETERMINE THE ACTUAL ATTITUDE OF THE QUADCOPTER
//------------------------------------------------
#include "ActualAttitude.hpp"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Mahony.h>
#include <Madgwick.h>

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

extern float offsetPitchRate, offsetRollRate, offsetYawRate; 

//------------------------
// IMU CALIBRATION VALUES
//------------------------

// Offsets applied to raw x/y/z mag values
 float mag_offsets[3] = { 9.16F, -30.86F, 44.09F };

// Soft iron error compensation matrix
 float mag_softiron_matrix[3][3] = { {  0.939,  -0.020,  -0.018 },
									 {  -0.020,  1.030, -0.006 },
									 {  -0.018, -0.006,  1.034 } };

 float mag_field_strength = 38.18F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Filter sample rate
int updateFreq = 250;

// Filter type
//Mahony filter;
Madgwick filter;

// Global variables 
float actualPitch = 0, actualRoll = 0, actualYaw = 0;
float actualPitchRate = 0, actualRollRate = 0, actualYawRate = 0;

void IMUInitialization()
{
	if(!gyro.begin(GYRO_RANGE_250DPS))
	{
		Serial.println("Ooops, no gyro detected ... Check your wiring!");
		while(1);
	}

	if(!accelmag.begin(ACCEL_RANGE_4G))
	{
		Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
		while(1);
	}

	filter.begin(updateFreq);
	Serial.println("IMU initialization Completed");
}

void GetActualAttitude()
{
	 sensors_event_t gyro_event;
	 sensors_event_t accel_event;
	 sensors_event_t mag_event;
	 gyro.getEvent(&gyro_event);
	 accelmag.getEvent(&accel_event, &mag_event);

	 // Apply mag offset compensation (base values in uTesla)
	 float x = mag_event.magnetic.x - mag_offsets[0];
	 float y = mag_event.magnetic.y - mag_offsets[1];
	 float z = mag_event.magnetic.z - mag_offsets[2];

	 // Apply mag soft iron error compensation
	 float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
	 float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
	 float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

	 // Apply gyro zero-rate error compensation
	 float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
	 float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
	 float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

	 // The filter library expects gyro data in degrees/s, but adafruit sensor
	 // uses rad/s so we need to convert them first (or adapt the filter lib
	 // where they are being converted)
	 gx *= 57.2958F;
	 gy *= 57.2958F;
	 gz *= 57.2958F;

	filter.update(gx, gy, gz,
			accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
			mx, my, mz);

	actualRoll = filter.getPitch() + 2.5;
	actualPitch = filter.getRoll() + 2.8;
	actualYaw = -1*filter.getYaw();

	actualPitchRate =  gyro_event.gyro.x * (180/3.14) - offsetPitchRate;
	actualRollRate = gyro_event.gyro.y * (180/3.14) - offsetRollRate;
	actualYawRate = -1*gyro_event.gyro.z * (180/3.14) - offsetYawRate;
}