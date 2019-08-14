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
extern float offsetAccX, offsetAccY, offsetAccZ;

//------------------------
// IMU CALIBRATION VALUES
//------------------------

// Offsets applied to raw x/y/z mag values
float mag_offsets[3] = { -20.02F, -26.84F, 48.86F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.953,  -0.0019,  0.009 },
									 {  -0.059,  1.022, -0.024 },
									 {  0.009, -0.024,  1.028  } };

float mag_field_strength = 48.75F;

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
float accelerationX = 0, accelerationY = 0, accelerationZ = 0;
float magAcceleration = 0;

// Variables for filter 
float averageZ = 0, sumZ = 0;
float storage[25] = {};
int counter = 0;

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

	actualRoll = filter.getPitch();
	actualPitch = filter.getRoll();
	actualYaw = -1*filter.getYaw();

	actualPitchRate =  gx - offsetPitchRate;
	actualRollRate = gy - offsetRollRate;
	actualYawRate = -1*gz - offsetYawRate;

	accelerationX = -accel_event.acceleration.y - offsetAccX;
	accelerationY = -accel_event.acceleration.x - offsetAccY;
	accelerationZ = accel_event.acceleration.z - offsetAccZ;

	magAcceleration = sqrt(accelerationX*accelerationX + accelerationY*accelerationY + accelerationZ*accelerationZ);

	 // Average the acceleration data 
	 sumZ += accelerationZ;
	 sumZ -= storage[counter];
	 averageZ = sumZ/25;
	 storage[counter] = accelerationZ;
	 if(counter == 24)
	 	counter = 0;
	 else
	 	counter++;
}