/////////////////////////////////////////////////////////
// ATTITUDE CONTROLLER
/////////////////////////////////////////////////////////
// This subroutine takes in the desired and actual pitch roll and yaw rates to determine the motor outputs

// Global Variables 
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern volatile unsigned int throttle_Pulse, activateMotor;


// Variables
int escPulse1, escPulse2, escPulse3, escPulse4;

// Pitch 
int errorPitch; 
int pitchPulse;
int last_errorPitch;
int pid_max_pitch = 300;
float Ipitch;

int pPitch = 2;
int dPitch = 18;
int iPitch = .02;

// Roll
int errorRoll; 
int rollPulse;
int last_errorRoll;
int pid_max_roll = 300;
int Iroll;

int pRoll = pPitch;
int dRoll = dPitch;
int iRoll = iPitch;

// Yaw 
int errorYaw; 
int yawPulse;
int last_errorYaw;
int pid_max_yaw = 300;
int Iyaw;

int pYaw = 4;
int dYaw = 0;
int iYaw = 0.02;

void GetAttitudeController()
{

	// Pitch
	errorPitch = desiredPitchRate - actualPitchRate;
	Ipitch += iPitch*errorPitch; 
	pitchPulse = pPitch*errorPitch + dPitch*(errorPitch - last_errorPitch) + Ipitch;
	last_errorPitch = errorPitch; 

	// Bound PID Pitch output
	if( pitchPulse > pid_max_pitch)
	{
		pitchPulse = pid_max_pitch;
	}

	if( pitchPulse < -pid_max_pitch )
	{
		pitchPulse = -pid_max_pitch;
	}


	// Roll 
	errorRoll = desiredRollRate - actualRollRate;
	Iroll += iRoll*errorRoll;
	rollPulse = pRoll*errorRoll + dRoll*(errorRoll - last_errorRoll) + Iroll;
	last_errorRoll = errorRoll;

	// Bound PID Roll output 
	if( rollPulse > pid_max_roll)
	{
		rollPulse = pid_max_roll;
	}

	if( rollPulse < -pid_max_roll )
	{
		rollPulse = -pid_max_roll;
	}


	// Yaw
	errorYaw = desiredYawRate - actualYawRate;
	Iyaw += iYaw*errorYaw;
	yawPulse = pYaw*errorYaw +dYaw*(errorYaw - last_errorYaw) + Iyaw;
	last_errorYaw = errorYaw;

	// Bound PID YAW output  
	if( yawPulse > pid_max_yaw)
	{
		yawPulse = pid_max_yaw;
	}

	if( yawPulse < -pid_max_yaw )
	{
		yawPulse = -pid_max_yaw;
	}

	// Calculate pulses to motors
	escPulse1 = throttle_Pulse - rollPulse + pitchPulse + yawPulse;
	escPulse2 = throttle_Pulse - rollPulse - pitchPulse - yawPulse;
	escPulse3 = throttle_Pulse + rollPulse - pitchPulse + yawPulse; 
	escPulse4 = throttle_Pulse + rollPulse + pitchPulse - yawPulse;

	return;

}

/////////////////////////////////////////////////////////
// BOUND PULSE
/////////////////////////////////////////////////////////
// This is a safey feature in the code so the pulses sent to motors won't exced the
// the max or min of the motors. In this case the min of the motors is 1000 while the
// max is 2000. Also will activate the motors.

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
	if(activateMotor < 1100)
	{
		escPulse1 = 1000;
		escPulse2 = 1000;
		escPulse3 = 1000;
		escPulse4 = 1000;
	}

	return;
}
