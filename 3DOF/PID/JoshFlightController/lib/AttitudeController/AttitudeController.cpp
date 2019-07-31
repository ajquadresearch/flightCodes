//-----------------------------------------------------------------------------------------
// THIS SUBROUTINE TAKES THE DESIRED AND ACTUAL RATE TO DETERMINE THE ROTOR PULSE USING PID
// THE PULSE OUTPUT IS THEN BOUNDED BY A MIN AND MAX VALUE 
//------------------------------------------------------------------------------------------
// Global Variables 
extern int desiredPitchRate, desiredRollRate, desiredYawRate;
extern float actualPitchRate, actualRollRate, actualYawRate;
extern volatile unsigned int throttle_Pulse, activateMotor;


// Variables
int escPulse1 = 0, escPulse2 = 0, escPulse3 = 0, escPulse4 = 0;

// Pitch 
int errorPitch = 0, errorRoll = 0, errorYaw = 0;
int pitchPulse = 0, rollPulse = 0, yawPulse = 0;
int last_errorPitch = 0, last_errorRoll = 0, last_errorYaw = 0;
int pid_max_pitch = 250, pid_max_roll = 250, pid_max_yaw = 250;
float Ipitch = 0, Iroll = 0, Iyaw = 0;


// Gains
int pPitch = 2;
int dPitch = 18;
int iPitch = .02;

int pRoll = pPitch;
int dRoll = dPitch;
int iRoll = iPitch;

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
