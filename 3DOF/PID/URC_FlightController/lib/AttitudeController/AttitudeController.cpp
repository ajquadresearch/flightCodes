/////////////////////////////////////////////////////////
// PID 
/////////////////////////////////////////////////////////
// Controller corrects for angular rates to converge to desired hand held contoller rates


// Global Variables 
extern int inputPitch, inputRoll, inputYaw;
extern float pitch_rate, roll_rate, yaw_rate;
extern volatile unsigned int throttle_Pulse;



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

void getPID()
{

	// Pitch
	errorPitch = inputPitch - pitch_rate;
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
	errorRoll = inputRoll - roll_rate;
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
	errorYaw = inputYaw - yaw_rate;
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

}