/////////////////////////////////////////////////////////
// INPUT
/////////////////////////////////////////////////////////
// Calculate the input signal needed to be sent the the PID function

// Global Variables 
extern volatile unsigned int roll_ratePulse;
extern volatile unsigned int pitch_ratePulse;
extern volatile unsigned int yaw_ratePulse;
extern float pitch, roll;

// Local Variables


bool autoLevel = true;

// Pitch
int autoPitch; 
int inputPitch;

// Roll 
int autoRoll;
int inputRoll;

// Yaw
int inputYaw;

void getInput()
{
	// Pitch bandwith of 16
	if(pitch_ratePulse > 1508)
	{
		inputPitch = 1508 - pitch_ratePulse;
	}

	else if(pitch_ratePulse < 1492)
	{
		inputPitch = 1492 - pitch_ratePulse;
	} 
	
	else
	{
		inputPitch = 0;
	}

	// Roll bandwith of 16
	if(roll_ratePulse > 1508)
	{
		inputRoll = roll_ratePulse - 1508;
	}

	else if(roll_ratePulse < 1492)
	{
		inputRoll = roll_ratePulse - 1492;
	} 
	
	else
	{
		inputRoll = 0;
	}

	// Yaw bandwidth of 16
	if(yaw_ratePulse > 1508)
	{
		inputYaw = yaw_ratePulse - 1508;
	}

	else if(yaw_ratePulse < 1492)
	{
		inputYaw = yaw_ratePulse - 1492 ;
	} 
	
	else
	{
		inputYaw = 0;
	}

	inputYaw /= 3;

	// AutoLevel
	autoPitch = 15*pitch;
	autoRoll = 15*roll;

	if (autoLevel == false)
	{
		autoPitch = 0;
		autoRoll = 0;
	}

	inputPitch -= autoPitch;
	inputPitch /= 3; 

	inputRoll -= autoRoll;
	inputRoll /= 3;
}