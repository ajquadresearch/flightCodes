/////////////////////////////////////////////////////////
// INPUT
/////////////////////////////////////////////////////////
// Calculate the input signal needed to be sent the the PID function

// Global Variables 
extern volatile unsigned int roll_ratePulse;
extern volatile unsigned int pitch_ratePulse;
extern volatile unsigned int yaw_ratePulse;
extern float actualPitch, actualRoll;

// Local Variables
bool autoLevel = true;

// Pitch
int autoPitch; 
int desiredPitchRate;

// Roll 
int autoRoll;
int desiredRollRate;

// Yaw
int desiredYawRate;

void getInput()
{
	// Pitch bandwith of 16
	if(pitch_ratePulse > 1508)
	{
		desiredPitchRate = 1508 - pitch_ratePulse;
	}

	else if(pitch_ratePulse < 1492)
	{
		desiredPitchRate = 1492 - pitch_ratePulse;
	} 
	
	else
	{
		desiredPitchRate = 0;
	}

	// Roll bandwith of 16
	if(roll_ratePulse > 1508)
	{
		desiredRollRate = roll_ratePulse - 1508;
	}

	else if(roll_ratePulse < 1492)
	{
		desiredRollRate = roll_ratePulse - 1492;
	} 
	
	else
	{
		desiredRollRate = 0;
	}

	// Yaw bandwidth of 16
	if(yaw_ratePulse > 1508)
	{
		desiredYawRate = yaw_ratePulse - 1508;
	}

	else if(yaw_ratePulse < 1492)
	{
		desiredYawRate = yaw_ratePulse - 1492 ;
	} 
	
	else
	{
		desiredYawRate = 0;
	}

	desiredYawRate /= 3;

	// AutoLevel
	autoPitch = 15*actualPitch;
	autoRoll = 15*actualRoll;

	if (autoLevel == false)
	{
		autoPitch = 0;
		autoRoll = 0;
	}

	desiredPitchRate -= autoPitch;
	desiredPitchRate /= 3; 

	desiredRollRate -= autoRoll;
	desiredRollRate /= 3;

	return;
}