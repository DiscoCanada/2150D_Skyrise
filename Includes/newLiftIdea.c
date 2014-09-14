int raisePrevError = 0;
int raiseIntegral = 0;
void raisePID(int position)
{
	int error = position - nMotorEncoder[rtdrive1];
	int derivative = error - raisePrevError;
	raiseIntegral = error + raiseIntegral;
	if(raiseIntegral > 70)raiseIntegral = 70;
	if(raiseIntegral < -70) raiseIntegral = -70;
	fdrive(error*2.5+raiseIntegral*0.1+derivative*1);
}

float potToDegree()
{
	return (SensorValue[pot]/21.276)-127.735;
}

float horizontalDistanceToDrive(int initValue)
{
	return initValue-28*cosDegrees(potToDegree());
}
float Y1 = 0;
float X2 = 0;
bool savedDrivePositionTrue = false;
bool hasDriveReset = false;
int savedDrivePositionL = 0;
int savedDrivePositionR = 0;
int test = 0;
void driveOpControlComp()
{
	if(abs(vexRT[Ch3])<=30 && abs(vexRT[Ch1]) <= 30&&(vexRT[Btn8U]==1||vexRT[Btn7L]==1||vexRT[Btn8D]==1))
		{
		if(vexRT[Btn8U]==1)
			{
			if(!hasDriveReset)
				{
				test = 28*cosDegrees(potToDegree());
				resetDriveSpecific((horizontalDistanceToDrive(test)/(2.75*PI))*(TPR_SPEED));
				hasDriveReset = true;
			}

			raisePID((horizontalDistanceToDrive(test)/(2.75*PI))*(TPR_SPEED));
		}
		if(vexRT[Btn8D]==1)
			{
			if(!hasDriveReset)
				{
				test = 28*cosDegrees(potToDegree());
				resetDriveSpecific((horizontalDistanceToDrive(test)/(2.75*PI))*(TPR_SPEED));
				hasDriveReset = true;
			}

			raisePID((horizontalDistanceToDrive(test)/(2.75*PI))*(TPR_SPEED)*1.1);
		}
		if(vexRT[Btn7L]==1)
			{
			if(!savedDrivePositionTrue)
				{
				hasDriveReset = false;
				savedDrivePositionL = nMotorEncoder[lftdrive1];
				savedDrivePositionR = nMotorEncoder[rtdrive1];
				savedDrivePositionTrue = true;
			}
			else brakePID(savedDrivePositionL,savedDrivePositionR);
		}
	}
	else
	{
		hasDriveReset = false;
		savedDrivePositionTrue = false;
		if(driveScheme == 0&&vexRT[Btn8U]==0&&vexRT[Btn8D]==0)
		{
			rDrive(vexRT[Ch3]-vexRT[Ch1]);
			lDrive(vexRT[Ch3]+vexRT[Ch1]);
		}
		else
		{
			rDrive(vexRT[Ch2]);
			lDrive(vexRT[Ch3]);
		}
	}
}
