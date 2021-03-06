
int ignoreliftError = 1;
int liftErrorMax = 40;
float integralLift = 0;
int prevErrorLift = 0;
int errorLift = 0;

//Set Arm Power
void armPo(int power) {
	motor[aBotL] = motor[aBotR] = motor[aTopL] = motor[aTopR] = power;
}

int ticksLift;
int liftTarget = SensorValue[aBotL];
//PID for intake
task LiftPID()
{
	while(true)
	{
		ticksLift = SensorValue[aBotL];
		errorLift = liftTarget - ticksLift;
		integralLift = errorLift;

		if(abs(integralLift) < ignoreDriveError) {
			integralLift = 0;
		}
		if(abs(integralLift) > driveErrorMax) {
			integralLift = 0;
		}
		int derivativeLift = errorLift - prevErrorLift;

		prevErrorLift = errorLift;

		armPo(kP_LIFT * errorLift + kI_LIFT * integralLift + kD_LIFT * derivativelift);
	}
}
