//lift PID Variables
//float liftKp = 0.925;
//float liftKi = 0.015;
//float liftKd = 0.3;
int previousError = 0;
float errorSum = 0;
int powApplied = 0;

// Eli Wu Helped Me

//Lift
void lift(int power)
{
	motor[rtLift1] = power;
	motor[rtLift2] = power;
	motor[lftLift2] = power;
	motor[lftLift1] = power;
}

//Reset the PID Variables
void PIDreset()
{
	previousError = 0;
	errorSum = 0;
}

// Trajectory Calculator
void skyriseTrajectory()
{

}

//All Lift Stuff Should be 45* Degrees w/ a 5 Degree Error
//PID
void liftPID(int target)
{
	int error = target - SensorValue[rtLiftPot];
	int derivative = error - previousError;
	powApplied = (error*LIFT_kP + errorSum*LIFT_kI + derivative*LIFT_kD);
	lift(powApplied);
	previousError = error;
	errorSum += error;
	if(errorSum > 70)errorSum = 70;
	if(errorSum < -70)errorSum = -70;
}

//Task to set lift to skyrise height
task liftGoalPreset()
{
	while (true)
	{
		liftPID(1230);
		wait1Msec(5);
	}
}

bool liftDownTaskStarted = false;

task liftDownPreset()
{
	liftDownTaskStarted = true;
	while (SensorValue[rtLiftZero] != 1 && SensorValue[lftLiftZero] != 1)
	{
		lift(-127);
	}
	lift(-127);
	wait1Msec(300);
	lift(-10);
	liftDownTaskStarted = false;
	stopTask(liftDownPreset);
}

task liftMiddlePreset()
{
	while (true)
	{
		liftPID(800);
		wait1Msec(5);
	}
}

task liftSmallPreset()
{
	while (true)
	{
		liftPID(200);
		wait1Msec(5);
	}
}

task liftZero()
{
	while (true)
	{
		if (SensorValue[rtLiftZero] == 1)
		{
			nMotorEncoder[rtLift2] = 0;
		}
			wait1Msec(15);
	}
}

//Position hold variables; i is the loop counter, and position is the current position hold target
int position = 0;
bool savedPosition = false;
//bool isHangMode = false;

void liftOpControlComp()
{
	if (SensorValue[rtLiftZero] == 1)
	{
		SensorValue[rtLiftPot] = 0;
	}

	if (SensorValue[lftLiftZero] == 1)
	{
		SensorValue[lftLiftPot] = 0;
	}
	//Moves the lift full speed up
	if (vexRT[Btn6U] == 1)
	{
		stopTask(liftDownPreset);
		liftDownTaskStarted = false;
		if (SensorValue[rtLiftPot]>LIFT_SCORE_ANGLE)
		{
			lift(10);
		}
		else
		{
			lift(127);
			savedPosition = false;
		}
	}

	//Moves the lift full speed down
	if (vexRT[Btn6D] == 1)
	{
		stopTask(liftDownPreset);
		if (SensorValue[rtLiftPot]<30)
		{
			lift(-127);
		}
		else
		{
			lift(-127);
		}
		savedPosition = false;
	}
	if (vexRT[Btn7R] == 1)
	{
		stopTask(liftDownPreset);
		liftDownTaskStarted = false;
		savedPosition = true;
		position = 800;
	}
	if (vexRT[Btn7U] == 1)
	{
		stopTask(liftDownPreset);
		liftDownTaskStarted = false;
		savedPosition = true;
		position = 1000;
	}
	if (vexRT[Btn7D] == 1)
	{
		if (!liftDownTaskStarted)
		{
			startTask(liftDownPreset);
		}
	}
	//If no buttons pertaining to the lift are pressed, hold the lift at current position if not down, otherwise, power down lift
		if (vexRT[Btn6D] == 0 && vexRT[Btn6U] == 0 && vexRT[Btn8R] == 0 && !liftDownTaskStarted)
		{
			if (!savedPosition)
			{
				position = SensorValue[rtLiftPot];
				savedPosition = true;
			}

			if (SensorValue[rtLiftPot]>LIFT_SCORE_ANGLE)
			{
				position = MAX_LIFT_ANGLE;
			}

			if (SensorValue[rtLiftPot] <= 5)
			{
				lift(-10);
			}
			else
			{
				liftPID(position);
			}
		}
}
