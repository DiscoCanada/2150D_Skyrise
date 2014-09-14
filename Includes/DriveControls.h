
int driveL = 0;
int driveR = 0;
int maxSpeed = 127;
void resetDrive()
{
	nMotorEncoder[rtDrive2] = 0;
	nMotorEncoder[lftDrive2] = 0;
	driveL = 0;
	driveR = 0;
}
//Drive the right motors at power
void Rdrive(int power)
{
	motor[rtDrive1] = power;
	motor[rtDrive2] = power;
}

//Drive the left motors at power
void Ldrive(int power)
{
	motor[lftDrive1] = -power;
	motor[lftDrive2] = -power;
}

//Fdrive is a misnomer... more like driveMotors at power
void fdrive(int power)
{
	Rdrive(power);
	Ldrive(power);
}

int iL = 0;
int iR = 0;
task DrivePID()
{
	nMotorEncoder[rtDrive2] = 0;
	nMotorEncoder[lftDrive2] = 0;
	int dLP = 0;
	int dRP = 0;
	int dL = 0;
	int dR = 0;
	while (bIfiAutonomousMode)
	{
		dL = driveL - nMotorEncoder[lftDrive2];
		dR = driveR - nMotorEncoder[rtDrive2];
		iL += dL;
		iR += dR;
		if (iL > 50) iL = 50;
		if (iR > 50) iR = 50;
		if (iL < -50) iL = -50;
		if (iR < -50) iR = -50;
		float left = dL*IME_kP + (dL - dLP)*IME_kD + iL*IME_kI;
		float right = dR*IME_kP + (dR - dRP)*IME_kD + iR*IME_kI;
		if (right>maxSpeed) right = maxSpeed;
		if (right<-maxSpeed) right = -maxSpeed;
		if (left>maxSpeed) left = maxSpeed;
		if (left<-maxSpeed) left = -maxSpeed;
		Rdrive(right);
		Ldrive(left);
		dLP = dL;
		dRP = dR;
		wait1Msec(50);
	}
}

void drive(int left, int right, /*int speed*/)
{
	//maxSpeed = speed;
	iL = 0;
	iR = 0;
	int encoderLeft = (left / (4.0*PI))*(TPR_TORQUE);
	int encoderRight = (right / (4.0*PI))*(TPR_TORQUE);
	driveL += encoderLeft;
	driveR += encoderRight;
}

void turnPID(float angle, int direction, /*int speed*/)
{
	//maxSpeed = speed;
	int turnTicks = angle*TURN_CONSTANT;
	if (direction == RIGHT)
	{
		drive(turnTicks, -turnTicks);
	}
	if (direction == LEFT)
	{
		drive(-turnTicks, turnTicks);
	}
}

//return the left encoder distance
float leftdistance()
{
	return ((float)nMotorEncoder[lftDrive2] / (TPR_TORQUE)) * (4.0*PI);
}

//return right encoder distance
float rightdistance()
{
	return ((float)nMotorEncoder[rtDrive2] / (TPR_TORQUE)) * (4.0*PI);
}

//distance variable for distancedrive
float dist;

//drive a certain distance based on encoders
void distancedrive(float inches, int direction, int speed)
{
	nMotorEncoder[rtDrive2] = 0;
	while (true)
	{
		dist = abs(rightdistance());
		if (direction == 1)
		{
			if (dist<inches)
			{
				fdrive(speed);
			}
			else
			{
				fdrive(0);
				break;
			}
		}
		if (direction == 0)
		{
			if (dist<inches)
			{
				fdrive(-speed);
			}
			else
			{
				fdrive(0);
				break;
			}
		}
		wait1Msec(25);
	}
}

void brakePID(int positionL, int positionR){
	int errorL = positionL - nMotorEncoder[lftDrive2];
	int errorR = positionR - nMotorEncoder[rtDrive2];
	Rdrive(errorR*1.1);
	Ldrive(errorL*1.1);
}

//Turn a certain number of degrees based on encoders
void turn(float degrees, int direction)
{
	float inches = (degrees / 20)*PI;
	inches = (degrees / 360)*DRIVE_CIRCLE_CIRCUMFERENCE;
	nMotorEncoder[rtDrive2] = 0;
	nMotorEncoder[lftDrive2] = 0;
	if (direction == RIGHT)
	{
		while (true){
			float leftdist = abs(leftdistance());
			float rightdist = abs(rightdistance());
				if (leftdist<inches)
				{
				Ldrive(127);
				}
			if (rightdist<inches)
			{
				Rdrive(-127);
			}
			else
			{
				fdrive(0);
				break;
			}
		}
	}
	if (direction == LEFT)
	{
		while (true)
		{
			float leftdist = abs(leftdistance());
			float rightdist = abs(rightdistance());
			if (leftdist<inches)
			{
				Ldrive(-127);
			}
			if (rightdist<inches)
			{
				Rdrive(127);
			}
			else
			{
				fdrive(0);
				break;
			}
		}
	}
}

//float Y1 = 0;
//float X2 = 0;
bool savedDrivePositionTrue = false;
int savedDrivePositionL = 0;
int savedDrivePositionR = 0;

void driveOpControlComp()
{
	if (abs(vexRT[Ch3]) <= 5 && abs(vexRT[Ch1]) <= 5 && vexRT[Btn7L] == 1)
	{
		if (!savedDrivePositionTrue)
		{
			savedDrivePositionL = nMotorEncoder[lftDrive2];
			savedDrivePositionR = nMotorEncoder[rtDrive2];
			savedDrivePositionTrue = true;
		}
		else brakePID(savedDrivePositionL, savedDrivePositionR);
	}
	else
	{
		savedDrivePositionTrue = false;
		Rdrive(vexRT[Ch3] - vexRT[Ch1]);
		Ldrive(vexRT[Ch3] + vexRT[Ch1]);
	}
}
