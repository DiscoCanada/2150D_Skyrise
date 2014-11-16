int rfMult = -1;
int lfMult = -1;
int encoderTicks = HS_TPR;

int deadZone = 15;

float driveKp = 1.0;
float driveKi = 0.0;
float driveKd = 0.0;

float fwd=0;
float strafe=0;
float clockwise=0;

int ignoreDriveError = 1;
int driveErrorMax = 40;

float integralBR = 0;
float integralBL = 0;

int prevErrorBR = 0;
int prevErrorBL = 0;

int tickIncrease = 20;
//Reset All Encoder
float lx() {
	return (vexRT[ch4]^3)/(127^2);
}
float ly() {
	return (vexRT[ch3]^3)/(127^2);
}
float rx() {
	return (vexRT[ch1]^3)/(127^2);
}
void clearEncoders () {
	nMotorEncoder[br] = 0;
	nMotorEncoder[bl] = 0;
}

//Set target value in ticks
void driveTicks(int fwdDrive, int rDrive, int rotateDrive)
{
	fwd = fwdDrive*sqrt(2);
	strafe = rDrive*sqrt(2);
	clockwise = rotateDrive*sqrt(2);
}

//Set drive target in inches
void driveIn(int fwdDrive, int rDrive, int rotateDrive)
{
	float fakefwd = (fwdDrive*sqrt(2)*4*PI)/encoderTicks;
	float fakeright = (rDrive*sqrt(2)*4*PI)/encoderTicks;
	fwd = fakefwd;
	strafe = fakeright;
	clockwise = rotateDrive;
}

int errorBR;
int errorBL;

task drivePID()
{
	while(true)
	{
		int ticksBR = nMotorEncoder[br];
		int ticksBL = nMotorEncoder[bl];

		int targetBR = fwd - clockwise + strafe;
		int targetBL = fwd + clockwise - strafe;

		errorBR = targetBR + ticksBR;
		errorBL = targetBL - ticksBL;

		integralBR = errorBR;
		integralBL = errorBL;

		if(abs(integralBR) < ignoreDriveError) {
			integralBR = 0;
		}
		if(abs(integralBR) > driveErrorMax) {
			integralBR = 0;
		}


		if(abs(integralBL) < ignoreDriveError) {
			integralBL = 0;
		}
		if(abs(integralBL) > driveErrorMax) {
			integralBL = 0;
		}

		int derivativeBR = errorBR - prevErrorBR;
		int derivativeBL = errorBL - prevErrorBL;

		prevErrorBR = errorBR;
		prevErrorBL = errorBL;

		writeDebugStreamLine("%f,%f",-1*(kP_DRIVE * errorBR + kI_DRIVE * integralBR + kD_DRIVE * derivativeBR), -1*(kP_DRIVE * errorBL + kI_DRIVE * integralBL + kD_DRIVE * derivativeBL));

		motor[br] = (kP_DRIVE * errorBR + kI_DRIVE * integralBR + kD_DRIVE * derivativeBR);
		motor[bl] = (kP_DRIVE * errorBL + kI_DRIVE * integralBL + kD_DRIVE * derivativeBL);
		motor[fl] = (kP_DRIVE * errorBL + kI_DRIVE * integralBL + kD_DRIVE * derivativeBL);
		motor[fr] = (kP_DRIVE * errorBR + kI_DRIVE * integralBR + kD_DRIVE * derivativeBR);

		wait1Msec(TIME_DELAY);
	}
}

//float average = abs((nMotorEncoder[bl] + nMotorEncoder[br]) / 2);
//float ticksToInches = WHEEL_CIRCUMFERENCE4/HS_TPR;
//float distTrav = sqrt(2)*average*ticksToInches;

float leftdistance()
{
	return ((float)nMotorEncoder[bl] / (HS_TPR)) * (4.0*PI) * sqrt(2);
}

float rightdistance()
{
	return ((float)nMotorEncoder[br] / (HS_TPR)) * (4.0*PI) * sqrt(2);
}
