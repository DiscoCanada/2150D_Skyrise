int encoderTicks = 627.2;

int deadZone = 50;

float driveKp = 5.0;
float driveKi = 0.0;
float driveKd = 0.0;

float fwd=0;
float strafe=0;
float clockwise=0;

int ignoreDriveError = 1;
int driveErrorMax = 40;

float integralLF = 0;
float integralLB = 0;
float integralRF = 0;
float integralRB = 0;

int prevErrorLF = 0;
int prevErrorLB = 0;
int prevErrorRF = 0;
int prevErrorRB = 0;

//Reset All Encoders
void clearEncoders () {
	nMotorEncoder[rB] = 0;
	nMotorEncoder[rF] = 0;
	nMotorEncoder[lF] = 0;
	nMotorEncoder[lB] = 0;
}

//Set Arm Power
void armPo(int power) {
	motor[lru] = motor[lrd] = motor[llu] = motor[lld] = power;
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

//PID for arm
task armPID()
{

}
int errorRB;
int errorLB;
int errorRF;
int errorLF;
//PID for drive
task drivePID() {
	while(true) {
		int ticksRF = nMotorEncoder[rF];
		int ticksLB = nMotorEncoder[lB];
		int ticksLF = nMotorEncoder[lF];
		int ticksRB = nMotorEncoder[rB];

		int targetRB = fwd - clockwise + strafe;
		int targetLB = fwd + clockwise - strafe;
		int targetRF = fwd - clockwise - strafe;
		int targetLF = fwd + clockwise + strafe;

		errorRB = targetRB - ticksRB;
		errorLB = targetLB - ticksLB;
		errorRF = targetRF - ticksRF;
		errorLF = targetLF - ticksLF;

		integralRB = errorRB;
		integralLB = errorLB;
		integralRF = errorRF;
		integralLF = errorLF;

		if(abs(integralRB) < ignoreDriveError) {
			integralRB = 0;
		}
		if(abs(integralRB) > driveErrorMax) {
			integralRB = 0;
		}


		if(abs(integralRF) < ignoreDriveError) {
			integralRF = 0;
		}
		if(abs(integralRF) > driveErrorMax) {
			integralRF = 0;
		}


		if(abs(integralLB) < ignoreDriveError) {
			integralLB = 0;
		}
		if(abs(integralLB) > driveErrorMax) {
			integralLB = 0;
		}


		if(abs(integralLF) < ignoreDriveError) {
			integralLF = 0;
		}
		if(abs(integralLF) > driveErrorMax) {
			integralLF = 0;
		}

		int derivativeRF = errorRF - prevErrorRF;
		int derivativeRB = errorRB - prevErrorRB;
		int derivativeLF = errorLF - prevErrorLF;
		int derivativeLB = errorLB - prevErrorLB;

		prevErrorRF = errorRF;
		prevErrorRB = errorRB;
		prevErrorLB = errorLB;
		prevErrorLF = errorLF;
		writeDebugStreamLine("%f,%f",-1*(driveKp * errorLF + driveKi * integralLF + driveKd * derivativeLF), -1*(driveKp * errorRF + driveKi * integralRF + driveKd * derivativeRF));
		motor[rF] = (driveKp * errorRF + driveKi * integralRF + driveKd * derivativeRF);
		motor[rB] = -1*(driveKp * errorRB + driveKi * integralRB + driveKd * derivativeRB);
		motor[lF] = (driveKp * errorLF + driveKi * integralLF + driveKd * derivativeLF);
		motor[lB] = 1*(driveKp * errorLB + driveKi * integralLB + driveKd * derivativeLB);
	}
}

int battVoltage;
task usercontrol()
{
	// User control code here, inside the loop
	clearEncoders();
	startTask(drivePID);

	while (true)
	{
		battVoltage = nImmediateBatteryLevel/1000.0;

		/*if(abs(vexRT[Ch3]) < deadZone && abs(vexRT[Ch4]) < deadZone && abs(vexRT[Ch1]) < deadZone)
		{
			fwd = 0;
			strafe = 0;
			clockwise = 0;
		}
		else {
			motor[rB] = vexRT[Ch3] - vexRT[Ch1] + vexRT[Ch4];
			motor[lB] = vexRT[Ch3] + vexRT[Ch1] - vexRT[Ch4];
			motor[rF] = vexRT[Ch3] - vexRT[Ch1] - vexRT[Ch4];
			motor[lF] = vexRT[Ch3] + vexRT[Ch1] + vexRT[Ch4];
		}*/
		if(abs(vexRT[Ch3]) > deadZone) 
		{
			fwd = vexRT[Ch3];
		}
		else {
			fwd = 0;
			nMotorEncoder[lB] = 0;
			nMotorEncoder[lF] = 0;
			nMotorEncoder[rB] = 0;
			nMotorEncoder[rF] = 0;
		}		
		if(abs(vexRT[Ch4]) > deadZone)
		{
			strafe = vexRT[Ch4];
		}
		else {
			strafe = 0;
			nMotorEncoder[lB] = 0;
			nMotorEncoder[lF] = 0;
			nMotorEncoder[rB] = 0;
			nMotorEncoder[rF] = 0;
		}
		if(abs(vexRT[Ch1]) > deadZone)
		{
			clockwise =  vexRT[Ch1];
		}
		else {
			clockwise = 0;
			nMotorEncoder[lB] = 0;
			nMotorEncoder[lF] = 0;
			nMotorEncoder[rB] = 0;
			nMotorEncoder[rF] = 0;
		}
		/*
		motor[rB] = vexRT[Ch3] - vexRT[Ch1] + vexRT[Ch4];
		motor[lB] = vexRT[Ch3] + vexRT[Ch1] - vexRT[Ch4];
		motor[rF] = vexRT[Ch3] - vexRT[Ch1] - vexRT[Ch4];
		motor[lF] = vexRT[Ch3] + vexRT[Ch1] + vexRT[Ch4];*/

		if(vexRT[Btn6U] == 1) {
			armPo(127);
		}
		else if(vexRT[Btn6D] == 1) {
			armPo(-127);
		}
		else {
			armPo(0);
		}

		//Simple PID free drive for testing
		/*
		motor[rB] = fwd - clockwise + strafe;
		motor[lB] = fwd + clockwise - strafe;
		motor[rF] = fwd - clockwise - strafe;
		motor[lF] = fwd + clockwise + strafe;
		/**/
	}
}
