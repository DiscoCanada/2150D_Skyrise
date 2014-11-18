#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    turnGyro,       sensorGyro)
#pragma config(Sensor, dgtl1,  s_claw,         sensorDigitalOut)
#pragma config(Sensor, I2C_1,  bl,             sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  br,             sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_3,  aBotL,          sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_4,  aBotR,          sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_5,  aTopR,          sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_6,  aTopL,          sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           intakeL,       tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           aTopL,         tmotorVex393_MC29, openLoop, encoderPort, I2C_6)
#pragma config(Motor,  port3,           aBotL,         tmotorVex393_MC29, openLoop, encoderPort, I2C_3)
#pragma config(Motor,  port4,           fl,            tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port5,           bl,            tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port6,           br,            tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port7,           fr,            tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           aBotR,         tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_4)
#pragma config(Motor,  port9,           aTopR,         tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_5)
#pragma config(Motor,  port10,          intakeR,       tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!

// TODO: Make sure code is ready for holonomic(Test on My Drivetrain)
// TODO: Make sure rerun/PID is not messed up.

int trueSpeed(int power) {
	int tsArray[128] =
	{
  	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 		20, 21, 21, 21, 22, 22, 22, 23, 24, 24,
 		25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
 		28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
 		33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
 		37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
 		41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
 		46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
 		52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
 		61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
 		71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
 		80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
 		88, 89, 89, 90, 90,127,127,127
	};
	return ((power>0)?1:-1)*tsArray[power*((power>0)?1:-1)];
}

void arcadeDrive(int forward, int strafe, int turn) //forward controls distance(for/back)
{
	motor[fl] = forward - strafe + turn;
	motor[bl] = forward - strafe - turn;
	motor[fr] = forward + strafe + turn;
	motor[br] = forward + strafe - turn;
}

/*
   /FL		FR\


	 \BL		BR/
	Starting off, motors are all "positive"
	going forward. The only time they arent,
	is when they are in reverse, in which all
	values will be negative. When factoring in
	the strafe value, the robot will move left/right
	based on the same principal of +127 & -127.
	Lastly, turning comes into play, when the
	robot's values are either all - or all positive,
	and then it will turn in place.
*/

#define LCD_LEFT 1
#define LCD_CENTER 2
#define LCD_RIGHT 4

int auto = 0;

task ProgramChooser()
{

	bool picked = false;

	bLCDBacklight = true;
  clearLCDLine(0);
	clearLCDLine(1);

	int max = 3;
	int min = 0;

	while(!picked) {

		if(nLCDButtons == LCD_LEFT) {
			while(nLCDButtons != 0) {}
			auto --;
		}
		if(nLCDButtons == LCD_RIGHT) {
			while(nLCDButtons != 0) {}
			auto ++;
		}
		if(nLCDButtons == LCD_CENTER) {
			while(nLCDButtons != 0) {}
			picked = true;
		}

		if(auto < min) auto = max;
		if(auto > max) auto = min;
		string autoName = "";
		switch(auto) {
			case 0: autoName = "S RED"; break;
			case 1: autoName = "N RED"; break;
			case 2: autoName = "S BLUE"; break;
			case 3: autoName = "N BLUE"; break;
		}
		displayLCDCenteredString(0,autoName);
		displayLCDCenteredString(1,"<            >");
	}

}

void pre_auton() {
	bStopTasksBetweenModes = false;
	startTask(ProgramChooser);
}

void tank(int left, int right) {
	motor[fl] = left;
	motor[bl] = left;
	motor[fr] = right;
	motor[br] = right;
}

void arm(int power) {

	motor[aBotL] = power;
	motor[aTopL] = power;
	motor[aBotR] = power;
	motor[aTopR] = power;
}

void intake() {
	motor[intakeL] = 127;
	motor[intakeR] = 127;
}

void outtake() {
	motor[intakeL] = -127;
	motor[intakeR] = -127;
}

void notake() {
	motor[intakeL] = 0;
	motor[intakeR] = 0;
}

void intakeP(int power) {
	motor[intakeL] = power;
	motor[intakeR] = power;
}

//--START AUTONOMOUS ARRAYS--
int autonomous1[150][6] = {{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
};
int autonomous2[150][6] = {{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
};
int autonomous3[150][6] = {{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
};
int autonomous4[150][6] = {{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
{0.000000,0.000000,-1.000000,0.000000,0,0}
};
//--END AUTONOMOUS ARRAYS--

int driveL = 0;
int driveR = 0;
int armTBot = 0;
int armTTop = 0;

// P loop for the arm's control.
task ArmPID()
{
	nMotorEncoder[aBotR] = 0;
	nMotorEncoder[aTopR] = 0;
	nMotorEncoder[aBotL] = 0;
	nMotorEncoder[aTopL] = 0;
	while(true)
	{
		if(abs(armTBot-nMotorEncoder[aBotR])>40 && abs(armTBot-nMotorEncoder[aBotL])>40 && abs(armTTop-nMotorEncoder[aTopL])>40 && abs(armTTop-nMotorEncoder[aTopR])>40)
		{
			if(nMotorEncoder[aBotR] < armTBot && nMotorEncoder[aBotL] < armTBot && nMotorEncoder[aTopL] < armTTop && nMotorEncoder[aTopR] < armTTop) {
				arm(127);
			}
			else
			{
				arm(-127);
			}
		}
		else
		{
			arm(17);
		}
		wait1Msec(50);
	}
}

// PD Loop Controls drive position
task DrivePID()
{
	nMotorEncoder[bl] = 0;
	nMotorEncoder[br] = 0;
	int dLP = 0;
	int dRP = 0;
	while(true)
	{
		int dL = driveL-nMotorEncoder[bl];
		int dR = driveR-nMotorEncoder[br];
		float kP = 0.3;
		float kD = 0.33;
		float left = dL*kP+(dL-dLP)*kD;
		float right = dR*kP+(dR-dRP)*kD;
		tank(left,right);
		dLP = dL;
		dRP = dR;
		wait1Msec(100);
	}
}
// Replay <-- One of the most amazing pices of software,
// which allows for the user to record their autonomous into an array
void replay1()
{
	startTask(DrivePID);
	startTask(ArmPID);
	for(int i = 0; i < 150; i ++)
	{
		driveL = autonomous1[i][0];
		driveR = autonomous1[i][1];
		armTBot = autonomous1[i][2];
		armTTop = autonomous1[i][3];
		intakeP(autonomous1[i][4]);
		SensorValue[s_claw] = autonomous1[i][5];
		wait1Msec(100);
	}
	stopTask(DrivePID);
	stopTask(ArmPID);
	tank(0,0);
}

void replay2()
{
	startTask(DrivePID);
	startTask(ArmPID);
	for(int i = 0; i < 150; i ++)
	{
		driveL = autonomous2[i][0];
		driveR = autonomous2[i][1];
		armTBot = autonomous2[i][2];
		armTTop = autonomous2[i][3];
		intakeP(autonomous2[i][4]);
		SensorValue[s_claw] = autonomous2[i][5];
		wait1Msec(100);
	}
	stopTask(DrivePID);
	stopTask(ArmPID);
	tank(0,0);
}

void replay3()
{
	startTask(DrivePID);
	startTask(ArmPID);
	for(int i = 0; i < 150; i ++)
	{
		driveL = autonomous3[i][0];
		driveR = autonomous3[i][1];
		armTBot = autonomous3[i][2];
		armTTop = autonomous3[i][3];
		intakeP(autonomous3[i][4]);
		SensorValue[s_claw] = autonomous3[i][5];
		wait1Msec(100);
	}
	stopTask(DrivePID);
	stopTask(ArmPID);
	tank(0,0);
}

void replay4()
{
	startTask(DrivePID);
	startTask(ArmPID);
	for(int i = 0; i < 150; i ++)
	{
		driveL = autonomous4[i][0];
		driveR = autonomous4[i][1];
		armTBot = autonomous4[i][2];
		armTTop = autonomous4[i][3];
		intakeP(autonomous4[i][4]);
		SensorValue[s_claw] = autonomous4[i][5];
		wait1Msec(100);
	}
	stopTask(DrivePID);
	stopTask(ArmPID);
	tank(0,0);
}

task autonomous()
{
	switch(auto)
	{
		case 0: replay1(); break;
		case 1: replay2(); break;
		case 2: replay3(); break;
		case 3: replay4(); break;
	}

}

bool braking = false;
bool recording = false;

task ActiveBrake()
{
	braking = true;
	tank(0,0);
	wait1Msec(500);
	int dLP = nMotorEncoder[bl];
	int dRP = nMotorEncoder[br];
	while(true)
	{
		int dL = dLP-nMotorEncoder[bl];
		int dR = dRP-nMotorEncoder[br];
		float kP = 0.4;
		float left = dL*kP;
		float right = dR*kP;
		if(!recording) tank(left,right);
		wait1Msec(50);
	}
}

// This records the autonomous and prints it to a debug table.
task Record()
{
	nMotorEncoder[bl] = 0;
	nMotorEncoder[br] = 0;
	nMotorEncoder[aBotR] = 0;
	nMotorEncoder[aTopR] = 0;
	while(motor[bl] == 0
		&& motor[br] == 0
		&& nMotorEncoder[aBotR] == 0
		&& nMotorEncoder[aTopR] == 0
		&& motor[intakeL] == 0) { wait1Msec(1); }
	writeDebugStream("int autonomous[150][5] = {");
	for(int i = 0; i < 150; i ++)
	{
		int dL = nMotorEncoder[bl]; // Left Side
		int dR = nMotorEncoder[br]; // Right Side
		int aPB = nMotorEncoder[aBotR]; // Bottom R
		int aPT = nMotorEncoder[aTopR]; // Top R
		int iP = motor[intakeL]; // Intake Speed
		int cP = SensorValue[s_claw]; // Sensor Value
		if(i > 0) writeDebugStream(",");
		writeDebugStream("{%f,%f,%f,%f,%f,%d}\n",dL,dR,aPB,aPT,iP,cP);
		autonomous1[i][0] = dL;
		autonomous1[i][1] = dR;
		autonomous1[i][2] = aPB;
		autonomous1[i][3] = aPT;
		autonomous1[i][4] = iP;
		autonomous1[i][5] = cP;
		wait1Msec(100);
	}
	recording = false;
	writeDebugStream("}");
}

// Controls user shiz
task usercontrol()
{
	while(true)
		{
		if(trueSpeed(vexRT[Ch3]) == 0 && trueSpeed(vexRT[Ch1]) == 0) {
			if(!braking) startTask(ActiveBrake);
		}
		else
		{
			if(braking) stopTask(ActiveBrake);
			arcadeDrive(trueSpeed(vexRT[Ch3]),trueSpeed(vexRT[Ch4]),trueSpeed(vexRT[Ch1]));
			braking = false;
		}

		int armP = 16;
		if(vexRT[Btn5U]) {
			armP = 127;
		}else
		if(vexRT[Btn5D]) {
			armP = -127;
		}else
		if(abs(nMotorEncoder[aBotR]) < 40) {
			armP = -16;
		}
		motor[aBotL] = armP;
		motor[aTopL] = armP;
		motor[aBotR] = armP;
		motor[aTopR] = armP;

		int intakeP = 0;
		if(vexRT[Btn6U])
		{
			intakeP = 127;
		}
		if(vexRT[Btn6D])
		{
			intakeP = -127;
		}
		motor[intakeL] = intakeP;
		motor[intakeR] = intakeP;

		SensorValue[s_claw] = vexRT[Btn8D];

		displayLCDCenteredString(0,"Dark Matter");
		displayLCDCenteredString(1,"2150D");

		if(vexRT[Btn8U])
		{
			recording = true;
			startTask(Record);
		}

		if(vexRT[Btn7D])
		{
			nMotorEncoder[aBotR] = 0;
			nMotorEncoder[aTopR] = 0;
		}

		if(vexRT[Btn7U])
		{
			stopTask(ActiveBrake);
			//replay1();
		}

	}
}
