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
#pragma config(Motor,  port4,           fl,            tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           bl,            tmotorVex393HighSpeed_MC29, openLoop, reversed, encoderPort, I2C_1)
#pragma config(Motor,  port6,           br,            tmotorVex393HighSpeed_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port7,           fr,            tmotorVex393HighSpeed_MC29, openLoop)
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

void arcadeDrive(int forward, int right, int slightright) {
	motor[fl] = forward + right + slightright/2;
	motor[bl] =  forward + right + slightright/2;
	motor[fr] = forward - right - slightright/2;
	motor[br] =  forward - right - slightright/2;
}

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
int autonomous1[150][5] = {{0.000000,0.000000,-1.000000,0.000000,0}
,{0.000000,0.000000,69.000000,0.000000,0}
,{0.000000,0.000000,150.000000,0.000000,0}
,{0.000000,0.000000,210.000000,0.000000,0}
,{0.000000,0.000000,266.000000,0.000000,0}
,{0.000000,0.000000,332.000000,0.000000,0}
,{0.000000,0.000000,375.000000,0.000000,0}
,{0.000000,0.000000,375.000000,127.000000,0}
,{0.000000,0.000000,356.000000,127.000000,0}
,{0.000000,0.000000,262.000000,127.000000,0}
,{0.000000,0.000000,162.000000,127.000000,0}
,{0.000000,0.000000,64.000000,127.000000,0}
,{0.000000,0.000000,-35.000000,127.000000,0}
,{1.000000,0.000000,-96.000000,127.000000,0}
,{1.000000,0.000000,-79.000000,127.000000,0}
,{1.000000,0.000000,-58.000000,127.000000,0}
,{1.000000,0.000000,-22.000000,127.000000,0}
,{1.000000,0.000000,70.000000,127.000000,0}
,{1.000000,0.000000,152.000000,0.000000,0}
,{1.000000,0.000000,214.000000,0.000000,0}
,{1.000000,0.000000,272.000000,0.000000,0}
,{1.000000,0.000000,339.000000,0.000000,0}
,{1.000000,0.000000,405.000000,0.000000,0}
,{1.000000,0.000000,452.000000,0.000000,0}
,{11.000000,12.000000,453.000000,0.000000,0}
,{56.000000,56.000000,453.000000,0.000000,0}
,{88.000000,84.000000,453.000000,0.000000,0}
,{144.000000,139.000000,453.000000,0.000000,0}
,{216.000000,195.000000,453.000000,0.000000,0}
,{293.000000,201.000000,453.000000,0.000000,0}
,{357.000000,201.000000,453.000000,0.000000,0}
,{413.000000,181.000000,453.000000,0.000000,0}
,{458.000000,140.000000,453.000000,0.000000,0}
,{504.000000,97.000000,453.000000,0.000000,0}
,{568.000000,72.000000,453.000000,0.000000,0}
,{641.000000,112.000000,453.000000,0.000000,0}
,{716.000000,163.000000,453.000000,0.000000,0}
,{800.000000,229.000000,453.000000,0.000000,0}
,{888.000000,310.000000,453.000000,0.000000,0}
,{972.000000,392.000000,453.000000,0.000000,0}
,{1037.000000,451.000000,453.000000,0.000000,0}
,{1079.000000,490.000000,453.000000,0.000000,0}
,{1102.000000,532.000000,453.000000,0.000000,0}
,{1102.000000,540.000000,458.000000,0.000000,0}
,{1086.000000,517.000000,460.000000,0.000000,0}
,{1054.000000,485.000000,460.000000,0.000000,0}
,{1003.000000,433.000000,460.000000,0.000000,0}
,{943.000000,372.000000,460.000000,0.000000,0}
,{873.000000,309.000000,460.000000,0.000000,0}
,{796.000000,266.000000,460.000000,0.000000,0}
,{720.000000,269.000000,460.000000,0.000000,0}
,{663.000000,284.000000,459.000000,0.000000,0}
,{624.000000,308.000000,459.000000,0.000000,0}
,{590.000000,342.000000,459.000000,0.000000,0}
,{565.000000,401.000000,459.000000,0.000000,0}
,{555.000000,469.000000,459.000000,0.000000,0}
,{609.000000,536.000000,459.000000,0.000000,0}
,{670.000000,606.000000,459.000000,0.000000,0}
,{741.000000,678.000000,459.000000,0.000000,0}
,{826.000000,755.000000,459.000000,0.000000,0}
,{914.000000,831.000000,459.000000,0.000000,0}
,{997.000000,847.000000,459.000000,0.000000,0}
,{1056.000000,846.000000,459.000000,0.000000,0}
,{1111.000000,828.000000,459.000000,0.000000,0}
,{1152.000000,793.000000,459.000000,0.000000,0}
,{1195.000000,758.000000,459.000000,0.000000,0}
,{1236.000000,721.000000,459.000000,0.000000,0}
,{1269.000000,685.000000,459.000000,0.000000,0}
,{1299.000000,705.000000,459.000000,0.000000,0}
,{1337.000000,731.000000,464.000000,0.000000,0}
,{1388.000000,778.000000,495.000000,0.000000,0}
,{1450.000000,839.000000,524.000000,0.000000,0}
,{1517.000000,903.000000,566.000000,0.000000,0}
,{1588.000000,974.000000,613.000000,-127.000000,0}
,{1661.000000,1044.000000,659.000000,-127.000000,0}
,{1732.000000,1117.000000,707.000000,-127.000000,0}
,{1804.000000,1191.000000,726.000000,-127.000000,0}
,{1878.000000,1261.000000,726.000000,-127.000000,0}
,{1937.000000,1321.000000,726.000000,-127.000000,0}
,{1975.000000,1358.000000,726.000000,-127.000000,0}
,{2003.000000,1392.000000,726.000000,-127.000000,0}
,{2041.000000,1430.000000,726.000000,-127.000000,0}
,{2061.000000,1438.000000,726.000000,-127.000000,0}
,{2063.000000,1438.000000,726.000000,-127.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
};
int autonomous2[150][5] = {{0.000000,0.000000,-1.000000,0.000000,0}
,{0.000000,0.000000,69.000000,0.000000,0}
,{0.000000,0.000000,150.000000,0.000000,0}
,{0.000000,0.000000,210.000000,0.000000,0}
,{0.000000,0.000000,266.000000,0.000000,0}
,{0.000000,0.000000,332.000000,0.000000,0}
,{0.000000,0.000000,375.000000,0.000000,0}
,{0.000000,0.000000,375.000000,127.000000,0}
,{0.000000,0.000000,356.000000,127.000000,0}
,{0.000000,0.000000,262.000000,127.000000,0}
,{0.000000,0.000000,162.000000,127.000000,0}
,{0.000000,0.000000,64.000000,127.000000,0}
,{0.000000,0.000000,-35.000000,127.000000,0}
,{1.000000,0.000000,-96.000000,127.000000,0}
,{1.000000,0.000000,-79.000000,127.000000,0}
,{1.000000,0.000000,-58.000000,127.000000,0}
,{1.000000,0.000000,-22.000000,127.000000,0}
,{1.000000,0.000000,70.000000,127.000000,0}
,{1.000000,0.000000,152.000000,0.000000,0}
,{1.000000,0.000000,214.000000,0.000000,0}
,{1.000000,0.000000,272.000000,0.000000,0}
,{1.000000,0.000000,339.000000,0.000000,0}
,{1.000000,0.000000,405.000000,0.000000,0}
,{1.000000,0.000000,452.000000,0.000000,0}
,{11.000000,12.000000,453.000000,0.000000,0}
,{56.000000,56.000000,453.000000,0.000000,0}
,{88.000000,84.000000,453.000000,0.000000,0}
,{144.000000,139.000000,453.000000,0.000000,0}
,{216.000000,195.000000,453.000000,0.000000,0}
,{293.000000,201.000000,453.000000,0.000000,0}
,{357.000000,201.000000,453.000000,0.000000,0}
,{413.000000,181.000000,453.000000,0.000000,0}
,{458.000000,140.000000,453.000000,0.000000,0}
,{504.000000,97.000000,453.000000,0.000000,0}
,{568.000000,72.000000,453.000000,0.000000,0}
,{641.000000,112.000000,453.000000,0.000000,0}
,{716.000000,163.000000,453.000000,0.000000,0}
,{800.000000,229.000000,453.000000,0.000000,0}
,{888.000000,310.000000,453.000000,0.000000,0}
,{972.000000,392.000000,453.000000,0.000000,0}
,{1037.000000,451.000000,453.000000,0.000000,0}
,{1079.000000,490.000000,453.000000,0.000000,0}
,{1102.000000,532.000000,453.000000,0.000000,0}
,{1102.000000,540.000000,458.000000,0.000000,0}
,{1086.000000,517.000000,460.000000,0.000000,0}
,{1054.000000,485.000000,460.000000,0.000000,0}
,{1003.000000,433.000000,460.000000,0.000000,0}
,{943.000000,372.000000,460.000000,0.000000,0}
,{873.000000,309.000000,460.000000,0.000000,0}
,{796.000000,266.000000,460.000000,0.000000,0}
,{720.000000,269.000000,460.000000,0.000000,0}
,{663.000000,284.000000,459.000000,0.000000,0}
,{624.000000,308.000000,459.000000,0.000000,0}
,{590.000000,342.000000,459.000000,0.000000,0}
,{565.000000,401.000000,459.000000,0.000000,0}
,{555.000000,469.000000,459.000000,0.000000,0}
,{609.000000,536.000000,459.000000,0.000000,0}
,{670.000000,606.000000,459.000000,0.000000,0}
,{741.000000,678.000000,459.000000,0.000000,0}
,{826.000000,755.000000,459.000000,0.000000,0}
,{914.000000,831.000000,459.000000,0.000000,0}
,{997.000000,847.000000,459.000000,0.000000,0}
,{1056.000000,846.000000,459.000000,0.000000,0}
,{1111.000000,828.000000,459.000000,0.000000,0}
,{1152.000000,793.000000,459.000000,0.000000,0}
,{1195.000000,758.000000,459.000000,0.000000,0}
,{1236.000000,721.000000,459.000000,0.000000,0}
,{1269.000000,685.000000,459.000000,0.000000,0}
,{1299.000000,705.000000,459.000000,0.000000,0}
,{1337.000000,731.000000,464.000000,0.000000,0}
,{1388.000000,778.000000,495.000000,0.000000,0}
,{1450.000000,839.000000,524.000000,0.000000,0}
,{1517.000000,903.000000,566.000000,0.000000,0}
,{1588.000000,974.000000,613.000000,-127.000000,0}
,{1661.000000,1044.000000,659.000000,-127.000000,0}
,{1732.000000,1117.000000,707.000000,-127.000000,0}
,{1804.000000,1191.000000,726.000000,-127.000000,0}
,{1878.000000,1261.000000,726.000000,-127.000000,0}
,{1937.000000,1321.000000,726.000000,-127.000000,0}
,{1975.000000,1358.000000,726.000000,-127.000000,0}
,{2003.000000,1392.000000,726.000000,-127.000000,0}
,{2041.000000,1430.000000,726.000000,-127.000000,0}
,{2061.000000,1438.000000,726.000000,-127.000000,0}
,{2063.000000,1438.000000,726.000000,-127.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
};
int autonomous3[150][5] = {{0.000000,0.000000,-1.000000,0.000000,0}
,{0.000000,0.000000,69.000000,0.000000,0}
,{0.000000,0.000000,150.000000,0.000000,0}
,{0.000000,0.000000,210.000000,0.000000,0}
,{0.000000,0.000000,266.000000,0.000000,0}
,{0.000000,0.000000,332.000000,0.000000,0}
,{0.000000,0.000000,375.000000,0.000000,0}
,{0.000000,0.000000,375.000000,127.000000,0}
,{0.000000,0.000000,356.000000,127.000000,0}
,{0.000000,0.000000,262.000000,127.000000,0}
,{0.000000,0.000000,162.000000,127.000000,0}
,{0.000000,0.000000,64.000000,127.000000,0}
,{0.000000,0.000000,-35.000000,127.000000,0}
,{1.000000,0.000000,-96.000000,127.000000,0}
,{1.000000,0.000000,-79.000000,127.000000,0}
,{1.000000,0.000000,-58.000000,127.000000,0}
,{1.000000,0.000000,-22.000000,127.000000,0}
,{1.000000,0.000000,70.000000,127.000000,0}
,{1.000000,0.000000,152.000000,0.000000,0}
,{1.000000,0.000000,214.000000,0.000000,0}
,{1.000000,0.000000,272.000000,0.000000,0}
,{1.000000,0.000000,339.000000,0.000000,0}
,{1.000000,0.000000,405.000000,0.000000,0}
,{1.000000,0.000000,452.000000,0.000000,0}
,{11.000000,12.000000,453.000000,0.000000,0}
,{56.000000,56.000000,453.000000,0.000000,0}
,{88.000000,84.000000,453.000000,0.000000,0}
,{144.000000,139.000000,453.000000,0.000000,0}
,{216.000000,195.000000,453.000000,0.000000,0}
,{293.000000,201.000000,453.000000,0.000000,0}
,{357.000000,201.000000,453.000000,0.000000,0}
,{413.000000,181.000000,453.000000,0.000000,0}
,{458.000000,140.000000,453.000000,0.000000,0}
,{504.000000,97.000000,453.000000,0.000000,0}
,{568.000000,72.000000,453.000000,0.000000,0}
,{641.000000,112.000000,453.000000,0.000000,0}
,{716.000000,163.000000,453.000000,0.000000,0}
,{800.000000,229.000000,453.000000,0.000000,0}
,{888.000000,310.000000,453.000000,0.000000,0}
,{972.000000,392.000000,453.000000,0.000000,0}
,{1037.000000,451.000000,453.000000,0.000000,0}
,{1079.000000,490.000000,453.000000,0.000000,0}
,{1102.000000,532.000000,453.000000,0.000000,0}
,{1102.000000,540.000000,458.000000,0.000000,0}
,{1086.000000,517.000000,460.000000,0.000000,0}
,{1054.000000,485.000000,460.000000,0.000000,0}
,{1003.000000,433.000000,460.000000,0.000000,0}
,{943.000000,372.000000,460.000000,0.000000,0}
,{873.000000,309.000000,460.000000,0.000000,0}
,{796.000000,266.000000,460.000000,0.000000,0}
,{720.000000,269.000000,460.000000,0.000000,0}
,{663.000000,284.000000,459.000000,0.000000,0}
,{624.000000,308.000000,459.000000,0.000000,0}
,{590.000000,342.000000,459.000000,0.000000,0}
,{565.000000,401.000000,459.000000,0.000000,0}
,{555.000000,469.000000,459.000000,0.000000,0}
,{609.000000,536.000000,459.000000,0.000000,0}
,{670.000000,606.000000,459.000000,0.000000,0}
,{741.000000,678.000000,459.000000,0.000000,0}
,{826.000000,755.000000,459.000000,0.000000,0}
,{914.000000,831.000000,459.000000,0.000000,0}
,{997.000000,847.000000,459.000000,0.000000,0}
,{1056.000000,846.000000,459.000000,0.000000,0}
,{1111.000000,828.000000,459.000000,0.000000,0}
,{1152.000000,793.000000,459.000000,0.000000,0}
,{1195.000000,758.000000,459.000000,0.000000,0}
,{1236.000000,721.000000,459.000000,0.000000,0}
,{1269.000000,685.000000,459.000000,0.000000,0}
,{1299.000000,705.000000,459.000000,0.000000,0}
,{1337.000000,731.000000,464.000000,0.000000,0}
,{1388.000000,778.000000,495.000000,0.000000,0}
,{1450.000000,839.000000,524.000000,0.000000,0}
,{1517.000000,903.000000,566.000000,0.000000,0}
,{1588.000000,974.000000,613.000000,-127.000000,0}
,{1661.000000,1044.000000,659.000000,-127.000000,0}
,{1732.000000,1117.000000,707.000000,-127.000000,0}
,{1804.000000,1191.000000,726.000000,-127.000000,0}
,{1878.000000,1261.000000,726.000000,-127.000000,0}
,{1937.000000,1321.000000,726.000000,-127.000000,0}
,{1975.000000,1358.000000,726.000000,-127.000000,0}
,{2003.000000,1392.000000,726.000000,-127.000000,0}
,{2041.000000,1430.000000,726.000000,-127.000000,0}
,{2061.000000,1438.000000,726.000000,-127.000000,0}
,{2063.000000,1438.000000,726.000000,-127.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
};
int autonomous4[150][5] = {{0.000000,0.000000,-1.000000,0.000000,0}
,{0.000000,0.000000,69.000000,0.000000,0}
,{0.000000,0.000000,150.000000,0.000000,0}
,{0.000000,0.000000,210.000000,0.000000,0}
,{0.000000,0.000000,266.000000,0.000000,0}
,{0.000000,0.000000,332.000000,0.000000,0}
,{0.000000,0.000000,375.000000,0.000000,0}
,{0.000000,0.000000,375.000000,127.000000,0}
,{0.000000,0.000000,356.000000,127.000000,0}
,{0.000000,0.000000,262.000000,127.000000,0}
,{0.000000,0.000000,162.000000,127.000000,0}
,{0.000000,0.000000,64.000000,127.000000,0}
,{0.000000,0.000000,-35.000000,127.000000,0}
,{1.000000,0.000000,-96.000000,127.000000,0}
,{1.000000,0.000000,-79.000000,127.000000,0}
,{1.000000,0.000000,-58.000000,127.000000,0}
,{1.000000,0.000000,-22.000000,127.000000,0}
,{1.000000,0.000000,70.000000,127.000000,0}
,{1.000000,0.000000,152.000000,0.000000,0}
,{1.000000,0.000000,214.000000,0.000000,0}
,{1.000000,0.000000,272.000000,0.000000,0}
,{1.000000,0.000000,339.000000,0.000000,0}
,{1.000000,0.000000,405.000000,0.000000,0}
,{1.000000,0.000000,452.000000,0.000000,0}
,{11.000000,12.000000,453.000000,0.000000,0}
,{56.000000,56.000000,453.000000,0.000000,0}
,{88.000000,84.000000,453.000000,0.000000,0}
,{144.000000,139.000000,453.000000,0.000000,0}
,{216.000000,195.000000,453.000000,0.000000,0}
,{293.000000,201.000000,453.000000,0.000000,0}
,{357.000000,201.000000,453.000000,0.000000,0}
,{413.000000,181.000000,453.000000,0.000000,0}
,{458.000000,140.000000,453.000000,0.000000,0}
,{504.000000,97.000000,453.000000,0.000000,0}
,{568.000000,72.000000,453.000000,0.000000,0}
,{641.000000,112.000000,453.000000,0.000000,0}
,{716.000000,163.000000,453.000000,0.000000,0}
,{800.000000,229.000000,453.000000,0.000000,0}
,{888.000000,310.000000,453.000000,0.000000,0}
,{972.000000,392.000000,453.000000,0.000000,0}
,{1037.000000,451.000000,453.000000,0.000000,0}
,{1079.000000,490.000000,453.000000,0.000000,0}
,{1102.000000,532.000000,453.000000,0.000000,0}
,{1102.000000,540.000000,458.000000,0.000000,0}
,{1086.000000,517.000000,460.000000,0.000000,0}
,{1054.000000,485.000000,460.000000,0.000000,0}
,{1003.000000,433.000000,460.000000,0.000000,0}
,{943.000000,372.000000,460.000000,0.000000,0}
,{873.000000,309.000000,460.000000,0.000000,0}
,{796.000000,266.000000,460.000000,0.000000,0}
,{720.000000,269.000000,460.000000,0.000000,0}
,{663.000000,284.000000,459.000000,0.000000,0}
,{624.000000,308.000000,459.000000,0.000000,0}
,{590.000000,342.000000,459.000000,0.000000,0}
,{565.000000,401.000000,459.000000,0.000000,0}
,{555.000000,469.000000,459.000000,0.000000,0}
,{609.000000,536.000000,459.000000,0.000000,0}
,{670.000000,606.000000,459.000000,0.000000,0}
,{741.000000,678.000000,459.000000,0.000000,0}
,{826.000000,755.000000,459.000000,0.000000,0}
,{914.000000,831.000000,459.000000,0.000000,0}
,{997.000000,847.000000,459.000000,0.000000,0}
,{1056.000000,846.000000,459.000000,0.000000,0}
,{1111.000000,828.000000,459.000000,0.000000,0}
,{1152.000000,793.000000,459.000000,0.000000,0}
,{1195.000000,758.000000,459.000000,0.000000,0}
,{1236.000000,721.000000,459.000000,0.000000,0}
,{1269.000000,685.000000,459.000000,0.000000,0}
,{1299.000000,705.000000,459.000000,0.000000,0}
,{1337.000000,731.000000,464.000000,0.000000,0}
,{1388.000000,778.000000,495.000000,0.000000,0}
,{1450.000000,839.000000,524.000000,0.000000,0}
,{1517.000000,903.000000,566.000000,0.000000,0}
,{1588.000000,974.000000,613.000000,-127.000000,0}
,{1661.000000,1044.000000,659.000000,-127.000000,0}
,{1732.000000,1117.000000,707.000000,-127.000000,0}
,{1804.000000,1191.000000,726.000000,-127.000000,0}
,{1878.000000,1261.000000,726.000000,-127.000000,0}
,{1937.000000,1321.000000,726.000000,-127.000000,0}
,{1975.000000,1358.000000,726.000000,-127.000000,0}
,{2003.000000,1392.000000,726.000000,-127.000000,0}
,{2041.000000,1430.000000,726.000000,-127.000000,0}
,{2061.000000,1438.000000,726.000000,-127.000000,0}
,{2063.000000,1438.000000,726.000000,-127.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
,{2063.000000,1438.000000,726.000000,0.000000,0}
};
//--END AUTONOMOUS ARRAYS--

int driveL = 0;
int driveR = 0;
int armT = 0;

// Redo
task ArmPID() {
	nMotorEncoder[aBotR] = 0;
	while(true) {
		if(abs(armT-nMotorEncoder[aBotR])>40) {
			if(nMotorEncoder[aBotR] < armT) {
				arm(127);
			}else
			{
				arm(-127);
			}
		}else
		{
			arm(16);
		}
		wait1Msec(50);
	}
}

task DrivePID() {
	nMotorEncoder[bl] = 0;
	nMotorEncoder[br] = 0;
	int dLP = 0;
	int dRP = 0;
	while(true) {
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

void replay1() {
	startTask(DrivePID);
	startTask(ArmPID);
	for(int i = 0; i < 150; i ++) {
		driveL = autonomous1[i][0];
		driveR = autonomous1[i][1];
		armT = autonomous1[i][2];
		intakeP(autonomous1[i][3]);
		SensorValue[s_claw] = autonomous1[i][4];
		wait1Msec(100);
	}
	stopTask(DrivePID);
	stopTask(ArmPID);
	tank(0,0);
}

void replay2() {
	startTask(DrivePID);
	startTask(ArmPID);
	for(int i = 0; i < 150; i ++) {
		driveL = autonomous2[i][0];
		driveR = autonomous2[i][1];
		armT = autonomous2[i][2];
		intakeP(autonomous2[i][3]);
		SensorValue[s_claw] = autonomous2[i][4];
		wait1Msec(100);
	}
	stopTask(DrivePID);
	stopTask(ArmPID);
	tank(0,0);
}

void replay3() {
	startTask(DrivePID);
	startTask(ArmPID);
	for(int i = 0; i < 150; i ++) {
		driveL = autonomous3[i][0];
		driveR = autonomous3[i][1];
		armT = autonomous3[i][2];
		intakeP(autonomous3[i][3]);
		SensorValue[s_claw] = autonomous3[i][4];
		wait1Msec(100);
	}
	stopTask(DrivePID);
	stopTask(ArmPID);
	tank(0,0);
}

void replay4() {
	startTask(DrivePID);
	startTask(ArmPID);
	for(int i = 0; i < 150; i ++) {
		driveL = autonomous4[i][0];
		driveR = autonomous4[i][1];
		armT = autonomous4[i][2];
		intakeP(autonomous4[i][3]);
		SensorValue[s_claw] = autonomous4[i][4];
		wait1Msec(100);
	}
	stopTask(DrivePID);
	stopTask(ArmPID);
	tank(0,0);
}

task autonomous() {

	switch(auto) {
		case 0: replay1(); break;
		case 1: replay2(); break;
		case 2: replay3(); break;
		case 3: replay4(); break;
	}

}

bool braking = false;
bool recording = false;

task ActiveBrake() {
	braking = true;
	tank(0,0);
	wait1Msec(500);
	int dLP = nMotorEncoder[bl];
	int dRP = nMotorEncoder[br];
	while(true) {
		int dL = dLP-nMotorEncoder[bl];
		int dR = dRP-nMotorEncoder[br];
		float kP = 0.4;
		float left = dL*kP;
		float right = dR*kP;
		if(!recording) tank(left,right);
		wait1Msec(50);
	}
}

task Record() {
	nMotorEncoder[bl] = 0;
	nMotorEncoder[br] = 0;
	nMotorEncoder[aBotR] = 0;
	while(motor[bl] == 0
		&& motor[br] == 0
		&& nMotorEncoder[aBotR] == 0
		&& motor[intakeL] == 0) { wait1Msec(1); }
	writeDebugStream("int autonomous[150][5] = {");
	for(int i = 0; i < 150; i ++) {
		int dL = nMotorEncoder[bl];
		int dR = nMotorEncoder[br];
		int aP = nMotorEncoder[aBotR];
		int iP = motor[intakeL];
		int cP = SensorValue[s_claw];
		if(i > 0) writeDebugStream(",");
		writeDebugStream("{%f,%f,%f,%f,%d}\n",dL,dR,aP,iP,cP);
		autonomous1[i][0] = dL;
		autonomous1[i][1] = dR;
		autonomous1[i][2] = aP;
		autonomous1[i][3] = iP;
		autonomous1[i][4] = cP;
		wait1Msec(100);
	}
	recording = false;
	writeDebugStream("}");
}

task usercontrol()
{
	while(true) {
		if(trueSpeed(vexRT[Ch3]) == 0 && trueSpeed(vexRT[Ch1]) == 0) {
			if(!braking) startTask(ActiveBrake);
		}else
		{
			if(braking) stopTask(ActiveBrake);
			arcadeDrive(trueSpeed(vexRT[Ch3]),trueSpeed(vexRT[Ch1]),0);
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
		if(vexRT[Btn6U]) {
			intakeP = 127;
		}
		if(vexRT[Btn6D]) {
			intakeP = -127;
		}
		motor[intakeL] = intakeP;
		motor[intakeR] = intakeP;

		SensorValue[s_claw] = vexRT[Btn8D];

		displayLCDCenteredString(0,"Dark Matter");
		displayLCDCenteredString(1,"2150D");

		if(vexRT[Btn8U]) {
			recording = true;
			startTask(Record);
		}

		if(vexRT[Btn7D]) {
			nMotorEncoder[aBotR] = 0;
		}

		if(vexRT[Btn7U]) {
			stopTask(ActiveBrake);
			//replay1();
		}

	}
}
