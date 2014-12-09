#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    s_driveGyro,    sensorGyro)
#pragma config(Sensor, in2,    s_lfLL,         sensorLineFollower)
#pragma config(Sensor, in3,    s_lfL,          sensorLineFollower)
#pragma config(Sensor, in4,    s_lfM,          sensorLineFollower)
#pragma config(Sensor, in5,    s_lfR,          sensorLineFollower)
#pragma config(Sensor, in6,    s_lfRR,         sensorLineFollower)
#pragma config(Sensor, in7,    s_liftBotP,     sensorPotentiometer)
#pragma config(Sensor, in8,    s_liftTopP,     sensorPotentiometer)
#pragma config(Sensor, dgtl1,  p_intClaw,      sensorDigitalOut)
#pragma config(Sensor, dgtl6,  s_lftZero,      sensorTouch)
#pragma config(Sensor, dgtl7,  s_rtZero,       sensorTouch)
#pragma config(Sensor, dgtl8,  l_diable,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl9,  l_auton,        sensorLEDtoVCC)
#pragma config(Sensor, dgtl10, l_driver,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl11, l_lftT,         sensorLEDtoVCC)
#pragma config(Sensor, dgtl12, l_rtT,          sensorLEDtoVCC)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_4,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_5,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_6,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_7,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_8,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port2,           m_FL,          tmotorVex393HighSpeed_MC29, openLoop, driveRight, encoderPort, I2C_1)
#pragma config(Motor,  port3,           m_FR,          tmotorVex393HighSpeed_MC29, openLoop, reversed, driveLeft, encoderPort, I2C_2)
#pragma config(Motor,  port4,           m_BL,          tmotorVex393HighSpeed_MC29, openLoop, driveRight, encoderPort, I2C_3)
#pragma config(Motor,  port5,           m_BR,          tmotorVex393HighSpeed_MC29, openLoop, reversed, driveLeft, encoderPort, I2C_4)
#pragma config(Motor,  port6,           m_lftLB,       tmotorVex393_MC29, openLoop, encoderPort, I2C_5)
#pragma config(Motor,  port7,           m_rtLB,        tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_6)
#pragma config(Motor,  port8,           m_lftLT,       tmotorVex393_MC29, openLoop, encoderPort, I2C_8)
#pragma config(Motor,  port9,           m_rtLT,        tmotorVex393_MC29, openLoop, reversed, encoderPort, I2C_7)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//
/*
 _______  ____   _______  _______    ______
|       ||    | |       ||  _    |  |      |
|____   | |   | |   ____|| | |   |  |  _    |
____|   | |   | |  |____|| | |   | 	|	  |   |
| ______| |   | |_____  || |_|   |  | |_|   |
| |_____  |   |  _____| ||       |  |       |
|_______| |___| |_______||_______|  |______|
*/
#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(20)
#pragma userControlDuration(120)

#include "Vex_Competition_Includes.c"   //Main competition background code...do not modify!

#include "drivetrainControl.h"
#include "armControl.h"
#include "menu.h"
#include "auton.h"

/////////////////////////////////////////////////////////////////////////////////////////
//
//                          Pre-Autonomous Functions
//
// You may want to perform some actions before the competition starts. Do them in the
// following function.
//
/////////////////////////////////////////////////////////////////////////////////////////

void pre_auton()
{
	clearLCDLine(0);
	clearLCDLine(1);
	bLCDBacklight = true;
	calibrateGyro();
	startTask(Menu);

  // Set bStopTasksBetweenModes to false if you want to keep user created tasks running between
  // Autonomous and Tele-Op modes. You will need to manage all user created tasks if set to false.
  bStopTasksBetweenModes = true;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous Task
//
// This task is used to control your robot during the autonomous phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task autonomous()
{
		// Turn The LCD Backlight On
	bLCDBacklight = true;

	// Naming Convention, Due to Multiple Counts Running
	int finalCount = count;

	//Stop The Menu Task
	stopTask(Menu);

	//Consult The Menu Class to Run Autonomous Mode
	switch(finalCount)
	{
	case 0:
		{
			autonomous0();
			break;
		}
	case 1:
		{
			autonomous1();
			break;
		}
	case 2:
		{
			autonomous2();
			break;
		}
	case 3:
		{
			autonomous3();
			break;
		}
	case 4:
		{
			autonomous4();
			break;
		}
	case 5:
		{
			autonomous5();
			break;
		}
	case 6:
		{
			autonomous6();
			break;
		}
	case 7:
		{
			autonomous7();
			break;
		}
	case 8:
		{
			autonomous8();
			break;
		}
	case 9:
		{
			autonomous9();
			break;
		}
	case 10:
		{
			autonomous10();
			break;
		}
	case 11:
		{
			autonomous11();
			break;
		}
	case 12:
		{
			autonomous12();
			break;
		}
	case 13:
		{
			autonomous13();
			break;
		}
	case 14:
		{
			autonomous14();
			break;
		}
	case 15:
		{
			autonomous15();
			break;
		}
	case 16:
		{
			testing();
			break;
		}
	case 17:
		{
			programming();
			break;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 User Control Task
//
// This task is used to control your robot during the user control phase of a VEX Competition.
// You must modify the code to add your own robot specific commands here.
//
/////////////////////////////////////////////////////////////////////////////////////////

task usercontrol()
{
	// User control code here, inside the loop

	while(true)
	{
		// Testing Variables
		calibrateGyro();
	  startTask(handleEncoders);

		// Background PID 'n' Such
		startTask(armPID);
		startTask(trackXYT);

		//Driver Control
		startTask(userDriveHolo);
		startTask(userLiftRD4B);

		//Control Recorder
		if(vexRT[Btn7L] == 1)
		{
			startTask(recordAuto);
		}
	}
}
