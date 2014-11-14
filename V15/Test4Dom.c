#pragma config(Sensor, in1,    LfR,            sensorLineFollower)
#pragma config(Sensor, in2,    LfM,            sensorLineFollower)
#pragma config(Sensor, in3,    LfL,            sensorLineFollower)
#pragma config(Motor,  port1,           driveL,        tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port10,          driveR,        tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

int dom = 69;

task main()
{
	while(dom == dom)
	{
		if(SensorValue[LfM] > 50 && SensorValue[LfR] < 100)
		{
			motor[driveL] = 127;
			motor[driveR] = 0;
		}

		else if(SensorValue[LfM] > 50 && SensorValue[LfL] < 100)
		{
			motor[driveL] = 0;
			motor[driveR] = 127;
		}

		else if(SensorValue[LfM] < 50)
		{
			motor[driveL] = 127;
			motor[driveR] = 127;
		}
	}
}