/*
//Documentation
//Start and stop a drivePID task each time you want to call turning or moving
//To add a drive element, do drive(rotL, rotR);
//To add a turn element, do turnPID(degrees, Direction of turn);
//To set max Speed do maxSpeed = 0-127;
//To drive forward do fdrive(rot);
//To add an arm up, do startTask(arm(Height)Preset);
//To Reset Call resetAll();, ResetPID();, or ResetDrive;
// call lift(rot); for changing lift height.
// call startTask(liftHeightPresetxyz); to have the lift go to a certain preset height.
//For any other sensory input just call a while loop
*/

/* void alignGoalScore(){
	while (SensorValue[goalSonar] - 330>5){
		if (SensorValue[goalSonar]>330)fdrive(50);
		if (SensorValue[goalSonar]<330)fdrive(-50);
	}
	fdrive(0);
} */
void resetAll()
{
	PIDreset();
	resetDrive();
	startTask(DrivePID);
}
void autonomous0()
{

}

void autonomous1()
{

}

void autonomous2()
{

}
void autonomous3()
{

}

void autonomous4()
{

}
void autonomous5()
{

}
void autonomous6()
{

}
void autonomous7()
{

}

void autonomous8()
{

}

void autonomous9()
{

}

void autonomous10()
{

}
 
void autonomous11()
{

}

void autonomous12()
{

}

void autonomous13()
{

}

void autonomous14()
{

}

void autonomous15()
{

}
void testing()
{

}
void programming()
{

}