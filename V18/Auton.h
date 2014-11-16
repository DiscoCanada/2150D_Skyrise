
//Documentation
// 1. For Movement Enter: drive(distanceInInches, direction)
//	A) Direction Can Be:
//		- 1: Forward
//		- 2: Backward
//		- 3: Left
//		- 4: Right

void resetAll()
{
	startTask(drivePID);
	startTask(liftPID);
	startTask(intakeSet);
}

void autonomous0()
{
	resetAll();
	stopTask(intakeSet);
	driveIn(10, 0, 0);
	driveTicks(10, 392, 0);
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
