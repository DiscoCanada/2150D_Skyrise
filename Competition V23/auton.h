void resetAll()
{
	startTask(handleEncoders);
	startTask(trackXYT);
	startTask(ArmPID);
	calibrateGyro();
}

void autonomous0()
{
	resetAll();
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

task recordAuto()
{
	// Global Values
	int x = currpos.x;
	int y = currpos.y;
	int t = currpos.t;
	int rotationB = nMotorEncoder[m_lftLB];
	int rotationT = nMotorEncoder[m_lftLT];
	int intake = SensorValue[p_intClaw];

	// Set Encoder Values to 0
	nMotorEncoder[m_lftLB] = 0;
	nMotorEncoder[m_lftLT] = 0;
	nMotorEncoder[m_rtLB] = 0;
	nMotorEncoder[m_rtLT] = 0;

	// Start ArmPID task
	//TODO: Remove later because redundat
	startTask(armPID);

	// Print Values
	for(int i = 0; i < 150; i++)
	{
		writeDebugStream("setXYT(%f,%f,%d)\n",x,y,t );
		writeDebugStream("lift(%f, %d)\n", rotationB, rotationT);
		writeDebugStream("intake(%d)\n", intake);
		wait1Msec(100);
	}
}
