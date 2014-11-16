task intakeSet()
{
	SensorValue[sriseClaw] = 1;
	wait1Msec(1000);
	SensorValue[sriseClaw] = 0;
}

task intakeComp()
{
	if(vexRT[Btn6U] == 1)
	{
		SensorValue[sriseClaw] = 0;
	}
	else
	{
		SensorValue[sriseClaw] = 1;
	}
}
