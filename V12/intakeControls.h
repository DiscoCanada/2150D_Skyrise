void dump()
{
	if(vexRT[Btn8D] == 1)
	{
		SensorValue[dumpCubes] = 1;
	}
	else
	{
		SensorValue[dumpCubes] = 0;
	}
}

void diagVert()
{
	if(vexRT[Btn8U]== 1 &&SensorValue[dumpCubes] == 0)
	{
		SensorValue[DtoV] = DMODE;
	}
	else
	{
		SensorValue[DtoV] = VMODE;
	}
}

void intakeOpControlComp()
{
	dump();
	diagVert();
}
