int battVoltage;

task driveComp()
{
	clearEncoders();
	rfMult = 1;
	lfMult = 1;

	while(true)
	{
		battVoltage = nImmediateBatteryLevel/1000.0;

		if(abs(vexRT[Ch3]) > deadZone)
		{
			fwd = vexRT[Ch3];
		}
		else {
			fwd = 0;
			nMotorEncoder[bl] = 0;
			nMotorEncoder[br] = 0;
		}
		if(abs(vexRT[Ch4]) > deadZone)
		{
			strafe = vexRT[Ch4];
		}
		else {
			strafe = 0;
			nMotorEncoder[bl] = 0;
			nMotorEncoder[br] = 0;
		}
		if(abs(vexRT[Ch1]) > deadZone)
		{
			clockwise =  vexRT[Ch1];
		}
		else {
			clockwise = 0;
			nMotorEncoder[br] = 0;
			nMotorEncoder[bl] = 0;
		}
	}
}
