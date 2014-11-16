task armComp()
{
	while(true)
	{
		if(vexRT[Btn5U] == 1) {
			liftTarget = ticksLift + tickIncrease;
			hasLockedLift = false;
		}
		else if(vexRT[Btn5D] == 1) {
			liftTarget = ticksLift - tickIncrease;
			hasLockedLift = false;
		}
		else if(!hasLockedLift) {
			liftTarget = ticksLift;
			hasLockedLift = true;
		}

		if(vexRT[Btn5U] == 1) {
			armPo(127);
		}
		else if(vexRT[Btn5D] == 1) {
			armPo(-127);
		}
		else {
			armPo(0);
		}
		wait1Msec(TIME_DELAY);
	}
}
