int kLiftNum = 11;
int kCurrLiftVal = 0;
/* Lift Values */
// 0 = Starting Pos(Set to 0)
// 1 = Skyrise Holdfer
// 2 = First Level Drop Height
// 3 = Second Level Drop Height
// 4 = Third Level Drop Height
// 5 = Fourth Level Drop Height
// 6 = Fifth Level Drop Height
// 7 = Sixth Level Drop Height
// 8 = Seventh Level Drop Height
// 9 = Lowest Tower
// 10 = Medium Tower
// 11 = Tallest Tower

void arm(int power)
{
	motor[m_lftLB] = power;
	motor[m_lftLT] = power;
	motor[m_rtLB] = power;
	motor[m_rtLT] = power;
}

int armTBot = 0;
int armTTop = 0;

// P loop for the arm's control.
task ArmPID()
{
	nMotorEncoder[m_lftLB] = 0;
	nMotorEncoder[m_lftLT] = 0;
	nMotorEncoder[m_rtLB] = 0;
	nMotorEncoder[m_rtLT] = 0;
	while(true)
	{
		if(abs(armTBot-nMotorEncoder[m_rtLB])>40 && abs(armTBot-nMotorEncoder[m_lftLB])>40 && abs(armTTop-nMotorEncoder[m_lftLT])>40 && abs(armTTop-nMotorEncoder[m_rtLT])>40)
		{
			if(nMotorEncoder[m_rtLB] < armTBot && nMotorEncoder[m_lftLB] < armTBot && nMotorEncoder[m_lftLT] < armTTop && nMotorEncoder[m_rtLT] < armTTop)
			{
				arm(127);
			}
			else
			{
				arm(-127);
			}
		}
		else
		{
			arm(17);
		}
		wait1Msec(25);
	}
}

task userLiftRD4B()
{
	// Toggle System
	while(kCurrLiftValue < kLiftNum)
	{
		// Lift Up
		if(vexRT[Btn5U]==1)
		{
				kCurrLiftValue++;
				while(vexRT[Btn5U]{}
		}

		// Lift Down
		if(vexRT[Btn5D]==1)
		{
			kCurrLiftValue--;
			while(vexRT[Btn5D]{}
		}

		// Normalize Values
		if(kCurrValue < 0)
		{
			kCurrValue == 0;
		}

		if(kCurrLiftValue == 0)
		{
			// Bring Lift To Starting/Gathering Pos
			lift(0, 0);

			// Set Encoders To Zero
			if(s_lftZero == 1 && s_rtZero == 1)
			{
				nMotorEncoder[m_lftLB] = 0;
				nMotorEncoder[m_lftLT] = 0;
				nMotorEncoder[m_rtLB] = 0;
				nMotorEncoder[m_rtLT] = 0;
			}
			else if(s_lftZero == 0 || s_rtZero == 0)
			{
				arm(-127);
			}

			// Bring Lift To Zero
			lift(0, 0);
		}

		if(kCurrLiftValue == 1)
		{
				lift(100 , 100);
		}

		if(kCurrLiftValue == 2)
		{
				lift(0, 0);
		}

		if(kCurrLiftValue == 3)
		{
				lift(110, 110);
		}

		wait1Msec(25);
	}

	// Control Buttons
	int power = vexRT[Btn6U]*127 - vexRT[Btn6D]*127;
	arm(power);
}

// Intake Control
task userIntakeSrise()
{
	SensorValue(p_intClaw) = vexRT[Btn8L];
}

void lift(int rotationB, int rotationT)
{
	armTBot = rotationB;
	armTTop = rotationT;
}
