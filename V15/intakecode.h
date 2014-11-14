void deployIntake()
{
  motor[intake] = 127;
  wait1Msec(1000);
}

task userIntake()
{
  const TVexJoysticks kBtnU8 = Btn8U;
  const TVexJoysticks kBtnD8 = Btn8D;
  while(true)
    {
      //Intake Up
      motor[intake] = 127*vexRT[kBtnU8]

      //Intake Down
      motor[intake] = -127*vexRT[kBtnD8]

      wait1Msec(25);
    }
}
