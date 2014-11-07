// Init
float liftKp = 0.3;
float kLiftAngle;
float kLiftHeight;
bool liftFinished = true;
int kELGoal; // Elevator Left
int kERGoal; // Elevator Right
int kALGoal; // Arm Left
int kARGoal; // Arm Right

void liftElevator(int position)
{
  if(position == 1)
  {

  }
  else if(position == 2)
  {

  }
  else if(position == 3)
  {

  }
  else if(position == 4)
  {

  }
  else if(position == 5)
  {

  }
  else if(position == 6)
  {

  }
  else if(position == 7)
  {

  }
  else if(position == 8)
  {

  }
  else if(position == 9)
  {

  }
}

// Pos 1 = Skyrise 1
// Pos 2 = Skyrise 2
// Pos 3 = Skyrise 3
// Pos 4 = Skyrise 4
// Pos 5 = Skyrise 5
// Pos 6 = Low Gray Tower
// Pos 7 = Middle Low Tower
// Pos 8 = Middle Tall Tower
// Pos 9 = Tall Tower


void liftArm(int kLiftAngle)
{
  if(kLiftAngle == 1)
  {

  }
  else if(kLiftAngle == 2)
  {

  }
  else if(kLiftAngle == 3)
  {

  }
  else if(kLiftAngle == 4)
  {

  }
}

task liftPIDArm()
{
  while(1)
  {
    while((abs(kALGoal-SensorValue(al))>50) && abs(abs(kARGoal-SensorValue(ar))>50)) //probably should do all 4 but i was lazy
    {
      finished=false;
      motor[al]=kPLift*(kALGoal-SensorValue(al));
      motor[ar]=kPLift*(kARGaol-SensorValue(ar));
    }
    if(abs(kALGoal-SensorValue(al))<=50 && abs(kARGoal-SensorValue(ar))<=50)
    {
      finished=true;
      kALGoal=0;
      kARGoal=0;
      SensorValue(al)=0;
      SensorValue(ar)=0;
    }
  }
}

task liftPIDElevator()
{
  while(1)
  {
    while((abs(kELGoal-SensorValue(el))>50) && abs(abs(kERGoal-SensorValue(er))>50)) //probably should do all 4 but i was lazy
    {
      finished=false;
      motor[el]=kPLift*(kALGoal-SensorValue(el));
      motor[er]=kPLift*(kARGaol-SensorValue(er));
    }
    if(abs(kELGoal-SensorValue(el))<=50 && abs(kERGoal-SensorValue(er))<=50)
    {
      finished=true;
      kELGoal=0;
      kERGoal=0;
      SensorValue(el)=0;
      SensorValue(er)=0;
    }
  }
}

task userLiftElevator()
{
  const TVexJoysticks kChY2 = Ch2; // y-axis joystick channel
  const TVexJoysticks kBtnU7 = Btn7U;
  const TVexJoysticks kBtnD7 = Btn7D;
  int i = 1;
  while(true)
    {
      y = vexRT[kChY2]; // y component

      motor[el] =  y;
      motor[er] = -y;

      // if btn 7U is pressed, then lift to next height. Make sure it has a way to decrease i++.
      if(vexRT[kBtnU7] == 1)
      {
        i++;
        liftElevator(i);
      }
      else if(vexRT[kBtnD7])
      {
        i--;
        liftElevator(i);
      }

      if(i > 1)
      {
          i = 1;
      }
    }
}
// TODO: Implement a method to build skyrises by just pressing a button. In this case just do an i++.


task userLiftArm()
{
  const TVexJoysticks kBtnU8 = Btn8U; // y-axis joystick channel
  const TVexJoysticks kBtnD8 = Btn8D;
  const TVexJoysticks kBtnL8 = Btn8L;
  const TVexJoysticks kBtnR8 = Btn8R;
  int i = 1;
  while(true)
    {
      y = vexRT[kChY2]; // y component

      motor[el] = vexRT[kBtnU8];
      motor[er] = vexRT[kBtnD8];

      // if btn 7U is pressed, then lift to next height. Make sure it has a way to decrease i++.
      if(vexRT[kBtnL8] == 1)
      {
        i++;
        liftArm(i);
      }
      else if(vexRT[kBtnR8])
      {
        i--;
        liftArm(i);
      }

      if(i > 1)
      {
          i = 1;
      }
    }
}
