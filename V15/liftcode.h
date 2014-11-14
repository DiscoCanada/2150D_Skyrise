// Init
float kPLift = 0.3;
float kLiftAngle;
float kLiftHeight;
bool liftFinished = true;

int kELGoal; // Elevator Left
int kERGoal; // Elevator Right
int kALGoal; // Arm Left
int kARGoal; // Arm Right

task userLiftElevator()
{
  while(true)
    {
    	int balSpeed = 20;
			while(vexRT[Btn5D] == 0 && vexRT[Btn5U] == 0)
			{
				motor[el] = balSpeed;
				motor[er] = balSpeed;
			}

    	// Elevator Up
    	motor[er] = vexRT[Btn5U]*127; // Arm Elevator Up
    	motor[el] = vexRT[Btn5U]*127; // Arm Elevator Up

    	// Elevator Down
    	motor[er] = vexRT[Btn5D]*-127; // Arm Elevator Down
    	motor[el] = vexRT[Btn5D]*-127; // Arm Elevator Down
      wait1Msec(25);


    }
}
// TODO: Implement a method to build skyrises by just pressing a button. In this case just do an i++.


task userLiftArm()
{
  while(true)
    {
    	int balSpeed = 20;
			while(vexRT[Btn6D] == 0 && vexRT[Btn6U] == 0)
			{
				motor[el] = balSpeed;
				motor[er] = balSpeed;
			}

    	// Arm Up
    	motor[ar] = vexRT[Btn6U]*127; // Arm Right Up
    	motor[al] = vexRT[Btn6U]*127; // Arm Left Up

    	// Arm Down
    	motor[ar] = vexRT[Btn6D]*-127; // Arm Right Down
    	motor[al] = vexRT[Btn6D]*-127; // Arm Left Down
      wait1Msec(25);
    }
}

void elevatorUp(int timeSec)
{
	motor[el] = 127;
	motor[er] = 127;
	wait1Msec(timeSec);
	motor[el] = 0;
	motor[er] = 0;
}

void armUp(int timeSec)
{
	motor[al] = 127;
	motor[ar] = 127;
	wait1Msec(timeSec);
	motor[al] = 0;
	motor[ar] = 0;
}

      // if btn 7U is pressed, then lift to next height. Make sure it has a way to decrease i++.
      /*
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
      */
