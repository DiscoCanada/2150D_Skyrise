void lift(int speed, int time)
{
  motor[m_lftLB] = speed;
  motor[m_lftLT] = speed;
  motor[m_rtLB] = speed;
  motor[m_rtLT] = speed;
  wait1Msec(time);
  motor[m_lftLB] = 16;
  motor[m_lftLT] = 16;
  motor[m_rtLB] = 16;
  motor[m_rtLT] = 16;
}

void driveForward(int speed, int time)
{

}

void driveBackwards(int speed, int time)
{

}

void strafeLeft(int speed, int time)
{

}

void strafeRight(int speed, int time)
{

}

void turnLeft(int degrees, int time)
{

}

void turnRight(int degrees, int time)
{

}
