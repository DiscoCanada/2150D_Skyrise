/*==============================================================================*\
||Written By: Harrison Freni, Emma Taulli, and Madeleine Gradney.               ||
||With Help From: Jordan Kiesel(BNS), Elias Wu(BNSw), and Griffin Tabor(BNS)    ||
||Team 2150D is a Robotics Team based out of Chadwick School loacted on the     ||
||Palos Verdes Peninsula. Team 2150D was recreated during the Skyrise Season    ||
\*=============================================================================*/

#include <FirmwareVersion.h> // the "TMotorType" values below have changed a bit between versions of RobotC

#if kRobotCVersionNumeric >= 400 // Newer RobotC
#define tmotorVex269 9989
#define tmotorVex393 9990
#define tmotorVex393HighSpeed 9991
#else // Older RobotC
#define tmotorVex269_HBridge 9992
#define tmotorVex269_MC29 9993
#define tmotorVex393_HBridge 9994
#define tmotorVex393_MC29 9995
#define tmotorVex393HighSpeed_HBridge 9996
#define tmotorVex393HighSpeed_MC29 9997
#endif
// Set Motor Values 'n' Stuff
#if kRobotCVersionNumeric < 426
#define tmotorVex393TurboSpeed_HBridge 9998
#define tmotorVex393TurboSpeed_MC29 9999
#endif

const ubyte kNumMotors = 10; // Don't Change This Unless You have Some magical powers and have added more ports to the cortex.
const tMotor kMotorPort[kNumMotors] = {port1, port2, port3, port4, port5, port6, port7, port8, port9, port10};

const tSensors kGyroPort = s_Gyro; // Assign the Gyro Port Here
const ubyte kNumliftMotorsPerSegment = 2; // Assign Number of Motors Per Side Here
const tMotor kLiftMotorPorts[2][kNumLiftMotorsPerSegemnt] = {{port6, port7}, {port8, port9}}; // {{left motors}, {right motors}}

float liftTicksPerRev = 0; // Set TPR to 0 so RobotC can determine the stuffs
bool liftDone = false; // Boolean used to re-evalueate if the robot's operation is done.
bool turnDone = false; // Boolean used to re-evalueate if the robot's operation is done.
word maxSpeed = 127; // Again. Don't change this unless you have mystical powers, in which you then can.
word liftSpeed = 0; // Set Current lift Speed to 0 for Later Modification
word turnSpeed = 0; // Set Turn Speed to 0 Unless moving, which then gets changed
long motorEncoder[kNumMotors]; // Pls, This is very important, it keeps track of which motors use encoders, See liftPID method for more info.
long setDistance = 0;
float setGyro = 0.0;

task liftPIDElevator()
{
  const ubyte kDelay = 25;
  const tSensors kEncoderMotorPort[4] = {port2, port3, port4, port5}; // {left encoder x1,left encoder x2, right encoder x1,right encoder x2}
  const TMotorTypes kMotorType = motorType[kEncoderMotorPort[0]]; // save encoder motor's type
  const float kP = 1.0;
  const float kI = 0.5;
  const float kD = 0.5;
  const float kL = 50.0;
  const short kDoneThreshold = 10;
  const short kSettleTime = 250;

  long error = 0;
  long prevError = 0;
  long position = 0;
  float p = 0.0;
  //float i = 0.0;
  //float d = 0.0;

  short speedL = 0;
  short speedR = 0;

  // ==== determine number of ticks per revolution ====
  if(kMotorType == tmotorVex269 || kMotorType == tmotorVex269_HBridge || kMotorType == tmotorVex269_MC29)
    liftTicksPerRev = 240.448; // 269
    else if(kMotorType == tmotorVex393 || kMotorType == tmotorVex393_HBridge || kMotorType == tmotorVex393_MC29)
    liftTicksPerRev = 627.2; // 393
    else if(kMotorType == tmotorVex393HighSpeed || kMotorType == tmotorVex393HighSpeed_HBridge || kMotorType == tmotorVex393HighSpeed_MC29)
    liftTicksPerRev = 392; // 393 high speed
    else if(kMotorType == tmotorVex393TurboSpeed_HBridge || kMotorType == tmotorVex393TurboSpeed_MC29)
    liftTicksPerRev = 261.333; // 393 turbo speed

    while(true)
    {
      error = kHeight - abs(nMotorEncoder[m_lftLB]);

      //setheightkP(goalIME-current IME)
      p = error;
      //i = abs(i + error) < kL ? i + error : kL;
      //d = error - prevError;

      liftSpeed = p*kP //+ i*kI + d*kD;

      float leftSideElev8 = setHeight;
      float rightSideElev8 = setHeight;

      //rightlift=(normal input) -kP(rightIME-leftIME)
      //leftlift=normal input


      for(ubyte j=0; j<kNumliftMotorsPerAngle; j++)
      {
        motor[kliftMotorPort[j][0]] = trueSpeed(leftSideElev8); // FL
        motor[kliftMotorPort[j][1]] = trueSpeed(rightSideElev8); // BL
      }

      prevError = error;

      if(error < kDoneThreshold)
      {
        if(time1[T1] > kSettleTime)
        {
          liftDone = true;
        }
        else
        {
          clearTimer(T1);
        }
      }
      wait1Msec(kDelay);
  }
}

void liftToHeight(string position, const Word kSpeed = 127 )
{
  kHeight = 0;

  if (position = "LOW_GRAY_TUBE")
  {
    kHeight = x;
  }
    else if(position = "MED_GRAY_TUBE1")
  {
    kHeight = x;
  }
    else if(position = "MED_GRAY_TUBE2")
  {
    kHeight = x;
  }
    else if(position = "HIGH_GRAY_TUBE")
  {
    kHeight = x;
  }
  else if (position = "SKYRISE1")
  {
    kHeight = x;
  }
  else if (position = "SKYRISE2")
  {
    kHeight = x;
  }
  else if (position = "SKYRISE3")
  {
    kHeight = x;
  }
  else if (position = "SKYRISE4")
  {
    kHeight = x;
  }
  else if (position = "SKYRISE5")
  {
    kHeight = x;
  }

  maxSpeed = abs(kSpeed);
  setDistance += kHeight < 0 ? kHeight : kHeight*sgn(kSpeed);
}

task turnPID()
{
  const ubyte kDelay = 25;
  const float kP = 0.545;
  const float kI = 0.0586;
  const float kD = 3.37;
  const float kL = 50.0;
  const float kDoneThreshold = 1.0;
  const short kSettleTime = 250;

  float error = 0.0;
  float prevError = 0.0;
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;

  while(true)
  {
    error = setGyro - SensorValue[kGyroPort];

    p = error;
    i = abs(i + error) < kL ? i + error : kL;
    d = error - prevError;

    turnSpeed = p*kP + i*kI + d*kD;

    prevError = error;

    if(error < kDoneThreshold)
    {
      if(time1[T2] > kSettleTime)
      {
        turnDone = true;
      }
      else
      {
        clearTimer(T2);
      }
    }
    wait1Msec(kDelay);
  }
}
