const ubyte kNumMotors = 10;
const tMotor kMotorPort[kNumMotors] = {port1, port2, port3, port4, port5, port6, port7, port8, port9, port10};
const tSensors kGyroPort = in1;
const ubyte kNumDriveMotorsPerSide = 2;
const tMotor kDriveMotorPorts[2][kNumDriveMotorsPerSide] = {{port2, port3}, {port8, port9}}; // {{left motors}, {right motors}}

bool driveDone = false;
bool turnDone = false;
short maxSpeed = 127;
short driveSpeed = 0;
short turnSpeed = 0;
long motorEncoder[kNumMotors];
long setDistance = 0;
float setGyro = 0.0;

sbyte trueSpeed(long input)
{
  const sbyte speed = (sbyte)limit(input, 127);
  const ubyte speedArray[128] =
  {
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
     0, 21, 21, 21, 22, 22, 22, 23, 24, 24,
    25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
    28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
    33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
    37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
    41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
    46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
    52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
    71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
    80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
    88, 89, 89, 90, 90,127,127,127
  };
  return sgn(speed)*speedArray[sgn(speed)*speed];
}

float limit(float input, float maxLimit, float minLimit = 2097200.0)
{
  if(input > maxLimit)
    return maxLimit;
  if(minLimit == 2097200.0)
    minLimit = -maxLimit;
  if(input < minLimit)
    return minLimit;
  return input;
}

void calibrateGyro()
{
  // completely clear out any previous sensor readings by setting the port to "sensorNone"
  SensorType[kGyroPort] = sensorNone;
  wait1Msec(1000);
  SensorType[kGyroPort] = sensorGyro; // reconfigure as sensorGyro
  wait1Msec(2000); // wait for calibration: ROBOT MUST STAY STILL

  SensorScale[kGyroPort] = 138; // adjust SensorScale to correct the scaling for your gyro
  SensorFullCount[kGyroPort] = 3599; // fix rollover to be "...3598, 3599, 0, 1..."
}

task handleEncoders()
{
  const ubyte kDelay = 10;
  signed int prevValue[kNumMotors];
  signed int encoderValue = 0;

  for(ubyte i=0; i<kNumMotors; i++)
  {
    motorEncoder[i] = 0;
    prevValue[i] = 0;
  }
  while(true)
  {
    for(ubyte i=0; i<kNumMotors; i++)
    {
      encoderValue = nMotorEncoder[kMotorPort[i]];
      if(abs(encoderValue) > 32000)
      {
        nMotorEncoder[kMotorPort[i]] = 0;
        prevValue[i] -= encoderValue;
        encoderValue = nMotorEncoder[kMotorPort[i]];
      }
      motorEncoder[i] += encoderValue - prevValue[i];
      prevValue[i] = encoderValue;
    }
    wait1Msec(kDelay);
  }
}

task gyroPID()
{
  const ubyte kDelay = 25;
  const float kP = 1.0;
  const float kI = 0.5;
  const float kD = 0.5;
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
      if(time1[T2] > kSettleTime)
        turnDone = true;
    else
      clearTimer(T2);

    wait1Msec(kDelay);
  }
}

task drivePID()
{
  const ubyte kDelay = 25;
  const tSensors kEncoderPorts[2] = {port3, port8}; // {left encoder, right encoder}
  const float kP = 1.0; // Retune
  const float kI = 0.5; // Retune
  const float kD = 0.5; // Retune
  const float kL = 50.0;
  const short kDoneThreshold = 10;
  const short kSettleTime = 250;

  long error = 0;
  long prevError = 0;
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;
  short speedL = 0;
  short speedR = 0;

  while(true)
  {
    error = setDistance - (motorEncoder[kEncoderPorts[0]] + motorEncoder[kEncoderPorts[1]])/2;

    p = error;
    i = abs(i + error) < kL ? i + error : kL;
    d = error - prevError;

    driveSpeed = p*kP + i*kI + d*kD;

    speedL = limit(driveSpeed - turnSpeed, maxSpeed);
    speedR = limit(speedL + 2*turnSpeed, maxSpeed);

    for(ubyte j=0; j<kNumDriveMotorsPerSide; j++) {
      motor[kDriveMotorPorts[j][0]] = trueSpeed(speedL);
      motor[kDriveMotorPorts[j][1]] = trueSpeed(speedR);
    }
    prevError = error;

    if(error < kDoneThreshold)
      if(time1[T1] > kSettleTime)
        driveDone = true;
    else
      clearTimer(T1);

    wait1Msec(kDelay);
  }
}

void drive(const long distance, const short speed = 127)
{
  maxSpeed = speed;
  setDistance = distance;
}

void turn(const float angle, const short speed = 127)
{
  maxSpeed = speed;
  setGyro = angle;
}

void waitForDrive()
{
  driveDone = false;

  while(!driveDone)
    wait1Msec(25);
}

void waitForTurn()
{
  turnDone = false;

  while(!turnDone)
    wait1Msec(25);
}

void driverControl()
{
	// Manage Tasks
	startTask(handleEncoders);
	stopTask(gyroPID);
	stopTask(drivePID);

	motor[lftDrive1] = -vexRT[Ch3] - vexRT[Ch4];
	motor[lftDrive2] = -vexRT[Ch3] - vexRT[Ch4];
	motor[rtDrive2] = vexRT[Ch3] + vexRT[Ch4];
	motor[rtDrive1] = vexRT[Ch3] + vexRT[Ch4];
}

//return the left encoder distance
float leftdistance()
{
	return ((float)nMotorEncoder[lftDrive2] / (TPR_TORQUE)) * (4.0 * PI);
}

//return right encoder distance
float rightdistance()
{
	return ((float)nMotorEncoder[rtDrive2] / (TPR_TORQUE)) * (4.0 * PI);
}

//task main() {
//  calibrateGyro();
//  startTask(handleEncoders);
//  startTask(gyroPID);
//  startTask(drivePID);

//  //drive(1000);
//  //waitForDrive();
//  //turn(90.0, 100);
//  //waitForTurn();
//}
