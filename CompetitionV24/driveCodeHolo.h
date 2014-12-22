#include <FirmwareVersion.h> // the "TMotorType" values below have changed a bit between versions of RobotC

#if kRobotCVersionNumeric >= 400
#define tmotorVex269 9989
#define tmotorVex393 9990
#define tmotorVex393HighSpeed 9991
#else
#define tmotorVex269_HBridge 9992
#define tmotorVex269_MC29 9993
#define tmotorVex393_HBridge 9994
#define tmotorVex393_MC29 9995
#define tmotorVex393HighSpeed_HBridge 9996
#define tmotorVex393HighSpeed_MC29 9997
#endif
#if kRobotCVersionNumeric < 426
#define tmotorVex393TurboSpeed_HBridge 9998
#define tmotorVex393TurboSpeed_MC29 9999
#endif
const ubyte kNumMotors = 10;
const tMotor kMotorPort[kNumMotors] = {port1, port2, port3, port4, port5, port6, port7, port8, port9, port10};

// edit these constants to your robot's specs
const tSensors kGyroPort = s_gyro;
const ubyte kNumLiftEncoder = 2;
const tSensors kLiftEncoderPort[kNumLiftEncoder] = {m_lftBL, lftBR}; // {left encoder, right encoder}
const ubyte kLiftMotorsPerSide = 2;
const tMotor kLiftMotorPort[2][kLiftMotorsPerSide] = {{port5, port6}, {port7, port8}}; // {{left motors}, {right motors}}
const ubyte kNumEncodersPerSide = 2;
const tSensors kEncoderMotorPort[2][kNumEncodersPerSide] = {{port1, port2}, {port4, port5}}; // {{left encoders}, {right encoders}}
const ubyte kDriveMotorsPerSide = 2;
const tMotor kDriveMotorPort[4][kDriveMotorsPerSide] = {{port1}, {port2}, {port3}, {port4}}; // {{left motors}, {right motors}}
const float kWheelDiameter = 2.75; // inches

float driveTicksPerRev = 0.0;
bool driveDone = false;
bool turnDone = false;
bool liftDone = false;
word maxDriveSpeed = 127;
word maxLiftSpeed = 127;
word turnSpeed = 0;
word syncSpeed = 0;
long motorEncoder[kNumMotors];
long setDistance = 0;
long polarityFL = 0;
long polarityFR = 0;
long polarityBL = 0;
long polarityBR = 0;
float setGyro = 0.0;
word setLift = 0;

// used to limit the input to the given bounds (max limit is ± if no min limit is given)
float limit(const float kInput, const float kMaxLimit, float minLimit = 2097200.0)
{
  if(kInput > kMaxLimit)
    return kMaxLimit;
  if(minLimit == 2097200.0)
    minLimit = -kMaxLimit;
  if(kInput < minLimit)
    return minLimit;
  return kInput;
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

void drive(const float kInches, const float kPolarity, const word kSpeed = 127) {
  const long kDistance = kInches/driveTicksPerRev*kWheelDiameter*PI; // convert from inches to ticks

  if(kPolarity == 0)
  {
    // Run Motors Regularly
    polarityFL = 1;
    polarityFR = -1;
    polarityBL = 1;
    polarityBR = -1;
  }
  else if(kPolarity == 1)
  {
    polarityFL = -1;
    polarityFR = 1;
    polarityBL = -1;
    polarityBR = 1;
  }
  else if(kPolarity == 2)
  {
    polarityFL = 1;
    polarityFR = -1;
    polarityBL = -1;
    polarityBR = 1;
  }
  else if(kPolarity == 3)
  {
    // Run Motors in Right
    polarityFL = -1;
    polarityFR = 1;
    polarityBL = 1;
    polarityBR = -1;
  }
  else if(kPolarity > 3 || kPolarity < 0)
  {
    // Drive Forward
    // This is a Failsafe
    polarityFL = 1;
    polarityFR = -1;
    polarityBL = 1;
    polarityBR = -1;
  }

  maxDriveSpeed = abs(kSpeed);
  setDistance += kDistance < 0 ? kDistance : kDistance*sgn(kSpeed);
}

void turn(const float kAngle, const word kSpeed = 127) {
  maxDriveSpeed = abs(kSpeed);
  setGyro += kAngle < 0 ? kAngle : kAngle*sgn(kSpeed);
}

void lift(const word kHeight, const word kSpeed = 127) {
  maxLiftSpeed = abs(kSpeed);
  setLift = kHeight;
}

void waitForDrive() {
  driveDone = false;

  while(!driveDone)
    wait1Msec(25);
}

void waitForTurn() {
  turnDone = false;

  while(!turnDone)
    wait1Msec(25);
}

void waitForLift() {
  liftDone = false;

  while(!liftDone)
    wait1Msec(25);
}

// stores encoder values in an array, allowing values >32,767, up to 2,147,483,647
task handleEncoders() {
  const ubyte kDelay = 25;
  signed int prevValue[kNumMotors];
  signed int encoderValue = 0;

  for(ubyte i=0; i<kNumMotors; i++) {
    motorEncoder[i] = 0;
    prevValue[i] = 0;
  }
  while(true) {
    for(ubyte i=0; i<kNumMotors; i++) {
      encoderValue = nMotorEncoder[kMotorPort[i]];
      if(abs(encoderValue) > 32000) {
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

task gyroPID() {
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

  while(true) {
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
      ClearTimer(T2);

    wait1Msec(kDelay);
  }
}

task drivePID() {
  const ubyte kDelay = 25;
  const TMotorTypes kMotorType = motorType[kEncoderMotorPort[0][0]]; // save encoder motors' type
  const float kP = 1.0;
  const float kI = 0.5;
  const float kD = 0.5;
  const float kL = 50.0;
  const short kDoneThreshold = 10;
  const short kSettleTime = 250;

  long error = 0;
  long prevError = 0;
  float avgEncoder = 0.0;
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;
  word driveSpeed = 0;
  word speedFL = 0;
  word speedFR = 0;
  word speedBL = 0;
  word speedBR = 0;

  // ==== determine number of ticks per revolution ====
  if(kMotorType == tmotorVex269 || kMotorType == tmotorVex269_HBridge || kMotorType == tmotorVex269_MC29)
    driveTicksPerRev = 240.448; // 269
  else if(kMotorType == tmotorVex393 || kMotorType == tmotorVex393_HBridge || kMotorType == tmotorVex393_MC29)
    driveTicksPerRev = 627.2; // 393
  else if(kMotorType == tmotorVex393HighSpeed || kMotorType == tmotorVex393HighSpeed_HBridge || kMotorType == tmotorVex393HighSpeed_MC29)
    driveTicksPerRev = 392; // 393 high speed
  else if(kMotorType == tmotorVex393TurboSpeed_HBridge || kMotorType == tmotorVex393TurboSpeed_MC29)
    driveTicksPerRev = 261.333; // 393 turbo speed

  while(true) {
    avgEncoder = (motorEncoder[kEncoderMotorPort[0][0]] + motorEncoder[kEncoderMotorPort[1][0]])/2;
    for(ubyte j=1; j<kNumEncodersPerSide; j++)
      avgEncoder = (avgEncoder + motorEncoder[kEncoderMotorPort[0][j]] + motorEncoder[kEncoderMotorPort[1][j]])/3;
    error = setDistance - avgEncoder;

    p = error;
    i = abs(i + error) < kL ? i + error : kL;
    d = error - prevError;

    driveSpeed = p*kP + i*kI + d*kD;

    speedFL = limit(driveSpeed, maxSpeed);
    speedBL = limit(driveSpeed, maxSpeed);
    speedFR = limit(driveSpeed, maxSpeed);
    speedBR = limit(driveSpeed, maxSpeed);

    for(ubyte j=0; j<kDriveMotorsPerSide; j++) {
      motor[kDriveMotorPort[0][j]] = speedFL * polarityFL;
      motor[kDriveMotorPort[1][j]] = speedFR * polarityFR;
      motor[kDriveMotorPort[2][j]] = speedBL * polarityBL;
      motor[kDriveMotorPort[3][j]] = speedBR * polarityBR;
    }
    prevError = error;

    if(error < kDoneThreshold)
      if(time1[T1] > kSettleTime)
        driveDone = true;
    else
      ClearTimer(T1);

    wait1Msec(kDelay);
  }
}

task syncPID() {
  const ubyte kDelay = 25;
  const float kP = 1.0;
  const float kI = 0.5;
  const float kD = 0.5;
  const float kL = 50.0;

  float error = 0.0;
  float prevError = 0.0;
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;

  while(true) {
    error = nMotorEncoder[kLiftEncoderPort[1]] - nMotorEncoder[kLiftEncoderPort[0]];

    p = error;
    i = abs(i + error) < kL ? i + error : kL;
    d = error - prevError;

    syncSpeed = p*kP + i*kI + d*kD;

    prevError = error;

    wait1Msec(kDelay);
  }
}

task liftPID() {
  const ubyte kDelay = 25;
  const float kP = 1.0;
  const float kI = 0.5;
  const float kD = 0.5;
  const float kL = 50.0;
  const short kDoneThreshold = 10;
  const short kSettleTime = 250;

  float avgEncoder = 0.0;
  long error = 0;
  long prevError = 0;
  float p = 0.0;
  float i = 0.0;
  float d = 0.0;
  word liftSpeed = 0;
  word speedL = 0;
  word speedR = 0;

  while(true) {
    avgEncoder = nMotorEncoder[kLiftEncoderPort[0]];
    for(ubyte j=1; j<kNumLiftEncoders; j++)
    {
      avgEncoder = (avgPotentiometer + nMotorEncoder[kLiftEncoderPort[j]])/2;
    }

    error = setLift - avgEncoder;

    p = error;
    i = abs(i + error) < kL ? i + error : kL;
    d = error - prevError;

    liftSpeed = p*kP + i*kI + d*kD;

    speedR = limit(liftSpeed + syncSpeed, maxLiftSpeed);
    speedL = limit(speedR - 2*syncSpeed, maxLiftSpeed);
    speedR = limit(speedL + 2*syncSpeed, maxLiftSpeed);

    for(ubyte j=0; j<kLiftMotorsPerSide; j++) {
      motor[kLiftMotorPort[0][j]] = speedL;
      motor[kLiftMotorPort[1][j]] = speedR;
    }
    prevError = error;

    if(error < kDoneThreshold)
      if(time1[T3] > kSettleTime)
        liftDone = true;
    else
      ClearTimer(T3);

    wait1Msec(kDelay);
  }
}

task main() {
  calibrateGyro();
  StartTask(handleEncoders);
  StartTask(gyroPID);
  StartTask(drivePID);
  StartTask(syncPID);
  StartTask(liftPID);

  drive(100, 1);
  waitForDrive();
  turn(90.0, 100);
  waitForTurn();
  lift(1000);
  waitForLift();
}
