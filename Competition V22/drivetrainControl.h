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

#define LCD_LEFT 1
#define LCD_CENTER 2
#define LCD_RIGHT 4

const ubyte kNumMotors = 10;
const tMotor kMotorPort[kNumMotors] = {port1, port2, port3, port4, port5, port6, port7, port8, port9, port10};

// edit these constants to your robot's specs
const tSensors kGyroPort = in1;
const ubyte kNumWheels = 4; // number of drive wheels
const ubyte kNumMotorsPerWheel = 2; // max number of motors per wheel
const tMotor kNone = -1; // used for indicating the lack of an additional motor
const tMotor kDriveMotorPort[kNumWheels][kNumMotorsPerWheel] = { // drive motor ports/names
  {port2, kNone}, // front-left
  {port3, kNone}, // front-right
  {port4, kNone}, // back-left
  {port5, kNone}  // back-right
};
const ubyte kNumDriveEncoders = 4;
const tMotor kDriveEncoderMotorPort[kNumDriveEncoders] = {port1, port2, port3, port4};
// tankDrive: {90.0,90.0} // hDrive: {90.0,0.0,90.0} // kiwiDrive: {60.0,120.0,0.0}
// xDrive OR mecanumDrive: {135.0,45.0,135.0,45} // plusDrive: {90.0,0.0,90.0,0.0}
const float kDriveEncoderMotorAngle[kNumDriveEncoders] = {135.0, 45.0, 135.0, 45.0};
const float kWheelDiameter = 4.0; // inches
const bool kDriveIME = true; // set to false if using quadrature encoders
const ubyte kDefaultLoopDelay = 25;

bool doUseGyro = true; // enable gyroscopic sensor
float gyroOffsetDriver = 90.0; // degrees CCW from +x axis robot faces at gyro=0

typedef struct {
  float x,y,t;
} coordinates;

typedef enum {
  tankDrive = 0,
  hDrive = 1,
  kiwiDrive = 2,
  xDrive = 3,
  mecanumDrive = 4,
  plusDrive = 5,
} driveStyle;

const driveStyle kDriveType = xDrive;

coordinates currPos = {0.0,0.0,0.0};
coordinates destPos = {0.0,0.0,0.0};
coordinates diffPos = {0.0,0.0,0.0};

word maxDriveSpeed = 127;
float driveTicksPerRev = 0.0;
float gyroOffset = 0.0;
long motorEncoder[kNumMotors];

float limit(const float kInput, const float kMaxLimit, float minLimit = 2097200.0) {
  if(kInput > kMaxLimit)
    return kMaxLimit;
  if(minLimit == 2097200.0)
    minLimit = -kMaxLimit;
  if(kInput < minLimit)
    return minLimit;
  return kInput;
}

// compensates for non-linearity of control value vs speed curve
word trueSpeed(const long kInput) {
  const word speed = (word)limit(kInput, 127);
  const word kSpeedArray[128] = {
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
  return sgn(speed)*kSpeedArray[sgn(speed)*speed];
}


// determine number of ticks per revolution for given motor
float ticksPerRev(const tMotor kPort) {
  const TMotorTypes kMotorType = motorType[kMotorPort[kPort]];
  float ticks = 0;

  if(kDriveIME) {
	  if(kMotorType == tmotorVex269 || kMotorType == tmotorVex269_HBridge || kMotorType == tmotorVex269_MC29)
	    ticks = 240.448; // 269
	  else if(kMotorType == tmotorVex393 || kMotorType == tmotorVex393_HBridge || kMotorType == tmotorVex393_MC29)
	    ticks = 627.2; // 393
	  else if(kMotorType == tmotorVex393HighSpeed || kMotorType == tmotorVex393HighSpeed_HBridge || kMotorType == tmotorVex393HighSpeed_MC29)
	    ticks = 392.0; // 393 high speed
	  else if(kMotorType == tmotorVex393TurboSpeed_HBridge || kMotorType == tmotorVex393TurboSpeed_MC29)
	    ticks = 261.333; // 393 turbo speed
	}
	else
	  ticks = 360.0; // quadrature encoders

  return ticks;
}

float ticksToInches(const float kEncoderTicks) {
  return kEncoderTicks*driveTicksPerRev*kWheelDiameter*PI; // convert from ticks to inches
}

void calibrateGyro() {
  // completely clear out any previous sensor readings by setting the port to "sensorNone"
  SensorType[kGyroPort] = sensorNone;
  wait1Msec(1000);
  SensorType[kGyroPort] = sensorGyro; // reconfigure as sensorGyro
  wait1Msec(2000); // wait for calibration: ROBOT MUST STAY STILL

  SensorScale[kGyroPort] = 138; // adjust SensorScale to correct the scaling for your gyro
  SensorFullCount[kGyroPort] = 3599; // fix rollover to be "...3598, 3599, 0, 1..."
}

void setXYT(float x, float y, float t)
{
  destPos.x = x;
  destPos.y = y;
  destPos.t = t;
}

// stores encoder values in an array, allowing values >32,767, up to 2,147,483,647
task handleEncoders() {
  const ubyte kLoopDelay = kDefaultLoopDelay;

  int prevValue[kNumMotors];
  int encoderValue = 0;

  for(ubyte i=0; i<kNumMotors; i++) {
    nMotorEncoder[kMotorPort[i]] = 0;
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
    wait1Msec(kLoopDelay);
  }
}


task trackXYT() {
  const float kP = 1.0;
  const float kI = 0.5;
  const float kD = 1.0;
  const float kL = 50.0;
  const float kRotationScale = 1.0;
  const ubyte kLoopDelay = kDefaultLoopDelay;

  float gyro=0.0,lastGyro=0.0,avgGyro=0.0,
        robotX=0.0,robotY=0.0,
        radius=0.0,theta=0.0,
        fieldX=0.0,fieldY=0.0,
        error=0.0,prevError[kNumWheels],
        p=0.0,i[kNumWheels],d=0.0;
  ubyte xDivider=0,yDivider=0;
  word speed=0;
  long encoderValue[kNumDriveEncoders],
      lastEncoderValue[kNumDriveEncoders],
      deltaEncoderValue[kNumDriveEncoders];

  for(ubyte j=0; j<kNumDriveEncoders; j++) {
    encoderValue[j] =
    lastEncoderValue[j] =
    deltaEncoderValue[j] = 0.0;
  }
  for(ubyte j=0; j<kNumWheels; j++) {
    prevError[j] =
    i[j] = 0.0;
  }

  if(kDriveType == tankDrive || kDriveType == xDrive || kDriveType == mecanumDrive || kDriveType == plusDrive) {
    xDivider = 2;
    yDivider = 2;
  }
  else if(kDriveType == hDrive || kDriveType == kiwiDrive) {
    xDivider = 1;
    if(kDriveType == hDrive)
      yDivider = 2;
    else
      yDivider = 1;
  }

  while(true) {
    // save gyro and encoder values
    gyro = gyroOffset + SensorValue[kGyroPort]/10.0;
    for(ubyte j=0; j<kNumDriveEncoders; j++)
      encoderValue[j] = motorEncoder[kDriveEncoderMotorPort[j]];

    avgGyro = (gyro + lastGyro)/2; // calculate average gyro

    // calculate (x,y) movement with respect to robot
    robotX = robotY = 0.0;
    for(ubyte j=0; j<kNumDriveEncoders; j++) {
      deltaEncoderValue[j] = encoderValue[j] - lastEncoderValue[j];

      robotX += deltaEncoderValue[j]*cosDegrees(kDriveEncoderMotorAngle[j]);
      robotY += deltaEncoderValue[j]*sinDegrees(kDriveEncoderMotorAngle[j]);
    }
    robotX /= xDivider;
    robotY /= yDivider;

    // convert cartesian to polar
    radius = sqrt(pow(robotX,2) + pow(robotY,2)); // r^2 = x^2 + y^2
    theta = atan2(robotY,robotX)*180.0/PI; // tan(t) = y/x

    // add gyro value to get (x,y) movement with respect to field
    theta += avgGyro;
    fieldX = radius*cosDegrees(theta);
    fieldY = radius*sinDegrees(theta);

    // update current position (after converting from encoder ticks to inches)
    currPos.x += ticksToInches(fieldX);
    currPos.y += ticksToInches(fieldY);
    currPos.t = gyro;

    // update difference from destination
    diffPos.x = destPos.x - currPos.x;
    diffPos.y = destPos.y - currPos.y;
    diffPos.t = destPos.t - currPos.t;

    // convert from cartesian to polar
    radius = sqrt(pow(diffPos.x,2) + pow(diffPos.y,2)); // r^2 = x^2 + y^2
    theta = atan2(diffPos.y,diffPos.x)*180.0/PI; // tan(t) = y/x

    // subtract robot angle to get (x,y) difference with respect to robot
    theta -= gyro;
    robotX = radius*cosDegrees(theta);
    robotY = radius*sinDegrees(theta);

    // PID for moving to destination
    for(ubyte j=0; j<kNumWheels; j++) {
      error = robotX*cosDegrees(kDriveEncoderMotorAngle[j]) + robotY*sinDegrees(kDriveEncoderMotorAngle[j]) + diffPos.t*kRotationScale;

      p = error;
      i[j] = abs(i[j] + error) < kL ? i[j] + error : kL;
      d = error - prevError[j];

      speed = p*kP + i*kI + d*kD;

      prevError[j] = error;
    }

    // update "last" sensor reading variables
    for(ubyte i=0; i<kNumDriveEncoders; i++)
      lastEncoderValue[i] = encoderValue[i];
    lastGyro = gyro;

    wait1Msec(kLoopDelay);
  }
}


task userDriveHolo() {
  const TVexJoysticks kChY = Ch3; // y-axis joystick channel
  const TVexJoysticks kChX = Ch4; // x-axis joystick channel
  const TVexJoysticks kChR = Ch1; // rotation joystick channel
  const short kDelay = 25; // milliseconds for loop delay
  const ubyte kNumWheels = 4; // number of drive wheels
  const ubyte kNumMotorsPerWheel = 2; // max number of motors per wheel
  const tMotor kNone = -1; // used for indicating the lack of an additional motor
  const tMotor kMotorPort[kNumWheels][kNumMotorsPerWheel] = { // drive motor ports/names
    {port1, kNone}, // front-left
    {port2, kNone}, // front-right
    {port3, kNone}, // back-left
    {port4, kNone}  // back-right
  };

  word x,y,r;
  float gyro,radius,theta,a,b,wheelSpeed[kNumWheels],topSpeed;

  while(true) {
    // ==== collect joystick & sensor values ====
    x = vexRT[kChX]; // x component
    y = vexRT[kChY]; // y component
    r = vexRT[kChR]; // rotation
    gyro = gyroOffsetDriver + (doUseGyro ? SensorValue[kGyroPort]/10.0 : 0.0); // if using gyro, scale its value to degrees

    // ==== convert joystick values to polar ====
    radius = sqrt(pow(x,2) + pow(y,2)); // r = sqrt(x^2 + y^2)
    theta = atan2(y,x)*180.0/PI; // t = arctan(y/x) [converted from radians to degrees]

    theta -= gyro; // adjust for gyro angle

    // ==== calculate opposite-side speeds ====
    a = (cosDegrees(theta + 90.0) + sinDegrees(theta + 90.0))*radius; // front-left and back-right
    b = (cosDegrees(theta) + sinDegrees(theta))*radius; // front-right and back-left

    // ==== set speeds, including rotation ====
    wheelSpeed[0] = a + r; // front-left
    wheelSpeed[1] = b - r; // front-right
    wheelSpeed[2] = b + r; // back-left
    wheelSpeed[3] = a - r; // back-right

    // ==== normalize speeds ====
    topSpeed = 0.0;
    for(ubyte i=0; i<kNumWheels; i++)
      if(abs(wheelSpeed[i]) > topSpeed)
        topSpeed = abs(wheelSpeed[i]); // find highest desired speed
    if(topSpeed > 127.0)
      for(ubyte i=0; i<kNumWheels; i++)
        wheelSpeed[i] /= topSpeed/127.0; // downscale all speeds so none are above 127

    // ==== update motor powers ====
    for(ubyte i=0; i<kNumWheels; i++) // cycle through all wheels
      for(ubyte j=0; j<kNumMotorsPerWheel; j++) // cycle through all motors for each wheel
        if(kMotorPort[i][j] != kNone) // check existence of motor
          motor[kMotorPort[i][j]] = (word)wheelSpeed[i]; // update motor power

    wait1Msec(kDelay);
  }
}
