/*======================================================*\
|                   Drive Controls V 4                   |
| Written By: Harrison Freni on Team 2150D               |
| What it does: This year, our robot utilizes a holonomic|
| drivetrain, which made us write up some custom PID     |
| control schemes for our robot. This allows us to write |
| intricate autonomous routines that can score really    |
| well.                                                  |
| Protected Under the LGPL v3                            |
| Helpers: Emma Taulli, Madeliene Gradney, Devin Codon   |
| Jordan Taylor-Kiesel (Team BNS), and Griffin Tabor(BNS)|
\*======================================================*/

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

// Controller 1/2, Stick L/R, Axis X/Y
#define C1LX vexRT[Ch4]
#define C1LY vexRT[Ch3]
#define C1RX vexRT[Ch1]



/*======================================================*\
|                          Variables                     |
| All of these variables allow for fast/easy programming |
| for our PID Loops for the Drive and Lift. Recently, We |
| have developed some code that makes the robot's speed  |
| far more linear than that of the standard Vex kit.     |
\*======================================================*/

// 1. Don't Change This, It tells the cortex how many motor ports there are.
const ubyte kNumMotors = 10;
const tMotor kMotorPort[kNumMotors] = {port1, port2, port3, port4, port5, port6, port7, port8, port9, port10};

// 2. Change These At Will
const tSensors kGyroPort = in1; // Change in1 to either: Gyro's Name, or Sensor Port, Here We Assume That the Gyro is on Analog Port 1
const ubyte kNumDriveMotorsPerAngle = 1; // Since We are Using A Holonomic Drive, We created a way to Make Use of an Array to store motor info.
const tMotor kDriveMotorPorts[4][kNumDriveMotorsPerAngle] = {{port2}, {port3}, {port4}, {port5}}; // {{left motors}, {right motors}} This is a 2D array, so just fill in motor info
const float kWheelDiameter = 3.25; // Set The Wheel Diameter To Its Size
const float kWheelCircumference = kWheelDiameter * PI; // This gets the circumference of the wheel

// 3. Some More Stuff That Gets Called At A Later Period in Time, Please DON'T Change These.
float driveTicksPerRev = 0; // Set TPR to 0 so RobotC can determine the actual TPR based on previous input.
bool driveDone = false; // Boolean used to re-evalueate if the robot's driving/strafing operation is done.
bool turnDone = false; // Boolean used to re-evalueate if the robot's truning operation is done.
word maxSpeed = 127; // Again. Don't change this unless you have mystical powers, in which you then can.
word driveSpeed = 0; // Set Current Drive Speed to 0 for Later Modification
word turnSpeed = 0; // Set Turn Speed to 0 Unless moving, which then gets changed
long motorEncoderDrive[kNumMotors]; // Keeps Track of Which Motors Use Encoders
long setDistance = 0; // Don't Change This
float setGyro = 0.0; // Don't Change This
float encoderAverage = abs(nMotorEncoder[FL] + nMotorEncoder[FR] + nMotorEncoder[BL] + nMotorEncoder[BR]) / 4;
float polaritySpeedFL = 0;
float polaritySpeedBL = 0;
float polaritySpeedFR = 0;
float polaritySpeedBR = 0;

// 4. used to limit the input to the given bounds (max limit is ± if no min limit is given)
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

// 5. This Calibrates The Gyro, but It takes some time.
void calibrateGyro()
{
    // completely clear out any previous sensor readings by setting the port to "sensorNone"
    SensorType[kGyroPort] = sensorNone;
    wait1Msec(1000);
    SensorType[kGyroPort] = sensorGyro; // reconfigure as sensorGyro
    wait1Msec(2000); // wait for calibration: ROBOT MUST STAY STILL

    SensorScale[kGyroPort] = 125; // adjust SensorScale to correct the scaling for your gyro // You actually have to tune this well, unless you love a badly performing robot <3
    SensorFullCount[kGyroPort] = 3599; // fix rollover to be "...3598, 3599, 0, 1..."
}

// 6. Stores encoder values in an array, allowing values >32,767, up to 2,147,483,647
task handleEncoders() {
    const ubyte kDelay = 25;
    signed int prevValue[kNumMotors];
    signed int encoderValue = 0;

    for(ubyte i=0; i<kNumMotors; i++)
    {
        motorEncoderDrive[i] = 0;
        prevValue[i] = 0;
    }
    while(true) {
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

void driveForward()
{
    polaritySpeedFL =  1;
    polaritySpeedBL = -1;
    polaritySpeedFR = -1;
    polaritySpeedBR =  1;
}

void driveBackward()
{
    polaritySpeedFL = -1;
    polaritySpeedBL =  1;
    polaritySpeedFR =  1;
    polaritySpeedBR = -1;
}

void strafeLeft()
{
    polaritySpeedFL =  1;
    polaritySpeedBL =  1;
    polaritySpeedFR = -1;
    polaritySpeedBR = -1;
}

void strafeRight()
{
    polaritySpeedFL = -1;
    polaritySpeedBL = -1;
    polaritySpeedFR =  1;
    polaritySpeedBR =  1;
}

/*=======================================================*\
|                          PID TASKS                      |
| The subsequent tasks will allow for the robot's         |
| movement be slightly more predictable, which allows us  |
| to write far more complex/efficient autonomous routines |
| We use calculus to calculate the current error(How far  |
| away we are from our target value.) Our previous errors |
| (how far away we WERE) and Future (How far away we will |
| be). This uses a very simple to understand equation     |
\*=======================================================*/

task drivePID()
{
    const ubyte kDelay = 25;
    const float kP = 0.5;
    const float kI = 0.6;
    const float kD = 0.3;
    const float kL = 50.0;
    const tSensors kEncoderMotorPort[4] = {port2, port3, port4, port5}; // {left encoder x1,left encoder x2, right encoder x1,right encoder x2}
    const TMotorTypes kMotorType = motorType[kEncoderMotorPort[0]]; // save encoder motor's type
    const float kDoneThreshold = 1.0;
    const short kSettleTime = 250;

    long speedFL = 0;
    long speedBL = 0;
    long speedFR = 0;
    long speedBR = 0;

    float error = 0.0;
    float prevError = 0.0;
    float p = 0.0;
    float i = 0.0;
    float d = 0.0;

        // ==== determine number of ticks per revolution ====
    if(kMotorType == tmotorVex269 || kMotorType == tmotorVex269_HBridge || kMotorType == tmotorVex269_MC29)
        driveTicksPerRev = 240.448; // 269
    else if(kMotorType == tmotorVex393 || kMotorType == tmotorVex393_HBridge || kMotorType == tmotorVex393_MC29)
        driveTicksPerRev = 627.2; // 393
    else if(kMotorType == tmotorVex393HighSpeed || kMotorType == tmotorVex393HighSpeed_HBridge || kMotorType == tmotorVex393HighSpeed_MC29)
        driveTicksPerRev = 392; // 393 high speed
    else if(kMotorType == tmotorVex393TurboSpeed_HBridge || kMotorType == tmotorVex393TurboSpeed_MC29)
        driveTicksPerRev = 261.333; // 393 turbo speed

    while(true)
    {
    	  error = setDistance - SensorValue[kGyroPort];

        p = error; // Calculates How Far Away Yo Have To Go
        i = abs(i + error) < kL ? i + error : kL;
        d = error - prevError;

        driveSpeed = p*kP + i*kI + d*kD;

        speedFL = limit(driveSpeed, maxSpeed);
        speedBL = limit(driveSpeed, maxSpeed);
        speedFR = limit(driveSpeed, maxSpeed);
        speedBR = limit(driveSpeed, maxSpeed);

        // Set These To True Speed Later

        for(ubyte j=0; j<kNumDriveMotorsPerAngle; j++)
        {
            motor[kDriveMotorPorts[j][0]] = speedFL * polaritySpeedFL; // FL
            motor[kDriveMotorPorts[j][0]] = speedBL * polaritySpeedBL; // BL
            motor[kDriveMotorPorts[j][0]] = speedFR * polaritySpeedFR; // FR
            motor[kDriveMotorPorts[j][0]] = speedBR * polaritySpeedBR; // BR
        }
        prevError = error;

        if(error < kDoneThreshold)
            if(time1[T2] > kSettleTime)
                turnDone = true;
            else
                clearTimer(T2);

        wait1Msec(kDelay);

		}
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
            if(time1[T2] > kSettleTime)
                turnDone = true;
            else
                clearTimer(T2);

        wait1Msec(kDelay);

		}
}

void drive(short kDirection, const float kInches, const word kSpeed = 127)
{
	// This Sets The Drive
// On Drive:
// 0 is Forward
// 1 is Backward
// 2 is Strafe Left
// 3 is Strafe Right

    const long kDistance = kInches / kWheelCircumference * driveTicksPerRev;

    if(kDirection == 0)
    {
        driveForward();
    }
    else if(kDirection == 1)
    {
        driveBackward();
    }
    else if(kDirection == 2)
    {
        strafeLeft();
    }
    else if(kDirection == 3)
    {
        strafeRight();
    }

    maxSpeed = abs(kSpeed);
    setDistance += kDistance < 0 ? kDistance : kDistance*sgn(kSpeed);
}

void turn(const float kAngle, const word kSpeed = 127)
{
    maxSpeed = abs(kSpeed);
    setGyro += kAngle < 0 ? kAngle : kAngle*sgn(kSpeed);
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

void getDistanceTraveled()
{
    float kInch = kWheelCircumference * encoderAverage / driveTicksPerRev;
    float revolutions = encoderAverage / driveTicksPerRev;
    float distanceTravel = kInch * revolutions;
}

void driveControl()
{
    while(true)
    {
        // Deadzone
        if(abs(C1LX) < 15)
        {
            motor[FL] = 0;
            motor[BL] = 0;
            motor[FR] = 0;
            motor[BR] = 0;
        }
        // Deadzone
        if(abs(C1LY) < 15)
        {
            motor[FL] = 0;
            motor[BL] = 0;
            motor[FR] = 0;
            motor[BR] = 0;
        }
        // Deadzone
        if(abs(C1RX) < 15)
        {
            motor[FL] = 0;
            motor[BL] = 0;
            motor[FR] = 0;
            motor[BR] = 0;
        }
  
    }
}
