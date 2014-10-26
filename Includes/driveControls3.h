//  ROBOT SEEN FROM ABOVE
//
//        ♦ FRONT ♦
//      ♦           ♦
//    ♦  FL       FR  ♦
//            ↑
//           ←X→
//            ↓
//    ♦  BL       BR  ♦
//      ♦           ♦
//        ♦       ♦
/*
//==============================================================================\\
||Written By: Harrison Freni, Emma Taulli, and Madeleine Gradney.               ||
||With Help From: Jordan Kiesel(BNS), Elias Wu(BNSw), and Griffin Tabor(BNS)    ||
||Team 2150D is a Robotics Team based out of Chadwick School loacted on the     ||
||Palos Verdes Peninsula. Team 2150D was recreated during the Skyrise Season    ||
 \\=============================================================================//
*/

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
// Defines Controller Values
#define C1LX vexRT[Ch4]
#define C1LY vexRT[Ch3]
#define C1RX vexRT[Ch1]

const ubyte kNumMotors = 10; // Don't Change This Unless You have Some magical powers and have added more ports to the cortex.
const tMotor kMotorPort[kNumMotors] = {port1, port2, port3, port4, port5, port6, port7, port8, port9, port10};

const tSensors kGyroPort = in1; // Assign the Gyro Port Here
const ubyte kNumDriveMotorsPerAngle = 1; // Assign Number of Motors Per Side Here
const tMotor kDriveMotorPorts[4][kNumDriveMotorsPerAngle] = {{port2}, {port3}, {port4}, {port5}}; // {{left motors}, {right motors}}
const float kWheelDiameter = 3.25; // Set The Wheel Diameter To Its Size
const float kWheelCircumference = kWheelDiameter * PI; // This gets the circumference of the wheel

float driveTicksPerRev = 0; // Set TPR to 0 so RobotC can determine the stuffs
bool driveDone = false; // Boolean used to re-evalueate if the robot's operation is done.
bool turnDone = false; // Boolean used to re-evalueate if the robot's operation is done.
word maxSpeed = 127; // Again. Don't change this unless you have mystical powers, in which you then can.
word driveSpeed = 0; // Set Current Drive Speed to 0 for Later Modification
word turnSpeed = 0; // Set Turn Speed to 0 Unless moving, which then gets changed
long motorEncoder[kNumMotors]; // Pls, This is very important, it keeps track of which motors use encoders, See drivePID method for more info.
long setDistance = 0;
float setGyro = 0.0;


// Placeholder for X/Y Code Ideas
// x = nMotorEncoder[wheel] * cos(wheelAngle) // Do this for each wheel
// y = nMotorEncoder[wheel] * sin(wheelAngle) // ^^^^^^^^^^^^^^

// This creates an amazing array that makes the robot's speed/torque curve so linear I can't even #Whitegirl
sbyte trueSpeed(long input) // Use included Truespeed Writer to Create A better TS array for your robot.
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

// This Creates A Limit to which the encoders can reach, which is really high, because for whatever reason, RobotC messed up the perfectly fine encoders :C
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

// This Gives The Gyro A Zero Position, This should only be used at match start, this callibrates the Gyro TY BASED GYRO!!!!!
void calibrateGyro()
{
    // completely clear out any previous sensor readings by setting the port to "sensorNone"
    SensorType[kGyroPort] = sensorNone;
    wait1Msec(1000);
    SensorType[kGyroPort] = sensorGyro; // reconfigure as sensorGyro
    wait1Msec(2000); // wait for calibration: ROBOT MUST STAY STILL
    
    SensorScale[kGyroPort] = 138; // adjust SensorScale to correct the scaling for your gyro // You actually have to tune this well, unless you love a badly performing robot <3s
    SensorFullCount[kGyroPort] = 3599; // fix rollover to be "...3598, 3599, 0, 1..."
}

// stores encoder values in an array, allowing values >32,767, up to 2,147,483,647
task handleEncoders() {
    const ubyte kDelay = 25;
    signed int prevValue[kNumMotors];
    signed int encoderValue = 0;
    
    for(ubyte i=0; i<kNumMotors; i++)
    {
        motorEncoder[i] = 0;
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
/*
//======================================================================\\
|| This task makes sure the gyroscope on the robot is tuned in a certain||
|| way, in which it allows for extremely nice turning. This was created ||
|| for teams to create a robot that can turn easily.                    ||
\\======================================================================//
 */

task gyroPID()
{
    const ubyte kDelay = 25;
    const float kP = 1.0; // Yep... You have To Tune This :P YAY!!!! If You have never tuned PID before, pls read manual if U have, Then Pls Do So Now Starting with P
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

/*
 //======================================================================\\
 || This task makes sure the drivetrain functions properly in terms of   ||
 || forward and backward movement. The lateral movement is controlled by ||
 || the strafePID task, which keeps X constant, and moves along the Y    ||
 \\======================================================================//
 */

task drivePID()
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
    float i = 0.0;
    float d = 0.0;

    short speedFL = 0;
    short speedBL = 0;
    short speedFR = 0;
    short speedBR = 0;
    
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
        error = setDistance - currentDistance;
        
        p = error;
        i = abs(i + error) < kL ? i + error : kL;
        d = error - prevError;
        
        driveSpeed = p*kP + i*kI + d*kD;
        
        speedFL = limit(error * kP, maxSpeed);
        speedBL = limit(error * kP, maxSpeed);
        speedFR = limit(error * kP, maxSpeed);
        speedBR = limit(error * kP, maxSpeed);
        
        
        
        for(ubyte j=0; j<kNumDriveMotorsPerAngle; j++) {
            motor[kDriveMotorPort[j][0]] = trueSpeed(speedFL); // FL
            motor[kDriveMotorPort[j][0]] = trueSpeed(speedBL); // BL
            motor[kDriveMotorPort[j][0]] = trueSpeed(speedFR); // FR
            motor[kDriveMotorPort[j][0]] = trueSpeed(speedBR); // BR
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


void drive(string direction, const float kInches, const word kSpeed = 127)
{
    const long kDistance = kInches/driveTicksPerRev*kWheelCircumference;
    if(direction = "forward")
    {
        FL =  1 * limit(error * kP, maxSpeed);
        BL = -1 * limit(error * kP, maxSpeed);
        FR = -1 * limit(error * kP, maxSpeed);
        BR =  1 * limit(error * kP, maxSpeed);
    }
    else if(direction = "backward")
    {
        FL = -1;
        BL = 1;
        FR = 1;
        BR = -1;
    }
    maxSpeed = abs(kSpeed);
    setDistance += kDistance < 0 ? kDistance : kDistance*sgn(kSpeed);
}

void turn(const float kAngle, const word kSpeed = 127)
{
    maxSpeed = abs(kSpeed);
    setGyro += kAngle < 0 ? kAngle : kAngle*sgn(kSpeed);
}

// When Calculating Travel Distance, One wants to take the delta X value and add it to the delta Y valueand then multiply that by the sqrt of 2
// Add all encoder values, take distance and avg it, take the avg
void calculateDistanceTravel()
{
    float kInch = kWheelCircumference * motorEncoder[] / TPR_TORQUE;
    float revolutions = motorEncoder[port1] / TPR_TORQUE;
    float distanceTravel = kInch * revolutions;

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

// Redesign These Later For Comeptition
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

// Operator Control for Drive

void robotDriveComp()
{
    // Task Manager
    startTask(handleEncoders);
    stopTask(gyroPID);
    stopTask(drivePID);
    
    // Y component, X component, Rotation
    motor[FL] =  C1LY + C1LX + C1RX;
    motor[FR] = -C1LY + C1LX + C1RX;
    motor[BR] = -C1LY - C1LX + C1RX;
    motor[BL] =  C1LY - C1LX + C1RX;
        
    // Motor values can only be updated every 20ms
    wait10Msec(2);
}

// Documentation
// This Code Needs To Add X and Y together, and then divide by 2
// x = nMotorEncoder[wheel] * cos(wheelAngle)
// y = nMotorEncoder[wheel] * sin(wheelAngle)
/*
 if i want to go to 1000 ime value
 and im at 200 ime value
 then i only need to travel 800
*/