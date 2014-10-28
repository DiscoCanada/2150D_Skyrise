/*
 //==============================================================================\\
 ||Written By: Harrison Freni, Emma Taulli, and Madeleine Gradney.               ||
 ||With Help From: Jordan Kiesel(BNS), Elias Wu(BNSw), and Griffin Tabor(BNS)    ||
 ||Team 2150D is a Robotics Team based out of Chadwick School loacted on the     ||
 ||Palos Verdes Peninsula. Team 2150D was recreated during the Skyrise Season    ||
 \\=============================================================================//
 */

#include <FirmwareVersion.h> // the "TMotorType" values below have changed a bit between versions of RobotC

// Controller 1/2, Stick L/R, Axis X/Y
// Defines Controller Values
const ubyte kNumMotors = 10; // Don't Change This Unless You have Some magical powers and have added more ports to the cortex.
const tMotor kMotorPort[kNumMotors] = {port1, port2, port3, port4, port5, port6, port7, port8, port9, port10};

const ubyte kNumliftMotorsPerSegment = 1; // Assign Number of Motors Per Side Here
const tMotor kElevatorPorts[2][kNumliftMotorsPerSegment] = {{port6}, {port7}}; // {{left motors}, {right motors}};

bool liftDone = false; // Boolean used to re-evalueate if the robot's operation is done.
word maxSpeedLift = 127; // Again. Don't change this unless you have mystical powers, in which you then can.
word liftSpeed = 0; // Set Current lift Speed to 0 for Later Modification
word syncSpeed = 0; // Set Current sync Speed to 0 for Later Modification
long motorEncoder[kNumMotors]; // Pls, This is very important, it keeps track of which motors use encoders, See liftPID method for more info.
long setHeight = 0;
float kHeight = 0;

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
    float i = 0.0;
    float d = 0.0;

    short leftSideElev8 = 0;
    short rightSideElev8 = 0;

    while(true)
    {
        error = kHeight - motorEncoder[port7];

        //setheightkP(goalIME-current IME)
        p = error;
        i = abs(i + error) < kL ? i + error : kL;
        d = error - prevError;

        liftSpeed = p*kP + i*kI + d*kD;

        leftSideElev8  = setHeight;
        rightSideElev8 = setHeight;

        //rightlift=(normal input) -kP(rightIME-leftIME)
        //leftlift=normal input


        for(ubyte j=0; j<kNumliftMotorsPerSegment; j++)
        {
            motor[kElevatorPorts[j][0]] = leftSideElev8; // FL
            motor[kElevatorPorts[j][0]] = rightSideElev8; // BL
        }

        prevError = error;

        if(error < kDoneThreshold)
            if(time1[T1] > kSettleTime)
                liftDone = true;
            else
                clearTimer(T1);

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
        error = SensorValue[armLiftRPot] - SensorValue[armLiftLPot];

        p = error;
        i = abs(i + error) < kL ? i + error : kL;
        d = error - prevError;

        syncSpeed = p*kP + i*kI + d*kD;

        prevError = error;

        wait1Msec(kDelay);
    }
}

//void liftToHeight(string position, const Word kSpeed = 127 )
//{
    //kHeight = 0;

    //if (position = "LOW_GRAY_TUBE")
    //{
    //    kHeight = x;
    //}
    //else if(position = "MED_GRAY_TUBE1")
    //{
    //    kHeight = x;
    //}
    //else if(position = "MED_GRAY_TUBE2")
    //{
    //    kHeight = x;
    //}
    //else if(position = "HIGH_GRAY_TUBE")
    //{
    //    kHeight = x;
    //}
    //else if (position = "SKYRISE1")
    //{
    //    kHeight = x;
    //}
    //else if (position = "SKYRISE2")
    //{
    //    kHeight = x;
    //}
    //else if (position = "SKYRISE3")
    //{
    //    kHeight = x;
    //}
    //else if (position = "SKYRISE4")
    //{
    //    kHeight = x;
    //}
    //else if (position = "SKYRISE5")
    //{
    //    kHeight = x;
    //}

    //maxSpeed = abs(kSpeed);
    //setDistance += kHeight < 0 ? kHeight : kHeight*sgn(kSpeed);
//}
