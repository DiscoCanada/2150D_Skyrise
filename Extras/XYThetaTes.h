/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*                        Copyright (c) Jordan Kiesel                          */
/*                                   2014                                      */
/*                            All Rights Reserved                              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Module:     HolonomicDriveControl.c                                      */
/*    Author:     Jordan Kiesel                                                */
/*    Created:    10 Aug 2014                                                  */
/*                                                                             */
/*    Revisions:                                                               */
/*                V1.00  10 Aug 2014 - Initial release                         */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    The author is supplying this software for use with the VEX cortex        */
/*    control system. This file can be freely distributed and teams are        */
/*    authorized to freely use this program, however, it is requested that     */
/*    improvements or additions be shared with the VEX community via the VEX   */
/*    Forum. Please acknowledge the work of the authors when appropriate.      */
/*    Thanks.                                                                  */
/*                                                                             */
/*    Licensed under the Apache License, Version 2.0 (the "License");          */
/*    you may not use this file except in compliance with the License.         */
/*    You may obtain a copy of the License at                                  */
/*                                                                             */
/*      http://www.apache.org/licenses/LICENSE-2.0                             */
/*                                                                             */
/*    Unless required by applicable law or agreed to in writing, software      */
/*    distributed under the License is distributed on an "AS IS" BASIS,        */
/*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*    See the License for the specific language governing permissions and      */
/*    limitations under the License.                                           */
/*                                                                             */
/*    The author can be contacted on the VEX Forum as LegoMindstormsmaniac     */
/*    email: legomindstormsmaniac@gmail.com                                    */
/*    Member of VEXU team BNS, Bots 'n' Stuff Robotics, Princeton NJ.          */
/*    Mentor for VRC team 24, Super Sonic Sparks, New Windsor MD.              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*    Description:                                                             */
/*                                                                             */
/*    The userDriveHolo task is designed to control a holonomic drive and      */
/*    utilizes a gyroscopic sensor to determine the direction to drive in      */
/*    order to move in the direction the user pushes the joystick, regardless  */
/*    of the robot's current orientation. Gyroscopic sensor values, joystick   */
/*    values, and preset drive statistics are used to calculate the required   */
/*    speed for each wheel in order to achieve the desired movement.           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

const tSensors kGyroPort = in1; // gyroscopic sensor port/name

bool doUseGyro = true; // enable gyroscopic sensor
float gyroOffset = 90.0; // degrees CCW from +x axis robot faces at gyro=0

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
        {port3, port5}, // back-left
        {port4, port6}  // back-right
    };
    
    word x,y,r;
    float gyro,radius,theta,a,b,wheelSpeed[kNumWheels],topSpeed;
    
    while(true) {
        // ==== collect joystick & sensor values ====
        x = vexRT[kChX]; // x component
        y = vexRT[kChY]; // y component
        r = vexRT[kChR]; // rotation
        gyro = gyroOffset + (doUseGyro ? SensorValue[kGyroPort]/10.0 : 0.0); // if using gyro, scale its value to degrees
        
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

task main() {
    calibrateGyro();
    startTask(userDriveHolo);
    
    while(true)
        wait1Msec(100);
}