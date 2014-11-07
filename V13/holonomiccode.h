// FL Is Same
// Reverse FR
// Reverse BL
// BR Is Same

const tSensors kGyroPort = in1; // gyroscopic sensor port/name

float kP=0.3;
float kDriveWidth = 7.5;
float kConversionFactor=627.2/(3.14*3.25*1.41)
float kTurnConversionFactor = nMotorEncoder[br]/kTicksPerRev*kWheelSize*pow(PI,2)*kDriveWidth/360;
float kAngles;
float gyroOffset = 90.0; // degrees CCW from +x axis robot faces at gyro=0
bool finished=true;
bool doUseGyro = false; // enable gyroscopic sensor
int kFlGoal;
int kFrGoal;
int kBlGoal;
int kBrGoal;
int kAnglesFR;
int kAnglesFL;
int kAnglesBL;
int kAnglesBR;

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

void turn(float angle, float direction)// Direction 1 = (Right 0 - 180) Direction 2 = (Left 181 - 360)
{
    if(direction == 1) // Turn Right
    {
        while(!finished){}
        kAnglesFL = angle * kTurnConversionFactor;
        -kAnglesFR = angle * kTurnConversionFactor;
        -kAnglesBL = angle * kTurnConversionFactor;
        kAnglesBR = angle * kTurnConversionFactor;
    }
    if (direction == 2)
    {
        while(!finished){}
        -kAnglesFL = angle * kTurnConversionFactor;
        kAnglesFR = angle * kTurnConversionFactor;
        kAnglesBL = angle * kTurnConversionFactor;
        -kAnglesBR = angle * kTurnConversionFactor;
    }
}

void forward(float distance)
{
    while(!finished){}
    kFlGoal=distance*kConversionFactor;
    kFrGoal=distance*kConversionFactor;
    kBlGoal=distance*kConversionFactor;
    kBrGoal=distance*kConversionFactor;
}
void backward(float distance)
{
    while(!finished){}
    kFlGoal=-distance*kConversionFactor;
    kFrGoal=-distance*kConversionFactor;
    kBlGoal=-distance*kConversionFactor;
    kBrGoal=-distance*kConversionFactor;
}
void strafeleft(float distance)
{
    while(!finished){}
    kFlGoal=-distance*kConversionFactor;
    kFrGoal=distance*kConversionFactor;
    kBlGoal=distance*kConversionFactor;
    kBrGoal=-distance*kConversionFactor;
}
void straferight(float distance)
{
    while(!finished){}
    kFlGoal=distance*kConversionFactor;
    kFrGoal=-distance*kConversionFactor;
    kBlGoal=-distance*kConversionFactor;
    kBrGoal=distance*kConversionFactor;
}

task drivePID()
{
	while(1)
    {
        while((abs(kFlGoal-SensorValue(fl))>50) && abs(abs(kFrGoal-SensorValue(fr))>50) && (abs(kBlGoal-SensorValue(bl))>50) && (abs(kBrGoal-SensorValue(br))>50)) //probably should do all 4 but i was lazy
		{
			finished=false;
            motor[fl]=kP*(flgoal-SensorValue(fl));
            motor[fr]=kP*(frgoal-SensorValue(fr));
            motor[bl]=kP*(blgoal-SensorValue(bl));
            motor[br]=kP*(brgoal-SensorValue(br));
        }
        if(abs(kFlGoal-SensorValue(fl))<=50 && abs(kFrGoal-SensorValue(fr))<=50 && abs(kBlGoal-SensorValue(bl))<=50 && abs(kBrGoal-SensorValue(br))<=50)
        {
            finished=true;
            flgoal=0;
            frgoal=0;
            blgoal=0;
            brgoal=0;
            SensorValue(fl)=0;
            SensorValue(fr)=0;
            SensorValue(bl)=0;
            SensorValue(br)=0;
        }
    }
}

// So circumference formula is PI*d
task turnPID()
{
    while (1)
    {
        while ((abs(kAnglesFL - SensorValue(fl))>50) && (abs(kAnglesFR - SensorValue(fr)>50) && (abs(kAnglesBL - SensorValue(bl)>50) && (abs(kAnglesBR - SensorValue(br)>50))
        {
            finished=false;
            motor[fl]=kP*(kAnglesFL-SensorValue(fl));
            motor[fr]=kP*(kAnglesFR-SensorValue(fr));
            motor[bl]=kP*(kAnglesBL-SensorValue(bl));
            motor[br]=kP*(kAnglesBR-SensorValue(br));
        }
        if((abs(kAnglesFL - SensorValue(fl))<=50) && (abs(kAnglesFR - SensorValue(fr)<=50) && (abs(kAnglesBL - SensorValue(bl)<=50) && (abs(kAnglesBR - SensorValue(br)<=50))
        {
            finished=true;
            kAnglesFL = 0;
            kAnglesFR = 0;
            kAnglesBL = 0;
            kAnglesBR = 0;
            SensorValue(fl) = 0;
            SensorValue(fr) = 0;
            SensorValue(bl) = 0;
            SensorValue(br) = 0;
        }
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
        {fl, kNone}, // front-left
        {fr, kNone}, // front-right
        {bl, kNone}, // back-left
        {br, kNone}  // back-right
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
//find distance from center of robot to center of a wheel
//PI*kDriveWidth/360.
