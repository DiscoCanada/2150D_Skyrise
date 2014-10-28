/*==========================================*\
|                   SENSOR PID               |
| This was written as a proof of concept, may|
| not work well.                             |
| Written By: Harrison Freni(2150D)          |
\*==========================================*/

long desiredLine = 0;

task lineSensorPID()
{
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
    
    while(true)
    {
        error = desiredLine - sensorValue[lineSensor1];
        
        p = error;
        i = abs(i + error) < kL ? i + error : kL;
        d = error - prevError;
        
        lineSensorSpeed = p*kP + i*kI + d*kD;
        
        prevError = error;
        
        wait1Msec(kDelay);
    }
}

void lineDerp()
{
    desiredLine = 0; // Enter Value Here
}