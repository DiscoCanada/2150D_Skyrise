// Autotuner Ideas


long kP = 0;
long kI = 0;
long kD = 0;
long tU = 0; // User Input for now
long kU = 0; // User Input for now

// P loop tuning
void tuneP()
{
    kP = 0.5 * kU;
}

// PI loop tuning
void tunePI()
{
    kP = 0.45 * kU;
    kI = 1.2 * kP / tU;
}

// PD loop tuning
void tunePD()
{
    kP = 0.8 * kU;
    kD = kp * tU / 8;
}

// PID loop tuning
void tunePID()
{
    kP = 0.60 * kU;
    kI = 2 * kP * tU;
    kD = kP * tU / 8;
}

//Passen Rule loop tuning
void tunePassen()
{
    kP = 0.7 * kU;
    kI = 0.4 * kP * tU;
    kD = 0.15 * kP * tU;
}

// Some Overshoot tuning
void tuneOvershoot()
{
    kP = 0.33 * kU;
    kI = 2 * kP / tU;
    kD = kP * tU / 3;
}

// No Overshoot tuning
void tuneOvershoot()
{
    kP = 0.2 * kU;
    kI = 2 * kP / tU;
    kD = kP * tU / 3;
}