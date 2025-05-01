#include "drv8825.h"

Stepper stepper[MAXSTEPPERS];
StepperControl vertControl = {.lock = PTHREAD_MUTEX_INITIALIZER, .cond = PTHREAD_COND_INITIALIZER, .hasWork = false};
StepperControl horizControl = {.lock = PTHREAD_MUTEX_INITIALIZER, .cond = PTHREAD_COND_INITIALIZER, .hasWork = false};

// Drive stepper motor with acceleration
bool driveStepsAccel(long steps, int dir, StepperNumber steppernr)
{
    /*if ((steppernr != HORIZ && steppernr != VERT) || dir < 0 || dir > 1)
        return false;
    if (steppernr == HORIZ)
        printf("--> horiz: ");
    else
        printf("--> vert: ");
    printf("drive steps: %ld direction: %d\n", steps, dir);*/
    digitalWrite(stepper[steppernr].dirPin, dir);

    int halfSteps = steps / 2;
    int currentPulseDuration = DEFAULT_MAXPULSE;

    // Acceleration phase
    for (int i = 0; i < halfSteps; i++)
    {
        float t = (float)i / halfSteps;
        currentPulseDuration = DEFAULT_MAXPULSE - (DEFAULT_MAXPULSE - DEFAULT_MINPULSE) * (0.5f - 0.5f * cosf(M_PI * t * DEFAULT_ACCELRATE));

        digitalWrite(stepper[steppernr].stepPin, HIGH);
        delayMicroseconds(currentPulseDuration / 2);
        digitalWrite(stepper[steppernr].stepPin, LOW);
        delayMicroseconds(currentPulseDuration / 2);
    }

    // Constant speed phase
    currentPulseDuration = DEFAULT_MINPULSE;
    for (int i = halfSteps; i < steps - halfSteps; i++)
    {
        digitalWrite(stepper[steppernr].stepPin, HIGH);
        delayMicroseconds(currentPulseDuration / 2);
        digitalWrite(stepper[steppernr].stepPin, LOW);
        delayMicroseconds(currentPulseDuration / 2);
    }

    // Deceleration phase
    for (int i = steps - halfSteps; i < steps; i++)
    {
        float t = (float)(steps - i) / halfSteps;
        currentPulseDuration = DEFAULT_MAXPULSE - (DEFAULT_MAXPULSE - DEFAULT_MINPULSE) * (0.5f - 0.5f * cosf(M_PI * t * DEFAULT_ACCELRATE));

        digitalWrite(stepper[steppernr].stepPin, HIGH);
        delayMicroseconds(currentPulseDuration / 2);
        digitalWrite(stepper[steppernr].stepPin, LOW);
        delayMicroseconds(currentPulseDuration / 2);
    }

    return true;
}

// Thread function for controlling stepper motors
void *stepperThreadFunc(void *arg)
{
    StepperControl *ctrl = (StepperControl *)arg;

    while (1)
    {
        pthread_mutex_lock(&ctrl->lock);
        while (!ctrl->hasWork)
        {
            pthread_cond_wait(&ctrl->cond, &ctrl->lock);
        }

        // Copy the current parameters to avoid race conditions
        StepperParams work = ctrl->params;
        ctrl->hasWork = false;
        pthread_mutex_unlock(&ctrl->lock);

        //printf("Processing motor %d with %ld steps\n", work.stepperNr, work.steps);

        // Perform the steps
        driveStepsAccel(work.steps, work.dir, work.stepperNr);
    }
    return NULL;
}

// Function to set a new vertical angle
void setVertNewAngle(double myAngle, double objectAngle)
{
    double calcAngle= getShortestAngel(myAngle,objectAngle);

    pthread_mutex_lock(&vertControl.lock);
    vertControl.params.stepperNr = VERT;
    if (calcAngle > 0)
        vertControl.params.dir = DIR_CW;
    else
        vertControl.params.dir = DIR_CCW;
    long steps = (long)(fabs(calcAngle) / (VERTDEGPERSTEP / (double)stepper[VERT].microsteps));
    //printf(" --> vert angle: %f calc steps = %ld\n", calcAngle, steps);
    vertControl.params.steps = steps;
    vertControl.hasWork = true;
    pthread_cond_signal(&vertControl.cond); // Notify the thread
    pthread_mutex_unlock(&vertControl.lock);
}

// Function to set a new horizontal angle
void setHorizNewAngle(double myAngle, double objectAngle)
{
    double calcAngle= getShortestAngel(myAngle,objectAngle);

    pthread_mutex_lock(&horizControl.lock);
    horizControl.params.stepperNr = HORIZ;
    if (calcAngle > 0)
        horizControl.params.dir = DIR_CCW;
    else
        horizControl.params.dir = DIR_CW;
    long steps = (long)(fabs(calcAngle) / (HORIZDEGPERSTEP / (double)stepper[HORIZ].microsteps));
    //printf(" --> horiz angle: %f calc steps = %ld\n", calcAngle, steps);
    horizControl.params.steps = steps;
    horizControl.hasWork = true;
    pthread_cond_signal(&horizControl.cond); // Notify the thread
    pthread_mutex_unlock(&horizControl.lock);
}

// Enable a stepper motor
bool enableStepper(StepperNumber steppernr)
{
    if (steppernr == HORIZ || steppernr == VERT)
    {
        digitalWrite(stepper[steppernr].enablePin, HIGH);
        printf("stepper %d enabled\n", steppernr);
        return true;
    }
    return false;
}

// Disable a stepper motor
bool disableStepper(StepperNumber steppernr)
{
    if (steppernr >= 0 && steppernr < MAXSTEPPERS)
    {
        digitalWrite(stepper[steppernr].enablePin, LOW);
        printf("stepper %d disabled\n", steppernr);
        return true;
    }
    return false;
}

// Initialize a stepper motor
bool initStepper(int step, int enable, int dir, int microsteps, StepperNumber steppernr)
{
    if (steppernr == HORIZ || steppernr == VERT)
    {
        stepper[steppernr].dirPin = dir;
        stepper[steppernr].enablePin = enable;
        stepper[steppernr].stepPin = step;
        stepper[steppernr].microsteps = microsteps;
        stepper[steppernr].stepDelayUs = 0;

        pinMode(stepper[steppernr].dirPin, OUTPUT);
        pinMode(stepper[steppernr].enablePin, OUTPUT);
        pinMode(stepper[steppernr].stepPin, OUTPUT);

        digitalWrite(stepper[steppernr].enablePin, LOW);
        return true;
    }
    return false;
}

// helper functions
double getShortestAngel(double myAngle, double objectAngle)
{
    double calcAngle;
    if ((myAngle - objectAngle) >= 180)
        calcAngle = myAngle - objectAngle - 360;
    else if ((myAngle - objectAngle) <= -180)
        calcAngle = myAngle - objectAngle + 360;
    else
        calcAngle = myAngle - objectAngle;
    return calcAngle;
}