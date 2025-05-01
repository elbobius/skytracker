#ifndef DRV8825_H
#define DRV8825_H

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>
#include <stdbool.h>
#include <wiringPi.h>

#define MAXSTEPPERS 2
#define DEFAULTSTEPDELAY 80

#define VERTDIRPIN 23
#define VERTENPIN 26
#define VERTSTEPPIN 24

#define HORIZDIRPIN 5
#define HORIZENPIN 7
#define HORIZSTEPPIN 1

#define VERTMICROSTEPS 32
#define HORIZMICROSTEPS 32

#define VERTDEGPERSTEP 0.189
#define HORIZDEGPERSTEP 1.26

#define DEFAULT_MINPULSE 300    // Example min pulse in microseconds
#define DEFAULT_MAXPULSE 600   // Example max pulse in microseconds
#define DEFAULT_ACCELRATE 0.4f   // Example acceleration rate (adjust as needed)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// direction enumeration
typedef enum { 
    DIR_CW = 1, 
    DIR_CCW = 0 
} StepperDirection;

typedef enum { 
    HORIZ = 1, 
    VERT = 0 
} StepperNumber;

// StepperParams structure for motor control
typedef struct {
    long steps;
    int dir;
    int maxPulseUs;
    int minPulseUs;
    float accelRate;
    int stepperNr;
} StepperParams;

// StepperControl structure for thread synchronization
typedef struct {
    StepperParams params;
    pthread_mutex_t lock;
    pthread_cond_t cond;
    bool hasWork;
} StepperControl;

// Stepper structure for holding stepper motor info
typedef struct {
    int dirPin;
    int enablePin;
    int stepPin;
    int microsteps;
    int stepDelayUs;
} Stepper;

// Global variables for stepper parameters
extern Stepper stepper[MAXSTEPPERS];
extern StepperControl vertControl;
extern StepperControl horizControl;

// Function prototypes
void *stepperThreadFunc(void *arg);
bool driveStepsAccel(long steps, int dir, StepperNumber steppernr);
void setVertNewAngle(double myAngle,double objectAngle);
void setHorizNewAngle(double myAngle,double objectAngle);
bool enableStepper(StepperNumber steppernr);
bool disableStepper(StepperNumber steppernr);
bool initStepper(int step, int enable, int dir, int microsteps, StepperNumber steppernr);
double getShortestAngel(double myAngle, double objectAngle);

#endif // DRV8825_H
