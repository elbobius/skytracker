#include "drv8825.h"
#include "orientation.h"
#include "gps_sensor.h"
#include "starfinder.h"
#include <wiringPi.h>
#include <stdio.h>
#include <pthread.h>
#include <math.h>

int main()
{
    if (wiringPiSetup() < 0)
    {
        printf("wiringPi setup failed\n");
        return 1;
    }

    // Initialize the stepper motors
    initStepper(VERTSTEPPIN, VERTENPIN, VERTDIRPIN, VERTMICROSTEPS, VERT);      // pins and parameters for vertical motor
    initStepper(HORIZSTEPPIN, HORIZENPIN, HORIZDIRPIN, HORIZMICROSTEPS, HORIZ); // pins and parameters for horizontal motor

    disableStepper(VERT);
    disableStepper(HORIZ);

    if (!lsm6ds3trc_init())
    {
        fprintf(stderr, "Failed to initialize LSM6DS3TRC\n");
        exit(1);
    }
    if (!lis3mdl_init())
    {
        fprintf(stderr, "Failed to initialize LIS3MDL\n");
        exit(1);
    }

    gps_init();

    // Create threads
    pthread_t vertMotorThread, horizMotorThread;
    if (pthread_create(&vertMotorThread, NULL, stepperThreadFunc, &vertControl))
    {
        perror("Failed to create vertMotorThread");
        return 1;
    }

    if (pthread_create(&horizMotorThread, NULL, stepperThreadFunc, &horizControl))
    {
        perror("Failed to create horizMotorThread");
        return 1;
    }

    pthread_t vertSensorThread;
    if (pthread_create(&vertSensorThread, NULL, sensor_thread, NULL) != 0)
    {
        perror("Failed to create vertSensorThread");
        return 1;
    }

    pthread_t gpsSensorThread;
    if (pthread_create(&gpsSensorThread, NULL, gps_thread, NULL) != 0)
    {
        perror("Failed to create gpsSensorThread");
        return 1;
    }
    // get gps data before steppers are activated
    double gpsLa = 0;
    double gpsLo = 0;
    double gpsAl = 0;
    int usedSat = 0;
    int viewedSat = 0;
    while (gpsLa == 0 || gpsLo == 0 || gpsAl == 0 || usedSat < 3)
    {
        get_average_gps_data(&gpsLa, &gpsLo, &gpsAl);
        get_satalite_data(&usedSat, &viewedSat);
        printf("waiting for gps data\n");
        printf("Satelites used: %d\n", usedSat);
        printf("Satelites in view: %d\n", viewedSat);
        delay(100);
    }
    printf("GPS Latitude: %.3f\n", gpsLa);
    printf("GPS Longitude: %.3f\n", gpsLo);
    printf("GPS Altitude: %.3f\n", gpsAl);

    // activate the stepper motors
    enableStepper(0);
    enableStepper(1);
    //disableStepper(0);
    //disableStepper(1);
    // loop to continously get sensor data and conrtrole the stepper motor according to the data
    for (int i = 0; i < 50; i++)
    {
        sleep(5);
        double myAz = get_average_heading();
        double myAl = get_average_elevation();
        double altObject;
        double azObject;
        get_object_angles(gpsLa, gpsLo, "moon", &altObject, &azObject);
        printf("*****************************************************************\n");
        printf("Azimuth(vertical):    %.3f  Object: %0.3f  dif: %0.3f\n", myAz, azObject, getShortestAngel(azObject,myAz));
        printf("Altitude(horizontal): %.3f  Object: %0.3f  dif: %0.3f\n", myAl, altObject, getShortestAngel(altObject,myAl));

        setVertNewAngle(altObject,myAl);    // sets vertParams.steps, and the thread picks it up
        setHorizNewAngle(azObject,myAz);    // sets horizParams.steps, and the thread picks it up
    }
    disableStepper(0);
    disableStepper(1);
    // Wait for threads to finish
    pthread_join(vertMotorThread, NULL);
    pthread_join(horizMotorThread, NULL);

    // detach from endless threads
    pthread_detach(vertSensorThread);
    pthread_detach(gpsSensorThread);

    gps_cleanup();

    return 0;
}