#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <gps.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

typedef struct
{
    double latitude;
    double longitude;
    double altitude;
    int satellites_used;
    int satellites_visible;
    bool has_fix;
} GpsFix;

typedef struct
{
    double latitude;
    double longitude;
    double altitude;
} GpsPosition;

void* gps_thread(void* arg);
bool get_gps_data(GpsFix *fix);
void get_average_gps_data(double *latitude, double *longitude, double *altitude);
void get_satalite_data(int* used, int* inView);

bool gps_init();
void gps_cleanup();

#endif