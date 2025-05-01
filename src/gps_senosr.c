#include "gps_sensor.h"

#define BUFFER_SIZE 100

struct gps_data_t *gps_data_ptr;

GpsPosition gpsBuffer[BUFFER_SIZE];
int gpsHead = 0;
int gpsCount = 0;
pthread_mutex_t gpsBuffer_mutex = PTHREAD_MUTEX_INITIALIZER;
int satUsed = 0;
int satInView = 0;

void *gps_thread(void *arg)
{
    while (1)
    {
        GpsFix newData;
        if (get_gps_data(&newData))
        {
            if (newData.has_fix)
            {
                pthread_mutex_lock(&gpsBuffer_mutex);
                gpsBuffer[gpsHead].altitude = newData.altitude;
                gpsBuffer[gpsHead].longitude = newData.longitude;
                gpsBuffer[gpsHead].latitude = newData.latitude;
                satUsed = newData.satellites_used;
                satInView = newData.satellites_visible;
                gpsHead = (gpsHead + 1) % BUFFER_SIZE;
                if (gpsCount < BUFFER_SIZE)
                    gpsCount++;
                pthread_mutex_unlock(&gpsBuffer_mutex);
            }
        }
    }
}

bool get_gps_data(GpsFix *fix)
{
    if (gps_waiting(gps_data_ptr, 5000000))
    {
        if (gps_read(gps_data_ptr, NULL, 0) == -1)
        {
            fprintf(stderr, "Error reading GPS data.\n");
            return false;
        }

        if (!isnan(gps_data_ptr->fix.latitude) && !isnan(gps_data_ptr->fix.longitude))
        {
            fix->latitude = gps_data_ptr->fix.latitude;
            fix->longitude = gps_data_ptr->fix.longitude;
            fix->altitude = gps_data_ptr->fix.altitude;
            fix->satellites_used = gps_data_ptr->satellites_used;
            fix->satellites_visible = gps_data_ptr->satellites_visible;
            fix->has_fix = true;
        }
        else
        {
            fix->has_fix = false;
        }
        return true;
    }
    return false;
}
void get_average_gps_data(double *latitude, double *longitude, double *altitude)
{

    double sumLat = 0;
    double sumLon = 0;
    double sumAlt = 0;
    pthread_mutex_lock(&gpsBuffer_mutex);
    for (int i = 0; i < gpsCount; i++)
    {
        sumLat += gpsBuffer[i].latitude;
        sumLon += gpsBuffer[i].longitude;
        sumAlt += gpsBuffer[i].altitude;
    }
    pthread_mutex_unlock(&gpsBuffer_mutex);
    *latitude = (gpsCount > 0) ? sumLat / gpsCount : 0.0;
    *longitude = (gpsCount > 0) ? sumLon / gpsCount : 0.0;
    *altitude = (gpsCount > 0) ? sumAlt / gpsCount : 0.0;
}

void get_satalite_data(int *used, int *inView)
{
    *used = satUsed;
    *inView = satInView;
}

bool gps_init()
{
    gps_data_ptr = malloc(sizeof(struct gps_data_t));
    if (!gps_data_ptr)
    {
        fprintf(stderr, "Memory allocation for gps_data_ptr failed.\n");
        return false;
    }
    if (gps_open("localhost", "2947", gps_data_ptr) != 0)
    {
        fprintf(stderr, "Error: Could not connect to gpsd.\n");
        return false;
    }
    gps_stream(gps_data_ptr, WATCH_ENABLE | WATCH_JSON, NULL);
    return true;
}

void gps_cleanup()
{
    gps_stream(gps_data_ptr, WATCH_DISABLE, NULL);
    gps_close(gps_data_ptr);
}