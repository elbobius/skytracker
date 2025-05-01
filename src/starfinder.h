#ifndef STARFINDER_H
#define STARFINDER_H

#include <gps.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int get_object_angles(double lat, double lon, const char *object, double *alt, double *az);

#endif // STARFINDER_H