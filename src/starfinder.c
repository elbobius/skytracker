#include "starfinder.h"

int get_object_angles(double lat, double lon, const char *object, double *alt, double *az)
{
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
             "python3 python/moonposition.py %f %f %s",
             lat, lon, object);

    FILE *fp = popen(cmd, "r");
    if (!fp)
    {
        perror("Failed to run Python script");
        return -1;
    }

    if (fscanf(fp, "%lf,%lf", alt, az) != 2)
    {
        fprintf(stderr, "Failed to parse output from Python script\n");
        pclose(fp);
        return -1;
    }

    pclose(fp);
    return 0;
}