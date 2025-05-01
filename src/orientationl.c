
#include "orientation.h"

#define BUFFER_SIZE 500

// Ring buffer
// float pitchBuffer[BUFFER_SIZE];
// float rollBuffer[BUFFER_SIZE];
float headBuffer[BUFFER_SIZE];
float elevBuffer[BUFFER_SIZE];
/*
int pitchHead = 0;
int pitchCount = 0;
int rollHead = 0;
int rollCount = 0;
*/
int headHead = 0;
int headCount = 0;
int elevHead = 0;
int elevCount = 0;
/*
pthread_mutex_t pitchBuffer_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t rollBuffer_mutex = PTHREAD_MUTEX_INITIALIZER;
*/
pthread_mutex_t headBuffer_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t elevBuffer_mutex = PTHREAD_MUTEX_INITIALIZER;
int fd_lsm6ds3trc, fd_lis3mdl;

// Thread function: reads sensor and writes to ring buffer
void *sensor_thread(void *arg)
{
    while (1)
    {
        /*
        float pitch;
        float roll;
        get_pitch_roll(&pitch, &roll);
        */

        float heading;
        float elevation;
        get_heading_orientation(&heading, &elevation);

        /*
        pthread_mutex_lock(&pitchBuffer_mutex);
        pitchBuffer[pitchHead] = pitch;
        pitchHead = (pitchHead + 1) % BUFFER_SIZE;
        if (pitchCount < BUFFER_SIZE)
            pitchCount++;
        pthread_mutex_unlock(&pitchBuffer_mutex);

        pthread_mutex_lock(&rollBuffer_mutex);
        rollBuffer[rollHead] = roll;
        rollHead = (rollHead + 1) % BUFFER_SIZE;
        if (rollCount < BUFFER_SIZE)
            rollCount++;
        pthread_mutex_unlock(&rollBuffer_mutex);
        */
        pthread_mutex_lock(&headBuffer_mutex);
        headBuffer[headHead] = heading;
        headHead = (headHead + 1) % BUFFER_SIZE;
        if (headCount < BUFFER_SIZE)
            headCount++;
        pthread_mutex_unlock(&headBuffer_mutex);

        pthread_mutex_lock(&elevBuffer_mutex);
        elevBuffer[elevHead] = elevation;
        elevHead = (elevHead + 1) % BUFFER_SIZE;
        if (elevCount < BUFFER_SIZE)
            elevCount++;
        pthread_mutex_unlock(&elevBuffer_mutex);

        usleep(5000); // 20ms between reads
    }
    return NULL;
}
// read lsm6ds3trc sensor pitch and roll
void get_pitch_roll(float *pitch, float *roll)
{
    float ax, ay, az;
    lsm6ds3trc_read_accel(fd_lsm6ds3trc, &ax, &ay, &az);

    // Standard pitch/roll calculation for right-handed XYZ:
    *pitch = atan2(-ax, sqrt(ay * ay + az * az)); // Rotation around Y
    *roll = atan2(ay, az);                        // Rotation around X
}
// read lis3mdl && lsm6ds3trc sensor and calculate heading and orientation
void get_heading_orientation(float *heading_deg, float *elevation_deg)
{
    // --- Read accelerometer ---
    float accX, accY, accZ;
    lsm6ds3trc_read_accel(fd_lsm6ds3trc, &accX, &accY, &accZ);

    // --- Read magnetometer ---
    float magX, magY, magZ;
    lis3mdl_read_magnet(fd_lis3mdl, &magX, &magY, &magZ);

    // Align LIS3MDL axes to LSM6DS3 orientation
    magX = -magX;
    magY = -magY;
    // printf("magx: %f magy: %f\n", magX, magY);

    // --- Compute pitch & roll ---
    float pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));
    float roll = atan2(accY, accZ);
    // printf("pitch: %f roll: %f\n", pitch, roll);
    //  --- Tilt compensation ---
    float magX_comp = magX * cos(pitch) + magZ * sin(pitch);
    float magY_comp = magX * sin(roll) * sin(pitch) + magY * cos(roll) - magZ * sin(roll) * cos(pitch);

    // --- Heading (azimuth) ---
    float heading = atan2(-magY_comp, magX_comp);
    *heading_deg = DEG(heading);
    //printf("azimuth = %f\n", DEG(heading));
    // --- Elevation (as vertical tilt) ---
    float elevation = atan2(accZ, sqrt(accX * accX + accY * accY));
    *elevation_deg = DEG(elevation);
    //printf("altitude = %f\n", DEG(elevation));
}

/*
// Get average of pitch buffer
float get_average_pitch()
{
    pthread_mutex_lock(&pitchBuffer_mutex);
    float sum = 0;
    for (int i = 0; i < pitchCount; i++)
    {
        sum += pitchBuffer[i];
    }
    float avg = (pitchCount > 0) ? sum / pitchCount : 0.0;
    pthread_mutex_unlock(&pitchBuffer_mutex);
    return avg;
}

// Get average of roll buffer
float get_average_roll()
{
    pthread_mutex_lock(&rollBuffer_mutex);
    float sum = 0;
    for (int i = 0; i < rollCount; i++)
    {
        sum += rollBuffer[i];
    }
    float avg = (rollCount > 0) ? sum / rollCount : 0.0;
    pthread_mutex_unlock(&rollBuffer_mutex);
    return avg;
}
*/
// Get average of heading buffer
float get_average_heading()
{
    pthread_mutex_lock(&headBuffer_mutex);
    float sum = 0;
    for (int i = 0; i < headCount; i++)
    {
        sum += headBuffer[i];
    }
    float avg = (headCount > 0) ? sum / headCount : 0.0;
    pthread_mutex_unlock(&headBuffer_mutex);
    if (avg < 0)
        avg += 360.0;
    return avg;
}
// Get average of elevation buffer
float get_average_elevation()
{
    pthread_mutex_lock(&elevBuffer_mutex);
    float sum = 0;
    for (int i = 0; i < elevCount; i++)
    {
        sum += elevBuffer[i];
    }
    float avg = (elevCount > 0) ? sum / elevCount : 0.0;
    pthread_mutex_unlock(&elevBuffer_mutex);
    if (avg < 0)
        avg += 360.0;
    return avg;
}

//**************************************** lsm6ds3tr ***********************************************************

static int16_t read16(int fd, int reg)
{
    int l = wiringPiI2CReadReg8(fd, reg);
    int h = wiringPiI2CReadReg8(fd, reg + 1);
    return (int16_t)((h << 8) | l);
}

bool lsm6ds3trc_init()
{
    fd_lsm6ds3trc = wiringPiI2CSetup(LSM6DS3TRC_ADDR);
    if (fd_lsm6ds3trc < 0)
        return false;

    int whoami = wiringPiI2CReadReg8(fd_lsm6ds3trc, LSM6DS3TRC_WHO_AM_I);
    if (whoami != 0x6A)
        return false;

    wiringPiI2CWriteReg8(fd_lsm6ds3trc, LSM6DS3TRC_CTRL1_XL, 0xA0); // Accel: 208 Hz
    wiringPiI2CWriteReg8(fd_lsm6ds3trc, LSM6DS3TRC_CTRL2_G, 0xA0);  // Gyro: 208 Hz
    return true;
}

bool lsm6ds3trc_read_accel(int fd, float *x, float *y, float *z)
{
    *x = read16(fd, LSM6DS3TRC_OUTX_L_XL) * 0.061f / 1000.0f;
    *y = read16(fd, LSM6DS3TRC_OUTX_L_XL + 2) * 0.061f / 1000.0f;
    *z = read16(fd, LSM6DS3TRC_OUTX_L_XL + 4) * 0.061f / 1000.0f;
    return true;
}

bool lsm6ds3trc_read_gyro(int fd, float *x, float *y, float *z)
{
    *x = read16(fd, LSM6DS3TRC_OUTX_L_G) * 8.75f / 1000.0f;
    *y = read16(fd, LSM6DS3TRC_OUTX_L_G + 2) * 8.75f / 1000.0f;
    *z = read16(fd, LSM6DS3TRC_OUTX_L_G + 4) * 8.75f / 1000.0f;
    return true;
}

float lsm6ds3trc_calc_pitch(float ax, float ay, float az)
{
    return atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
}

float lsm6ds3trc_calc_roll(float ay, float az)
{
    return atan2f(ay, az) * 180.0f / M_PI;
}

//**************************************** lis3mdl ***********************************************************

bool lis3mdl_init()
{
    fd_lis3mdl = wiringPiI2CSetup(LIS3MDL_ADDR);
    if (fd_lis3mdl < 0)
        return false;

    int whoami = wiringPiI2CReadReg8(fd_lis3mdl, LIS3MDL_WHO_AM_I);
    if (whoami != 0x3D)
        return false;
    /*
    wiringPiI2CWriteReg8(fd_lis3mdl, 0x20, 0b11110000); // CTRL_REG1
    wiringPiI2CWriteReg8(fd_lis3mdl, 0x21, 0b00000000); // CTRL_REG2
    wiringPiI2CWriteReg8(fd_lis3mdl, 0x22, 0b00000000); // CTRL_REG3
    wiringPiI2CWriteReg8(fd_lis3mdl, 0x23, 0b00001100); // CTRL_REG4
    */
    wiringPiI2CWriteReg8(fd_lis3mdl, LIS3MDL_CTRL_REG1, 0x70); // Temp disabled, ultra-high-perf XY, 10 Hz ODR
    wiringPiI2CWriteReg8(fd_lis3mdl, LIS3MDL_CTRL_REG2, 0x00); // ±4 gauss
    wiringPiI2CWriteReg8(fd_lis3mdl, LIS3MDL_CTRL_REG3, 0x00); // Continuous conversion mode

    return true;
}

bool lis3mdl_read_magnet(int fd, float *x, float *y, float *z)
{
    // Read raw 16-bit values from LIS3MDL registers (little-endian)
    int16_t rawX = wiringPiI2CReadReg8(fd, 0x28) | (wiringPiI2CReadReg8(fd, 0x29) << 8);
    int16_t rawY = wiringPiI2CReadReg8(fd, 0x2A) | (wiringPiI2CReadReg8(fd, 0x2B) << 8);
    int16_t rawZ = wiringPiI2CReadReg8(fd, 0x2C) | (wiringPiI2CReadReg8(fd, 0x2D) << 8);

    // Convert raw values to Gauss (±4 gauss scale => 0.14 mG/LSB)
    // Datasheet: 0.14 mG/LSB for ±4 gauss => 0.00014 Gauss
    *x = rawX * 0.00014;
    *y = rawY * 0.00014;
    *z = rawZ * 0.00014;

    return true;{}
}
