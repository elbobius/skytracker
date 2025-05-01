#ifndef ORIENTATION_H
#define ORIENTATION_H

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG(x) ((x) * 180.0f / M_PI)

#define LSM6DS3TRC_ADDR 0x6A // Default I2C address
#define LSM6DS3TRC_WHO_AM_I 0x0F
#define LSM6DS3TRC_CTRL1_XL 0x10
#define LSM6DS3TRC_CTRL2_G 0x11
#define LSM6DS3TRC_OUTX_L_G 0x22
#define LSM6DS3TRC_OUTX_L_XL 0x28

#define LIS3MDL_ADDR 0x1C // Default I2C address
#define LIS3MDL_WHO_AM_I 0x0F
#define LIS3MDL_CTRL_REG1 0x20
#define LIS3MDL_CTRL_REG2 0x21
#define LIS3MDL_CTRL_REG3 0x22
#define LIS3MDL_OUT_X_L 0x28

void* sensor_thread(void* arg);
void get_pitch_roll(float* pitch, float* roll);
void get_heading_orientation(float* heading_deg, float* elevation_deg);
float get_average_roll();
float get_average_pitch();
float get_average_heading();
float get_average_elevation();

bool lsm6ds3trc_init();
bool lsm6ds3trc_read_accel(int fd, float *x, float *y, float *z);
bool lsm6ds3trc_read_gyro(int fd, float *x, float *y, float *z);
float lsm6ds3trc_calc_roll(float ay, float az);
float lsm6ds3trc_calc_pitch(float ax, float ay, float az);

bool lis3mdl_init();
bool lis3mdl_read_magnet(int fd, float *x, float *y, float *z);
#endif
