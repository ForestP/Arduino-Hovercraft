#ifndef IMU_UTILS_H
#define IMU_UTILS_H
#include "MPU9250.h"

extern MPU9250 IMU;
extern void imu_setup();
extern void imu_calibrate_magbias();
extern void imu_update();

#endif

