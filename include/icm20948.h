#ifndef __ICM20948_H__
#define __ICM20948_H__

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <gsl/gsl_fit.h>

#include "icm20948_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {  
    IMU_EN_SENSOR_TYPE_NULL = 0,
    IMU_EN_SENSOR_TYPE_ICM20948,
    IMU_EN_SENSOR_TYPE_MAX
} imu_sensor_t;

typedef enum {
    GYRO_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
    GYRO_RANGE_500_DEG,  ///< +/- 500 deg/s
    GYRO_RANGE_1000_DEG, ///< +/- 1000 deg/s
    GYRO_RANGE_2000_DEG, ///< +/- 2000 deg/s
} icm20948_gyro_range_t;

typedef enum {
    ACCEL_RANGE_2G,  ///< +/- 2g (default value)
    ACCEL_RANGE_4G,  ///< +/- 4g
    ACCEL_RANGE_8G,  ///< +/- 8g
    ACCEL_RANGE_16G, ///< +/- 16g
} icm20948_accel_range_t;

typedef enum {
    LOW_PASS_ORDER_2,
    LOW_PASS_ORDER_4,
    LOW_PASS_ORDER_6,
} icm20948_low_pass_order_t;

typedef struct {
    const uint8_t dlpf_en;
    const icm20948_low_pass_order_t dlpf_order;
    const icm20948_gyro_range_t gyro_range;
    const icm20948_accel_range_t accel_range;
} icm20948_config_t;

typedef struct {
    float ax, ay, az,
          gx, gy, gz,
          mx, my, mz,
          temp;
} icm20948_data_t;

typedef struct {
    double gx, gy, gz;
    double ax_c0, ax_c1, ax_cov00, ax_cov01, ax_cov11,
           ay_c0, ay_c1, ay_cov00, ay_cov01, ay_cov11,
           az_c0, az_c1, az_cov00, az_cov01, az_cov11;
} icm20948_offset_t;

typedef struct {
    int i2c_fd;       // i2c device in Linux
    const char* i2c_device_path;
    icm20948_config_t* config;
    icm20948_data_t* data;
    icm20948_offset_t* offset;
} icm20948;

int icm20948_init(icm20948* device);
int icm20948_close(icm20948* device);

int icm20948_get_sensors(const icm20948* device);
int icm20948_get_sensors_calibrated(const icm20948* device);

#ifdef __cplusplus
}
#endif

#endif //__ICM20948_H__
