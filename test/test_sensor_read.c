#include "icm20948.h"

int main(void) {
    icm20948_data_t data;

    icm20948_config_t config = {1, LOW_PASS_ORDER_2, GYRO_RANGE_500_DEG, ACCEL_RANGE_2G};

    icm20948_offset_t offset = {
        // gx, gy, gz (3 total)
        .gx = 0.001727,
        .gy = 0.024315,
        .gz = -0.008772,
        // a(xyz)_c0, c1, cov00, cov01, cov11 (15 total)
        .ax_c0 = 0.001567,
        .ax_c1 = 1.002769,
        .ax_cov00 = 0.000057,
        .ax_cov01 = 0.000058,
        .ax_cov11 = 0.000230,
        .ay_c0 = 0.017529,
        .ay_c1 = 0.993337,
        .ay_cov00 = 0.001211,
        .ay_cov01 = 0.001250,
        .ay_cov11 = 0.004643,
        .az_c0 = -0.036225,
        .az_c1 = 0.986753,
        .az_cov00 = 0.000407,
        .az_cov01 = -0.000029,
        .az_cov11 = 0.000792
    };

    icm20948 imu = {-1, "/dev/i2c-1", &config, &data, &offset};

    // Initialize MPU6050
    if (icm20948_init(&imu) < 0) {
        printf("Failed to initialize IMU\n");
        return 1;
    }
    
    printf("Successfully initialized IMU\n");
    
    // Read data continuously
    while (1) {
	// read temperature
	if (icm20948_get_sensors_calibrated(&imu) == 0) {
		printf("Accel X / Y / Z (G): %f / %f / %f\n",
            data.ax, data.ay, data.az);
		printf("Gyro X / Y / Z (rad/s): %f / %f / %f\n",
            data.gx, data.gy, data.gz);
        printf("Magnetometer X / Y / Z (uT): %f / %f / %f\n",
            data.mx, data.my, data.mz);
	}
        
        usleep(100000); // 10ms delay
    }
    
    icm20948_close(&imu);
    return 0;
}
