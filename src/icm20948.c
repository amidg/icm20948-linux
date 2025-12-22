#include "icm20948.h"

// basic functions
static int read_i2c_(
	const int fd,
    const uint8_t device_addr,
    const uint8_t* addr,
    const uint8_t size_bytes,
    uint8_t* result) {
	// check if FD is valid
	if (fd < 0) return -1;
	if (!addr) return -1;
	if (!result) return -1;

    // check bus access
    if (ioctl(fd, I2C_SLAVE, device_addr) < 0) {
        printf("Failed to capture bus / device");
        return -1;
    }

	// write the register addr then read from it
	if (write(fd, addr, 1) != 1) return -1;
	if (read(fd, result, size_bytes) != size_bytes) return -1;
	return 0;
}

static int write_i2c_(
	const int fd,
    const uint8_t device_addr,
    const uint8_t* buffer,
    const uint8_t size_bytes) {
	// assumptions:
	// - first member of the buffer is the address, not the buffer value itself
	if (fd < 0) return -1;
	if (!buffer) return -1;

    // check bus access
    if (ioctl(fd, I2C_SLAVE, device_addr) < 0) {
        printf("Failed to capture bus / device");
        return -1;
    }

	// write buffer
	if (write(fd, buffer, size_bytes) != size_bytes) return -1;
	return 0;
}

// advanced i2c functions
static int icm20948_read_master_i2c_(
    const int i2c_fd,
    const uint8_t slave_addr,
    const uint8_t reg_addr,
    const uint8_t len,
    uint8_t* data);

static int icm20948_write_master_i2c_(
    const int i2c_fd,
    const uint8_t slave_addr,
    const uint8_t reg_addr,
    const uint8_t* data,
    const uint8_t len);

// translation unit functions
static int icm20948_whoami_(const icm20948* device);
static int icm20948_reset_(const icm20948* device);
static int icm20948_set_gyro_range_(const icm20948* device);
static int icm20948_set_accel_range_(const icm20948* device);
static int icm20948_setup_magnetometer_(const icm20948* device);

// API functions
int icm20948_close(icm20948* device) {
    if (!device) return -1;
    if (device->i2c_fd >= 0) {
        close(device->i2c_fd);
        device->i2c_fd = -1;
        return 0;
    }
    return -1;
}

int icm20948_init(icm20948* device) {
    // check
    if (!device || !device->i2c_device_path) {
	    printf("Requested NULL device");
	    return -1;
    }

    // Open I2C device
    device->i2c_fd = open(device->i2c_device_path, O_RDWR);
    if (device->i2c_fd < 0) {
        printf("Failed to open I2C device\n");
        icm20948_close(device);
        return -1;
    }

    // set the register bank 0
    const uint8_t select_bank0[2] = {REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0};
    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, select_bank0, sizeof(select_bank0)) != 0)
        return -1;
    
    // who am i
    if (icm20948_whoami_(device) != 0) {
        printf("Failed to set WHO_AM_I\n");
        icm20948_close(device);
        return -1;
    }

    // reset
    // page 8 of datasheet -> register 107 will be set to 0x40 = 64
    if (icm20948_reset_(device) != 0) {
        printf("Failed to reset device\n");
        icm20948_close(device);
        return -1;
    }

    // set gyro range
    if (icm20948_set_gyro_range_(device) != 0) {
	printf("Failed to set gyro range\n");
        icm20948_close(device);
        return -1;
    }

    // set accel range
    if (icm20948_set_accel_range_(device) != 0) {
	    printf("Failed to set accel range\n");
        icm20948_close(device);
        return -1;
    }

    // set the register bank 0
    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, select_bank0, sizeof(select_bank0)) != 0)
        return -1;

    // setup magnetometer
    if (icm20948_setup_magnetometer_(device) != 0) {
            printf("Failed to setup magnetometer\n");
        icm20948_close(device);
        return -1;
    }

    return 0;
}

// read 14-bit buffer from the IMU
int icm20948_get_sensors(const icm20948* device) {
 	// checks
	if (!device || !device->data) return -1;

    // read main device = 14 bytes
    // Accelerometer:
    // ACCEL_XOUT_H, ACCEL_XOUT_L
    // ACCEL_YOUT_H, ACCEL_YOUT_L
    // ACCEL_ZOUT_H, ACCEL_ZOUT_L
    // Gyroscope:
    // GYRO_XOUT_H, GYRO_XOUT_L
    // GYRO_XOUT_H, GYRO_XOUT_L
    // GYRO_XOUT_H, GYRO_XOUT_L
    // Temperature:
    // TEMP_OUT_H, TEMP_OUT_L
    uint8_t buffer[14];
    const uint8_t reg = REG_ADD_ACCEL_XOUT_H;
    if (read_i2c_(device->i2c_fd, I2C_ADD_ICM20948, &reg, 14, buffer) != 0) {
        printf("Failed to read main sensor");
        return -1;
    }

    // read accelerometer
	static double accel_scale = 1;
	if (device->config != NULL) {
		switch (device->config->accel_range) {
		case ACCEL_RANGE_2G:
			accel_scale = ACCEL_SSF_AT_FS_2g;
			break;
		case ACCEL_RANGE_4G:
			accel_scale = ACCEL_SSF_AT_FS_4g;
			break;
		case ACCEL_RANGE_8G:
			accel_scale = ACCEL_SSF_AT_FS_8g;
			break;
		case ACCEL_RANGE_16G:
			accel_scale = ACCEL_SSF_AT_FS_16g;
			break;
		}
	}
    device->data->ax = ((int16_t)((buffer[0] << 8) | buffer[1])) / accel_scale;
	device->data->ay = ((int16_t)((buffer[2] << 8) | buffer[3])) / accel_scale;
	device->data->az = ((int16_t)((buffer[4] << 8) | buffer[5])) / accel_scale;

    // read gyroscope
	static double gyro_scale = 1;
	if (device->config != NULL) {
		switch (device->config->gyro_range) {
		case GYRO_RANGE_250_DEG:
			gyro_scale = GYRO_SSF_AT_FS_250DPS;
			break;
		case GYRO_RANGE_500_DEG:
			gyro_scale = GYRO_SSF_AT_FS_500DPS;
            break;
		case GYRO_RANGE_1000_DEG:
			gyro_scale = GYRO_SSF_AT_FS_1000DPS;
			break;
		case GYRO_RANGE_2000_DEG:
			gyro_scale = GYRO_SSF_AT_FS_2000DPS;
			break;
		}
	}
    device->data->gx =
        (M_PI * ((int16_t)((buffer[6] << 8) | buffer[7])) / gyro_scale) / 180.0;
	device->data->gy =
        (M_PI * ((int16_t)((buffer[8] << 8) | buffer[9])) / gyro_scale) / 180.0;
	device->data->gz =
        (M_PI * ((int16_t)((buffer[10] << 8) | buffer[11])) / gyro_scale) / 180.0;

    // TODO (Dmitrii)
    // Implement temperature
    //device->data->temp = (int16_t)((buffer[6] << 8) | buffer[7]) / 340.0f + 21.0f;

    // read magnetometer using slave external sensors
    // 0x10 = ST1
    // 0x11 - 0x16 = H(X/Y/Z)(L/H)
    // 0x17 = dummy
    // 0x18 = ST2
    // According to AK09916 datasheet (page 23) user must read ST2 register after every read
    // This will be automatically read
    uint8_t mag_buf[9];
    if (icm20948_read_master_i2c_(
            device->i2c_fd,
            I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
            REG_ADD_MAG_ST1,
            1,
            &mag_buf[0]) != 0) {
        return -1;
    } else {
        if ((mag_buf[0] & REG_VAL_MAG_READY) == 0) return 0;
    }

    if (icm20948_read_master_i2c_(
            device->i2c_fd,
            I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
            REG_ADD_MAG_DATA,
            8,
            &mag_buf[1]) != 0) {
        return -1;
    }

    device->data->mx =  ((int16_t)((mag_buf[2] << 8) | mag_buf[1])) * MAG_SSF_AT_FS_4900uT;
    device->data->my = -((int16_t)((mag_buf[4] << 8) | mag_buf[3])) * MAG_SSF_AT_FS_4900uT;
    device->data->mz = -((int16_t)((mag_buf[6] << 8) | mag_buf[5])) * MAG_SSF_AT_FS_4900uT;

    return 0;
}

int icm20948_get_sensors_calibrated(const icm20948* device) {
 	// checks
	if (!device || !device->data) return -1;

    // get sensors
    if (icm20948_get_sensors(device) != 0) return -1;

    // apply calibrated data
    int status = 0;
    //if (device->offset != NULL) {
    //    // apply to the gyroscope
    //    device->data->gx -= device->offset->gx;
    //    device->data->gy -= device->offset->gy;
    //    device->data->gz -= device->offset->gz;

    //    // apply gsl_fit_linear_est
    //    double error;
    //    status = gsl_fit_linear_est(
    //        device->data->ax,
    //        device->offset->ax_c0,
    //        device->offset->ax_c1,
    //        device->offset->ax_cov00,
    //        device->offset->ax_cov01,
    //        device->offset->ax_cov11,
    //        &(device->data->ax),
    //        &error
    //    );

    //    status = gsl_fit_linear_est(
    //        device->data->ay,
    //        device->offset->ay_c0,
    //        device->offset->ay_c1,
    //        device->offset->ay_cov00,
    //        device->offset->ay_cov01,
    //        device->offset->ay_cov11,
    //        &(device->data->ay),
    //        &error
    //    );

    //    status = gsl_fit_linear_est(
    //        device->data->az,
    //        device->offset->az_c0,
    //        device->offset->az_c1,
    //        device->offset->az_cov00,
    //        device->offset->az_cov01,
    //        device->offset->az_cov11,
    //        &(device->data->az),
    //        &error
    //    );
    //}

    return status;
}

// Static functions
static int icm20948_whoami_(const icm20948* device) {
    // Verify device by reading WHO_AM_I register
    uint8_t who_am_i = 0;
    const uint8_t reg = REG_ADD_WIA;
    if (read_i2c_(device->i2c_fd, I2C_ADD_ICM20948, &reg, 1, &who_am_i) != 0) {
        printf("Failed to set WHO_AM_I address on the i2c bus: %d\n", who_am_i);
        return -1;
    }

    if (who_am_i != REG_VAL_WIA) return -1;

    return 0;
}

static int icm20948_reset_(const icm20948* device) {
    // checks
    if (!device) return -1;

    // buffers
    const uint8_t sel_bank0[2] = {REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0};
    const uint8_t pwr_mgmt1_rst[2] = {REG_ADD_PWR_MIGMT_1,  REG_VAL_ALL_RGE_RESET};
    const uint8_t pwr_mgmt1_run_mode[2] = {REG_ADD_PWR_MIGMT_1,  REG_VAL_RUN_MODE};

    // user bank 0 reset
    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, sel_bank0, 2) != 0) return -1;
    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, pwr_mgmt1_rst, 2) != 0) return -1;
    usleep(100000); // 100ms
    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, pwr_mgmt1_run_mode, 2) != 0) return -1;
    usleep(100000); // 100ms

    return 0;
}

// translation units
static int icm20948_set_gyro_range_(const icm20948* device) {
	// checks
	if (!device) return -1;

    // buffers
	const uint8_t sel_bank2[2] = {REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2};
    const uint8_t smplrt_div[2] = {REG_ADD_GYRO_SMPLRT_DIV, 0x07};
    uint8_t gyro_config[2] = {REG_ADD_GYRO_CONFIG_1};
    
    // set the bank
    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, sel_bank2, 2) != 0) return -1;

    // set the divisor
    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, smplrt_div, 2) != 0) return -1;

    // set the configuration
    if (!device->config) {
        gyro_config[1] = REG_VAL_BIT_GYRO_FS_250DPS;
    } else {
        // set gyro range
        switch (device->config->gyro_range) {
            case GYRO_RANGE_250_DEG:
                gyro_config[1] |= REG_VAL_BIT_GYRO_FS_250DPS;
                break;
            case GYRO_RANGE_500_DEG:
                gyro_config[1] |= REG_VAL_BIT_GYRO_FS_250DPS;
                break;
            case GYRO_RANGE_1000_DEG:
                gyro_config[1] |= REG_VAL_BIT_GYRO_FS_1000DPS;
                break;
            case GYRO_RANGE_2000_DEG:
                gyro_config[1] |= REG_VAL_BIT_GYRO_FS_2000DPS;
                break;
            default:
                gyro_config[1] |= REG_VAL_BIT_GYRO_FS_250DPS;
                break;
        };

        if (device->config->dlpf_en != 0) {
            gyro_config[1] |= REG_VAL_BIT_GYRO_DLPF;
            switch (device->config->dlpf_order) {
                case LOW_PASS_ORDER_2:
                    gyro_config[1] |= REG_VAL_BIT_GYRO_DLPCFG_2;
                    break;
                case LOW_PASS_ORDER_4:
                    gyro_config[1] |= REG_VAL_BIT_GYRO_DLPCFG_4;
                    break;
                case LOW_PASS_ORDER_6:
                    gyro_config[1] |= REG_VAL_BIT_GYRO_DLPCFG_6;
                    break;
                default:
                    gyro_config[1] |= REG_VAL_BIT_GYRO_DLPCFG_2;
                    break;
            };
        }
    }

    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, gyro_config, 2) != 0) return -1;
	usleep(100000); // wait 100ms
    
    // check current value
    uint8_t test_value = 0;
    if (read_i2c_(device->i2c_fd, I2C_ADD_ICM20948, gyro_config, 1, &test_value) != 0 ||
        (test_value & gyro_config[1]) != gyro_config[1]) {
        printf("Failed to set gyro configuration of the ICM20948: %d\n", test_value);
        return -1;
    }

    return 0;
}

static int icm20948_set_accel_range_(const icm20948* device) {
    // checks
    if (!device) return -1;
    
    // buffers
    const uint8_t sel_bank2[2] = {REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2};
    const uint8_t smplrt_div[2] = {REG_ADD_ACCEL_SMPLRT_DIV_2, 0x07};
    uint8_t accel_config[2] = {REG_ADD_ACCEL_CONFIG};
    
    // set the bank
    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, sel_bank2, 2) != 0) return -1;
    
    // set sample rate divider
    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, smplrt_div, 2) != 0) return -1;
    
    // set the configuration
    if (!device->config) {
        accel_config[1] = REG_VAL_BIT_ACCEL_FS_2g;
    } else {
        // set gyro range
        switch (device->config->accel_range) {
            case ACCEL_RANGE_2G:
                accel_config[1] |= REG_VAL_BIT_ACCEL_FS_2g;
                break;
            case ACCEL_RANGE_4G:
                accel_config[1] |= REG_VAL_BIT_ACCEL_FS_4g;
                break;
            case ACCEL_RANGE_8G:
                accel_config[1] |= REG_VAL_BIT_ACCEL_FS_8g;
                break;
            case ACCEL_RANGE_16G:
                accel_config[1] |= REG_VAL_BIT_ACCEL_FS_16g;
                break;
            default:
                accel_config[1] |= REG_VAL_BIT_ACCEL_FS_2g;
                break;
        };

        if (device->config->dlpf_en != 0) {
            accel_config[1] |= REG_VAL_BIT_ACCEL_DLPF;
            switch (device->config->dlpf_order) {
                case LOW_PASS_ORDER_2:
                    accel_config[1] |= REG_VAL_BIT_ACCEL_DLPCFG_2;
                    break;
                case LOW_PASS_ORDER_4:
                    accel_config[1] |= REG_VAL_BIT_ACCEL_DLPCFG_4;
                    break;
                case LOW_PASS_ORDER_6:
                    accel_config[1] |= REG_VAL_BIT_ACCEL_DLPCFG_6;
                    break;
                default:
                    accel_config[1] |= REG_VAL_BIT_ACCEL_DLPCFG_2;
                    break;
            };
        }
    }

    if (write_i2c_(device->i2c_fd, I2C_ADD_ICM20948, accel_config, 2) != 0) return -1;
	usleep(100000); // wait 100ms
    
    // check current value
    uint8_t test_value = 0;
    if (read_i2c_(device->i2c_fd, I2C_ADD_ICM20948, accel_config, 1, &test_value) != 0 ||
        (test_value & accel_config[1]) != accel_config[1]) {
        printf("Failed to set accel configuration of the ICM20948: %d\n", test_value);
        return -1;
    }
    
    return 0;
}

static int icm20948_setup_magnetometer_(const icm20948* device) {
    if (!device) return -1;

    // buffers
    uint8_t addr = I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ;
    uint8_t reg = REG_ADD_MAG_WIA1;
    uint8_t ret[2];

    // check if magnetometer is present
    if (icm20948_read_master_i2c_(device->i2c_fd, addr, reg, 2, ret) != 0) return -1;
    if (!((ret[0] == REG_VAL_MAG_WIA1) && ( ret[1] == REG_VAL_MAG_WIA2))) return -1;

    // reset magnetometer
    addr = I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE;
    reg = REG_ADD_MAG_CNTL3;
    const uint8_t soft_rst_val[1] = {REG_VAL_MAG_RESET};
    if (icm20948_write_master_i2c_(device->i2c_fd, addr, reg, soft_rst_val, 1) != 0) return -1;
    usleep(100000); // wait 100ms

    // setup magnetometer
    // if using Single Measurement mode device will transition to sleep after data is read
    addr = I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE;
    reg = REG_ADD_MAG_CNTL2;
    const uint8_t val[1] = {REG_VAL_MAG_MODE_100HZ};
    if (icm20948_write_master_i2c_(device->i2c_fd, addr, reg, val, 1) != 0) return -1;
    usleep(100000); // wait 100ms

    // check if mode is set correctly
    addr = I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ;
    if (icm20948_read_master_i2c_(device->i2c_fd, addr, reg, 1, ret) != 0) return -1;
    if (val[0] > REG_VAL_MAG_MODE_SM && (ret[0] & val[0]) != val[0]) return -1;
    if (val[0] == REG_VAL_MAG_MODE_SM && ret[0] != 0) return -1;

    return 0;
}

static int icm20948_read_master_i2c_(
    const int i2c_fd,
    const uint8_t slave_addr,
    const uint8_t reg_addr,
    const uint8_t len,
    uint8_t* data)
{
    uint8_t buf[2];
    uint8_t temp;

    // Switch to Bank 3
    buf[0] = REG_ADD_REG_BANK_SEL;
    buf[1] = REG_VAL_REG_BANK_3;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    // Configure I2C Slave 0
    buf[0] = REG_ADD_I2C_SLV0_ADDR;
    buf[1] = slave_addr;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    buf[0] = REG_ADD_I2C_SLV0_REG;
    buf[1] = reg_addr;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    buf[0] = REG_ADD_I2C_SLV0_CTRL;
    buf[1] = REG_VAL_BIT_SLV0_EN | len;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    // Switch back to Bank 0
    buf[0] = REG_ADD_REG_BANK_SEL;
    buf[1] = REG_VAL_REG_BANK_0;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    // Enable I2C Master
    buf[0] = REG_ADD_USER_CTRL;
    if (read_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 1, &temp) != 0)
        return -1;
    temp |= REG_VAL_BIT_I2C_MST_EN;
    buf[1] = temp;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    usleep(5000); // 5 ms delay

    // Disable I2C Master
    buf[0] = REG_ADD_USER_CTRL;
    if (read_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 1, &temp) != 0)
        return -1;
    temp &= ~REG_VAL_BIT_I2C_MST_EN;
    buf[1] = temp;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    // Read result from EXT_SENS_DATA_00
    buf[0] = REG_ADD_EXT_SENS_DATA_00;
    if (read_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, len, data) != 0)
        return -1;

    // Clear SLV0 control register
    buf[0] = REG_ADD_REG_BANK_SEL;
    buf[1] = REG_VAL_REG_BANK_3;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    buf[0] = REG_ADD_I2C_SLV0_CTRL;
    if (read_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 1, &temp) != 0)
        return -1;
    temp &= ~(REG_VAL_BIT_I2C_MST_EN & REG_VAL_BIT_MASK_LEN);
    buf[1] = temp;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    // Switch back to Bank 0
    buf[0] = REG_ADD_REG_BANK_SEL;
    buf[1] = REG_VAL_REG_BANK_0;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0)
        return -1;

    return 0;
}

int icm20948_write_master_i2c_(
    const int i2c_fd,
    const uint8_t slave_addr,
    const uint8_t reg_addr,
    const uint8_t* data,
    const uint8_t len)
{
    uint8_t buf[2];
    uint8_t temp;

    if (!data || len == 0) return -1;
    
    // Switch to Bank 3
    buf[0] = REG_ADD_REG_BANK_SEL;
    buf[1] = REG_VAL_REG_BANK_3;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    // Configure I2C Slave 1 (for writing)
    buf[0] = REG_ADD_I2C_SLV0_ADDR;
    buf[1] = slave_addr;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    buf[0] = REG_ADD_I2C_SLV0_REG;
    buf[1] = reg_addr;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    const uint8_t new_buf_sz = len + 1;
    uint8_t buffer[new_buf_sz];
    buffer[0] = REG_ADD_I2C_SLV0_DO;
    memcpy(&buffer[1], data, len);
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buffer, new_buf_sz) != 0) return -1;

    // Enable I2C Slave 1 (write operation)
    buf[0] = REG_ADD_I2C_SLV0_CTRL;
    buf[1] = REG_VAL_BIT_SLV0_EN | 1;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    // Switch back to Bank 0
    buf[0] = REG_ADD_REG_BANK_SEL;
    buf[1] = REG_VAL_REG_BANK_0;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    // Enable I2C Master
    buf[0] = REG_ADD_USER_CTRL;
    if (read_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 1, &temp) != 0) return -1;
    temp |= REG_VAL_BIT_I2C_MST_EN;
    buf[1] = temp;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    usleep(5000); // 5 ms delay

    // Disable I2C Master
    buf[0] = REG_ADD_USER_CTRL;
    if (read_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 1, &temp) != 0) return -1;
    temp &= ~REG_VAL_BIT_I2C_MST_EN;
    buf[1] = temp;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    // Clear I2C Slave 1 control register
    buf[0] = REG_ADD_REG_BANK_SEL;
    buf[1] = REG_VAL_REG_BANK_3;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    buf[0] = REG_ADD_I2C_SLV0_CTRL;
    if (read_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 1, &temp) != 0) return -1;
    temp &= ~(REG_VAL_BIT_I2C_MST_EN & REG_VAL_BIT_MASK_LEN);
    buf[1] = temp;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    // Switch back to Bank 0
    buf[0] = REG_ADD_REG_BANK_SEL;
    buf[1] = REG_VAL_REG_BANK_0;
    if (write_i2c_(i2c_fd, I2C_ADD_ICM20948, buf, 2) != 0) return -1;

    return 0;
}
