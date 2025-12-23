# icm20948-linux

Pure C99 driver for the ICM20948 (Accelerometer + Gyroscope) and AK09916 (Magnetometer). This is heavily inspired by the Waveshare's code for the ICM20948 userspace Linux driver, however introduces the following improvements:
- Select specific I2C system device in Linux
- Runtime configuration of the gyroscope, accelerometer and magnetometer parameters
- Calibration with linear fit using GNU Scientific Library

This was designed for the [Waveshare's IMX219-83 Stero Camera](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera) and tested on Jetson Nano (B01 / 4GB model).

## How to build?

### Dependencies

GNU Scientific Library (tested on Ubuntu 18.04, Ubuntu 24.04 and Fedora 43)

Fedora: `sudo dnf install gsl-devel` \
Ubuntu: `sudo apt-get install libgsl-dev gsl-bin`

### Build demo

```bash
mkdir build && cd build
cmake -DBUILD_DEMO=ON ..
cmake --build .
./sensor_test
```

### Integrate into CMake (supports both C99 and >=C++11)

```
include(FetchContent)

FetchContent_Declare(
    icm20948
    GIT_REPOSITORY https://github.com/amidg/icm20948-linux.git
    GIT_TAG main  # or use a specific tag/commit
)

FetchContent_MakeAvailable(icm20948)
```

## How to use?

### Calibration

Requires 4 orientations and uses GSL to perform correct linear fit for all three axis. Test calibration program is provided for your convenience and is tested on Jetson Nano:
1. Execute `./calibrate` inside build directory
2. Press SPACE and enter for the first measurement
3. Repeat 4 times according to the directions specified in the code (Z UP, Z side and X down, Z down, Y up) or modify code according to your need
4. Receive calibration numbers. They will look something like

Example parameters

| Parameter | Value     |
|-----------|-----------|
| **Gyro X** | 0.001727 |
| **Gyro Y** | 0.024315 |
| **Gyro Z** | -0.008772 |
| **Accel X c0** | 0.001567 |
| **Accel X c1** | 1.002769 |
| **Accel X cov00** | 0.000057 |
| **Accel X cov01** | 0.000058 |
| **Accel X cov11** | 0.000230 |
| **Accel Y c0** | 0.017529 |
| **Accel Y c1** | 0.993337 |
| **Accel Y cov00** | 0.001211 |
| **Accel Y cov01** | 0.001250 |
| **Accel Y cov11** | 0.004643 |
| **Accel Z c0** | -0.036225 |
| **Accel Z c1** | 0.986753 |
| **Accel Z cov00** | 0.000407 |
| **Accel Z cov01** | -0.000029 |
| **Accel Z cov11** | 0.000792 |

### API

- `icm20948_init(icm20948* device)` allows to initialize device with your configuration parameters. If configuration parameters are not provided, it will use default ones. See datasheets in the docs section

- `icm20948_close(icm20948* device)` closes file descriptor in Linux and frees up I2c bus

- `icm20948_get_sensors(const icm20948* device)` reads "raw" uncalibrated values

- `icm20948_get_sensors_calibrated(const icm20948* device)` reads calibrated values. Estimation is done using GNU Scientific Library (GSL) via linear fit algorithm

## Acknowledgements

The following FOSS contributors inspired me to make this project:

- Waveshare's team. Please buy their products.
- mtmal's [C++ driver](https://github.com/mtmal/ICM20948-linux-i2c)
