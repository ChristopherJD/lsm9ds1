/*
 * Copyright (C) Your copyright.
 *
 * Author: Christopher Jordan-Denny
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file
 * @author Christopher Jordan-Denny
 * @date
 * @brief Functions to access the lsm9ds1.
 *
 * Initializes the LSM9DS1 for the Raspberry Pi 3B+. Currently the device is wired
 * to the first spi device. Sets up the magnetometer, accelerometer and gyroscope.
 * Provides functions to read and write the data collected on the LSM9DS1.
 */

#ifndef LSM9DS1_H_
#define LSM9DS1_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define DEVICE "/dev/spidev0.0"

#define SPI_READ 0x80
#define SPI_WRITE 0x0

/**
 * @brief Temperature returned from the LSM9DS1
 */
typedef int16_t lsm9ds1_temperature_t;

typedef struct accelerometer_data {
	int16_t x;
	int16_t y;
	int16_t z;
}accelerometer_data_t;

typedef struct mag_data {
	int16_t x;
	int16_t y;
	int16_t z;
}mag_data_t;

/**
 * @brief Contains the possible error codes returned by the functions.
 *
 */
typedef enum lsm9ds1_status {
	LSM9DS1_SUCCESS = 0,
	LSM9DS1_NOT_FOUND = -1,
	LSM9DS1_SPI_BUS_XFER_ERROR = -2,
	LSM9DS1_UNABLE_TO_GET_SPI_MODE = -3,
	LSM9DS1_MODE_3_NOT_SET = -4,
	LSM9DS1_NUM_BITS_NOT_SET = -5,
	LSM9DS1_CLOCK_NOT_SET = -6,
	LSM9DS1_BUS_NOT_SUPPORTED = -7,
	LSM9DS1_UNKNOWN_ERROR = -8,
	LSM9DS1_ALREADY_INIT_ERROR = -9,
	LSM9DS1_UNKNOWN_SUB_DEVICE = -10,
	LSM9DS1_NOT_INITIALIZED = -11,
	LSM9DS1_UNSUPPORTED_OP = -12,
	LSM9DS1_UKNOWN_ACCEL_RANGE = -13,
	LSM9DS1_BUS_NOT_INTIALIZED = -14,
	LSM9DS1_UKNOWN_GAIN_RANGE = -15,
} lsm9ds1_status_t;

typedef enum {
	LSM9DS1_READ, LSM9DS1_WRITE,
} lsm9ds1_xfer_t;

typedef enum lsm9ds1_bus_t {
	LSM9DS1_SPI_BUS, LSM9DS1_I2C_BUS,
} lsm9ds1_bus_t;

/**
 * @brief Sub device of the LSM9DS1
 *
 * The LSM9DS1 contains two sub-devices. The accelerometer gyroscope combo and
 * the magnetometer. The value 0 is used as an uknown device.
 *
 */
typedef enum {
	LSM9DS1_UNKNOWN_DEVICE = 0,
	LSM9DS1_ACCEL_GYRO = 0x68,
	LSM9DS1_MAG = 0x3D,
} lsm9ds1_devices_t;

#define LSM9DS1_REGISTER_WHO_AM_I 0x0FU

typedef enum {
	LSM9DS1_REGISTER_WHO_AM_I_XG = 0x0F,
	LSM9DS1_REGISTER_CTRL_REG1_G = 0x10,
	LSM9DS1_REGISTER_CTRL_REG2_G = 0x11,
	LSM9DS1_REGISTER_CTRL_REG3_G = 0x12,
	LSM9DS1_REGISTER_TEMP_OUT_L = 0x15,
	LSM9DS1_REGISTER_TEMP_OUT_H = 0x16,
	LSM9DS1_REGISTER_STATUS_REG = 0x17,
	LSM9DS1_REGISTER_OUT_X_L_G = 0x18,
	LSM9DS1_REGISTER_OUT_X_H_G = 0x19,
	LSM9DS1_REGISTER_OUT_Y_L_G = 0x1A,
	LSM9DS1_REGISTER_OUT_Y_H_G = 0x1B,
	LSM9DS1_REGISTER_OUT_Z_L_G = 0x1C,
	LSM9DS1_REGISTER_OUT_Z_H_G = 0x1D,
	LSM9DS1_REGISTER_CTRL_REG4 = 0x1E,
	LSM9DS1_REGISTER_CTRL_REG5_XL = 0x1F,
	LSM9DS1_REGISTER_CTRL_REG6_XL = 0x20,
	LSM9DS1_REGISTER_CTRL_REG7_XL = 0x21,
	LSM9DS1_REGISTER_CTRL_REG8 = 0x22,
	LSM9DS1_REGISTER_CTRL_REG9 = 0x23,
	LSM9DS1_REGISTER_CTRL_REG10 = 0x24,

	LSM9DS1_REGISTER_OUT_X_L_XL = 0x28,
	LSM9DS1_REGISTER_OUT_X_H_XL = 0x29,
	LSM9DS1_REGISTER_OUT_Y_L_XL = 0x2A,
	LSM9DS1_REGISTER_OUT_Y_H_XL = 0x2B,
	LSM9DS1_REGISTER_OUT_Z_L_XL = 0x2C,
	LSM9DS1_REGISTER_OUT_Z_H_XL = 0x2D,

} lsm9ds1AccGyroRegisters_t;

typedef enum {

	LSM9DS1_REGISTER_WHO_AM_I_M = 0x0F,
	LSM9DS1_REGISTER_CTRL_REG1_M = 0x20,
	LSM9DS1_REGISTER_CTRL_REG2_M = 0x21,
	LSM9DS1_REGISTER_CTRL_REG3_M = 0x22,
	LSM9DS1_REGISTER_CTRL_REG4_M = 0x23,
	LSM9DS1_REGISTER_CTRL_REG5_M = 0x24,
	LSM9DS1_REGISTER_STATUS_REG_M = 0x27,
	LSM9DS1_REGISTER_OUT_X_L_M = 0x28,
	LSM9DS1_REGISTER_OUT_X_H_M = 0x29,
	LSM9DS1_REGISTER_OUT_Y_L_M = 0x2A,
	LSM9DS1_REGISTER_OUT_Y_H_M = 0x2B,
	LSM9DS1_REGISTER_OUT_Z_L_M = 0x2C,
	LSM9DS1_REGISTER_OUT_Z_H_M = 0x2D,
	LSM9DS1_REGISTER_CFG_M = 0x30,
	LSM9DS1_REGISTER_INT_SRC_M = 0x31,
} lsm9ds1MagRegisters_t;

typedef enum {
	LSM9DS1_ACCELRANGE_2G = (0 << 3),
	LSM9DS1_ACCELRANGE_16G = (1 << 3),
	LSM9DS1_ACCELRANGE_4G = (2 << 3),
	LSM9DS1_ACCELRANGE_8G = (3 << 3),
} lsm9ds1_accel_range_t;

typedef enum {
	LSM9DS1_ACCELDATARATE_POWERDOWN = (1 << 4),
	LSM9DS1_ACCELDATARATE_3_125HZ = (2 << 4),
	LSM9DS1_ACCELDATARATE_6_25HZ = (3 << 4),
	LSM9DS1_ACCELDATARATE_12_5HZ = (4 << 4),
	LSM9DS1_ACCELDATARATE_25HZ = (5 << 4),
	LSM9DS1_ACCELDATARATE_50HZ = (6 << 4),
	LSM9DS1_ACCELDATARATE_100HZ = (7 << 4),
	LSM9DS1_ACCELDATARATE_200HZ = (8 << 4),
	LSM9DS1_ACCELDATARATE_400HZ = (9 << 4),
	LSM9DS1_ACCELDATARATE_800HZ = (10 << 4),
	LSM9DS1_ACCELDATARATE_1600HZ = (11 << 4)
} lm9ds1AccelDataRate_t;

typedef enum {
	LSM9DS1_MAGGAIN_4GAUSS = (0 << 5),  // +/- 4 gauss
	LSM9DS1_MAGGAIN_8GAUSS = (1 << 5),  // +/- 8 gauss
	LSM9DS1_MAGGAIN_12GAUSS = (2 << 5),  // +/- 12 gauss
	LSM9DS1_MAGGAIN_16GAUSS = (3 << 5)   // +/- 16 gauss
} lsm9ds1_mag_gain_t;

typedef enum {
	LSM9DS1_MAGDATARATE_3_125HZ = (0 << 2),
	LSM9DS1_MAGDATARATE_6_25HZ = (1 << 2),
	LSM9DS1_MAGDATARATE_12_5HZ = (2 << 2),
	LSM9DS1_MAGDATARATE_25HZ = (3 << 2),
	LSM9DS1_MAGDATARATE_50HZ = (4 << 2),
	LSM9DS1_MAGDATARATE_100HZ = (5 << 2)
} lsm9ds1_mag_data_rate_t;

typedef enum {
	LSM9DS1_GYROSCALE_245DPS = (0 << 3), // +/- 245 degrees per second rotation
	LSM9DS1_GYROSCALE_500DPS = (1 << 3), // +/- 500 degrees per second rotation
	LSM9DS1_GYROSCALE_2000DPS = (3 << 3) // +/- 2000 degrees per second rotation
} lsm9ds1_gyro_scale_t;

typedef struct lsm9ds1_settings {
	lsm9ds1_accel_range_t range;
	lsm9ds1_mag_gain_t gain;
	lsm9ds1_gyro_scale_t scale;
}lsm9ds1_settings_t;

//TODO Make better header comments.

/**
 * @brief Determine which subdevice of the LSM9DS1 we are reading from.
 *
 * The sub-device can either be the gyroscope and accelerometer combo or a magnetometer.
 * The WHO_AMI register on the LSM9DS1 is read to determine which device is found.
 * The LSM9DS1 has two chip-selects, one for the gyroscope accelerometer combo and one,
 * for the magnetometer. @p device_id returns the device found see \ref lsm9ds1_devices_t.
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 * 		lsm9ds1_devices_t found_device = LSM9DS1_UNKNOWN_SUB_DEVICE;
 *
 *		status = lsm9ds1_init(LSM9DS1_SPI_BUS);
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 * 		status = lms9ds1_read_sub_device(found_device);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error getting sub-device!\n");
 * 		}
 * }
 * @endcode
 * @param device_id The discovered device id.
 * @return Returns the function status.
 * @see \ref lsm9ds1_status_t
 * @see \ref lsm9ds1_devices_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t lsm9ds1_read_sub_device(lsm9ds1_devices_t *device_id);

/** @brief Read the accelerometer in the coordinates x,y,z.
 *  @param accel_data The values read from the accelerometer.
 *  @return status
 */
lsm9ds1_status_t lsm9ds1_read_accel(accelerometer_data_t *accel_data);

/**
 * @brief Read the temperature from the LSM9DS1.
 *
 * TODO Add description of temp
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 * 		lsm9ds1_temperature_t temp = 0;
 *
 *		status = lsm9ds1_init(LSM9DS1_SPI_BUS);
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 * 		status = lms9ds1_read_temp(temp);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error getting sub-device!\n");
 * 		}
 * }
 * @endcode
 * @param device_id The discovered device id.
 * @return Returns the function status.
 * @see \ref lsm9ds1_status_t
 * @see \ref lsm9ds1_temperature_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t lsm9ds1_read_temp(lsm9ds1_temperature_t *temp);

/** @brief Read the magnetometer in the coordinates x,y,z.
 *  @param mag_data The values read from the magnetometer.
 *  @return status
 */
lsm9ds1_status_t lsm9ds1_read_mag(mag_data_t *mag_data);

/** @brief Initialize the lsm9ds1 in either SPI or I2C. Currently only SPI is supported.
 *  @param bus_type Either I2C or SPI.
 *  @param range The accelerometer range.
 *  @return status
 */
lsm9ds1_status_t lsm9ds1_init(lsm9ds1_bus_t bus_type,
		lsm9ds1_accel_range_t range, lsm9ds1_mag_gain_t gain,
		lsm9ds1_gyro_scale_t scale);

/** @brief Setup the accelerometer.
 *  @param range The accelerometer range.
 *  @return status
 */
lsm9ds1_status_t lsm9ds1_setup_accel(lsm9ds1_accel_range_t range);

lsm9ds1_status_t lsm9ds1_setup_mag(lsm9ds1_mag_gain_t gain);

lsm9ds1_status_t lsm9ds1_setup_gyro(lsm9ds1_gyro_scale_t scale);


#ifdef __cplusplus
}
#endif

#endif /* LSM9DS1_H_ */
