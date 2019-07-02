/* 
 * This file is part of the lsm9ds1 library (https://github.com/ChristopherJD/lsm9ds1.git).
 * Copyright (c) 2019 Christopher Jordan-Denny.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
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

#include <stdbool.h>
#include <stdint.h>

#ifndef _BUILD_VERSION
#define _BUILD_VERSION BUILD_VERSION
#endif

#define DEVICE "/dev/spidev0.0"

#define SPI_READ 0x80
#define SPI_WRITE 0x0

// Linear Acceleration: mg per LSB
#define LSM9DS1_ACCEL_MG_LSB_2G (0.061F)
#define LSM9DS1_ACCEL_MG_LSB_4G (0.122F)
#define LSM9DS1_ACCEL_MG_LSB_8G (0.244F)
#define LSM9DS1_ACCEL_MG_LSB_16G (0.732F)

	// Magnetic Field Strength: gauss range
#define LSM9DS1_MAG_MGAUSS_4GAUSS      (0.14F)
#define LSM9DS1_MAG_MGAUSS_8GAUSS      (0.29F)
#define LSM9DS1_MAG_MGAUSS_12GAUSS     (0.43F)
#define LSM9DS1_MAG_MGAUSS_16GAUSS     (0.58F)

// Angular Rate: dps per LSB
#define LSM9DS1_GYRO_DPS_DIGIT_245DPS      (0.00875F)
#define LSM9DS1_GYRO_DPS_DIGIT_500DPS      (0.01750F)
#define LSM9DS1_GYRO_DPS_DIGIT_2000DPS	   (0.07000F)

/**
 * @brief Temperature returned from the LSM9DS1
 */
typedef int16_t lsm9ds1_temperature_t;

typedef struct accelerometer_data {
	int16_t x;
	int16_t y;
	int16_t z;
} accelerometer_raw_data_t;

typedef struct accelerometer_converted_data_t {
	float x;
	float y;
	float z;
}accelerometer_converted_data_t;

typedef struct mag_data {
	int16_t x;
	int16_t y;
	int16_t z;
} mag_raw_data_t;

typedef struct mag_converted_data_t {
	float x;
	float y;
	float z;
}mag_converted_data_t;

typedef struct gyro_data {
	int16_t x;
	int16_t y;
	int16_t z;
} gyro_raw_data_t;

typedef struct gyro_converted_data_t {
	float x;
	float y;
	float z;
}gyro_converted_data_t;

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
	LSM9DS1_UNABLE_TO_SET_CS_DIR = -16,
	LSM9DS1_UNABLE_TO_OPEN_MAG_CS = -17,
	LSM9DS1_UNABLE_TO_SET_CS = -18,
	LSM9DS1_MALLOC_DEVICE_ERROR = -19,
	LSM9DS1_NO_BUS_FOUND = -20,
	LSM9DS1_MAG_ALREADY_RESET = -21,
	LSM9DS1_ACCEL_GYRO_ALREADY_RESET = -22,
} lsm9ds1_status_t;

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
} lsm9ds1_sub_device_t;

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

typedef enum lsm9ds1_xl_odr_t{
	LSM9DS1_XL_ODR_POWERDOWN = (0 << 5),
	LSM9DS1_XL_10HZ = (1 <<5),
	LSM9DS1_XL_50HZ = (2 << 5),
	LSM9DS1_XL_119HZ = (3 << 5),
	LSM9DS1_XL_238HZ = (4 << 5),
	LSM9DS1_XL_476HZ = (5 << 5),
	LSM9DS1_XL_952HZ = (6 << 5),
} lsm9ds1_xl_odr_t;

typedef enum {
	LSM9DS1_MAGGAIN_4GAUSS = (0 << 5),  // +/- 4 gauss
	LSM9DS1_MAGGAIN_8GAUSS = (1 << 5),  // +/- 8 gauss
	LSM9DS1_MAGGAIN_12GAUSS = (2 << 5),  // +/- 12 gauss
	LSM9DS1_MAGGAIN_16GAUSS = (3 << 5)   // +/- 16 gauss
} lsm9ds1_mag_gain_t;

typedef enum lsm9ds1_mag_odr_t{
	LSM9DS1_MAG_ODR_0_625HZ = (0 << 2),
	LSM9DS1_MAG_ODR_1_25HZ = (1 << 2),
	LSM9DS1_MAG_ODR_2_5HZ = (2 << 2),
	LSM9DS1_MAG_ODR_5HZ = (3 << 2),
	LSM9DS1_MAG_ODR_10HZ = (4 << 2),
	LSM9DS1_MAG_ODR_20HZ = (5 << 2),
	LSM9DS1_MAG_ODR_40HZ = (6 << 2),
	LSM9DS1_MAG_ODR_80HZ = (7 << 2)
} lsm9ds1_mag_odr_t;

typedef enum {
	LSM9DS1_GYROSCALE_245DPS = (0 << 3), // +/- 245 degrees per second rotation
	LSM9DS1_GYROSCALE_500DPS = (1 << 3), // +/- 500 degrees per second rotation
	LSM9DS1_GYROSCALE_2000DPS = (3 << 3) // +/- 2000 degrees per second rotation
} lsm9ds1_gyro_scale_t;

typedef struct lsm9ds1_accel_settings_t {
	lsm9ds1_accel_range_t range;
	float accel_mg_lsb;
	lsm9ds1_xl_odr_t odr;
} lsm9ds1_accel_settings_t;

typedef enum lsm9ds1_gyro_odr_t {
	LSM9DS1_GYRO_ODR_POWERDOWN = (0 << 5),
	LSM9DS1_GYRO_ODR_14_9HZ = (1 << 5),
	LSM9DS1_GYRO_ODR_59_5HZ = (2 << 5),
	LSM9DS1_GYRO_ODR_119HZ = (3 << 5),
	LSM9DS1_GYRO_ODR_238HZ = (4 << 5),
	LSM9DS1_GYRO_ODR_476HZ = (5 << 5),
	LSM9DS1_GYRO_ODR_952HZ = (6 << 5),
} lsm9ds1_gyro_odr_t;

typedef enum lsm9ds1_mag_op_mode_t {
	LSM9DS1_MAG_OP_MODE_CONTINUOUS = (0 << 0),
	LSM9DS1_MAG_OP_MODE_SINGLE_CONVERSION = (1 << 0),
	LSM9DS1_MAG_OP_MODE_POWER_DOWN = (2 << 0),
}lsm9ds1_mag_op_mode_t;

typedef enum lsm9ds1_mag_xy_op_mode_t {
	LSM9DS1_MAG_XY_OP_MODE_LOW_POWER = (0 << 5),
	LSM9DS1_MAG_XY_OP_MODE_MEDIUM_PERF = (1 << 5),
	LSM9DS1_MAG_XY_OP_MODE_HIGH_PERF = (2 << 5),
	LSM9DS1_MAG_XY_OP_MODE_ULTRA_PERF = (3 << 5),
}lsm9ds1_mag_xy_op_mode_t;

#define LSM9DS1_MAG_TEMP_COMP_ENABLE (1 << 7);
#define LSM9DS1_MAG_TEMP_COMP_DISABLE (0 << 7);

#define LSM9DS1_MAG_FAST_ODR_ENABLE (1 << 1);
#define LSM9DS1_MAG_FAST_ODR_DISABLE (0 << 1);

#define LSM9DS1_MAG_SELF_TEST_ENABLE (1 << 0);
#define LSM9DS1_MAG_SELF_TEST_DISABLE (0 << 0);

#define LSM9DS1_MAG_REBOOT (1 << 3);

#define LSM9DS1_MAG_RESET (1 << 2);

typedef enum lsm9ds1_mag_spi_mode_t {
	LSM9DS1_MAG_ONLY_WRITE = (0 << 2),
	LSM9DS1_MAG_READ_WRITE = (1 << 2),
}lsm9ds1_mag_spi_mode_t;

typedef struct lsm9ds1_mag_settings_t {
	float mag_mgauss;

	//CTRL REG 1
	uint8_t temp_comp_enable;
	lsm9ds1_mag_xy_op_mode_t xy_op_mode;
	lsm9ds1_mag_odr_t odr;
	uint8_t fast_odr_enable;
	uint8_t self_test_enable;

	//CTRL REG 2
	lsm9ds1_mag_gain_t gain;
	uint8_t reboot;
	uint8_t reset;

	//CTRL REG 3
	uint8_t i2c_disable;
	uint8_t low_power_mode;
	lsm9ds1_mag_spi_mode_t spi_mode;
	lsm9ds1_mag_op_mode_t op_mode;

	//CTRL REG 4

	//CTRL REG 5


}lsm9ds1_mag_settings_t;

typedef struct lsm9ds1_gyro_settings_t {
	lsm9ds1_gyro_scale_t scale;
	float gyro_dps_digit;
	lsm9ds1_gyro_odr_t odr;	//output data rate
}lsm9ds1_gyro_settings_t;

typedef struct lsm9ds1_settings {
	lsm9ds1_accel_settings_t accelerometer;
	lsm9ds1_mag_settings_t magnetometer;
	lsm9ds1_gyro_settings_t gyroscope;
} lsm9ds1_settings_t;

typedef struct lsm9ds1_data_t
{
	lsm9ds1_temperature_t temperature;
	accelerometer_raw_data_t accelerometer;
	mag_raw_data_t magnetometer;
	gyro_raw_data_t gyroscope;
}lsm9ds1_data_t;

typedef struct lsm9ds1_converted_data_t {
	float temperature;
	accelerometer_converted_data_t accelerometer;
	mag_converted_data_t magnetometer;
	gyro_converted_data_t gyroscope;
}lsm9ds1_converted_data_t;

typedef enum {
	LSM9DS1_READ, LSM9DS1_WRITE,
} lsm9ds1_xfer_t;

typedef enum lsm9ds1_xfer_bus_t {
	LSM9DS1_SPI_BUS, LSM9DS1_I2C_BUS, NUM_BUS_TYPES,
} lsm9ds1_xfer_bus_t;

typedef struct lsm9ds1_spi_settings_t {
	uint8_t mode;
	uint8_t bits;
	uint32_t speed;
	uint16_t spi_delay;
}lsm9ds1_spi_settings_t;

typedef struct lsm9ds1_i2c_settings_t {
	uint8_t bits;
}lsm9ds1_i2c_settings_t;

typedef struct lsm9ds1_spi_t {
	lsm9ds1_spi_settings_t settings;
	lsm9ds1_xfer_t op;
	uint8_t address;
	uint8_t tx;
	uint8_t rx[1];
}lsm9ds1_spi_t;

typedef struct lsm9ds1_i2c_t {
	lsm9ds1_i2c_settings_t settings;
	lsm9ds1_xfer_t op;
	uint8_t address;
	uint8_t tx;
	uint8_t rx[1];
}lsm9ds1_i2c_t;

typedef struct lsm9ds1_bus_t {
	bool initialized;
	int32_t fd;
	char device[256];
	lsm9ds1_spi_t spi;
	lsm9ds1_i2c_t i2c;
	lsm9ds1_sub_device_t current_sub_device;

	lsm9ds1_status_t (*transfer)(struct lsm9ds1_bus_t *self, lsm9ds1_xfer_t op, uint8_t address, uint8_t tx, uint8_t *rx);
	lsm9ds1_status_t (*cs_arbiter)(struct lsm9ds1_bus_t *self);
}lsm9ds1_bus_t;

/**
 * @brief Store data and configurations for the lsm9ds1 device.
 *
 * This structure stores all the information pertaining to the lsm9ds1.
 *	- Bus configuration (SPI/I2C) information
 *	- Gyroscope configuration information
 *	- Magnetometer configuration information
 *	- Accelerometer configuration information
 */
typedef struct lsm9ds1_device_t {
	bool initialized;	/* Set to true if the lsm9ds1 was able to setup the Gyroscope, accelerometer, and magnetometer. False otherwise. */
	lsm9ds1_settings_t settings;	/* Please see \ref lsm9ds1_settings_t for more information */
	lsm9ds1_xfer_bus_t xfer_bus;	/* Please see \ref lsm9ds1_xfer_bus_t for more information */
	lsm9ds1_data_t raw_data;	/* Please see \ref lsm9ds1_data_t for more information */
	lsm9ds1_converted_data_t converted_data;	/* Please see \ref lsm9ds1_converted_data_t for more information */
	lsm9ds1_bus_t bus;	/* Please see \ref lsm9ds1_bus_t for more information */

	lsm9ds1_status_t (*update_temp)(struct lsm9ds1_device_t *self);	/* Please see \ref update_temp for more information */
	lsm9ds1_status_t (*update_accel)(struct lsm9ds1_device_t *self);	/* Please see \ref update_accel for more information */
	lsm9ds1_status_t (*update_mag)(struct lsm9ds1_device_t *self);	/* Please see \ref update_mag for more information */
	lsm9ds1_status_t (*update_gyro)(struct lsm9ds1_device_t *self);	/* Please see \ref update_gyro for more information */
	lsm9ds1_status_t (*update)(struct lsm9ds1_device_t *self);		/* Please see \ref update for more information */
} lsm9ds1_device_t;

//TODO Make better header comments.

/**
 * @brief Read the temperature of the LSM9DS1.
 *
 * Updates the lsm9ds1_device_t structure with the current temperature. You
 * must first create this structure before reading.
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		lsm9ds1 = malloc(sizeof(lsm9ds1_device_t));
 *		status = lsm9ds1_init(lsm9ds1, LSM9DS1_SPI_BUS, LSM9DS1_ACCELRANGE_8G, LSM9DS1_MAGGAIN_8GAUSS, LSM9DS1_GYROSCALE_500DPS);
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 * 		status = lsm9ds1.update_temp(lsm9ds1);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error reading temperature!\n");
 * 		}
 *
 *		free(lsm9ds1);
 * }
 * @endcode
 * @param self The created instance of the lsm9ds1_device_t.
 * @return Returns the function status.
 * @see \ref lsm9ds1_status_t
 * @see \ref lsm9ds1_device_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t update_temp(lsm9ds1_device_t *self);

/**
 * @brief Read the accelerometer of the LSM9DS1.
 *
 * Updates the lsm9ds1_device_t structure with the current accelerometer reading. You
 * must first create this structure before reading.
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		lsm9ds1 = malloc(sizeof(lsm9ds1_device_t));
 *		status = lsm9ds1_init(lsm9ds1, LSM9DS1_SPI_BUS, LSM9DS1_ACCELRANGE_8G, LSM9DS1_MAGGAIN_8GAUSS, LSM9DS1_GYROSCALE_500DPS);
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 * 		status = lsm9ds1.update_accel(lsm9ds1);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error reading accelerometer!\n");
 * 		}
 *
 *		free(lsm9ds1);
 * }
 * @endcode
 * @param self The created instance of the lsm9ds1_device_t.
 * @return Returns the function status.
 * @see \ref lsm9ds1_status_t
 * @see \ref lsm9ds1_device_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t update_accel(lsm9ds1_device_t *self);

/**
 * @brief Read the magnetometer of the LSM9DS1.
 *
 * Updates the lsm9ds1_device_t structure with the current magnetometer reading. You
 * must first create this structure before reading.
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		lsm9ds1 = malloc(sizeof(lsm9ds1_device_t));
 *		status = lsm9ds1_init(lsm9ds1, LSM9DS1_SPI_BUS, LSM9DS1_ACCELRANGE_8G, LSM9DS1_MAGGAIN_8GAUSS, LSM9DS1_GYROSCALE_500DPS);
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 * 		status = lsm9ds1.update_mag(lsm9ds1);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error reading magnetometer!\n");
 * 		}
 *
 *		free(lsm9ds1);
 * }
 * @endcode
 * @param self The created instance of the lsm9ds1_device_t.
 * @return Returns the function status.
 * @see \ref lsm9ds1_status_t
 * @see \ref lsm9ds1_device_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t update_mag(lsm9ds1_device_t *self);

/**
 * @brief Read the gyroscope from the LSM9DS1.
 *
 * Updates the lsm9ds1_device_t structure with the current gyroscope reading. You
 * must first create this structure before reading.
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		lsm9ds1 = malloc(sizeof(lsm9ds1_device_t));
 *		status = lsm9ds1_init(lsm9ds1, LSM9DS1_SPI_BUS, LSM9DS1_ACCELRANGE_8G, LSM9DS1_MAGGAIN_8GAUSS, LSM9DS1_GYROSCALE_500DPS);
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 * 		status = lsm9ds1.update_accel(lsm9ds1);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error reading gyroscope!\n");
 * 		}
 *
 *		free(lsm9ds1);
 * }
 * @endcode
 * @param self The created instance of the lsm9ds1_device_t.
 * @return Returns the function status.
 * @see \ref lsm9ds1_status_t
 * @see \ref lsm9ds1_device_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t update_gyro(lsm9ds1_device_t *self);

/**
 * @brief Initialize the LSM9DS1.
 *
 * TODO Add description of initialization
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		lsm9ds1 = malloc(sizeof(lsm9ds1_device_t));
 *		status = lsm9ds1_init(lsm9ds1, LSM9DS1_SPI_BUS, LSM9DS1_ACCELRANGE_8G, LSM9DS1_MAGGAIN_8GAUSS, LSM9DS1_GYROSCALE_500DPS);
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 *		free(lsm9ds1);
 * }
 * @endcode
 * @param self The created instance of the lsm9ds1_device_t.
 * @return Returns the function status.
 * @see \ref lsm9ds1_status_t
 * @see \ref lsm9ds1_device_t
 * @see \ref lsm9ds1_xfer_bus_t
 * @see \ref lsm9ds1_accel_range_t
 # @see \ref lsm9ds1_mag_gain_t
 * @see \ref lsm9ds1_gyro_scale_t
 */
lsm9ds1_status_t lsm9ds1_init(lsm9ds1_device_t *self, lsm9ds1_xfer_bus_t bus_type,
                              lsm9ds1_accel_range_t range, lsm9ds1_mag_gain_t gain,
                              lsm9ds1_gyro_scale_t scale);

#ifdef __cplusplus
}
#endif

#endif /* LSM9DS1_H_ */
