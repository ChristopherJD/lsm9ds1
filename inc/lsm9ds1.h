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

#include "lsm9ds1_mag.h"
#include "lsm9ds1_gyro.h"
#include "lsm9ds1_accel.h"
#include "lsm9ds1_error.h"
#include "lsm9ds1_regs.h"
#include "lsm9ds1_bus.h"
#include "lsm9ds1_temp.h"
#include "lsm9ds1_debug.h"

#define _BUILD_VERSION BUILD_VERSION

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
} lsm9ds1_data_t;

typedef struct lsm9ds1_converted_data_t {
	float temperature;
	accelerometer_converted_data_t accelerometer;
	mag_converted_data_t magnetometer;
	gyro_converted_data_t gyroscope;
} lsm9ds1_converted_data_t;

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
