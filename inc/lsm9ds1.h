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
#include "lsm9ds1_debug.h"
#include "lsm9ds1_config.h"

#define _BUILD_VERSION BUILD_VERSION

/**
 * @brief Stores the raw data for each sub device.
 */
typedef struct lsm9ds1_data_t
{
	lsm9ds1_temperature_t temperature;
	accelerometer_raw_data_t accelerometer;
	mag_raw_data_t magnetometer;
	gyro_raw_data_t gyroscope;
} lsm9ds1_data_t;

/**
 * @brief Stores the converted data for each sub device.
 */
typedef struct lsm9ds1_converted_data_t {
	float temperature;
	accelerometer_converted_data_t accelerometer;
	mag_converted_data_t magnetometer;
	gyro_converted_data_t gyroscope;
} lsm9ds1_converted_data_t;

/**
 * @brief Data and configurations for the lsm9ds1 device.
 */
typedef struct lsm9ds1_device_t {
	bool initialized;
	lsm9ds1_settings_t settings;
	lsm9ds1_xfer_bus_t xfer_bus;
	lsm9ds1_data_t raw_data;
	lsm9ds1_converted_data_t converted_data;
	lsm9ds1_sub_devices_t sub_device;

	lsm9ds1_status_t (*update_temp)(struct lsm9ds1_device_t *self);
	lsm9ds1_status_t (*update_accel)(struct lsm9ds1_device_t *self);
	lsm9ds1_status_t (*update_mag)(struct lsm9ds1_device_t *self);
	lsm9ds1_status_t (*update_gyro)(struct lsm9ds1_device_t *self);
	lsm9ds1_status_t (*update)(struct lsm9ds1_device_t *self);
} lsm9ds1_device_t;

/**
 * @brief Read the temperature of the LSM9DS1.
 *
 * Get the current converted temperature reading
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		status = lsm9ds1_init();
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 *		lsm9ds1_temperature_t data = 0;
 * 		status = get_temp(&data);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error reading temperature!\n");
 * 		}
 *
 *		printf("Temperature: %f\n", data);
 *
 *		return status;
 * }
 * @endcode
 * @param data the converted data read from the temperature monitor
 * @see \ref lsm9ds1_temperature_t
 * @return Returns the function status as defined in \ref lsm9ds1_status_t.
 * @see \ref lsm9ds1_status_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t get_temp(lsm9ds1_temperature_t *temperature);

/**
 * @brief Read the accelerometer of the LSM9DS1.
 *
 * Get the current converted accelerometer reading. The X, Y and Z coordinates are
 * returned by this function.
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		status = lsm9ds1_init();
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 *		accelerometer_converted_data_t data = {0};
 * 		status = get_accel(&data);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error reading accelerometer!\n");
 * 		}
 *
 *		printf("Accelerometer x: %f\n", data.x);
 *		printf("Accelerometer y: %f\n", data.y);
 *		printf("Accelerometer z: %f\n", data.z);
 *
 *		return status;
 * }
 * @endcode
 * @param data the converted data read from the accelerometer.
 * @see \ref accelerometer_converted_data_t
 * @return Returns the function status as defined in \ref lsm9ds1_status_t.
 * @see \ref lsm9ds1_status_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t get_accel(accelerometer_converted_data_t *data);

/**
 * @brief Read the magnetometer of the LSM9DS1.
 *
 * Get the current converted magnetometer reading. The X, Y and Z coordinates are
 * returned by this function.
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		status = lsm9ds1_init();
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *		mag_converted_data_t data = {0};
 * 		status = get_mag(&data);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error reading magnetometer!\n");
 * 		}
 *
 *		printf("Magnetometer x: %f\n", data.x);
 *		printf("Magnetometer y: %f\n", data.y);
 *		printf("Magnetometer z: %f\n", data.z);
 *		
 *		return status;
 * }
 * @endcode
 * @param data the converted data read from the magnetometer.
 * @see \ref mag_converted_data_t
 * @return Returns the function status as defined in \ref lsm9ds1_status_t.
 * @see \ref lsm9ds1_status_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t get_mag(mag_converted_data_t *data);

/**
 * @brief Read the gyroscope from the LSM9DS1.
 *
 * Get the current converted gyroscope reading. The X, Y and Z coordinates are
 * returned by this function.
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		status = lsm9ds1_init();
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 *		gyro_converted_data_t data = {0};
 * 		status = get_gyro(&data);
 * 		if(status < 0) {
 * 			fprintf(stderr, "Error reading gyroscope!\n");
 * 		}
 *
 *		printf("Gyroscope x: %f\n", data.x);
 *		printf("Gyroscope y: %f\n", data.y);
 *		printf("Gyroscope z: %f\n", data.z);
 *
 *		return status;
 * }
 * @endcode
 * @param data the converted data read from the gyroscope.
 * @see \ref gyro_converted_data_t
 * @return Returns the function status as defined in \ref lsm9ds1_status_t.
 * @see \ref lsm9ds1_status_t
 * @note You must first initialize the lsm9ds1.
 * @see lsm9ds1_init
 */
lsm9ds1_status_t get_gyro(gyro_converted_data_t *data);

/**
 * @brief Initialize the LSM9DS1.
 *
 * Initialize the lsm9ds1 according to the configuration file found in
 * /etc/lsm9ds1.json. This function only has to be called once.
 *
 * Example Usage:
 * @code
 * #include <lsm9ds1.h>
 *
 * int main() {
 * 		lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
 *
 *		status = lsm9ds1_init();
 *		if(status < 0) {
 *			fprinf(stderr, "Error initializing lsm9ds1!\n");
 *		}
 *
 *		return status;
 * }
 * @endcode
 * @return Returns the function status as defined in \ref lsm9ds1_status_t.
 * @see \ref lsm9ds1_status_t
 */
lsm9ds1_status_t lsm9ds1_init();

#ifdef __cplusplus
}
#endif

#endif /* LSM9DS1_H_ */
