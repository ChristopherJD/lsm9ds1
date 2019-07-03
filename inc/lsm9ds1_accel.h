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

#ifndef LSM9DS1_ACCEL_H_
#define LSM9DS1_ACCEL_H_

// Linear Acceleration: mg per LSB
#define LSM9DS1_ACCEL_MG_LSB_2G (0.061F)
#define LSM9DS1_ACCEL_MG_LSB_4G (0.122F)
#define LSM9DS1_ACCEL_MG_LSB_8G (0.244F)
#define LSM9DS1_ACCEL_MG_LSB_16G (0.732F)

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

typedef struct lsm9ds1_accel_settings_t {
	lsm9ds1_accel_range_t range;
	float accel_mg_lsb;
	lsm9ds1_xl_odr_t odr;
} lsm9ds1_accel_settings_t;

#endif
