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

#ifndef LSM9DS1_GYRO_H_
#define LSM9DS1_GRYO_H_

// Angular Rate: dps per LSB
#define LSM9DS1_GYRO_DPS_DIGIT_245DPS      (0.00875F)
#define LSM9DS1_GYRO_DPS_DIGIT_500DPS      (0.01750F)
#define LSM9DS1_GYRO_DPS_DIGIT_2000DPS	   (0.07000F)

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

typedef enum {
	LSM9DS1_GYROSCALE_245DPS = (0 << 3), // +/- 245 degrees per second rotation
	LSM9DS1_GYROSCALE_500DPS = (1 << 3), // +/- 500 degrees per second rotation
	LSM9DS1_GYROSCALE_2000DPS = (3 << 3) // +/- 2000 degrees per second rotation
} lsm9ds1_gyro_scale_t;

typedef enum lsm9ds1_gyro_odr_t {
	LSM9DS1_GYRO_ODR_POWERDOWN = (0 << 5),
	LSM9DS1_GYRO_ODR_14_9HZ = (1 << 5),
	LSM9DS1_GYRO_ODR_59_5HZ = (2 << 5),
	LSM9DS1_GYRO_ODR_119HZ = (3 << 5),
	LSM9DS1_GYRO_ODR_238HZ = (4 << 5),
	LSM9DS1_GYRO_ODR_476HZ = (5 << 5),
	LSM9DS1_GYRO_ODR_952HZ = (6 << 5),
} lsm9ds1_gyro_odr_t;

typedef struct lsm9ds1_gyro_settings_t {
	lsm9ds1_gyro_scale_t scale;
	float gyro_dps_digit;
	lsm9ds1_gyro_odr_t odr;	//output data rate
}lsm9ds1_gyro_settings_t;

#endif