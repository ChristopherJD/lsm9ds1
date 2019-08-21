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
 * @brief Accelerometer functions and data.
 *
 * Setup function, and read function for the accelerometer.
 */

#ifndef LSM9DS1_SETTINGS_H_
#define LSM9DS1_SETTINGS_H_

#include "lsm9ds1_accel.h"
#include "lsm9ds1_mag.h"
#include "lsm9ds1_gyro.h"
#include "lsm9ds1_temp.h"

/**
 * @brief Stores the settings for each sub device.
 */
typedef struct lsm9ds1_settings {
	lsm9ds1_accel_settings_t accelerometer;
	lsm9ds1_mag_settings_t magnetometer;
	lsm9ds1_gyro_settings_t gyroscope;
} lsm9ds1_settings_t;

typedef struct lsm9ds1_sub_device_t {
	lsm9ds1_bus_t bus;
}lsm9ds1_sub_device_t;

/**
 * @brief LSM9DS1 sub device.
 */
typedef struct lsm9ds1_sub_devices_t {
	lsm9ds1_sub_device_t accelerometer;
	lsm9ds1_sub_device_t magnetometer;
}lsm9ds1_sub_devices_t;

#endif