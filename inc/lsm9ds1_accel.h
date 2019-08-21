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

#ifndef LSM9DS1_ACCEL_H_
#define LSM9DS1_ACCEL_H_

#include "lsm9ds1_common.h"

#define LSM9DS1_XL_DECIMATION_BIT_OFFSET 6
#define LSM9DS1_XL_DECIMATION_BIT_MASK __extension__ 0b11000000
typedef enum lsm9ds1_xl_decimation_t {
	LSM9DS1_XL_NO_DECIMATION = (0 << LSM9DS1_XL_DECIMATION_BIT_OFFSET),
	LSM9DS1_XL_2_SAMPLE_DECIMATION = (1 << LSM9DS1_XL_DECIMATION_BIT_OFFSET),
	LSM9DS1_XL_4_SAMPLE_DECIMATION = (2 << LSM9DS1_XL_DECIMATION_BIT_OFFSET),
	LSM9DS1_XL_8_SAMPLE_DECIMATION = (3 << LSM9DS1_XL_DECIMATION_BIT_OFFSET),
}lsm9ds1_xl_decimation_t;

#define LSM9DS1_XL_Z_ENABLE_BIT_OFFSET 5
#define LSM9DS1_XL_Z_ENABLE_BIT_MASK __extension__ 0b00100000
#define LSM9DS1_XL_Z_ENABLE (1 << LSM9DS1_Z_ENABLE_BIT_OFFSET)
#define LSM9DS1_XL_Z_DISABLE (0 << LSM9DS1_Z_ENABLE_BIT_OFFSET)

#define LSM9DS1_XL_Y_ENABLE_BIT_OFFSET 4
#define LSM9DS1_XL_Y_ENABLE_BIT_MASK __extension__ 0b00010000
#define LSM9DS1_XL_Y_ENABLE (1 << LSM9DS1_Y_ENABLE_BIT_OFFSET)
#define LSM9DS1_XL_Y_DISABLE (0 << LSM9DS1_Y_ENABLE_BIT_OFFSET)

#define LSM9DS1_XL_X_ENABLE_BIT_OFFSET 3
#define LSM9DS1_XL_X_ENABLE_BIT_MASK __extension__ 0b00001000
#define LSM9DS1_XL_X_ENABLE (1 << LSM9DS1_X_ENABLE_BIT_OFFSET)
#define LSM9DS1_XL_X_DISABLE (0 << LSM9DS1_X_ENABLE_BIT_OFFSET)

#define LSM9DS1_XL_ODR_BIT_OFFSET 5
#define LSM9DS1_XL_ODR_BIT_MASK __extension__ 0b11100000
typedef enum lsm9ds1_xl_odr_t{
	LSM9DS1_XL_ODR_POWERDOWN = (0 << LSM9DS1_XL_ODR_BIT_OFFSET),
	LSM9DS1_XL_10HZ = (1 <<LSM9DS1_XL_ODR_BIT_OFFSET),
	LSM9DS1_XL_50HZ = (2 << LSM9DS1_XL_ODR_BIT_OFFSET),
	LSM9DS1_XL_119HZ = (3 << LSM9DS1_XL_ODR_BIT_OFFSET),
	LSM9DS1_XL_238HZ = (4 << LSM9DS1_XL_ODR_BIT_OFFSET),
	LSM9DS1_XL_476HZ = (5 << LSM9DS1_XL_ODR_BIT_OFFSET),
	LSM9DS1_XL_952HZ = (6 << LSM9DS1_XL_ODR_BIT_OFFSET),
} lsm9ds1_xl_odr_t;

#define LSM9DS1_XL_RANGE_BIT_OFFSET 3
#define LSM9DS1_XL_RANGE_BIT_MASK __extension__ 0b00011000
typedef enum lsm9ds1_accel_range_t{
	LSM9DS1_ACCELRANGE_2G = (0 << LSM9DS1_XL_RANGE_BIT_OFFSET),
	LSM9DS1_ACCELRANGE_16G = (1 << LSM9DS1_XL_RANGE_BIT_OFFSET),
	LSM9DS1_ACCELRANGE_4G = (2 << LSM9DS1_XL_RANGE_BIT_OFFSET),
	LSM9DS1_ACCELRANGE_8G = (3 << LSM9DS1_XL_RANGE_BIT_OFFSET),
}lsm9ds1_accel_range_t;

#define LSM9DS1_XL_BANDWIDTH_SELECT_BIT_OFFSET 2
#define LSM9DS1_XL_BANDWIDTH_SELECT_BIT_MASK __extension__ 0b00000100
typedef enum lsm9ds1_xl_bandwidth_select_t {
	LSM9DS1_XL_BW_FROM_ODR = (0 << LSM9DS1_XL_BANDWIDTH_SELECT_BIT_OFFSET),
	LSM9DS1_XL_BW_FROM_FILTER = (0 << LSM9DS1_XL_BANDWIDTH_SELECT_BIT_OFFSET),
}lsm9ds1_xl_bandwidth_select_t;

#define LSM9DS1_XL_FILTER_BIT_OFFSET 0
#define LSM9DS1_XL_FILTER_BIT_MASK __extension__ 0b00000011
typedef enum lsm9ds1_xl_filter_t {
	LSM9DS1_XL_FILTER_408HZ = (0 << LSM9DS1_XL_FILTER_BIT_OFFSET),
	LSM9DS1_XL_FILTER_211HZ = (1 << LSM9DS1_XL_FILTER_BIT_OFFSET),
	LSM9DS1_XL_FILTER_105HZ = (2 << LSM9DS1_XL_FILTER_BIT_OFFSET),
	LSM9DS1_XL_FILTER_50HZ = (3 << LSM9DS1_XL_FILTER_BIT_OFFSET),
}lsm9ds1_xl_filter_t;

#define LSM9DS1_XL_HIGH_RESOLUTION_BIT_OFFSET 7
#define LSM9DS1_XL_HIGH_RESOLUTION_BIT_MASK __extension__ 0b10000000
#define LSM9DS1_XL_HIGH_RESOLUTION_ENABLE (1 << LSM9DS1_XL_HIGH_RESOLUTION_BIT_OFFSET)
#define LSM9DS1_XL_HIGH_RESOLUTION_DISABLE (0 << LSM9DS1_XL_HIGH_RESOLUTION_BIT_OFFSET)

#define LSM9DS1_XL_FDL_BIT_OFFSET 2
#define LSM9DS1_XL_FDL_BIT_MASK __extension__ 0b00000100
typedef enum lsm9ds1_xl_fdl_t {
	LSM9DS1_XL_FILTER_BYPASSED = (0 << LSM9DS1_XL_FDL_BIT_OFFSET),
	LSM9DS1_XL_FILTER_USED = (0 << LSM9DS1_XL_FDL_BIT_OFFSET),
}lsm9ds1_xl_fdl_t;

#define LSM9DS1_XL_HIGH_PASS_FILTER_BIT_OFFSET 0
#define LSM9DS1_XL_HIGH_PASS_FILTER_BIT_MASK __extension__ 0b00000001
#define LSM9DS1_XL_HIGH_PASS_FILTER_ENABLE (1 << LSM9DS1_XL_HIGH_PASS_FILETER_BIT_OFFSET)
#define LSM9DS1_XL_HIGH_PASS_FILTER_DISABLE (0 << LSM9DS1_XL_HIGH_PASS_FILETER_BIT_OFFSET)

#define SENSITIVITY_ACCELEROMETER_2  0.000061
#define SENSITIVITY_ACCELEROMETER_4  0.000122
#define SENSITIVITY_ACCELEROMETER_8  0.000244
#define SENSITIVITY_ACCELEROMETER_16 0.000732

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

typedef struct lsm9ds1_accel_settings_t {

	//CTRL REG 5 XL
	lsm9ds1_xl_decimation_t decimation;
	uint8_t z_enable;
	uint8_t y_enable;
	uint8_t x_enable;

	//CTRL REG 6 XL
	lsm9ds1_xl_odr_t odr;
	lsm9ds1_accel_range_t range;
	lsm9ds1_xl_bandwidth_select_t bw_select;
	lsm9ds1_xl_filter_t bw_filter;

	//CTRL REG 7 XL
	uint8_t high_resolution_enable;
	lsm9ds1_xl_fdl_t filtered_data_selection;
	uint8_t high_pass_filter_enable;

	float accel_resolution;
} lsm9ds1_accel_settings_t;

lsm9ds1_status_t lsm9ds1_read_accel(lsm9ds1_bus_t *bus, accelerometer_raw_data_t *raw_data);
lsm9ds1_status_t lsm9ds1_setup_accel(lsm9ds1_bus_t *bus, 
	lsm9ds1_accel_settings_t *settings, lsm9ds1_accel_range_t range);

#endif
