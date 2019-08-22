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
 * @brief Error codes returned by lsm9ds1 functions.
 */

#ifndef LSM9DS1_ERROR_H_
#define LSM9DS1_ERROR_H_

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
	LSM9DS1_BUS_ALREADY_OPEN = -23,
	LSM9DS1_CONFIG_FILE_NOT_FOUND = -24,
	LSM9DS1_CONFIG_FILE_SIZE_UKNOWN = -25,
	LSM9DS1_UNABLE_TO_READ_CONFIG = -26,
	LSM9DS1_UNABLE_TO_PARSE_JSON = -27,
	LSM9DS1_NO_SETTINGS = -28,
	LSM9DS1_INVALID_SETTING = -29,
	LSM9DS1_NULL_PARAMETER = -30,
} lsm9ds1_status_t;

#endif
