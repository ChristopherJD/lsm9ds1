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
 *
 * Initializes the LSM9DS1 for the Raspberry Pi 3B+. Currently the device is wired
 * to the first spi device. Sets up the magnetometer, accelerometer and gyroscope.
 * Provides functions to read and write the data collected on the LSM9DS1.
 */

#ifndef LSM9DS1_COMMON_H_
#define LSM9DS1_COMMON_H_

#include <stdint.h>

#if DEBUG > 0
#include <stdio.h>
#define DEBUG_PRINT(fmt, ...) fprintf(stderr, "[%s:%d:%s()]: " fmt, \
		__FILE__, __LINE__, __func__, ## __VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) /* Don't do anything in release builds */
#endif

lsm9ds1_status_t read_bit_value(uint8_t value, uint8_t mask, uint8_t offset, uint8_t *read_value);

#endif