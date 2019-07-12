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
 * @brief Temperature functions and data.
 *
 * Read function for the temperature.
 */

#ifndef LSM9DS1_TEMP_H_
#define LSM9DS1_TEMP_H_

#include "lsm9ds1_common.h"
#include "lsm9ds1_error.h"

/**
 * @brief Temperature returned from the LSM9DS1
 */
typedef int16_t lsm9ds1_temperature_t;

lsm9ds1_status_t lsm9ds1_read_temp(lsm9ds1_bus_t *bus, lsm9ds1_temperature_t *raw_data);

#endif