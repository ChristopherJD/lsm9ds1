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
 * @brief Common functions used by the lsm9ds1.
 *
 * Select sub device, reset the device and bit operations.
 */

#ifndef LSM9DS1_COMMON_H_
#define LSM9DS1_COMMON_H_

#include <stdint.h>
#include "lsm9ds1_bus.h"

lsm9ds1_status_t is_correct_sub_device(lsm9ds1_bus_t *self, lsm9ds1_sub_device_id_t sub_device);
lsm9ds1_status_t read_bit_value(uint8_t value, uint8_t mask, uint8_t offset, uint8_t *read_value);
lsm9ds1_status_t lsm9ds1_soft_reset(lsm9ds1_bus_t *bus);

#endif