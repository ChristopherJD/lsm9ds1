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

#include "lsm9ds1_error.h"
#include "lsm9ds1_common.h"

lsm9ds1_status_t read_bit_value(uint8_t value, uint8_t mask, uint8_t offset, uint8_t *read_value) {

	*read_value = ((value & mask) >> offset);

	return LSM9DS1_SUCCESS;
}
