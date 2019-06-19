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

#ifndef LSM9DS1_PRIVATE_H_
#define LSM9DS1_PRIVATE_H_

#ifdef __cplusplus
extern "C"
{
#endif

lsm9ds1_status_t lsm9ds1_select_sub_device(lsm9ds1_device_t *self, lsm9ds1_sub_device_t sub_device);

#ifdef __cplusplus
}
#endif

#endif /* LSM9DS1_PRIVATE_H_ */