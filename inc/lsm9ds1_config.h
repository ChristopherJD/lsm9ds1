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
 * @brief 
 */

#ifndef LSM9DS1_CONFIG_H_
#define LSM9DS1_CONFIG_H_

#include <stdint.h>
#include "lsm9ds1_settings.h"

#define LSM9DS1_MAX_STR_SIZE 256
#define MAX_SPI_SPEED 15000000
#define LSM9DS1_CONFIG "/usr/etc/lsm9ds1.json"

typedef struct lsm9ds1_config_t {
	char name[LSM9DS1_MAX_STR_SIZE];
	lsm9ds1_settings_t settings;
	lsm9ds1_xfer_bus_t xfer_bus;
	lsm9ds1_sub_devices_t sub_device;
}lsm9ds1_config_t;

extern lsm9ds1_config_t glsm9ds1_config;

lsm9ds1_status_t init_config();

#endif
