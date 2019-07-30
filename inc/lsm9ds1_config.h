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
#include "lsm9ds1_error.h"

#define LSM9DS1_MAX_STR_SIZE 256
#define LSM9DS1_CONFIG "/etc/lsm9ds1.json"

struct lsm9ds1_spi_settings {
	uint32_t speed;
};

struct lsm9ds1_spi {
	struct lsm9ds1_spi_settings settings;
};

struct lsm9ds1_bus {
	uint8_t device[LSM9DS1_MAX_STR_SIZE];
	struct lsm9ds1_spi spi;
};

typedef struct lsm9ds1_config_t {
	uint8_t name[LSM9DS1_MAX_STR_SIZE];
	struct lsm9ds1_bus bus;
}lsm9ds1_config_t;

lsm9ds1_status_t parse_json(lsm9ds1_config_t *lsm9ds1_config);

#endif