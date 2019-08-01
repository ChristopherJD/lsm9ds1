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
 * @brief SPI and I2C Bus functions and data.
 *
 * Initializes and sets up the bus to the lsm9ds1. Provides read and write
 * functions to the bus.
 */

#ifndef LSM9DS1_BUS_H_
#define LSM9DS1_BUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "lsm9ds1_error.h"

#define ACCEL "/dev/spidev0.0"
#define MAG "/dev/spidev0.1"
#define MAG_CS 21	//Wiring Pi Pin number for the magnetometer CS.

#define SPI_READ 0x80
#define SPI_WRITE 0x0

/**
 * @brief Sub device of the LSM9DS1
 *
 * The LSM9DS1 contains two sub-devices. The accelerometer gyroscope combo and
 * the magnetometer. The value 0 is used as an uknown device.
 *
 */
typedef enum {
	LSM9DS1_UNKNOWN_DEVICE = 0,
	LSM9DS1_ACCEL_GYRO = 0x68,
	LSM9DS1_MAG = 0x3D,
} lsm9ds1_sub_device_id_t;

typedef enum {
	LSM9DS1_READ, LSM9DS1_WRITE,
} lsm9ds1_xfer_t;

typedef enum lsm9ds1_xfer_bus_t {
	LSM9DS1_SPI_BUS, LSM9DS1_I2C_BUS, NUM_BUS_TYPES,
} lsm9ds1_xfer_bus_t;

typedef struct lsm9ds1_spi_settings_t {
	uint8_t mode;
	uint8_t bits;
	uint32_t speed;
	uint16_t spi_delay;
}lsm9ds1_spi_settings_t;

typedef struct lsm9ds1_i2c_settings_t {
	uint8_t bits;
}lsm9ds1_i2c_settings_t;

typedef struct lsm9ds1_i2c_t {
	bool initialized;
	//Future use
}lsm9ds1_i2c_t;

typedef struct lsm9ds1_spi_t {
	bool initialized;
	char name[256];
	int32_t fd;
	uint8_t address;
	uint8_t tx;
	uint8_t rx[1];
	lsm9ds1_xfer_t op;
	lsm9ds1_spi_settings_t settings;
}lsm9ds1_spi_t;

typedef struct lsm9ds1_bus_t {
	bool initialized;
	lsm9ds1_sub_device_id_t id;
	lsm9ds1_spi_t spi;
	lsm9ds1_i2c_t i2c;
}lsm9ds1_bus_t;

lsm9ds1_status_t lsm9ds1_write(lsm9ds1_bus_t *self, uint8_t register_addr, uint8_t tx);
lsm9ds1_status_t lsm9ds1_read(lsm9ds1_bus_t *self, uint8_t address);
lsm9ds1_status_t lsm9ds1_register_write(lsm9ds1_bus_t *self, uint8_t address, uint8_t mask, uint8_t value);
lsm9ds1_status_t init_spi(lsm9ds1_bus_t *self);


#endif
