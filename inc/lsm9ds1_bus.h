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
 * @brief Functions to access the lsm9ds1.
 *
 * Initializes the LSM9DS1 for the Raspberry Pi 3B+. Currently the device is wired
 * to the first spi device. Sets up the magnetometer, accelerometer and gyroscope.
 * Provides functions to read and write the data collected on the LSM9DS1.
 */

#ifndef LSM9DS1_BUS_H_
#define LSM9DS1_BUS_H_

#define DEVICE "/dev/spidev0.0"

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
} lsm9ds1_sub_device_t;

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

typedef struct lsm9ds1_spi_t {
	lsm9ds1_spi_settings_t settings;
	lsm9ds1_xfer_t op;
	uint8_t address;
	uint8_t tx;
	uint8_t rx[1];
}lsm9ds1_spi_t;

typedef struct lsm9ds1_i2c_t {
	lsm9ds1_i2c_settings_t settings;
	lsm9ds1_xfer_t op;
	uint8_t address;
	uint8_t tx;
	uint8_t rx[1];
}lsm9ds1_i2c_t;

typedef struct lsm9ds1_bus_t {
	bool initialized;
	int32_t fd;
	char device[256];
	lsm9ds1_spi_t spi;
	lsm9ds1_i2c_t i2c;
	lsm9ds1_sub_device_t current_sub_device;

	lsm9ds1_status_t (*transfer)(struct lsm9ds1_bus_t *self, lsm9ds1_xfer_t op, uint8_t address, uint8_t tx, uint8_t *rx);
	lsm9ds1_status_t (*cs_arbiter)(struct lsm9ds1_bus_t *self);
}lsm9ds1_bus_t;

#endif
