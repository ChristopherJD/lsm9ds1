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

#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <fcntl.h>

#include "lsm9ds1_bus.h"
#include "lsm9ds1_debug.h"

static lsm9ds1_status_t spi_transfer(lsm9ds1_spi_t *self, lsm9ds1_xfer_t op,
                                 uint8_t address, uint8_t tx) {

	int8_t ret = LSM9DS1_UNKNOWN_ERROR;
	uint8_t addr_xfer = 0;

	//Update bus info
	if(!self->initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	struct spi_ioc_transfer tr[2] = { 0 };

	self->op = op;
	self->tx = tx;

	DEBUG_PRINT("Rx Size: 0x%X\n", sizeof(self->rx));

	switch (op) {
		case LSM9DS1_READ:
			tr[1].rx_buf = (unsigned long)&(self->rx);
			tr[1].len = sizeof(self->rx);
			tr[1].speed_hz = self->settings.speed;
			tr[1].delay_usecs = self->settings.spi_delay;
			tr[1].bits_per_word = self->settings.bits;

			addr_xfer = (SPI_READ | address);
			break;
		case LSM9DS1_WRITE:
			tr[1].tx_buf = (unsigned long) &(self->tx);
			tr[1].len = sizeof(self->tx);
			tr[1].speed_hz = self->settings.speed;
			tr[1].delay_usecs = self->settings.spi_delay;
			tr[1].bits_per_word = self->settings.bits;

			addr_xfer = (SPI_WRITE | address);
			break;
		default:
			return LSM9DS1_UNSUPPORTED_OP;
	}

	// Setup the address to r/w
	self->address = addr_xfer;
	DEBUG_PRINT("Address: 0x%X\n", self->address);

	tr[0].tx_buf = (unsigned long) &(self->address);
	tr[0].len = sizeof(self->address);
	tr[0].speed_hz = self->settings.speed;
	tr[0].delay_usecs = self->settings.spi_delay;
	tr[0].bits_per_word = self->settings.bits;

	DEBUG_PRINT("fd: 0x%X\n", self->fd);
	ret = ioctl(self->fd, SPI_IOC_MESSAGE(2), tr);
	DEBUG_PRINT("TX: 0x%X\n", self->tx);
	DEBUG_PRINT("RX: 0x%X\n", self->rx[0]);
	if (ret < 1) {
		return LSM9DS1_SPI_BUS_XFER_ERROR;
	}

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_write(lsm9ds1_bus_t *self, uint8_t register_addr, uint8_t tx) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;
	lsm9ds1_xfer_t operation = LSM9DS1_WRITE;

	if(!self->initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	if(self->spi.initialized) {
		ret = spi_transfer(&(self->spi), operation, register_addr, tx);
	}
	else if(self->i2c.initialized) {
		// Not implemented
	}
	else {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	return ret;
}

lsm9ds1_status_t lsm9ds1_read(lsm9ds1_bus_t *self, uint8_t address) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;
	lsm9ds1_xfer_t operation = LSM9DS1_READ;

	if(!self->initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	if(self->spi.initialized) {
		ret = spi_transfer(&(self->spi), operation, address, 0);
	}
	else if(self->i2c.initialized) {
		// Not implemented 
	}
	else {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	return ret;
}

lsm9ds1_status_t lsm9ds1_register_write(lsm9ds1_bus_t *self, uint8_t address, uint8_t mask, uint8_t value) {

	uint8_t reg = 0;
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Get the initial register state
	status = lsm9ds1_read(self, address);
	if (status < 0) return status;
	reg = self->spi.rx[0];

	// Mask off with the value to be written
	reg &= ~(mask);
	reg |= value;

	status = lsm9ds1_write(self, address, value);
	if (status < 0) return status;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t init_spi(lsm9ds1_bus_t *self) {

	int8_t ret = -1;	// Function return codes.

	self->spi.settings.mode 		= 0;
	self->spi.settings.bits 		= 8;
	self->spi.settings.speed 		= 15000000;
	self->spi.settings.spi_delay 	= 0;

	switch(self->id) {
		case LSM9DS1_ACCEL_GYRO:
			strncpy(self->spi.name, ACCEL, sizeof(self->spi.name));
			break;
		case LSM9DS1_MAG:
			strncpy(self->spi.name, MAG, sizeof(self->spi.name));
			break;
		default:
			return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	DEBUG_PRINT("Device: %s\n", self->spi.name);

	self->spi.fd = open(self->spi.name, O_RDWR);
	if (self->spi.fd <= 0) {
		return LSM9DS1_NOT_FOUND;
	}

	DEBUG_PRINT("Device fd: %d\n", self->spi.fd);

	// Set to mode 3
	self->spi.settings.mode |= SPI_MODE_3;
	ret = ioctl(self->spi.fd, SPI_IOC_WR_MODE, &(self->spi.settings.mode));
	if (ret == -1) {
		return LSM9DS1_MODE_3_NOT_SET;
	}

#if DEBUG > 0
	ret = ioctl(self->spi.fd, SPI_IOC_RD_MODE, &(self->spi.settings.mode));
	if (ret == -1) {
		return LSM9DS1_MODE_3_NOT_SET;
	}
	DEBUG_PRINT("SPI_IOC_RD_MODE: 0x%X\n", self->spi.settings.mode);
#endif

	// Set the number of bits per word.
	ret = ioctl(self->spi.fd, SPI_IOC_WR_BITS_PER_WORD, &(self->spi.settings.bits));
	if (ret == -1) {
		return LSM9DS1_NUM_BITS_NOT_SET;
	}

	// Set the max clock speed in hz
	ret = ioctl(self->spi.fd, SPI_IOC_WR_MAX_SPEED_HZ, &(self->spi.settings.speed));
	if (ret == -1) {
		return LSM9DS1_CLOCK_NOT_SET;
	}

	self->spi.initialized = true;

	return LSM9DS1_SUCCESS;
}
