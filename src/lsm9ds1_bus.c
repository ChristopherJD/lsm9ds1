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
#include <wiringPi.h>

#include "lsm9ds1_bus.h"
#include "lsm9ds1_common.h"

static lsm9ds1_status_t lsm9ds1_mag_cs(int pin_state) {
	digitalWrite(MAG_CS, pin_state);
	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_setup_mag_cs() {

	//Setup for Mag CS
	(void)wiringPiSetup();

	(void)pinMode(MAG_CS, OUTPUT);

	// CS is logic low, so set value HIGH to have this device ignored until
	// otherwise decided.
	(void) lsm9ds1_mag_cs(HIGH);

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t cs_arbiter(lsm9ds1_bus_t *self) {

	int8_t ret = LSM9DS1_UNKNOWN_ERROR;

	switch (self->current_sub_device) {
	case LSM9DS1_MAG:

		self->spi.settings.mode |= SPI_NO_CS;
		ret = ioctl(self->fd, SPI_IOC_WR_MODE, &(self->spi.settings.mode));
		if (ret == -1) {
			return LSM9DS1_UNABLE_TO_SET_CS;
		}

		DEBUG_PRINT("Selecting LSM9DS1_MAG (0x%X)\n", self->current_sub_device);

		(void) lsm9ds1_mag_cs(LOW);

		break;
	case LSM9DS1_ACCEL_GYRO:
		//Make sure CS is selected.
		self->spi.settings.mode &= ~(SPI_NO_CS);
		ret = ioctl(self->fd, SPI_IOC_WR_MODE, &(self->spi.settings.mode));
		if (ret == -1) {
			return LSM9DS1_UNABLE_TO_SET_CS;
		}

		DEBUG_PRINT("Selecting LSM9DS1_ACCEL_GYRO (0x%X)\n", self->current_sub_device);

		break;
	default:
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t transfer(lsm9ds1_bus_t *self, lsm9ds1_xfer_t op,
                                 uint8_t address, uint8_t tx, uint8_t *rx) {

	int8_t ret = LSM9DS1_UNKNOWN_ERROR;

	// Select the chip select based on the current_sub_device.
	self->cs_arbiter(self);

	uint8_t addr_xfer = 0;

	struct spi_ioc_transfer tr[2] = { 0 };

	switch (op) {
	case LSM9DS1_READ:
		tr[1].rx_buf = (unsigned long) rx;
		tr[1].len = sizeof(*rx);
		tr[1].speed_hz = self->spi.settings.speed;
		tr[1].delay_usecs = self->spi.settings.spi_delay;
		tr[1].bits_per_word = self->spi.settings.bits;

		addr_xfer = (SPI_READ | address);
		break;
	case LSM9DS1_WRITE:
		tr[1].tx_buf = (unsigned long) &tx;
		tr[1].len = sizeof(tx);
		tr[1].speed_hz = self->spi.settings.speed;
		tr[1].delay_usecs = self->spi.settings.spi_delay;
		tr[1].bits_per_word = self->spi.settings.bits;

		if (NULL != rx) {
			rx = NULL;
		}

		addr_xfer = (SPI_WRITE | address);
		break;
	default:
		return LSM9DS1_UNSUPPORTED_OP;
	}

	// Setup the address to w/r
	tr[0].tx_buf = (unsigned long) &addr_xfer;
	tr[0].len = sizeof(tx);
	tr[0].speed_hz = self->spi.settings.speed;
	tr[0].delay_usecs = self->spi.settings.spi_delay;
	tr[0].bits_per_word = self->spi.settings.bits;

	ret = ioctl(self->fd, SPI_IOC_MESSAGE(2), tr);
	if (LSM9DS1_MAG == self->current_sub_device) {
		// Set back to high after the message was sent.
		(void) lsm9ds1_mag_cs(HIGH);
	}
	DEBUG_PRINT("TX: 0x%X\n", self->spi.tx);
	DEBUG_PRINT("RX: 0x%X\n", self->spi.rx[0]);
	if (ret < 1) {
		return LSM9DS1_SPI_BUS_XFER_ERROR;
	}

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_write(lsm9ds1_bus_t *self, uint8_t register_addr, uint8_t tx) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	//RX doesn't matter so give it NULL
	self->spi.op = LSM9DS1_WRITE;
	self->spi.tx = tx;
	ret = self->transfer(self, self->spi.op, self->spi.address, self->spi.tx, NULL);

	return ret;
}

lsm9ds1_status_t lsm9ds1_read(lsm9ds1_bus_t *self, uint8_t address) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	// tx doesn't matter, so just use 0.
	self->spi.op = LSM9DS1_READ;
	self->spi.address = address;
	ret = self->transfer(self, self->spi.op, self->spi.address, 0, self->spi.rx);

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

	// Setup the magnetometer, this only needs to be done once.
	DEBUG_PRINT("Setting up mag cs...\n");
	ret = lsm9ds1_setup_mag_cs();
	if (ret < 0) {
		DEBUG_PRINT("Error setting up mag cs!\n");
		return ret;
	}

	self->spi.settings.mode 		= 0;
	self->spi.settings.bits 		= 8;
	self->spi.settings.speed 		= 15000000;
	self->spi.settings.spi_delay 	= 0;
	self->transfer = transfer;
	self->cs_arbiter = cs_arbiter;

	strncpy(self->device, DEVICE, sizeof(self->device));

	DEBUG_PRINT("Device fd: %s\n", self->device);

	self->fd = open(self->device, O_RDWR);
	if (self->fd <= 0) {
		return LSM9DS1_NOT_FOUND;
	}

	DEBUG_PRINT("Device fd: %d\n", self->fd);

	// Set to mode 3
	self->spi.settings.mode |= SPI_CPOL | SPI_CPHA | SPI_NO_CS;
	ret = ioctl(self->fd, SPI_IOC_WR_MODE, &(self->spi.settings.mode));
	if (ret == -1) {
		return LSM9DS1_MODE_3_NOT_SET;
	}

#if DEBUG > 0
	ret = ioctl(self->fd, SPI_IOC_RD_MODE, &(self->spi.settings.mode));
	if (ret == -1) {
		return LSM9DS1_MODE_3_NOT_SET;
	}
	DEBUG_PRINT("SPI_IOC_RD_MODE: 0x%X\n", self->spi.settings.mode);
#endif

	// Set the number of bits per word.
	ret = ioctl(self->fd, SPI_IOC_WR_BITS_PER_WORD, &(self->spi.settings.bits));
	if (ret == -1) {
		return LSM9DS1_NUM_BITS_NOT_SET;
	}

	// Set the max clock speed in hz
	ret = ioctl(self->fd, SPI_IOC_WR_MAX_SPEED_HZ, &(self->spi.settings.speed));
	if (ret == -1) {
		return LSM9DS1_CLOCK_NOT_SET;
	}

	return LSM9DS1_SUCCESS;
}
