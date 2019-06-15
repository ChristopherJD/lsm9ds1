/*
 * Copyright (C) Your copyright.
 *
 * Author: Christopher Jordan-Denny
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


//TODO Add doxygen comments.
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>
#include <wiringPi.h>

#include "lsm9ds1.h"

#ifndef __GNUC__
#error "__GNUC__ not defined"
#else

#if DEBUG > 0
#define DEBUG_PRINT(fmt, ...) fprintf(stderr, "[%s:%d:%s()]: " fmt, \
		__FILE__, __LINE__, __func__, ## __VA_ARGS__)
#else
#define DEBUG_PRINT(fmt, ...) /* Don't do anything in release builds */
#endif

static bool bus_initialized = false;
static uint8_t num_calls = 0;

//TODO move these to the correct function
// static uint8_t mode = 0;
// static uint8_t bits = 8;
// static uint32_t speed = 15000000;
// static uint16_t spi_delay = 0;
static int32_t fd = -1;	// File descriptor for LSM9DS1

#define MAG_CS 21	//Wiring Pi Pin number

static lsm9ds1_status_t lsm9ds1_mag_cs(int pin_state) {
	digitalWrite(MAG_CS, pin_state);
	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t transfer(lsm9ds1_device_t *self, lsm9ds1_xfer_t op,
                                 uint8_t address, uint8_t tx, uint8_t *rx) {

	int8_t ret = LSM9DS1_UNKNOWN_ERROR;

	switch (self->current_sub_device) {
	case LSM9DS1_MAG:

		self->bus.spi.settings.mode |= SPI_NO_CS;
		ret = ioctl(fd, SPI_IOC_WR_MODE, &(self->bus.spi.settings.mode));
		if (ret == -1) {
			return LSM9DS1_UNABLE_TO_SET_CS;
		}

		DEBUG_PRINT("Selecting LSM9DS1_MAG (0x%X)\n", self->current_sub_device);

		(void) lsm9ds1_mag_cs(LOW);

		break;
	case LSM9DS1_ACCEL_GYRO:
		//Make sure CS is selected.
		self->bus.spi.settings.mode &= ~(SPI_NO_CS);
		ret = ioctl(fd, SPI_IOC_WR_MODE, &(self->bus.spi.settings.mode));
		if (ret == -1) {
			return LSM9DS1_UNABLE_TO_SET_CS;
		}

		DEBUG_PRINT("Selecting LSM9DS1_ACCEL_GYRO (0x%X)\n", self->current_sub_device);

		break;
	default:
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	uint8_t addr_xfer = 0;

	//TODO ensure sizes are correct
	struct spi_ioc_transfer tr[2] = { 0 };

	switch (op) {
	case LSM9DS1_READ:
		tr[1].rx_buf = (unsigned long) rx;
		tr[1].len = sizeof(*rx);
		tr[1].speed_hz = self->bus.spi.settings.speed;
		tr[1].delay_usecs = self->bus.spi.settings.spi_delay;
		tr[1].bits_per_word = self->bus.spi.settings.bits;

		addr_xfer = (SPI_READ | address);
		break;
	case LSM9DS1_WRITE:
		tr[1].tx_buf = (unsigned long) &tx;
		tr[1].len = sizeof(tx);
		tr[1].speed_hz = self->bus.spi.settings.speed;
		tr[1].delay_usecs = self->bus.spi.settings.spi_delay;
		tr[1].bits_per_word = self->bus.spi.settings.bits;

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
	tr[0].speed_hz = self->bus.spi.settings.speed;
	tr[0].delay_usecs = self->bus.spi.settings.spi_delay;
	tr[0].bits_per_word = self->bus.spi.settings.bits;

	ret = ioctl(fd, SPI_IOC_MESSAGE(2), tr);
	if (LSM9DS1_MAG == self->current_sub_device) {
		// Set back to high after the message was sent.
		(void) lsm9ds1_mag_cs(HIGH);
	}
	DEBUG_PRINT("TX: %d\n", self->bus.spi.tx);
	DEBUG_PRINT("RX: %d\n", self->bus.spi.rx[0]);
	if (ret < 1) {
		return LSM9DS1_SPI_BUS_XFER_ERROR;
	}

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_write(lsm9ds1_device_t *self, uint8_t register_addr, uint8_t tx) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	//RX doesn't matter so give it NULL
	self->bus.spi.op = LSM9DS1_WRITE;
	self->bus.spi.tx = tx;
	ret = transfer(self, self->bus.spi.op, self->bus.spi.address, self->bus.spi.tx, NULL);

	return ret;
}

static lsm9ds1_status_t lsm9ds1_read(lsm9ds1_device_t *self, uint8_t address) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	// tx doesn't matter, so just use 0.
	self->bus.spi.op = LSM9DS1_READ;
	self->bus.spi.address = address;
	ret = transfer(self, self->bus.spi.op, self->bus.spi.address, 0, self->bus.spi.rx);

	return ret;
}

lsm9ds1_status_t lsm9ds1_select_sub_device(lsm9ds1_device_t *self, lsm9ds1_sub_device_t sub_device) {

	lsm9ds1_status_t function_return = LSM9DS1_UNKNOWN_ERROR;

	// The mag accel and gyro id should be at the same offset, if not, we don't know what device we have.
	// We are comparing enums of different types, cast first since we want to do this.
	if (!((int) LSM9DS1_REGISTER_WHO_AM_I_XG
	        == (int) LSM9DS1_REGISTER_WHO_AM_I_M)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	self->current_sub_device = sub_device;

	// Discover device
	function_return = lsm9ds1_read(self, LSM9DS1_REGISTER_WHO_AM_I);
	lsm9ds1_sub_device_t found_device = self->bus.spi.rx[0];
	DEBUG_PRINT("Sub-device: (0x%X)\n", found_device);

	if(found_device != sub_device) return LSM9DS1_UNKNOWN_SUB_DEVICE;

	return function_return;
}

static lsm9ds1_status_t lsm9ds1_setup_mag_cs() {

	//TODO, verify the documentation is a crappy as I think.
	//Setup for Mag CS
	wiringPiSetup();

	pinMode(MAG_CS, OUTPUT);

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_setup_mag(lsm9ds1_device_t *self, lsm9ds1_mag_gain_t gain) {

	if (!bus_initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// CS logic low, so set to high to ignore.
	(void) lsm9ds1_mag_cs(HIGH);

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_MAG);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	uint8_t read_buffer = 0;
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_CTRL_REG2_M);

	if (read_status < 0) {
		return read_status;
	}

	DEBUG_PRINT("Reading LSM9DS1_REGISTER_CTRL_REG2_M: 0x%X\n", read_buffer);

	uint8_t reg = read_buffer;
	reg &= ~(__extension__ 0b01100000);
	reg |= gain;

	lsm9ds1_status_t write_status = LSM9DS1_UNKNOWN_ERROR;
	DEBUG_PRINT("Writing LSM9DS1_REGISTER_CTRL_REG2_M: 0x%X\n", reg);
	write_status = lsm9ds1_write(self, LSM9DS1_REGISTER_CTRL_REG2_M, reg);
	if (write_status < 0) {
		return write_status;
	}

#if DEBUG > 0
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_CTRL_REG2_M);
	if (read_status < 0) {
		return read_status;
	}

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG2_M: 0x%X\n", read_buffer);
#endif

	self->settings.magnetometer.gain = gain;

	switch (gain) {
	case LSM9DS1_MAGGAIN_4GAUSS:
		self->settings.magnetometer.mag_mgauss = LSM9DS1_MAG_MGAUSS_4GAUSS;
		break;
	case LSM9DS1_MAGGAIN_8GAUSS:
		self->settings.magnetometer.mag_mgauss = LSM9DS1_MAG_MGAUSS_8GAUSS;
		break;
	case LSM9DS1_MAGGAIN_12GAUSS:
		self->settings.magnetometer.mag_mgauss = LSM9DS1_MAG_MGAUSS_12GAUSS;
		break;
	case LSM9DS1_MAGGAIN_16GAUSS:
		self->settings.magnetometer.mag_mgauss = LSM9DS1_MAG_MGAUSS_16GAUSS;
		break;
	default:
		return LSM9DS1_UKNOWN_GAIN_RANGE;
	}

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_setup_accel(lsm9ds1_device_t *self, lsm9ds1_accel_range_t range) {

	if (!bus_initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	//TODO Figure out how to make this happen when setting up the gyro
	// Soft reset on the accelerometer and gyroscope
	lsm9ds1_write(self, LSM9DS1_REGISTER_CTRL_REG8, 0x05);

	// enable gyro continuous
  	lsm9ds1_write(self, LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0); // on XYZ

  	// Enable the accelerometer continous
  	lsm9ds1_write(self, LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38); // enable X Y and Z axis
	lsm9ds1_write(self, LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0); // 1 KHz out data rate, BW set by ODR, 408Hz anti-aliasing

	// Read the accelerometer.
	uint8_t read_buffer = 0;
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_CTRL_REG6_XL);

	if (read_status < 0) {
		return read_status;
	}

	DEBUG_PRINT("Reading LSM9DS1_REGISTER_CTRL_REG6_XL: 0x%X\n", read_buffer);

	uint8_t reg = read_buffer;
	reg &= ~( __extension__ 0b00011000);
	reg |= range;

	DEBUG_PRINT("Setting LSM9DS1_REGISTER_CTRL_REG6_XL: 0x%X\n", reg);
	lsm9ds1_status_t write_status = LSM9DS1_UNKNOWN_ERROR;
	write_status = lsm9ds1_write(self, LSM9DS1_REGISTER_CTRL_REG6_XL, reg);
	if (write_status < 0) {
		return write_status;
	}

#if DEBUG > 0
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_CTRL_REG6_XL);

	if (read_status < 0) {
		return read_status;
	}

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG6_XL: 0x%X\n", read_buffer);
#endif

	self->settings.accelerometer.range = range;

	switch (range) {
	case LSM9DS1_ACCELRANGE_2G:
		self->settings.accelerometer.accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
		break;
	case LSM9DS1_ACCELRANGE_4G:
		self->settings.accelerometer.accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
		break;
	case LSM9DS1_ACCELRANGE_8G:
		self->settings.accelerometer.accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
		break;
	case LSM9DS1_ACCELRANGE_16G:
		self->settings.accelerometer.accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_16G;
		break;
	default:
		return LSM9DS1_UKNOWN_ACCEL_RANGE;
	}

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_setup_gyro(lsm9ds1_device_t *self, lsm9ds1_gyro_scale_t scale) {

	if (!bus_initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	//TODO Cleanup
	// Soft reset on the magnetometer
	lsm9ds1_write(self, LSM9DS1_REGISTER_CTRL_REG8, 0x0C);

	lsm9ds1_write(self, LSM9DS1_REGISTER_CTRL_REG3_M, 0x00); // continuous mode

	// Read the accelerometer.
	uint8_t read_buffer = 0;
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_CTRL_REG1_G);

	if (read_status < 0) {
		return read_status;
	}

	DEBUG_PRINT("Reading LSM9DS1_REGISTER_CTRL_REG1_G: 0x%X\n", read_buffer);

	uint8_t reg = read_buffer;

	reg &= ~(__extension__ 0b00011000);
	reg |= scale;

	lsm9ds1_status_t write_status = LSM9DS1_UNKNOWN_ERROR;
	DEBUG_PRINT("Setting LSM9DS1_REGISTER_CTRL_REG1_G: 0x%X\n", reg);
	write_status = lsm9ds1_write(self, LSM9DS1_REGISTER_CTRL_REG1_G, reg);
	if (write_status < 0) {
		return write_status;
	}

#if DEBUG > 0
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_CTRL_REG1_G);

	if (read_status < 0) {
		return read_status;
	}
	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG1_G: 0x%X\n", read_buffer);
#endif

	self->settings.gyroscope.scale = scale;

	switch (scale) {
	case LSM9DS1_GYROSCALE_245DPS:
		self->settings.gyroscope.gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_245DPS;
		break;
	case LSM9DS1_GYROSCALE_500DPS:
		self->settings.gyroscope.gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_500DPS;
		break;
	case LSM9DS1_GYROSCALE_2000DPS:
		self->settings.gyroscope.gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_2000DPS;
		break;
	default:
		return LSM9DS1_UKNOWN_GAIN_RANGE;
	}

	return LSM9DS1_SUCCESS;
}
static lsm9ds1_status_t init_spi(lsm9ds1_device_t *self) {

	int8_t ret = -1;	// Function return codes.

	self->bus.spi.settings.mode 		= 0;
	self->bus.spi.settings.bits 		= 8;
	self->bus.spi.settings.speed 		= 15000000;
	self->bus.spi.settings.spi_delay 	= 0;


	fd = open(DEVICE, O_RDWR);
	if (fd < 0) {
		return LSM9DS1_NOT_FOUND;
	}

	// Set to mode 3
	self->bus.spi.settings.mode |= SPI_CPOL | SPI_CPHA | SPI_NO_CS;
	ret = ioctl(fd, SPI_IOC_WR_MODE, &(self->bus.spi.settings.mode));
	if (ret == -1) {
		return LSM9DS1_MODE_3_NOT_SET;
	}

#if DEBUG > 0
	ret = ioctl(fd, SPI_IOC_RD_MODE, &(self->bus.spi.settings.mode));
	if (ret == -1) {
		return LSM9DS1_MODE_3_NOT_SET;
	}
	DEBUG_PRINT("SPI_IOC_RD_MODE: 0x%X\n", self->bus.spi.settings.mode);
#endif

	// Set the number of bits per word.
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &(self->bus.spi.settings.bits));
	if (ret == -1) {
		return LSM9DS1_NUM_BITS_NOT_SET;
	}

	// Set the max clock speed in hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &(self->bus.spi.settings.speed));
	if (ret == -1) {
		return LSM9DS1_CLOCK_NOT_SET;
	}

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t init_i2c(lsm9ds1_device_t *self) {
	// Probably won't implement for raspberrypi
	return LSM9DS1_BUS_NOT_SUPPORTED;
}

static lsm9ds1_status_t lsm9ds1_is_init(bool *initialized) {

	bool has_been_init = false;
	if (num_calls > 0) {
		has_been_init = true;
	}

	*initialized = has_been_init;

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_init_bus(lsm9ds1_device_t *self, lsm9ds1_xfer_bus_t bus_type) {

	if (bus_initialized) {
		return LSM9DS1_SUCCESS;
	}

	//TODO decide if we need to remove
	// If we have already opened the fd then return early
	if (fd >= 0) {
		return LSM9DS1_SUCCESS;
	}

	// lsm9ds1 settings
	self->xfer_bus = bus_type;

	// Determine bus type being used.
	int8_t ret = -1;	// Function return codes.
	switch (self->xfer_bus) {
	case LSM9DS1_SPI_BUS:
		ret = init_spi(self);
		break;
	case LSM9DS1_I2C_BUS:
		ret = init_i2c(self);
		break;
	default:
		ret = LSM9DS1_BUS_NOT_SUPPORTED;
		break;

	}

	bus_initialized = true;
	return ret;
}

static lsm9ds1_status_t lsm9ds1_read_accel(lsm9ds1_device_t *self) {

	// Check that the lsm9ds1 has been initialized.
	bool lsm9ds1_initialized = false;
	(void) lsm9ds1_is_init(&lsm9ds1_initialized);
	if (!lsm9ds1_initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_X_L_XL);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_X_H_XL);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Y_L_XL);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Y_H_XL);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Z_L_XL);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Z_H_XL);
	if (read_status < 0) {
		return read_status;
	}
	int16_t zhi = self->bus.spi.rx[0];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8;
	xhi |= xlo;
	yhi <<= 8;
	yhi |= ylo;
	zhi <<= 8;
	zhi |= zlo;

	self->raw_data.accelerometer.x = xhi;
	self->raw_data.accelerometer.y = yhi;
	self->raw_data.accelerometer.z = zhi;

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_read_mag(lsm9ds1_device_t *self) {

	// Check that the lsm9ds1 has been initialized.
	bool lsm9ds1_initialized = false;
	(void) lsm9ds1_is_init(&lsm9ds1_initialized);
	if (!lsm9ds1_initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_MAG);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_X_L_M);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_X_H_M);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Y_L_M);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Y_H_M);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Z_L_M);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Z_H_M);
	if (read_status < 0) {
		return read_status;
	}
	int16_t zhi = self->bus.spi.rx[0];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8;
	xhi |= xlo;
	yhi <<= 8;
	yhi |= ylo;
	zhi <<= 8;
	zhi |= zlo;

	self->raw_data.magnetometer.x = xhi;
	self->raw_data.magnetometer.y = yhi;
	self->raw_data.magnetometer.z = zhi;

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_read_temp(lsm9ds1_device_t *self) {

	// Check that the lsm9ds1 has been initialized.
	bool lsm9ds1_initialized = false;
	(void) lsm9ds1_is_init(&lsm9ds1_initialized);
	if (!lsm9ds1_initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_TEMP_OUT_L);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = self->bus.spi.rx[0];

	DEBUG_PRINT("Temp xlo: 0x%X\n", xlo);

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_TEMP_OUT_H);
	if (read_status < 0) {
		return read_status;
	}
	lsm9ds1_temperature_t xhi = self->bus.spi.rx[0];

	DEBUG_PRINT("Temp xhi: 0x%X\n", xhi);

	xhi <<= 8;
	xhi |= xlo;

	// Shift values to create properly formed integer (low byte first)
	self->raw_data.temperature = xhi;

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_read_gyro(lsm9ds1_device_t *self) {

	// Check that the lsm9ds1 has been initialized.
	bool lsm9ds1_initialized = false;
	(void) lsm9ds1_is_init(&lsm9ds1_initialized);
	if (!lsm9ds1_initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	// Read the gryo.
	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_X_L_G);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_X_H_G);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Y_L_G);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Y_H_G);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Z_L_G);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(self, LSM9DS1_REGISTER_OUT_Z_H_G);
	if (read_status < 0) {
		return read_status;
	}
	int16_t zhi = self->bus.spi.rx[0];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8;
	xhi |= xlo;
	yhi <<= 8;
	yhi |= ylo;
	zhi <<= 8;
	zhi |= zlo;

	self->raw_data.gyroscope.x = xhi;
	self->raw_data.gyroscope.y = yhi;
	self->raw_data.gyroscope.z = zhi;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t update_temp(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_read_temp(self);

	self->converted_data.temperature = (21.0 + (((float)self->raw_data.temperature) / 8));

	return status;
}

lsm9ds1_status_t update_accel(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Get the raw data
	status = lsm9ds1_read_accel(self);

	DEBUG_PRINT("accel_mg_lsb: %f\n", self->settings.accelerometer.accel_mg_lsb);
	DEBUG_PRINT("Accel Raw (x): 0x%X\n", self->raw_data.accelerometer.x);
	DEBUG_PRINT("Accel Raw (y): 0x%X\n", self->raw_data.accelerometer.y);
	DEBUG_PRINT("Accel Raw (z): 0x%X\n", self->raw_data.accelerometer.z);

	// Convert the raw data
	self->converted_data.accelerometer.x = self->raw_data.accelerometer.x * self->settings.accelerometer.accel_mg_lsb;
	self->converted_data.accelerometer.y = self->raw_data.accelerometer.y * self->settings.accelerometer.accel_mg_lsb;
	self->converted_data.accelerometer.z = self->raw_data.accelerometer.z * self->settings.accelerometer.accel_mg_lsb;

	return status;
}

lsm9ds1_status_t update_mag(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Get the raw data
	status = lsm9ds1_read_mag(self);

	DEBUG_PRINT("mag_mgauss: %f\n", self->settings.magnetometer.mag_mgauss);
	DEBUG_PRINT("Mag Raw (x): 0x%X\n", self->raw_data.magnetometer.x);
	DEBUG_PRINT("Mag Raw (y): 0x%X\n", self->raw_data.magnetometer.y);
	DEBUG_PRINT("Mag Raw (z): 0x%X\n", self->raw_data.magnetometer.z);

	// Convert the raw data
	self->converted_data.magnetometer.x = self->raw_data.magnetometer.x * self->settings.magnetometer.mag_mgauss;
	self->converted_data.magnetometer.y = self->raw_data.magnetometer.y * self->settings.magnetometer.mag_mgauss;
	self->converted_data.magnetometer.z = self->raw_data.magnetometer.z * self->settings.magnetometer.mag_mgauss;

	return status;
}

lsm9ds1_status_t update_gyro(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Get the raw data
	status = lsm9ds1_read_gyro(self);

	DEBUG_PRINT("gyro_dps_digit: %f\n", self->settings.gyroscope.gyro_dps_digit);
	DEBUG_PRINT("Gyro Raw (x): 0x%X\n", self->raw_data.gyroscope.x);
	DEBUG_PRINT("Gyro Raw (y): 0x%X\n", self->raw_data.gyroscope.y);
	DEBUG_PRINT("Gyro Raw (z): 0x%X\n", self->raw_data.gyroscope.z);

	// Convert the raw data
	self->converted_data.gyroscope.x = self->raw_data.gyroscope.x * self->settings.gyroscope.gyro_dps_digit;
	self->converted_data.gyroscope.y = self->raw_data.gyroscope.y * self->settings.gyroscope.gyro_dps_digit;
	self->converted_data.gyroscope.z = self->raw_data.gyroscope.z * self->settings.gyroscope.gyro_dps_digit;

	return status;
}

lsm9ds1_status_t update(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = self->update_temp(self);
	if(status < 0) return status;

	self->update_accel(self);
	if(status < 0) return status;

	self->update_mag(self);
	if(status < 0) return status;

	self->update_gyro(self);
	if(status < 0) return status;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_init(lsm9ds1_device_t *self, lsm9ds1_xfer_bus_t bus_type,
                              lsm9ds1_accel_range_t range, lsm9ds1_mag_gain_t gain,
                              lsm9ds1_gyro_scale_t scale) {
	if (NULL == self) {
		return LSM9DS1_MALLOC_DEVICE_ERROR;
	}

	DEBUG_PRINT("Build Version: %s\n", _BUILD_VERSION);
	DEBUG_PRINT("Build Date/Time: %s %s\n", __DATE__, __TIME__);

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	// link functions
	self->update_temp = update_temp;
	self->update_accel = update_accel;
	self->update_mag = update_mag;
	self->update_gyro = update_gyro;
	self->update = update;

	//reset number of calls if necessary
	if (num_calls == UINT8_MAX) {
		num_calls = 0;
	}
	num_calls++;

#if DEBUG > 0
	const char *bus_names[NUM_BUS_TYPES] = {"SPI", "I2C"};
	DEBUG_PRINT("Initializing the lsm9ds1 %s bus...\n", bus_names[bus_type]);
#endif

	ret = lsm9ds1_init_bus(self, bus_type);
	if (ret < 0) {
		return ret;
	};

	DEBUG_PRINT("Setting up accelerometer... range(%d)\n", range);
	ret = lsm9ds1_setup_accel(self, range);
	if (ret < 0) {
		return ret;
	};

	DEBUG_PRINT("Setting up gyroscope... scale(%d)\n", scale);
	lsm9ds1_setup_gyro(self, scale);
	if (ret < 0) {
		return ret;
	};

	// Setup the magnetometer, this only needs to be done once.
	DEBUG_PRINT("Setting up mag cs...\n");
	ret = lsm9ds1_setup_mag_cs();
	if (ret < 0) {
		DEBUG_PRINT("Error setting up mag cs!\n");
		return ret;
	}

	ret = lsm9ds1_setup_mag(self, gain);
	if (ret < 0) {
		DEBUG_PRINT("Error setting up mag! (%d)\n", ret);
		return ret;
	};

	return LSM9DS1_SUCCESS;
}

#endif
