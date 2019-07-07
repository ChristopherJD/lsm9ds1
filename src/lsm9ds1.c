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
#include <time.h>
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

#define MAG_CS 21	//Wiring Pi Pin number for the magnetometer CS.

static lsm9ds1_status_t lsm9ds1_mag_cs(int pin_state) {
	digitalWrite(MAG_CS, pin_state);
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

static lsm9ds1_status_t lsm9ds1_write(lsm9ds1_bus_t *self, uint8_t register_addr, uint8_t tx) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	//RX doesn't matter so give it NULL
	self->spi.op = LSM9DS1_WRITE;
	self->spi.tx = tx;
	ret = self->transfer(self, self->spi.op, self->spi.address, self->spi.tx, NULL);

	return ret;
}

static lsm9ds1_status_t lsm9ds1_read(lsm9ds1_bus_t *self, uint8_t address) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	// tx doesn't matter, so just use 0.
	self->spi.op = LSM9DS1_READ;
	self->spi.address = address;
	ret = self->transfer(self, self->spi.op, self->spi.address, 0, self->spi.rx);

	return ret;
}

static lsm9ds1_status_t read_bit_value(uint8_t value, uint8_t mask, uint8_t offset, uint8_t *read_value) {

	*read_value = ((value & mask) >> offset);

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_register_write(lsm9ds1_bus_t *self, uint8_t address, uint8_t mask, uint8_t value) {

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

lsm9ds1_status_t lsm9ds1_select_sub_device(lsm9ds1_device_t *self, lsm9ds1_sub_device_t sub_device) {

	lsm9ds1_status_t function_return = LSM9DS1_UNKNOWN_ERROR;

	if (!self->bus.initialized) return LSM9DS1_BUS_NOT_INTIALIZED;

	// The mag accel and gyro id should be at the same offset, if not, we don't know what device we have.
	// We are comparing enums of different types, cast first since we want to do this.
	if (!((int) LSM9DS1_REGISTER_WHO_AM_I_XG
	        == (int) LSM9DS1_REGISTER_WHO_AM_I_M)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	self->bus.current_sub_device = sub_device;

	// Discover device
	function_return = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_WHO_AM_I);
	lsm9ds1_sub_device_t found_device = self->bus.spi.rx[0];
	DEBUG_PRINT("Sub-device: (0x%X)\n", found_device);

	if (found_device != sub_device) return LSM9DS1_UNKNOWN_SUB_DEVICE;

	return function_return;
}

static lsm9ds1_status_t lsm9ds1_soft_reset(lsm9ds1_device_t *self) {

	// We will only reset once.
	static bool accel_gyro_reset = false;
	static bool mag_reset = false;

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	if (accel_gyro_reset && (self->bus.current_sub_device == LSM9DS1_ACCEL_GYRO)) {
		return LSM9DS1_ACCEL_GYRO_ALREADY_RESET;
	}

	if (mag_reset && (self->bus.current_sub_device == LSM9DS1_MAG)) {
		return LSM9DS1_MAG_ALREADY_RESET;
	}

	switch (self->bus.current_sub_device) {
	case LSM9DS1_ACCEL_GYRO:
		// Soft reset on the accelerometer and gyroscope
		status = lsm9ds1_write(&(self->bus), LSM9DS1_REGISTER_CTRL_REG8, 0x05);
		if (status < 0) return status;
		accel_gyro_reset = true;
		break;
	case LSM9DS1_MAG:
		// Soft reset on the magnetometer
		status = lsm9ds1_write(&(self->bus), LSM9DS1_REGISTER_CTRL_REG8, 0x0C);
		if (status < 0) return status;
		mag_reset = true;
		break;
	default:
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	// Sleep for 10 microseconds to allow the lsm9ds1 to reset
	struct timespec sleep_time = {.tv_sec = 0, .tv_nsec = 10000};
	(void)nanosleep(&sleep_time, NULL);

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_setup_mag_cs() {

	//Setup for Mag CS
	(void)wiringPiSetup();

	(void)pinMode(MAG_CS, OUTPUT);

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_read_mag_settings(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG1_M);
	if (status < 0) {
		return status;
	}
	uint8_t bit_value = 0;
	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_TEMP_COMP_BIT_MASK, 
		LSM9DS1_MAG_TEMP_COMP_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.temp_comp_enable = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_XY_OP_MODE_BIT_MASK, 
		LSM9DS1_MAG_XY_OP_MODE_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.xy_op_mode = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_ODR_BIT_MASK, 
		LSM9DS1_MAG_ODR_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.odr = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_FAST_ODR_BIT_MASK, 
		LSM9DS1_MAG_FAST_ODR_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.fast_odr_enable = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_SELF_TEST_BIT_MASK, 
		LSM9DS1_MAG_SELF_TEST_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.self_test_enable = bit_value;
	
	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG1_M: 0x%X\n",
	            self->bus.spi.rx[0]);

	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG2_M);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_GAIN_BIT_MASK, 
		LSM9DS1_MAG_GAIN_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.gain = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_REBOOT_BIT_MASK, 
		LSM9DS1_MAG_REBOOT_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.reboot = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_RESET_BIT_MASK, 
		LSM9DS1_MAG_RESET_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.reset = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG2_M: 0x%X\n",
	            self->bus.spi.rx[0]);

	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG3_M);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_I2C_DISABLE_BIT_MASK, 
		LSM9DS1_MAG_I2C_DISABLE_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.i2c_disable = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_LOW_POWER_MODE_BIT_MASK, 
		LSM9DS1_MAG_LOW_POWER_MODE_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.low_power_mode = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_SPI_MODE_BIT_MASK, 
		LSM9DS1_MAG_SPI_MODE_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.spi_mode = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_OP_MODE_BIT_MASK, 
		LSM9DS1_MAG_OP_MODE_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.op_mode = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG3_M: 0x%X\n",
	            self->bus.spi.rx[0]);

	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG4_M);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_Z_OP_MODE_BIT_MASK, 
		LSM9DS1_MAG_Z_OP_MODE_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.z_op_mode = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_LITTLE_ENDIAN_BIT_MASK, 
		LSM9DS1_MAG_LITTLE_ENDIAN_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.little_endian_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG4_M: 0x%X\n",
	            self->bus.spi.rx[0]);

	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG5_M);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_FAST_READ_BIT_MASK, 
		LSM9DS1_MAG_FAST_READ_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.fast_read_enable = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_MAG_BLOCK_DATA_UPDATE_BIT_MASK, 
		LSM9DS1_MAG_BLOCK_DATA_UPDATE_BIT_OFFSET, &bit_value);
	self->settings.magnetometer.block_data_update_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG5_M: 0x%X\n",
	            self->bus.spi.rx[0]);

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_read_accel_settings(lsm9ds1_device_t *self) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG5_XL);
	if (status < 0) {
		return status;
	}

	uint8_t bit_value = 0;
	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_DECIMATION_BIT_MASK, 
		LSM9DS1_XL_DECIMATION_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.decimation = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_Z_ENABLE_BIT_MASK, 
		LSM9DS1_XL_Z_ENABLE_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.z_enable = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_Y_ENABLE_BIT_MASK, 
		LSM9DS1_XL_Y_ENABLE_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.y_enable = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_X_ENABLE_BIT_MASK, 
		LSM9DS1_XL_X_ENABLE_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.x_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG5_XL: 0x%X\n",
	            self->bus.spi.rx[0]);

	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG6_XL);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_ODR_BIT_MASK, 
		LSM9DS1_XL_ODR_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.odr = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_RANGE_BIT_MASK, 
		LSM9DS1_XL_RANGE_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.range = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_BANDWIDTH_SELECT_BIT_MASK, 
		LSM9DS1_XL_BANDWIDTH_SELECT_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.bw_select = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_FILTER_BIT_MASK, 
		LSM9DS1_XL_FILTER_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.bw_filter = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG6_XL: 0x%X\n",
	            self->bus.spi.rx[0]);

	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG7_XL);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_HIGH_RESOLUTION_BIT_MASK, 
		LSM9DS1_XL_HIGH_RESOLUTION_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.high_resolution_enable = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_FDL_BIT_MASK, 
		LSM9DS1_XL_FDL_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.filtered_data_selection = bit_value;

	(void)read_bit_value(self->bus.spi.rx[0], LSM9DS1_XL_HIGH_PASS_FILTER_BIT_MASK, 
		LSM9DS1_XL_HIGH_PASS_FILTER_BIT_OFFSET, &bit_value);
	self->settings.accelerometer.high_pass_filter_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG7_XL: 0x%X\n",
	            self->bus.spi.rx[0]);

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_setup_mag(lsm9ds1_device_t *self,
        lsm9ds1_mag_gain_t gain) {

	/**************************************************************************
	 * Checks and setup
	 *************************************************************************/
	// Bus must be initialized
	if (!self->bus.initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	// CS is logic low, so set value HIGH to have this device ignored until
	// otherwise decided.
	(void) lsm9ds1_mag_cs(HIGH);

	// Ensure we have the correct device
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
	status = lsm9ds1_select_sub_device(self, LSM9DS1_MAG);
	if (status < 0) {
		return status;
	}

	status = lsm9ds1_read_mag_settings(self);
	if (status < 0) {
		return status;
	}

	/**************************************************************************
	 * Setup continous mode, output data rate to 80 HZ (maximum), and the
	 * provided gain.
	 *************************************************************************/
	// Perform a reset on the current sub-device selected
	status = lsm9ds1_soft_reset(self);
	if (status < 0) return status;

	// Set the operational mode to continuous
	self->settings.magnetometer.op_mode = LSM9DS1_MAG_OP_MODE_CONTINUOUS;
	status = lsm9ds1_register_write(&(self->bus), LSM9DS1_REGISTER_CTRL_REG3_M,
	                                __extension__ 0b00000011,
	                                self->settings.magnetometer.op_mode);
	if (status < 0) return status;

	// Setup the output data rate
	self->settings.magnetometer.odr = LSM9DS1_MAG_ODR_80HZ;
	status = lsm9ds1_register_write(&(self->bus), LSM9DS1_REGISTER_CTRL_REG1_M,
	                                __extension__ 0b00011100,
	                                self->settings.magnetometer.odr);
	if (status < 0) return status;

	// Setup the Gain
	self->settings.magnetometer.gain = gain;
	status = lsm9ds1_register_write(&(self->bus), LSM9DS1_REGISTER_CTRL_REG2_M,
	                                __extension__ 0b01100000,
	                                self->settings.magnetometer.gain);
	if (status < 0) return status;

	// Decide the conversion value used based on provided gain
	switch (self->settings.magnetometer.gain) {
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

static lsm9ds1_status_t lsm9ds1_setup_accel(lsm9ds1_device_t *self,
        lsm9ds1_accel_range_t range) {

	/**************************************************************************
	 * Checks and setup
	 *************************************************************************/
	if (!self->bus.initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (status < 0) {
		return status;
	}

	// Reset the current sub-device selected
	status = lsm9ds1_soft_reset(self);
	if ((status != LSM9DS1_ACCEL_GYRO_ALREADY_RESET) && (status < 0)) {
		return status;
	}

	status = lsm9ds1_read_accel_settings(self);
	if (status < 0) {
		return status;
	}
	/**************************************************************************
	 * Setup output data rate to 952 HZ (maximum), and the provided range.
	 *************************************************************************/
	self->settings.accelerometer.odr = LSM9DS1_XL_952HZ;
	self->settings.accelerometer.range = range;
	uint8_t reg = (self->settings.accelerometer.range |
	               self->settings.accelerometer.odr);
	status = lsm9ds1_register_write(&(self->bus), LSM9DS1_REGISTER_CTRL_REG6_XL,
	                                __extension__ 0b11111000, reg);
	if (status < 0) return status;

	// Decide on the conversion value based on the provided range.
	switch (self->settings.accelerometer.range) {
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

static lsm9ds1_status_t lsm9ds1_setup_gyro(lsm9ds1_device_t *self,
        lsm9ds1_gyro_scale_t scale) {

	/**************************************************************************
	 * Checks and setup
	 *************************************************************************/
	if (!self->bus.initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (status < 0) {
		return status;
	}

	status = lsm9ds1_soft_reset(self);
	if ((status != LSM9DS1_ACCEL_GYRO_ALREADY_RESET) && (status < 0)) {
		return status;
	}

#if DEBUG > 0
	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG4);
	DEBUG_PRINT("Reading LSM9DS1_REGISTER_CTRL_REG4: 0x%X\n",
	            self->bus.spi.rx[0]);
#endif

	/**************************************************************************
	 * Setup output data rate to 952 HZ (maximum), and the provided scale.
	 *************************************************************************/
	// enable gyro continuous
	self->settings.gyroscope.odr = LSM9DS1_GYRO_ODR_952HZ;
	self->settings.gyroscope.scale = scale;
	uint8_t reg = (self->settings.gyroscope.odr |
	               self->settings.gyroscope.scale);
	status = lsm9ds1_register_write(&(self->bus), LSM9DS1_REGISTER_CTRL_REG1_G,
	                                __extension__ 0b11111000, reg);
	if (status < 0) return status;

#if DEBUG > 0
	status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_CTRL_REG1_G);

	if (status < 0) {
		return status;
	}
	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG1_G: 0x%X\n",
	            self->bus.spi.rx[0]);
#endif

	// Decide on the conversion value based on the provided scale
	switch (self->settings.gyroscope.scale) {
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

	strncpy(self->bus.device, DEVICE, sizeof(self->bus.device));

	DEBUG_PRINT("Device fd: %s\n", self->bus.device);

	self->bus.fd = open(self->bus.device, O_RDWR);
	if (self->bus.fd <= 0) {
		return LSM9DS1_NOT_FOUND;
	}

	DEBUG_PRINT("Device fd: %d\n", self->bus.fd);

	// Set to mode 3
	self->bus.spi.settings.mode |= SPI_CPOL | SPI_CPHA | SPI_NO_CS;
	ret = ioctl(self->bus.fd, SPI_IOC_WR_MODE, &(self->bus.spi.settings.mode));
	if (ret == -1) {
		return LSM9DS1_MODE_3_NOT_SET;
	}

#if DEBUG > 0
	ret = ioctl(self->bus.fd, SPI_IOC_RD_MODE, &(self->bus.spi.settings.mode));
	if (ret == -1) {
		return LSM9DS1_MODE_3_NOT_SET;
	}
	DEBUG_PRINT("SPI_IOC_RD_MODE: 0x%X\n", self->bus.spi.settings.mode);
#endif

	// Set the number of bits per word.
	ret = ioctl(self->bus.fd, SPI_IOC_WR_BITS_PER_WORD, &(self->bus.spi.settings.bits));
	if (ret == -1) {
		return LSM9DS1_NUM_BITS_NOT_SET;
	}

	// Set the max clock speed in hz
	ret = ioctl(self->bus.fd, SPI_IOC_WR_MAX_SPEED_HZ, &(self->bus.spi.settings.speed));
	if (ret == -1) {
		return LSM9DS1_CLOCK_NOT_SET;
	}

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t init_i2c(lsm9ds1_device_t *self) {
	// Probably won't implement for raspberrypi
	return LSM9DS1_BUS_NOT_SUPPORTED;
}

static lsm9ds1_status_t lsm9ds1_init_bus(lsm9ds1_device_t *self, lsm9ds1_xfer_bus_t bus_type) {

	if (self->bus.initialized) {
		return LSM9DS1_SUCCESS;
	}

	// If we have already opened the fd then return early
	if (self->bus.fd > 0) {
		return LSM9DS1_BUS_ALREADY_OPEN;
	}

	// lsm9ds1 settings
	self->xfer_bus = bus_type;

	// Determine bus type being used.
	int8_t ret = -1;	// Function return codes.
	switch (self->xfer_bus) {
	case LSM9DS1_SPI_BUS:
		ret = init_spi(self);
		if (ret < 0) return ret;
		break;
	case LSM9DS1_I2C_BUS:
		ret = init_i2c(self);
		if (ret < 0) return ret;
		break;
	default:
		ret = LSM9DS1_BUS_NOT_SUPPORTED;
		break;

	}

	self->bus.initialized = true;
	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_read_accel(lsm9ds1_device_t *self) {

	// Check that the lsm9ds1 has been initialized.
	if (!self->initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_X_L_XL);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_X_H_XL);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Y_L_XL);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Y_H_XL);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Z_L_XL);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Z_H_XL);
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
	if (!self->initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_MAG);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_X_L_M);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_X_H_M);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Y_L_M);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Y_H_M);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Z_L_M);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Z_H_M);
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
	if (!self->initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	DEBUG_PRINT("Temp initialized\n");

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_TEMP_OUT_L);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = self->bus.spi.rx[0];

	DEBUG_PRINT("Temp xlo: 0x%X\n", xlo);

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_TEMP_OUT_H);
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
	if (!self->initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(self, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	// Read the gryo.
	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_X_L_G);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_X_H_G);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Y_L_G);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Y_H_G);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Z_L_G);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = self->bus.spi.rx[0];

	read_status = lsm9ds1_read(&(self->bus), LSM9DS1_REGISTER_OUT_Z_H_G);
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
	if (status < 0) return status;

	self->update_accel(self);
	if (status < 0) return status;

	self->update_mag(self);
	if (status < 0) return status;

	self->update_gyro(self);
	if (status < 0) return status;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_init(lsm9ds1_device_t *self, lsm9ds1_xfer_bus_t bus_type,
                              lsm9ds1_accel_range_t range, lsm9ds1_mag_gain_t gain,
                              lsm9ds1_gyro_scale_t scale) {
	if (NULL == self) {
		return LSM9DS1_MALLOC_DEVICE_ERROR;
	}

	self->initialized = false;

	DEBUG_PRINT("Build Version: %s\n", _BUILD_VERSION);
	DEBUG_PRINT("Build Date/Time: %s %s\n", __DATE__, __TIME__);

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	// link functions
	self->update_temp = update_temp;
	self->update_accel = update_accel;
	self->update_mag = update_mag;
	self->update_gyro = update_gyro;
	self->update = update;
	self->bus.transfer = transfer;
	self->bus.cs_arbiter = cs_arbiter;

#if DEBUG > 0
	const char *bus_names[NUM_BUS_TYPES] = {"SPI", "I2C"};
	DEBUG_PRINT("Initializing the lsm9ds1 %s bus...\n", bus_names[bus_type]);
#endif

	self->bus.initialized = false;
	ret = lsm9ds1_init_bus(self, bus_type);
	if ((ret < 0) && (ret != LSM9DS1_BUS_ALREADY_OPEN) {
		return ret;
	};

	DEBUG_PRINT("Setting up accelerometer... range(%d)\n", range);
	ret = lsm9ds1_setup_accel(self, range);
	if (ret < 0) {
		DEBUG_PRINT("Error setting up accelerometer (%d)!\n", ret);
		return ret;
	};

	DEBUG_PRINT("Setting up gyroscope... scale(%d)\n", scale);
	lsm9ds1_setup_gyro(self, scale);
	if (ret < 0) {
		DEBUG_PRINT("Error setting up gyroscope (%d)!\n", ret);
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

	self->initialized = true;

	return LSM9DS1_SUCCESS;
}

#endif
