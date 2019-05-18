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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "lsm9ds1.h"

static bool bus_initialized = false;
static uint8_t num_calls = 0;
static lsm9ds1_settings_t device_settings = { 0 };

//TODO move these to the correct function
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 15000000;
static uint16_t delay = 0;
static int32_t fd = -1;	// File descriptor for LSM9DS0

static lsm9ds1_status_t transfer(lsm9ds1_xfer_t op, uint8_t address, uint8_t tx,
		uint8_t *rx) {

	int8_t ret = LSM9DS1_UNKNOWN_ERROR;

	uint8_t addr_xfer = 0;

	//TODO ensure sizes are correct
	struct spi_ioc_transfer tr[2] = { 0 };

	switch (op) {
	case LSM9DS1_READ:
		tr[1].rx_buf = (unsigned long) rx;
		tr[1].len = sizeof(*rx);
		tr[1].speed_hz = speed;
		tr[1].delay_usecs = delay;
		tr[1].bits_per_word = bits;

		addr_xfer = (SPI_READ | address);
		break;
	case LSM9DS1_WRITE:
		tr[1].tx_buf = (unsigned long) &tx;
		tr[1].len = sizeof(tx);
		tr[1].speed_hz = speed;
		tr[1].delay_usecs = delay;
		tr[1].bits_per_word = bits;

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
	tr[0].speed_hz = speed;
	tr[0].delay_usecs = delay;
	tr[0].bits_per_word = bits;

	ret = ioctl(fd, SPI_IOC_MESSAGE(2), tr);
#ifdef DEBUG
	printf("SPIDEV: transfer(%d)\n", ret);
	if(ret < 0) {
		perror("SPIDEV transfer:");
	}
#endif
	if(ret < 1) {
		return LSM9DS1_SPI_BUS_XFER_ERROR;
	}

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t init_spi(void) {

	int8_t ret = -1;	// Function return codes.

	fd = open(DEVICE, O_RDWR);
	if (fd < 0) {
		return LSM9DS1_NOT_FOUND;
	}

	// Set to mode 3
	mode |= SPI_CPOL | SPI_CPHA;
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) {
		return LSM9DS1_MODE_3_NOT_SET;
	}

	// Set the number of bits per word.
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) {
		return LSM9DS1_NUM_BITS_NOT_SET;
	}

	// Set the max clock speed in hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		return LSM9DS1_CLOCK_NOT_SET;
	}

	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t init_i2c(void) {
	// Probably won't implement for raspberrypi
	return LSM9DS1_BUS_NOT_SUPPORTED;
}

static lsm9ds1_status_t lsm9ds1_read(uint8_t register_addr, uint8_t *rx) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	// tx doesn't matter, so just use 0.
	ret = transfer(LSM9DS1_READ, register_addr, 0, rx);

	return ret;
}

//TODO fix write function.
static lsm9ds1_status_t lsm9ds1_write(uint8_t register_addr, uint8_t tx) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	//RX doesn't matter so give it NULL
	ret = transfer(LSM9DS1_WRITE, register_addr, tx, NULL);

	return ret;
}

lsm9ds1_status_t lsm9ds1_read_sub_device(lsm9ds1_devices_t *device_id) {

	lsm9ds1_status_t function_return = LSM9DS1_UNKNOWN_ERROR;
	uint8_t read_buffer = LSM9DS1_UNKNOWN_DEVICE;
	*device_id = LSM9DS1_UNKNOWN_DEVICE;

	// The mag accel and gyro id should be at the same offset, if not, we don't know what device we have.
	// We are comparing enums of different types, cast first since we want to do this.
	if (!((int)LSM9DS1_REGISTER_WHO_AM_I_XG == (int)LSM9DS1_REGISTER_WHO_AM_I_M)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	// Discover device
	function_return = lsm9ds1_read(LSM9DS1_REGISTER_WHO_AM_I, &read_buffer);
	lsm9ds1_devices_t found_device = read_buffer;
#ifdef DEBUG
	printf("Sub-device: (%d)\n", found_device);
#endif

	// Did we find a the gyro and accel combo or mag?
	if ((LSM9DS1_ACCEL_GYRO == found_device) || (LSM9DS1_MAG == found_device)) {
		*device_id = found_device;
	}

	return function_return;
}

static lsm9ds1_status_t lsm9ds1_is_init(bool *initialized) {

	bool has_been_init = false;
	if (num_calls > 0) {
		has_been_init = true;
	}

	*initialized = has_been_init;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_init_bus(lsm9ds1_bus_t bus_type) {

	if (bus_initialized) {
		return LSM9DS1_SUCCESS;
	}

	//TODO decide if we need to remove
	// If we have already opened the fd then return early
	if (fd >= 0) {
		return LSM9DS1_SUCCESS;
	}

	// Determine bus type being used.
	int8_t ret = -1;	// Function return codes.
	switch (bus_type) {
	case LSM9DS1_SPI_BUS:
		ret = init_spi();
		break;
	case LSM9DS1_I2C_BUS:
		ret = init_i2c();
		break;
	default:
		ret = LSM9DS1_BUS_NOT_SUPPORTED;
		break;

	}

	bus_initialized = true;
	return ret;
}

lsm9ds1_status_t lsm9ds1_init(lsm9ds1_bus_t bus_type,
		lsm9ds1_accel_range_t range, lsm9ds1_mag_gain_t gain,
		lsm9ds1_gyro_scale_t scale) {

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	//reset number of calls if necessary
	if (num_calls == UINT8_MAX) {
		num_calls = 0;
	}
	num_calls++;

#ifdef DEBUG
	printf("Initializing the lsm9ds1 bus...\n");
#endif
	ret = lsm9ds1_init_bus(bus_type);
	if (ret < 0) {
		return ret;
	};

#ifdef DEBUG
	printf("Setting up accelerometer... range(%d)\n", range);
#endif
	ret = lsm9ds1_setup_accel(range);
	if (ret < 0) {
		return ret;
	};

#ifdef DEBUG
	printf("Setting up gyroscope... scale(%d)\n", scale);
#endif
	lsm9ds1_setup_gyro(scale);
	if (ret < 0) {
		return ret;
	};

	//TODO currently don't have CS for mag.
	//	ret = lsm9ds1_setup_mag(gain);
	//	if (ret < 0) {
	//		return ret;
	//	};

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_read_accel(accelerometer_data_t *accel_data) {

	// Check that the lsm9ds1 has been initialized.
	bool lsm9ds1_initialized = false;
	(void) lsm9ds1_is_init(&lsm9ds1_initialized);
	if (!lsm9ds1_initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	lsm9ds1_devices_t sub_device = LSM9DS1_UNKNOWN_DEVICE;
	lsm9ds1_read_sub_device(&sub_device);
	if (!(LSM9DS1_ACCEL_GYRO == sub_device)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	// Read the accelerometer.
	uint8_t read_buffer = 0;
	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_X_L_XL, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_X_H_XL, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_Y_L_XL, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_Y_H_XL, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_Z_L_XL, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_Z_H_XL, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	int16_t zhi = read_buffer;

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8;
	xhi |= xlo;
	yhi <<= 8;
	yhi |= ylo;
	zhi <<= 8;
	zhi |= zlo;

	accel_data->x = xhi;
	accel_data->y = yhi;
	accel_data->z = zhi;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_read_mag(mag_data_t *mag_data) {

	// Check that the lsm9ds1 has been initialized.
	bool lsm9ds1_initialized = false;
	(void) lsm9ds1_is_init(&lsm9ds1_initialized);
	if (!lsm9ds1_initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	lsm9ds1_devices_t sub_device = LSM9DS1_UNKNOWN_DEVICE;
	lsm9ds1_read_sub_device(&sub_device);
	if (!(LSM9DS1_MAG == sub_device)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	// Read the accelerometer.
	uint8_t read_buffer = 0;
	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_X_L_G, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_X_H_G, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_Y_L_G, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_Y_H_G, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_Z_L_G, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_OUT_Z_H_G, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	int16_t zhi = read_buffer;

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8;
	xhi |= xlo;
	yhi <<= 8;
	yhi |= ylo;
	zhi <<= 8;
	zhi |= zlo;

	mag_data->x = xhi;
	mag_data->y = yhi;
	mag_data->z = zhi;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_read_temp(lsm9ds1_temperature_t *temp) {

	// Check that the lsm9ds1 has been initialized.
	bool lsm9ds1_initialized = false;
	(void) lsm9ds1_is_init(&lsm9ds1_initialized);
	if (!lsm9ds1_initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	lsm9ds1_devices_t sub_device = LSM9DS1_UNKNOWN_DEVICE;
	lsm9ds1_read_sub_device(&sub_device);
	if (!(LSM9DS1_ACCEL_GYRO == sub_device)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	// Read the accelerometer.
	uint8_t read_buffer = 0;
	read_status = lsm9ds1_read(LSM9DS1_REGISTER_TEMP_OUT_L, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = read_buffer;

	read_status = lsm9ds1_read(LSM9DS1_REGISTER_TEMP_OUT_H, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	lsm9ds1_temperature_t xhi = read_buffer;

	xhi <<= 8;
	xhi |= xlo;

	// Shift values to create properly formed integer (low byte first)
	*temp = xhi;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_setup_accel(lsm9ds1_accel_range_t range) {

	if (!bus_initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	lsm9ds1_devices_t sub_device = LSM9DS1_UNKNOWN_DEVICE;
	lsm9ds1_read_sub_device(&sub_device);
	if (!(LSM9DS1_ACCEL_GYRO == sub_device)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	// Read the accelerometer.
	uint8_t read_buffer = 0;
	read_status = lsm9ds1_read(LSM9DS1_REGISTER_CTRL_REG6_XL, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t reg = read_buffer;
	reg &= ~(0b00011000);
	reg |= range;

#ifdef DEBUG
	printf("Setting LSM9DS1_REGISTER_CTRL_REG6_XL: %d\n", reg);
#endif
	lsm9ds1_status_t write_status = LSM9DS1_UNKNOWN_ERROR;
	write_status = lsm9ds1_write(LSM9DS1_REGISTER_CTRL_REG6_XL, reg);
	if (write_status < 0) {
		return write_status;
	}

	switch (range) {
	case LSM9DS1_ACCELRANGE_2G:
		device_settings.range = range;
		break;
	case LSM9DS1_ACCELRANGE_4G:
		device_settings.range = range;
		break;
	case LSM9DS1_ACCELRANGE_8G:
		device_settings.range = range;
		break;
	case LSM9DS1_ACCELRANGE_16G:
		device_settings.range = range;
		break;
	default:
		return LSM9DS1_UKNOWN_ACCEL_RANGE;
		break;
	}

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_setup_mag(lsm9ds1_mag_gain_t gain) {

	if (!bus_initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	lsm9ds1_devices_t sub_device = LSM9DS1_UNKNOWN_DEVICE;
	lsm9ds1_read_sub_device(&sub_device);
	if (!(LSM9DS1_MAG == sub_device)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	// Read the accelerometer.
	uint8_t read_buffer = 0;
	read_status = lsm9ds1_read(LSM9DS1_REGISTER_CTRL_REG2_M, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}

	uint8_t reg = read_buffer;
	reg &= ~(0b01100000);
	reg |= gain;

	lsm9ds1_status_t write_status = LSM9DS1_UNKNOWN_ERROR;
	write_status = lsm9ds1_write(LSM9DS1_REGISTER_CTRL_REG2_M, reg);
	if (write_status < 0) {
		return write_status;
	}

	switch (gain) {
	case LSM9DS1_MAGGAIN_4GAUSS:
		device_settings.gain = gain;
		break;
	case LSM9DS1_MAGGAIN_8GAUSS:
		device_settings.gain = gain;
		break;
	case LSM9DS1_MAGGAIN_12GAUSS:
		device_settings.gain = gain;
		break;
	case LSM9DS1_MAGGAIN_16GAUSS:
		device_settings.gain = gain;
		break;
	default:
		return LSM9DS1_UKNOWN_GAIN_RANGE;
	}

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_setup_gyro(lsm9ds1_gyro_scale_t scale) {

	if (!bus_initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	lsm9ds1_devices_t sub_device = LSM9DS1_UNKNOWN_DEVICE;
	lsm9ds1_read_sub_device(&sub_device);
	if (!(LSM9DS1_ACCEL_GYRO == sub_device)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	// Read the accelerometer.
	uint8_t read_buffer = 0;
	read_status = lsm9ds1_read(LSM9DS1_REGISTER_CTRL_REG1_G, &read_buffer);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t reg = read_buffer;

	reg &= ~(0b00011000);
	reg |= scale;

	lsm9ds1_status_t write_status = LSM9DS1_UNKNOWN_ERROR;
	write_status = lsm9ds1_write(LSM9DS1_REGISTER_CTRL_REG1_G, reg);
	if (write_status < 0) {
		return write_status;
	}

	switch (scale) {
	case LSM9DS1_GYROSCALE_245DPS:
		device_settings.scale = LSM9DS1_GYROSCALE_245DPS;
		break;
	case LSM9DS1_GYROSCALE_500DPS:
		device_settings.scale = LSM9DS1_GYROSCALE_245DPS;
		break;
	case LSM9DS1_GYROSCALE_2000DPS:
		device_settings.scale = LSM9DS1_GYROSCALE_245DPS;
		break;
	default:
		return LSM9DS1_UKNOWN_GAIN_RANGE;
	}

	return LSM9DS1_SUCCESS;
}
