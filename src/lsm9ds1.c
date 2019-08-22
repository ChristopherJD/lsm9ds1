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

#include <stdlib.h>

#include "lsm9ds1.h"

#ifndef __GNUC__
#error "__GNUC__ not defined"
#else

static lsm9ds1_device_t glsm9ds1 = {0};

static lsm9ds1_status_t init_i2c(lsm9ds1_bus_t *self) {
	// Probably won't implement for raspberrypi
	return LSM9DS1_BUS_NOT_SUPPORTED;
}

static lsm9ds1_status_t close_i2c(lsm9ds1_bus_t *self) {
	// Probably won't implement for raspberrypi
	return LSM9DS1_BUS_NOT_SUPPORTED;
}

static lsm9ds1_status_t lsm9ds1_close_bus(lsm9ds1_sub_device_t *self, lsm9ds1_xfer_bus_t bus_type) {

	if (!self->bus.initialized) {
		return LSM9DS1_SUCCESS;
	}

	// Determine bus type being used.
	int8_t ret = -1;	// Function return codes.
	switch (bus_type) {
	case LSM9DS1_SPI_BUS:
		ret = close_spi(&(self->bus));
		if (ret < 0) return ret;
		break;
	case LSM9DS1_I2C_BUS:
		ret = close_i2c(&(self->bus));
		if (ret < 0) return ret;
		break;
	default:
		ret = LSM9DS1_BUS_NOT_SUPPORTED;
		break;
	}

	self->bus.initialized = false;
	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t lsm9ds1_init_bus(lsm9ds1_sub_device_t *self, lsm9ds1_xfer_bus_t bus_type) {

	if (self->bus.initialized) {
		return LSM9DS1_SUCCESS;
	}

	// Determine bus type being used.
	int8_t ret = -1;	// Function return codes.
	switch (bus_type) {
	case LSM9DS1_SPI_BUS:
		ret = init_spi(&(self->bus));
		if (ret < 0) return ret;
		break;
	case LSM9DS1_I2C_BUS:
		ret = init_i2c(&(self->bus));
		if (ret < 0) return ret;
		break;
	default:
		ret = LSM9DS1_BUS_NOT_SUPPORTED;
		break;
	}

	self->bus.initialized = true;
	return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t update_temp(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_read_temp(&(self->sub_device.accelerometer.bus), &(self->raw_data.temperature));

	self->converted_data.temperature = (21.0 + (((float)self->raw_data.temperature) / 8));

	return status;
}

static lsm9ds1_status_t update_accel(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Check that the bus has been initialized
	if (!self->sub_device.accelerometer.bus.initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	// Get the raw data
	status = lsm9ds1_read_accel(&(self->sub_device.accelerometer.bus), &(self->raw_data.accelerometer));

	DEBUG_PRINT("accel_mg_lsb: %f\n", self->settings.accelerometer.accel_resolution);
	DEBUG_PRINT("Accel Raw (x): 0x%X\n", self->raw_data.accelerometer.x);
	DEBUG_PRINT("Accel Raw (y): 0x%X\n", self->raw_data.accelerometer.y);
	DEBUG_PRINT("Accel Raw (z): 0x%X\n", self->raw_data.accelerometer.z);

	// Convert the raw data
	self->converted_data.accelerometer.x = self->raw_data.accelerometer.x * self->settings.accelerometer.accel_resolution;
	self->converted_data.accelerometer.y = self->raw_data.accelerometer.y * self->settings.accelerometer.accel_resolution;
	self->converted_data.accelerometer.z = self->raw_data.accelerometer.z * self->settings.accelerometer.accel_resolution;

	return status;
}

static lsm9ds1_status_t update_mag(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Check that the bus has been initialized
	if (!self->sub_device.magnetometer.bus.initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	// Get the raw data
	status = lsm9ds1_read_mag(&(self->sub_device.magnetometer.bus), &(self->raw_data.magnetometer));

	DEBUG_PRINT("mag_mgauss: %f\n", self->settings.magnetometer.mag_resolution);
	DEBUG_PRINT("Mag Raw (x): 0x%X\n", self->raw_data.magnetometer.x);
	DEBUG_PRINT("Mag Raw (y): 0x%X\n", self->raw_data.magnetometer.y);
	DEBUG_PRINT("Mag Raw (z): 0x%X\n", self->raw_data.magnetometer.z);

	// Convert the raw data
	self->converted_data.magnetometer.x = self->raw_data.magnetometer.x * self->settings.magnetometer.mag_resolution;
	self->converted_data.magnetometer.y = self->raw_data.magnetometer.y * self->settings.magnetometer.mag_resolution;
	self->converted_data.magnetometer.z = self->raw_data.magnetometer.z * self->settings.magnetometer.mag_resolution;

	return status;
}

static lsm9ds1_status_t update_gyro(lsm9ds1_device_t *self) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Check that the bus has been initialized
	if (!self->sub_device.accelerometer.bus.initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	// Get the raw data
	status = lsm9ds1_read_gyro(&(self->sub_device.accelerometer.bus), &(self->raw_data.gyroscope));

	DEBUG_PRINT("gyro_dps_digit: %f\n", self->settings.gyroscope.gyro_resolution);
	DEBUG_PRINT("Gyro Raw (x): 0x%X\n", self->raw_data.gyroscope.x);
	DEBUG_PRINT("Gyro Raw (y): 0x%X\n", self->raw_data.gyroscope.y);
	DEBUG_PRINT("Gyro Raw (z): 0x%X\n", self->raw_data.gyroscope.z);

	// Convert the raw data
	self->converted_data.gyroscope.x = self->raw_data.gyroscope.x * self->settings.gyroscope.gyro_resolution;
	self->converted_data.gyroscope.y = self->raw_data.gyroscope.y * self->settings.gyroscope.gyro_resolution;
	self->converted_data.gyroscope.z = self->raw_data.gyroscope.z * self->settings.gyroscope.gyro_resolution;

	return status;
}

static lsm9ds1_status_t update(lsm9ds1_device_t *self) {

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

lsm9ds1_status_t get_temp(lsm9ds1_temperature_t *temperature) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	if(NULL == temperature) return LSM9DS1_NULL_PARAMETER;

	status = update_temp(&glsm9ds1);
	if(status < 0) return status;

	*temperature = glsm9ds1.converted_data.temperature;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t get_accel(accelerometer_converted_data_t *data) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = update_accel(&glsm9ds1);
	if(status < 0) return status;

	*data = glsm9ds1.converted_data.accelerometer;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t get_mag(mag_converted_data_t *data) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = update_mag(&glsm9ds1);
	if(status < 0) return status;

	*data = glsm9ds1.converted_data.magnetometer;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t get_gyro(gyro_converted_data_t *data) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = update_gyro(&glsm9ds1);
	if(status < 0) return status;

	*data = glsm9ds1.converted_data.gyroscope;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_init() {

	glsm9ds1.initialized = false;

	DEBUG_PRINT("Build Version: %s\n", _BUILD_VERSION);
	DEBUG_PRINT("Build Date/Time: %s %s\n", __DATE__, __TIME__);

	lsm9ds1_status_t ret = LSM9DS1_UNKNOWN_ERROR;

	// link functions
	glsm9ds1.update_temp = update_temp;
	glsm9ds1.update_accel = update_accel;
	glsm9ds1.update_mag = update_mag;
	glsm9ds1.update_gyro = update_gyro;
	glsm9ds1.update = update;	

	ret = init_config();
	if(ret < 0) {
		return ret;
	}

	glsm9ds1.xfer_bus = glsm9ds1_config.xfer_bus;

#if DEBUG > 0
	const char *bus_names[NUM_BUS_TYPES] = {"SPI", "I2C"};
	DEBUG_PRINT("Initializing the lsm9ds1 %s bus...\n", bus_names[glsm9ds1.xfer_bus]);
#endif

	glsm9ds1.sub_device.accelerometer.bus.initialized = false;
	glsm9ds1.sub_device.magnetometer.bus.initialized = false;
	glsm9ds1.sub_device.accelerometer.bus.id = LSM9DS1_ACCEL_GYRO;
	glsm9ds1.sub_device.magnetometer.bus.id = LSM9DS1_MAG;

	ret = lsm9ds1_init_bus(&(glsm9ds1.sub_device.accelerometer), glsm9ds1.xfer_bus);
	if ((ret < 0) && (ret != LSM9DS1_BUS_ALREADY_OPEN)) {
		return ret;
	};
	ret = lsm9ds1_init_bus(&(glsm9ds1.sub_device.magnetometer), glsm9ds1.xfer_bus);
	if ((ret < 0) && (ret != LSM9DS1_BUS_ALREADY_OPEN)) {
		return ret;
	};

	DEBUG_PRINT("Setting up accelerometer... range(%d)\n", glsm9ds1_config.settings.accelerometer.range);
	ret = lsm9ds1_setup_accel(
		&(glsm9ds1.sub_device.accelerometer.bus), 
		&(glsm9ds1.settings.accelerometer), 
		glsm9ds1_config.settings.accelerometer.range
	);
	if (ret < 0) {
		DEBUG_PRINT("Error setting up accelerometer (%d)!\n", ret);
		return ret;
	};

	DEBUG_PRINT("Setting up gyroscope... scale(%d)\n", glsm9ds1_config.settings.gyroscope.scale);
	lsm9ds1_setup_gyro(&(glsm9ds1.sub_device.accelerometer.bus), 
		&(glsm9ds1.settings.gyroscope), 
		glsm9ds1_config.settings.gyroscope.scale
	);
	if (ret < 0) {
		DEBUG_PRINT("Error setting up gyroscope (%d)!\n", ret);
		return ret;
	};

	DEBUG_PRINT("Setting up Magnetometer... gain(%d)\n", glsm9ds1_config.settings.magnetometer.gain);
	ret = lsm9ds1_setup_mag(&(glsm9ds1.sub_device.magnetometer.bus), 
		&(glsm9ds1.settings.magnetometer), 
		glsm9ds1_config.settings.magnetometer.gain
	);
	if (ret < 0) {
		DEBUG_PRINT("Error setting up mag! (%d)\n", ret);
		return ret;
	};

	glsm9ds1.initialized = true;

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_close() {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_close_bus(&(glsm9ds1.sub_device.accelerometer), glsm9ds1.xfer_bus);
	status = lsm9ds1_close_bus(&(glsm9ds1.sub_device.magnetometer), glsm9ds1.xfer_bus);

	return status;
}

#endif
