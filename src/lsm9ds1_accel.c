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

#include "lsm9ds1_accel.h"
#include "lsm9ds1_regs.h"
#include "lsm9ds1_debug.h"

static lsm9ds1_status_t lsm9ds1_read_accel_settings(lsm9ds1_bus_t *bus, lsm9ds1_accel_settings_t *settings) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG5_XL);
	if (status < 0) {
		return status;
	}

	uint8_t bit_value = 0;
	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_DECIMATION_BIT_MASK, 
		LSM9DS1_XL_DECIMATION_BIT_OFFSET, &bit_value);
	settings->decimation = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_Z_ENABLE_BIT_MASK, 
		LSM9DS1_XL_Z_ENABLE_BIT_OFFSET, &bit_value);
	settings->z_enable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_Y_ENABLE_BIT_MASK, 
		LSM9DS1_XL_Y_ENABLE_BIT_OFFSET, &bit_value);
	settings->y_enable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_X_ENABLE_BIT_MASK, 
		LSM9DS1_XL_X_ENABLE_BIT_OFFSET, &bit_value);
	settings->x_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG5_XL: 0x%X\n",
	            bus->spi.rx[0]);

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG6_XL);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_ODR_BIT_MASK, 
		LSM9DS1_XL_ODR_BIT_OFFSET, &bit_value);
	settings->odr = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_RANGE_BIT_MASK, 
		LSM9DS1_XL_RANGE_BIT_OFFSET, &bit_value);
	settings->range = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_BANDWIDTH_SELECT_BIT_MASK, 
		LSM9DS1_XL_BANDWIDTH_SELECT_BIT_OFFSET, &bit_value);
	settings->bw_select = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_FILTER_BIT_MASK, 
		LSM9DS1_XL_FILTER_BIT_OFFSET, &bit_value);
	settings->bw_filter = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG6_XL: 0x%X\n",
	            bus->spi.rx[0]);

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG7_XL);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_HIGH_RESOLUTION_BIT_MASK, 
		LSM9DS1_XL_HIGH_RESOLUTION_BIT_OFFSET, &bit_value);
	settings->high_resolution_enable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_FDL_BIT_MASK, 
		LSM9DS1_XL_FDL_BIT_OFFSET, &bit_value);
	settings->filtered_data_selection = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_XL_HIGH_PASS_FILTER_BIT_MASK, 
		LSM9DS1_XL_HIGH_PASS_FILTER_BIT_OFFSET, &bit_value);
	settings->high_pass_filter_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG7_XL: 0x%X\n",
	            bus->spi.rx[0]);

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_setup_accel(lsm9ds1_bus_t *bus, 
	lsm9ds1_accel_settings_t *settings, lsm9ds1_accel_range_t range) {

	/**************************************************************************
	 * Checks and setup
	 *************************************************************************/
	if (!bus->initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	status = lsm9ds1_select_sub_device(bus, LSM9DS1_ACCEL_GYRO);
	if (status < 0) {
		return status;
	}

	// Reset the current sub-device selected
	status = lsm9ds1_soft_reset(bus);
	if ((status != LSM9DS1_ACCEL_GYRO_ALREADY_RESET) && (status < 0)) {
		return status;
	}

	status = lsm9ds1_read_accel_settings(bus, settings);
	if (status < 0) {
		return status;
	}
	/**************************************************************************
	 * Setup output data rate to 952 HZ (maximum), and the provided range.
	 *************************************************************************/
	settings->odr = LSM9DS1_XL_952HZ;
	settings->range = range;
	uint8_t reg = (settings->range |
	               settings->odr);
	status = lsm9ds1_register_write(bus, LSM9DS1_REGISTER_CTRL_REG6_XL,
	                                __extension__ 0b11111000, reg);
	if (status < 0) return status;

	// Decide on the conversion value based on the provided range.
	switch (settings->range) {
	case LSM9DS1_ACCELRANGE_2G:
		settings->accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
		break;
	case LSM9DS1_ACCELRANGE_4G:
		settings->accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
		break;
	case LSM9DS1_ACCELRANGE_8G:
		settings->accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
		break;
	case LSM9DS1_ACCELRANGE_16G:
		settings->accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_16G;
		break;
	default:
		return LSM9DS1_UKNOWN_ACCEL_RANGE;
	}

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_read_accel(lsm9ds1_bus_t *bus, accelerometer_raw_data_t *raw_data) {

	// Bus must be initialized
	if (!bus->initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(bus, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_X_L_XL);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_X_H_XL);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Y_L_XL);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Y_H_XL);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Z_L_XL);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Z_H_XL);
	if (read_status < 0) {
		return read_status;
	}
	int16_t zhi = bus->spi.rx[0];

	// Shift values to create properly formed integer (low byte first)
	xhi <<= 8;
	xhi |= xlo;
	yhi <<= 8;
	yhi |= ylo;
	zhi <<= 8;
	zhi |= zlo;

	raw_data->x = xhi;
	raw_data->y = yhi;
	raw_data->z = zhi;

	return LSM9DS1_SUCCESS;
}
