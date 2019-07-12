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

#include "lsm9ds1_gyro.h"
#include "lsm9ds1_regs.h"
#include "lsm9ds1_debug.h"

lsm9ds1_status_t lsm9ds1_read_gyro(lsm9ds1_bus_t *bus, gyro_raw_data_t *raw_data) {

	// Check that the lsm9ds1 has been initialized.
	if (!bus->initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(bus, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	// Read the gryo.
	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_X_L_G);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_X_H_G);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Y_L_G);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Y_H_G);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Z_L_G);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Z_H_G);
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

lsm9ds1_status_t lsm9ds1_setup_gyro(lsm9ds1_bus_t *bus, 
	lsm9ds1_gyro_settings_t *settings, lsm9ds1_gyro_scale_t scale) {

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

	status = lsm9ds1_soft_reset(bus);
	if ((status != LSM9DS1_ACCEL_GYRO_ALREADY_RESET) && (status < 0)) {
		return status;
	}

#if DEBUG > 0
	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG4);
	DEBUG_PRINT("Reading LSM9DS1_REGISTER_CTRL_REG4: 0x%X\n",
	            bus->spi.rx[0]);
#endif

	/**************************************************************************
	 * Setup output data rate to 952 HZ (maximum), and the provided scale.
	 *************************************************************************/
	// enable gyro continuous
	settings->odr = LSM9DS1_GYRO_ODR_952HZ;
	settings->scale = scale;
	uint8_t reg = (settings->odr |
	               settings->scale);
	status = lsm9ds1_register_write(bus, LSM9DS1_REGISTER_CTRL_REG1_G,
	                                __extension__ 0b11111000, reg);
	if (status < 0) return status;

#if DEBUG > 0
	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG1_G);

	if (status < 0) {
		return status;
	}
	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG1_G: 0x%X\n",
	            bus->spi.rx[0]);
#endif

	// Decide on the conversion value based on the provided scale
	switch (settings->scale) {
	case LSM9DS1_GYROSCALE_245DPS:
		settings->gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_245DPS;
		break;
	case LSM9DS1_GYROSCALE_500DPS:
		settings->gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_500DPS;
		break;
	case LSM9DS1_GYROSCALE_2000DPS:
		settings->gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_2000DPS;
		break;
	default:
		return LSM9DS1_UKNOWN_GAIN_RANGE;
	}

	return LSM9DS1_SUCCESS;
}