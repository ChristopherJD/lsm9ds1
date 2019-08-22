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

static lsm9ds1_status_t lsm9ds1_read_gyro_settings(lsm9ds1_bus_t *bus, lsm9ds1_gyro_settings_t *settings) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG1_G);
	if (status < 0) {
		return status;
	}

	uint8_t bit_value = 0;
	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_GYRO_ODR_BIT_MASK, 
		LSM9DS1_GYRO_ODR_BIT_OFFSET, &bit_value);
	settings->odr = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_GYRO_SCALE_BIT_MASK, 
		LSM9DS1_GYRO_SCALE_BIT_OFFSET, &bit_value);
	settings->scale = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG1_G: 0x%X\n",
	            bus->spi.rx[0]);

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG3_G);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_GYRO_LOW_POWER_MODE_BIT_MASK, 
		LSM9DS1_GYRO_LOW_POWER_MODE_BIT_OFFSET, &bit_value);
	settings->low_power_mode_enable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_GYRO_HIGH_PASS_FILTER_BIT_MASK, 
		LSM9DS1_GYRO_HIGH_PASS_FILTER_BIT_OFFSET, &bit_value);
	settings->high_pass_filter_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG3_G: 0x%X\n",
	            bus->spi.rx[0]);

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG4);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_GRYO_X_ENABLE_BIT_MASK, 
		LSM9DS1_GRYO_X_ENABLE_BIT_OFFSET, &bit_value);
	settings->x_enable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_GRYO_Y_ENABLE_BIT_MASK, 
		LSM9DS1_GRYO_Y_ENABLE_BIT_OFFSET, &bit_value);
	settings->y_enable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_GRYO_Z_ENABLE_BIT_MASK, 
		LSM9DS1_GRYO_Z_ENABLE_BIT_OFFSET, &bit_value);
	settings->z_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG4: 0x%X\n",
	            bus->spi.rx[0]);

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_read_gyro(lsm9ds1_bus_t *bus, gyro_raw_data_t *raw_data) {

	// Check that the lsm9ds1 has been initialized.
	if (!bus->initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = is_correct_sub_device(bus, LSM9DS1_ACCEL_GYRO);
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
	status = is_correct_sub_device(bus, LSM9DS1_ACCEL_GYRO);
	if (status < 0) {
		return status;
	}

	status = lsm9ds1_soft_reset(bus);
	if ((status != LSM9DS1_ACCEL_GYRO_ALREADY_RESET) && (status < 0)) {
		return status;
	}

	status = lsm9ds1_read_gyro_settings(bus, settings);
	if (status < 0) {
		return status;
	}

	/**************************************************************************
	 * Setup output data rate to 952 HZ (maximum), and the provided scale.
	 *************************************************************************/
	// enable gyro continuous
	settings->odr = LSM9DS1_GYRO_ODR_952HZ;
	settings->scale = scale;
	uint8_t reg = (settings->odr |
	               settings->scale);
	status = lsm9ds1_register_write(bus, LSM9DS1_REGISTER_CTRL_REG1_G,
		(LSM9DS1_GYRO_ODR_BIT_MASK | LSM9DS1_GYRO_SCALE_BIT_MASK), reg);
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
		settings->gyro_resolution = SENSITIVITY_GYROSCOPE_245;
		break;
	case LSM9DS1_GYROSCALE_500DPS:
		settings->gyro_resolution = SENSITIVITY_GYROSCOPE_500;
		break;
	case LSM9DS1_GYROSCALE_2000DPS:
		settings->gyro_resolution = SENSITIVITY_GYROSCOPE_2000;
		break;
	default:
		return LSM9DS1_UKNOWN_GAIN_RANGE;
	}

	return LSM9DS1_SUCCESS;
}