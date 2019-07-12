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

#include "lsm9ds1_mag.h"
#include "lsm9ds1_regs.h"
#include "lsm9ds1_debug.h"

static lsm9ds1_status_t lsm9ds1_read_mag_settings(lsm9ds1_bus_t *bus, lsm9ds1_mag_settings_t *settings) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG1_M);
	if (status < 0) {
		return status;
	}
	uint8_t bit_value = 0;
	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_TEMP_COMP_BIT_MASK, 
		LSM9DS1_MAG_TEMP_COMP_BIT_OFFSET, &bit_value);
	settings->temp_comp_enable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_XY_OP_MODE_BIT_MASK, 
		LSM9DS1_MAG_XY_OP_MODE_BIT_OFFSET, &bit_value);
	settings->xy_op_mode = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_ODR_BIT_MASK, 
		LSM9DS1_MAG_ODR_BIT_OFFSET, &bit_value);
	settings->odr = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_FAST_ODR_BIT_MASK, 
		LSM9DS1_MAG_FAST_ODR_BIT_OFFSET, &bit_value);
	settings->fast_odr_enable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_SELF_TEST_BIT_MASK, 
		LSM9DS1_MAG_SELF_TEST_BIT_OFFSET, &bit_value);
	settings->self_test_enable = bit_value;
	
	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG1_M: 0x%X\n",
	            bus->spi.rx[0]);

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG2_M);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_GAIN_BIT_MASK, 
		LSM9DS1_MAG_GAIN_BIT_OFFSET, &bit_value);
	settings->gain = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_REBOOT_BIT_MASK, 
		LSM9DS1_MAG_REBOOT_BIT_OFFSET, &bit_value);
	settings->reboot = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_RESET_BIT_MASK, 
		LSM9DS1_MAG_RESET_BIT_OFFSET, &bit_value);
	settings->reset = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG2_M: 0x%X\n",
	            bus->spi.rx[0]);

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG3_M);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_I2C_DISABLE_BIT_MASK, 
		LSM9DS1_MAG_I2C_DISABLE_BIT_OFFSET, &bit_value);
	settings->i2c_disable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_LOW_POWER_MODE_BIT_MASK, 
		LSM9DS1_MAG_LOW_POWER_MODE_BIT_OFFSET, &bit_value);
	settings->low_power_mode = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_SPI_MODE_BIT_MASK, 
		LSM9DS1_MAG_SPI_MODE_BIT_OFFSET, &bit_value);
	settings->spi_mode = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_OP_MODE_BIT_MASK, 
		LSM9DS1_MAG_OP_MODE_BIT_OFFSET, &bit_value);
	settings->op_mode = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG3_M: 0x%X\n",
	            bus->spi.rx[0]);

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG4_M);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_Z_OP_MODE_BIT_MASK, 
		LSM9DS1_MAG_Z_OP_MODE_BIT_OFFSET, &bit_value);
	settings->z_op_mode = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_LITTLE_ENDIAN_BIT_MASK, 
		LSM9DS1_MAG_LITTLE_ENDIAN_BIT_OFFSET, &bit_value);
	settings->little_endian_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG4_M: 0x%X\n",
	            bus->spi.rx[0]);

	status = lsm9ds1_read(bus, LSM9DS1_REGISTER_CTRL_REG5_M);
	if (status < 0) {
		return status;
	}

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_FAST_READ_BIT_MASK, 
		LSM9DS1_MAG_FAST_READ_BIT_OFFSET, &bit_value);
	settings->fast_read_enable = bit_value;

	(void)read_bit_value(bus->spi.rx[0], LSM9DS1_MAG_BLOCK_DATA_UPDATE_BIT_MASK, 
		LSM9DS1_MAG_BLOCK_DATA_UPDATE_BIT_OFFSET, &bit_value);
	settings->block_data_update_enable = bit_value;

	DEBUG_PRINT("Read back LSM9DS1_REGISTER_CTRL_REG5_M: 0x%X\n",
	            bus->spi.rx[0]);

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_setup_mag(lsm9ds1_bus_t *bus, 
	lsm9ds1_mag_settings_t *settings, lsm9ds1_mag_gain_t gain) {

	/**************************************************************************
	 * Checks and setup
	 *************************************************************************/
	// Bus must be initialized
	if (!bus->initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	// Ensure we have the correct device
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
	status = lsm9ds1_select_sub_device(bus, LSM9DS1_MAG);
	if (status < 0) {
		return status;
	}

	status = lsm9ds1_read_mag_settings(bus, settings);
	if (status < 0) {
		return status;
	}

	/**************************************************************************
	 * Setup continous mode, output data rate to 80 HZ (maximum), and the
	 * provided gain.
	 *************************************************************************/
	// Perform a reset on the current sub-device selected
	status = lsm9ds1_soft_reset(bus);
	if (status < 0) return status;

	// Set the operational mode to continuous
	settings->op_mode = LSM9DS1_MAG_OP_MODE_CONTINUOUS;
	status = lsm9ds1_register_write(bus, LSM9DS1_REGISTER_CTRL_REG3_M,
	                                __extension__ 0b00000011,
	                                settings->op_mode);
	if (status < 0) return status;

	// Setup the output data rate
	settings->odr = LSM9DS1_MAG_ODR_80HZ;
	status = lsm9ds1_register_write(bus, LSM9DS1_REGISTER_CTRL_REG1_M,
	                                __extension__ 0b00011100,
	                                settings->odr);
	if (status < 0) return status;

	// Setup the Gain
	settings->gain = gain;
	status = lsm9ds1_register_write(bus, LSM9DS1_REGISTER_CTRL_REG2_M,
	                                __extension__ 0b01100000,
	                                settings->gain);
	if (status < 0) return status;

	// Decide the conversion value used based on provided gain
	switch (settings->gain) {
	case LSM9DS1_MAGGAIN_4GAUSS:
		settings->mag_mgauss = LSM9DS1_MAG_MGAUSS_4GAUSS;
		break;
	case LSM9DS1_MAGGAIN_8GAUSS:
		settings->mag_mgauss = LSM9DS1_MAG_MGAUSS_8GAUSS;
		break;
	case LSM9DS1_MAGGAIN_12GAUSS:
		settings->mag_mgauss = LSM9DS1_MAG_MGAUSS_12GAUSS;
		break;
	case LSM9DS1_MAGGAIN_16GAUSS:
		settings->mag_mgauss = LSM9DS1_MAG_MGAUSS_16GAUSS;
		break;
	default:
		return LSM9DS1_UKNOWN_GAIN_RANGE;
	}

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t lsm9ds1_read_mag(lsm9ds1_bus_t *bus, mag_raw_data_t *raw_data) {

	// Check that the lsm9ds1 has been initialized.
	if (!bus->initialized) {
		return LSM9DS1_NOT_INITIALIZED;
	}

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(bus, LSM9DS1_MAG);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_X_L_M);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_X_H_M);
	if (read_status < 0) {
		return read_status;
	}
	int16_t xhi = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Y_L_M);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t ylo = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Y_H_M);
	if (read_status < 0) {
		return read_status;
	}
	int16_t yhi = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Z_L_M);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t zlo = bus->spi.rx[0];

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_OUT_Z_H_M);
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
