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

#include "lsm9ds1_temp.h"
#include "lsm9ds1_regs.h"
#include "lsm9ds1_debug.h"

lsm9ds1_status_t lsm9ds1_read_temp(lsm9ds1_bus_t *bus, lsm9ds1_temperature_t *temperature) {

	// Check that the lsm9ds1 has been initialized.
	if (!bus->initialized) {
		return LSM9DS1_BUS_NOT_INTIALIZED;
	}

	DEBUG_PRINT("Temp initialized\n");

	lsm9ds1_status_t read_status = LSM9DS1_UNKNOWN_ERROR;

	// Ensure we have the correct device
	read_status = lsm9ds1_select_sub_device(bus, LSM9DS1_ACCEL_GYRO);
	if (read_status < 0) {
		return read_status;
	}

	// Read the accelerometer.
	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_TEMP_OUT_L);
	if (read_status < 0) {
		return read_status;
	}
	uint8_t xlo = bus->spi.rx[0];

	DEBUG_PRINT("Temp xlo: 0x%X\n", xlo);

	read_status = lsm9ds1_read(bus, LSM9DS1_REGISTER_TEMP_OUT_H);
	if (read_status < 0) {
		return read_status;
	}
	lsm9ds1_temperature_t xhi = bus->spi.rx[0];

	DEBUG_PRINT("Temp xhi: 0x%X\n", xhi);

	xhi <<= 8;
	xhi |= xlo;

	// Shift values to create properly formed integer (low byte first)
	*temperature = xhi;

	return LSM9DS1_SUCCESS;
}