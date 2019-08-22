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

#include <time.h>

#include "lsm9ds1_error.h"
#include "lsm9ds1_common.h"
#include "lsm9ds1_regs.h"
#include "lsm9ds1_debug.h"

lsm9ds1_status_t read_bit_value(uint8_t value, uint8_t mask, uint8_t offset, uint8_t *read_value) {

	*read_value = ((value & mask) >> offset);

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t is_correct_sub_device(lsm9ds1_bus_t *bus, lsm9ds1_sub_device_id_t sub_device) {

	lsm9ds1_status_t function_return = LSM9DS1_UNKNOWN_ERROR;

	DEBUG_PRINT("Made it\n");

	if (!bus->initialized) return LSM9DS1_BUS_NOT_INTIALIZED;

	// The mag accel and gyro id should be at the same offset, if not, we don't know what device we have.
	// We are comparing enums of different types, cast first since we want to do this.
	if (!((int) LSM9DS1_REGISTER_WHO_AM_I_XG
	        == (int) LSM9DS1_REGISTER_WHO_AM_I_M)) {
		return LSM9DS1_UNKNOWN_SUB_DEVICE;
	}

	// Discover device
	function_return = lsm9ds1_read(bus, LSM9DS1_REGISTER_WHO_AM_I);
	lsm9ds1_sub_device_id_t found_device = bus->spi.rx[0];
	DEBUG_PRINT("Sub-device: (0x%X)\n", found_device);

	if (found_device != sub_device) return LSM9DS1_UNKNOWN_SUB_DEVICE;

	return function_return;
}

lsm9ds1_status_t lsm9ds1_soft_reset(lsm9ds1_bus_t *bus) {

	// We will only reset once.
	static bool accel_gyro_reset = false;
	static bool mag_reset = false;

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	if (accel_gyro_reset && mag_reset) {
		if(accel_gyro_reset) {return LSM9DS1_ACCEL_GYRO_ALREADY_RESET;}
		if(mag_reset) {return LSM9DS1_MAG_ALREADY_RESET;}
	}

	switch (bus->id) {
		case LSM9DS1_ACCEL_GYRO:
			// Soft reset on the accelerometer and gyroscope
			status = lsm9ds1_write(bus, LSM9DS1_REGISTER_CTRL_REG8, 0x05);
			if (status < 0) return status;
			accel_gyro_reset = true;
			break;
		case LSM9DS1_MAG:
			// Soft reset on the magnetometer
			status = lsm9ds1_write(bus, LSM9DS1_REGISTER_CTRL_REG8, 0x0C);
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
