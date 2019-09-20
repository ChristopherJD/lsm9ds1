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

/**
 * @file
 * @author Christopher Jordan-Denny
 * @date
 * @brief Gyroscope functions and data.
 *
 * Setup function, and read function for the gyroscope.
 */

#ifndef LSM9DS1_GYRO_H_
#define LSM9DS1_GRYO_H_

#include "lsm9ds1_bus.h"

// Angular Rate: dps per LSB
#define SENSITIVITY_GYROSCOPE_245    0.00875
#define SENSITIVITY_GYROSCOPE_500    0.0175
#define SENSITIVITY_GYROSCOPE_2000   0.07

#define LSM9DS1_GRYO_Z_ENABLE_BIT_OFFSET 5
#define LSM9DS1_GRYO_Z_ENABLE_BIT_MASK __extension__ 0b00100000
#define LSM9DS1_GRYO_Z_ENABLE (1 << LSM9DS1_GRYO_Z_ENABLE_BIT_OFFSET)
#define LSM9DS1_GRYO_Z_DISABLE (0 << LSM9DS1_GRYO_Z_ENABLE_BIT_OFFSET)

#define LSM9DS1_GRYO_Y_ENABLE_BIT_OFFSET 4
#define LSM9DS1_GRYO_Y_ENABLE_BIT_MASK __extension__ 0b00010000
#define LSM9DS1_GRYO_Y_ENABLE (1 << LSM9DS1_GRYO_Y_ENABLE_BIT_OFFSET)
#define LSM9DS1_GRYO_Y_DISABLE (0 << LSM9DS1_GRYO_Y_ENABLE_BIT_OFFSET)

#define LSM9DS1_GRYO_X_ENABLE_BIT_OFFSET 3
#define LSM9DS1_GRYO_X_ENABLE_BIT_MASK __extension__ 0b00001000
#define LSM9DS1_GRYO_X_ENABLE (1 << LSM9DS1_GRYO_X_ENABLE_BIT_OFFSET)
#define LSM9DS1_GRYO_X_DISABLE (0 << LSM9DS1_GRYO_X_ENABLE_BIT_OFFSET)

#define LSM9DS1_GYRO_HIGH_PASS_FILTER_BIT_OFFSET 6
#define LSM9DS1_GYRO_HIGH_PASS_FILTER_BIT_MASK __extension__ 0b01000000
#define LSM9DS1_GYRO_HIGH_PASS_FILTER_ENABLE (1 << LSM9DS1_GYRO_HIGH_PASS_FILETER_BIT_OFFSET)
#define LSM9DS1_GYRO_HIGH_PASS_FILTER_DISABLE (0 << LSM9DS1_GYRO_HIGH_PASS_FILETER_BIT_OFFSET)

#define LSM9DS1_GYRO_LOW_POWER_MODE_BIT_OFFSET 7
#define LSM9DS1_GYRO_LOW_POWER_MODE_BIT_MASK __extension__ 0b10000000
#define LSM9DS1_GYRO_LOW_POWER_MODE (1 << LSM9DS1_GRYO_LOW_POWER_MODE_BIT_OFFSET);

#define LSM9DS1_GYRO_ODR_BIT_OFFSET 5
#define LSM9DS1_GYRO_ODR_BIT_MASK __extension__ 0b11100000
typedef enum lsm9ds1_gyro_odr_t {
	LSM9DS1_GYRO_ODR_POWERDOWN = (0 << LSM9DS1_GYRO_ODR_BIT_OFFSET),
	LSM9DS1_GYRO_ODR_14_9HZ = (1 << LSM9DS1_GYRO_ODR_BIT_OFFSET),
	LSM9DS1_GYRO_ODR_59_5HZ = (2 << LSM9DS1_GYRO_ODR_BIT_OFFSET),
	LSM9DS1_GYRO_ODR_119HZ = (3 << LSM9DS1_GYRO_ODR_BIT_OFFSET),
	LSM9DS1_GYRO_ODR_238HZ = (4 << LSM9DS1_GYRO_ODR_BIT_OFFSET),
	LSM9DS1_GYRO_ODR_476HZ = (5 << LSM9DS1_GYRO_ODR_BIT_OFFSET),
	LSM9DS1_GYRO_ODR_952HZ = (6 << LSM9DS1_GYRO_ODR_BIT_OFFSET),
} lsm9ds1_gyro_odr_t;

#define LSM9DS1_GYRO_SCALE_BIT_OFFSET 3
#define LSM9DS1_GYRO_SCALE_BIT_MASK __extension__ 0b00011000
typedef enum {
	LSM9DS1_GYROSCALE_245DPS = (0 << LSM9DS1_GYRO_SCALE_BIT_OFFSET), // +/- 245 degrees per second rotation
	LSM9DS1_GYROSCALE_500DPS = (1 << LSM9DS1_GYRO_SCALE_BIT_OFFSET), // +/- 500 degrees per second rotation
	LSM9DS1_GYROSCALE_2000DPS = (3 << LSM9DS1_GYRO_SCALE_BIT_OFFSET) // +/- 2000 degrees per second rotation
} lsm9ds1_gyro_scale_t;

typedef enum {
	UKNOWN,
} lsm9ds1_gyro_bandwidth_select_t;

#define LSM9DS1_GYRO_HPCF_BIT_OFFSET 0
#define LSM9DS1_GYRO_HPCF_BIT_MASK __extension__ 0b00001111
typedef enum {
	LSM9DS1_GYRO_HPCF_DIV_2 = (0 << LSM9DS1_GYRO_HPCF_BIT_OFFSET),
	LSM9DS1_GYRO_HPCF_DIV_4 = (1 << LSM9DS1_GYRO_HPCF_BIT_OFFSET),
	LSM9DS1_GYRO_HPCF_DIV_8 = (2 << LSM9DS1_GYRO_HPCF_BIT_OFFSET),
	LSM9DS1_GYRO_HPCF_DIV_16 = (3 << LSM9DS1_GYRO_HPCF_BIT_OFFSET),
	LSM9DS1_GYRO_HPCF_DIV_32 = (4 << LSM9DS1_GYRO_HPCF_BIT_OFFSET),
	LSM9DS1_GYRO_HPCF_DIV_64 = (5 << LSM9DS1_GYRO_HPCF_BIT_OFFSET),
	LSM9DS1_GYRO_HPCF_DIV_128 = (6 << LSM9DS1_GYRO_HPCF_BIT_OFFSET),
	LSM9DS1_GYRO_HPCF_DIV_256 = (7 << LSM9DS1_GYRO_HPCF_BIT_OFFSET),
	LSM9DS1_GYRO_HPCF_DIV_512 = (8 << LSM9DS1_GYRO_HPCF_BIT_OFFSET),
	LSM9DS1_GYRO_HPCF_DIV_1024 = (9 << LSM9DS1_GYRO_HPCF_BIT_OFFSET)
}lsm9ds1_gyro_hpcf_t;

typedef struct gyro_data {
	int16_t x;
	int16_t y;
	int16_t z;
} gyro_raw_data_t;

typedef struct lsm9ds1_gyro_settings_t {

	// CTRL_REG1_G
	lsm9ds1_gyro_odr_t odr;	//output data rate
	lsm9ds1_gyro_scale_t scale;
	lsm9ds1_gyro_bandwidth_select_t bw_select;

	// CTRL_REG3_G
	uint8_t low_power_mode_enable;
	uint8_t high_pass_filter_enable;
	lsm9ds1_gyro_hpcf_t high_pass_cutoff;

	// CTRL_REG4
	uint8_t z_enable;
	uint8_t y_enable;
	uint8_t x_enable;

	float gyro_resolution;
}lsm9ds1_gyro_settings_t;

lsm9ds1_status_t lsm9ds1_read_gyro(lsm9ds1_bus_t *bus, gyro_raw_data_t *raw_data);
lsm9ds1_status_t lsm9ds1_setup_gyro(lsm9ds1_bus_t *bus, 
	lsm9ds1_gyro_settings_t *settings, lsm9ds1_gyro_scale_t scale);

#endif
