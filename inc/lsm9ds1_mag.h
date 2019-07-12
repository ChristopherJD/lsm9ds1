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
 * @brief Magnetometer functions and data.
 *
 * Setup function, and read function for the magnetometer.
 */

#ifndef LSM9DS1_MAG_H_
#define LSM9DS1_MAG_H_

#include "lsm9ds1_common.h"
#include "lsm9ds1_error.h"

#define LSM9DS1_MAG_TEMP_COMP_BIT_OFFSET 7
#define LSM9DS1_MAG_TEMP_COMP_BIT_MASK __extension__ 0b10000000
#define LSM9DS1_MAG_TEMP_COMP_ENABLE (1 << LSM9DS1_MAG_TEMP_COMP_BIT_OFFSET);
#define LSM9DS1_MAG_TEMP_COMP_DISABLE (0 << LSM9DS1_MAG_TEMP_COMP_BIT_OFFSET);

#define LSM9DS1_MAG_XY_OP_MODE_BIT_OFFSET 5
#define LSM9DS1_MAG_XY_OP_MODE_BIT_MASK __extension__ 0b01100000
typedef enum lsm9ds1_mag_xy_op_mode_t {
	LSM9DS1_MAG_XY_OP_MODE_LOW_POWER = (0 << LSM9DS1_MAG_XY_OP_MODE_BIT_OFFSET),
	LSM9DS1_MAG_XY_OP_MODE_MEDIUM_PERF = (1 << LSM9DS1_MAG_XY_OP_MODE_BIT_OFFSET),
	LSM9DS1_MAG_XY_OP_MODE_HIGH_PERF = (2 << LSM9DS1_MAG_XY_OP_MODE_BIT_OFFSET),
	LSM9DS1_MAG_XY_OP_MODE_ULTRA_PERF = (3 << LSM9DS1_MAG_XY_OP_MODE_BIT_OFFSET),
}lsm9ds1_mag_xy_op_mode_t;

#define LSM9DS1_MAG_ODR_BIT_OFFSET 2
#define LSM9DS1_MAG_ODR_BIT_MASK __extension__ 0b00011100
typedef enum lsm9ds1_mag_odr_t{
	LSM9DS1_MAG_ODR_0_625HZ = (0 << LSM9DS1_MAG_ODR_BIT_OFFSET),
	LSM9DS1_MAG_ODR_1_25HZ = (1 << LSM9DS1_MAG_ODR_BIT_OFFSET),
	LSM9DS1_MAG_ODR_2_5HZ = (2 << LSM9DS1_MAG_ODR_BIT_OFFSET),
	LSM9DS1_MAG_ODR_5HZ = (3 << LSM9DS1_MAG_ODR_BIT_OFFSET),
	LSM9DS1_MAG_ODR_10HZ = (4 << LSM9DS1_MAG_ODR_BIT_OFFSET),
	LSM9DS1_MAG_ODR_20HZ = (5 << LSM9DS1_MAG_ODR_BIT_OFFSET),
	LSM9DS1_MAG_ODR_40HZ = (6 << LSM9DS1_MAG_ODR_BIT_OFFSET),
	LSM9DS1_MAG_ODR_80HZ = (7 << LSM9DS1_MAG_ODR_BIT_OFFSET)
} lsm9ds1_mag_odr_t;

#define LSM9DS1_MAG_FAST_ODR_BIT_OFFSET 1
#define LSM9DS1_MAG_FAST_ODR_BIT_MASK __extension__ 0b00000010
#define LSM9DS1_MAG_FAST_ODR_ENABLE (1 << LSM9DS1_MAG_FAST_ODR_BIT_OFFSET);
#define LSM9DS1_MAG_FAST_ODR_DISABLE (0 << LSM9DS1_MAG_FAST_ODR_BIT_OFFSET);

#define LSM9DS1_MAG_SELF_TEST_BIT_OFFSET 0
#define LSM9DS1_MAG_SELF_TEST_BIT_MASK __extension__ 0b00000010
#define LSM9DS1_MAG_SELF_TEST_ENABLE (1 << LSM9DS1_MAG_SELF_TEST_BIT_OFFSET);
#define LSM9DS1_MAG_SELF_TEST_DISABLE (0 << LSM9DS1_MAG_SELF_TEST_BIT_OFFSET);

#define LSM9DS1_MAG_GAIN_BIT_OFFSET 5
#define LSM9DS1_MAG_GAIN_BIT_MASK __extension__ 0b01100000
typedef enum {
	LSM9DS1_MAGGAIN_4GAUSS = (0 << LSM9DS1_MAG_GAIN_BIT_OFFSET),  // +/- 4 gauss
	LSM9DS1_MAGGAIN_8GAUSS = (1 << LSM9DS1_MAG_GAIN_BIT_OFFSET),  // +/- 8 gauss
	LSM9DS1_MAGGAIN_12GAUSS = (2 << LSM9DS1_MAG_GAIN_BIT_OFFSET),  // +/- 12 gauss
	LSM9DS1_MAGGAIN_16GAUSS = (3 << LSM9DS1_MAG_GAIN_BIT_OFFSET)   // +/- 16 gauss
} lsm9ds1_mag_gain_t;

#define LSM9DS1_MAG_REBOOT_BIT_OFFSET 3
#define LSM9DS1_MAG_REBOOT_BIT_MASK __extension__ 0b00001000
#define LSM9DS1_MAG_REBOOT (1 << LSM9DS1_MAG_GAIN_BIT_OFFSET);

#define LSM9DS1_MAG_RESET_BIT_OFFSET 2
#define LSM9DS1_MAG_RESET_BIT_MASK __extension__ 0b00000100
#define LSM9DS1_MAG_RESET (1 << LSM9DS1_MAG_RESET_BIT_OFFSET);

#define LSM9DS1_MAG_I2C_DISABLE_BIT_OFFSET 7
#define LSM9DS1_MAG_I2C_DISABLE_BIT_MASK __extension__ 0b10000000
#define LSM9DS1_MAG_I2C_DISABLE (1 << LSM9DS1_MAG_I2C_DISABLE_BIT_OFFSET);
#define LSM9DS1_MAG_I2C_ENABLE (0 << LSM9DS1_MAG_I2C_DISABLE_BIT_OFFSET);

#define LSM9DS1_MAG_LOW_POWER_MODE_BIT_OFFSET 5
#define LSM9DS1_MAG_LOW_POWER_MODE_BIT_MASK __extension__ 0b00100000
#define LSM9DS1_MAG_LOW_POWER_MODE (1 << LSM9DS1_MAG_LOW_POWER_MODE_BIT_OFFSET);

#define LSM9DS1_MAG_SPI_MODE_BIT_OFFSET 2
#define LSM9DS1_MAG_SPI_MODE_BIT_MASK __extension__ 0b00000100
typedef enum lsm9ds1_mag_spi_mode_t {
	LSM9DS1_MAG_ONLY_WRITE = (0 << LSM9DS1_MAG_SPI_MODE_BIT_OFFSET),
	LSM9DS1_MAG_READ_WRITE = (1 << LSM9DS1_MAG_SPI_MODE_BIT_OFFSET),
}lsm9ds1_mag_spi_mode_t;

#define LSM9DS1_MAG_OP_MODE_BIT_OFFSET 0
#define LSM9DS1_MAG_OP_MODE_BIT_MASK __extension__ 0b00000100
typedef enum lsm9ds1_mag_op_mode_t {
	LSM9DS1_MAG_OP_MODE_CONTINUOUS = (0 << LSM9DS1_MAG_OP_MODE_BIT_OFFSET),
	LSM9DS1_MAG_OP_MODE_SINGLE_CONVERSION = (1 << LSM9DS1_MAG_OP_MODE_BIT_OFFSET),
	LSM9DS1_MAG_OP_MODE_POWER_DOWN = (2 << LSM9DS1_MAG_OP_MODE_BIT_OFFSET),
}lsm9ds1_mag_op_mode_t;

#define LSM9DS1_MAG_Z_OP_MODE_BIT_OFFSET 2
#define LSM9DS1_MAG_Z_OP_MODE_BIT_MASK __extension__ 0b00001100
typedef enum lsm9ds1_mag_z_op_mode_t {
	LSM9DS1_MAG_Z_OP_MODE_LOW_POWER = (0 << LSM9DS1_MAG_Z_OP_MODE_BIT_OFFSET),
	LSM9DS1_MAG_Z_OP_MODE_MEDIUM_PERF = (1 << LSM9DS1_MAG_Z_OP_MODE_BIT_OFFSET),
	LSM9DS1_MAG_Z_OP_MODE_HIGH_PERF = (2 << LSM9DS1_MAG_Z_OP_MODE_BIT_OFFSET),
	LSM9DS1_MAG_Z_OP_MODE_ULTRA_PERF = (3 << LSM9DS1_MAG_Z_OP_MODE_BIT_OFFSET),
}lsm9ds1_mag_z_op_mode_t;

#define LSM9DS1_MAG_LITTLE_ENDIAN_BIT_OFFSET 1
#define LSM9DS1_MAG_LITTLE_ENDIAN_BIT_MASK __extension__ 0b00000010
#define LSM9DS1_MAG_LITTLE_ENDIAN_ENABLE (1 << LSM9DS1_MAG_LITTLE_ENDIAN_BIT_OFFSET);
#define LSM9DS1_MAG_LITTLE_ENDIAN_DISABLE (0 << LSM9DS1_MAG_LITTLE_ENDIAN_BIT_OFFSET);

#define LSM9DS1_MAG_FAST_READ_BIT_OFFSET 7
#define LSM9DS1_MAG_FAST_READ_BIT_MASK __extension__ 0b10000000
#define LSM9DS1_MAG_FAST_READ_ENABLE (1 << LSM9DS1_MAG_FAST_READ_BIT_OFFSET);
#define LSM9DS1_MAG_FAST_READ_DISABLE (0 << LSM9DS1_MAG_FAST_READ_BIT_OFFSET);

#define LSM9DS1_MAG_BLOCK_DATA_UPDATE_BIT_OFFSET 6
#define LSM9DS1_MAG_BLOCK_DATA_UPDATE_BIT_MASK __extension__ 0b01000000
#define LSM9DS1_MAG_BLOCK_DATA_UPDATE_ENABLE (1 << LSM9DS1_MAG_BLOCK_DATA_UPDATE_BIT_OFFSET);
#define LSM9DS1_MAG_BLOCK_DATA_UPDATE_DISABLE (0 << LSM9DS1_MAG_BLOCK_DATA_UPDATE_BIT_OFFSET);

// Magnetic Field Strength: gauss range
#define LSM9DS1_MAG_MGAUSS_4GAUSS      (0.14F)
#define LSM9DS1_MAG_MGAUSS_8GAUSS      (0.29F)
#define LSM9DS1_MAG_MGAUSS_12GAUSS     (0.43F)
#define LSM9DS1_MAG_MGAUSS_16GAUSS     (0.58F)

typedef struct mag_data {
	int16_t x;
	int16_t y;
	int16_t z;
} mag_raw_data_t;

typedef struct mag_converted_data_t {
	float x;
	float y;
	float z;
}mag_converted_data_t;


typedef struct lsm9ds1_mag_settings_t {
	float mag_mgauss;

	//CTRL REG 1
	uint8_t temp_comp_enable;
	lsm9ds1_mag_xy_op_mode_t xy_op_mode;
	lsm9ds1_mag_odr_t odr;
	uint8_t fast_odr_enable;
	uint8_t self_test_enable;

	//CTRL REG 2
	lsm9ds1_mag_gain_t gain;
	uint8_t reboot;
	uint8_t reset;

	//CTRL REG 3
	uint8_t i2c_disable;
	uint8_t low_power_mode;
	lsm9ds1_mag_spi_mode_t spi_mode;
	lsm9ds1_mag_op_mode_t op_mode;

	//CTRL REG 4
	lsm9ds1_mag_z_op_mode_t z_op_mode;
	uint8_t little_endian_enable;

	//CTRL REG 5
	uint8_t fast_read_enable;
	uint8_t block_data_update_enable;

}lsm9ds1_mag_settings_t;

lsm9ds1_status_t lsm9ds1_read_mag(lsm9ds1_bus_t *bus, mag_raw_data_t *raw_data);
lsm9ds1_status_t lsm9ds1_setup_mag(lsm9ds1_bus_t *bus, 
	lsm9ds1_mag_settings_t *settings, lsm9ds1_mag_gain_t gain);

#endif