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
 * @brief Register addresses for the lsm9ds1.
 */

#ifndef LSM9DS1_REGS_H_
#define LSM9DS1_REGS_H_

#define LSM9DS1_REGISTER_WHO_AM_I 0x0FU

typedef enum {
	LSM9DS1_REGISTER_WHO_AM_I_XG = 0x0F,
	LSM9DS1_REGISTER_CTRL_REG1_G = 0x10,
	LSM9DS1_REGISTER_CTRL_REG2_G = 0x11,
	LSM9DS1_REGISTER_CTRL_REG3_G = 0x12,
	LSM9DS1_REGISTER_TEMP_OUT_L = 0x15,
	LSM9DS1_REGISTER_TEMP_OUT_H = 0x16,
	LSM9DS1_REGISTER_STATUS_REG = 0x17,
	LSM9DS1_REGISTER_OUT_X_L_G = 0x18,
	LSM9DS1_REGISTER_OUT_X_H_G = 0x19,
	LSM9DS1_REGISTER_OUT_Y_L_G = 0x1A,
	LSM9DS1_REGISTER_OUT_Y_H_G = 0x1B,
	LSM9DS1_REGISTER_OUT_Z_L_G = 0x1C,
	LSM9DS1_REGISTER_OUT_Z_H_G = 0x1D,
	LSM9DS1_REGISTER_CTRL_REG4 = 0x1E,
	LSM9DS1_REGISTER_CTRL_REG5_XL = 0x1F,
	LSM9DS1_REGISTER_CTRL_REG6_XL = 0x20,
	LSM9DS1_REGISTER_CTRL_REG7_XL = 0x21,
	LSM9DS1_REGISTER_CTRL_REG8 = 0x22,
	LSM9DS1_REGISTER_CTRL_REG9 = 0x23,
	LSM9DS1_REGISTER_CTRL_REG10 = 0x24,

	LSM9DS1_REGISTER_OUT_X_L_XL = 0x28,
	LSM9DS1_REGISTER_OUT_X_H_XL = 0x29,
	LSM9DS1_REGISTER_OUT_Y_L_XL = 0x2A,
	LSM9DS1_REGISTER_OUT_Y_H_XL = 0x2B,
	LSM9DS1_REGISTER_OUT_Z_L_XL = 0x2C,
	LSM9DS1_REGISTER_OUT_Z_H_XL = 0x2D,

} lsm9ds1AccGyroRegisters_t;

typedef enum {

	LSM9DS1_REGISTER_WHO_AM_I_M = 0x0F,
	LSM9DS1_REGISTER_CTRL_REG1_M = 0x20,
	LSM9DS1_REGISTER_CTRL_REG2_M = 0x21,
	LSM9DS1_REGISTER_CTRL_REG3_M = 0x22,
	LSM9DS1_REGISTER_CTRL_REG4_M = 0x23,
	LSM9DS1_REGISTER_CTRL_REG5_M = 0x24,
	LSM9DS1_REGISTER_STATUS_REG_M = 0x27,
	LSM9DS1_REGISTER_OUT_X_L_M = 0x28,
	LSM9DS1_REGISTER_OUT_X_H_M = 0x29,
	LSM9DS1_REGISTER_OUT_Y_L_M = 0x2A,
	LSM9DS1_REGISTER_OUT_Y_H_M = 0x2B,
	LSM9DS1_REGISTER_OUT_Z_L_M = 0x2C,
	LSM9DS1_REGISTER_OUT_Z_H_M = 0x2D,
	LSM9DS1_REGISTER_CFG_M = 0x30,
	LSM9DS1_REGISTER_INT_SRC_M = 0x31,
} lsm9ds1MagRegisters_t;

#endif