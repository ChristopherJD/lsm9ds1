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

#include <stdio.h>
#include <string.h>
#include "CUnit/Basic.h"
#include <stdlib.h>
#include "lsm9ds1.h"

/* The suite initialization function.
 *
 */
int init_lsm9ds1_suite(void) {

	(void)lsm9ds1_init();

	return 0;
}

/* The suite cleanup function.
 * Returns zero on success, non-zero otherwise.
 */
int clean_lsm9ds1_suite(void) {
	lsm9ds1_close();
	return 0;
}

void test_lsm9ds1_read_temp(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	lsm9ds1_temperature_t temperature = 0;
	status = lsm9ds1_get_temp(&temperature);

	printf("%d\n", status);

	CU_ASSERT(0 == status);
	printf("Temp: %f\n", temperature);
}

void test_lsm9ds1_read_accel(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	accelerometer_converted_data_t data = {0};
	status = lsm9ds1_get_accel(&data);
	printf("%d\n", status);

	CU_ASSERT(0 == status);
	printf("Accel X: %f\n", data.x);
	printf("Accel Y: %f\n", data.y);
	printf("Accel Z: %f\n", data.z);
}

void test_lsm9ds1_read_gyro(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	gyro_converted_data_t data = {0};
	status = lsm9ds1_get_gyro(&data);
	printf("%d\n", status);

	CU_ASSERT(0 == status);
	printf("Gyro X: %f\n", data.x);
	printf("Gyro Y: %f\n", data.y);
	printf("Gyro Z: %f\n", data.z);
}

void test_lsm9ds1_read_mag(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	mag_converted_data_t data = {0};
	status = lsm9ds1_get_mag(&data);
	printf("%d\n", status);

	CU_ASSERT(0 == status);
	printf("Mag X: %f\n", data.x);
	printf("Mag Y: %f\n", data.y);
	printf("Mag Z: %f\n", data.z);
}

int main() {
	CU_pSuite pSuite = NULL;

	/* initialize the CUnit test registry */
	if (CUE_SUCCESS != CU_initialize_registry())
		return CU_get_error();

	/* add a suite to the registry */
	pSuite = CU_add_suite("lsm9ds1 suite", init_lsm9ds1_suite,
	                      clean_lsm9ds1_suite);
	if (NULL == pSuite) {
		CU_cleanup_registry();
		return CU_get_error();
	}

	/* add the tests to the suite */
	if ((NULL == CU_add_test(pSuite, "test lsm9ds1_read_temp", test_lsm9ds1_read_temp))
	        || (NULL == CU_add_test(pSuite, "test lsm9ds1_read_accel", test_lsm9ds1_read_accel))
	        || (NULL == CU_add_test(pSuite, "test lsm9ds1_read_gyro", test_lsm9ds1_read_gyro))
	        || (NULL == CU_add_test(pSuite, "test lsm9ds1_read_mag", test_lsm9ds1_read_mag))

	   ) {
		CU_cleanup_registry();
		return CU_get_error();
	}

	/* Run all tests using the CUnit Basic interface */
	CU_basic_set_mode(CU_BRM_VERBOSE);
	CU_basic_run_tests();
	CU_cleanup_registry();
	return CU_get_error();
}

