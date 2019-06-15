/*
 *  Simple example of a CUnit unit test.
 *
 *  This program (crudely) demonstrates a very simple "black box"
 *  test of the standard library functions fprintf() and fread().
 *  It uses suite initialization and cleanup functions to open
 *  and close a common temporary file used by the test functions.
 *  The test functions then write to and read from the temporary
 *  file in the course of testing the library functions.
 *
 *  The 2 test functions are added to a single CUnit suite, and
 *  then run using the CUnit Basic interface.  The output of the
 *  program (on CUnit version 2.0-2) is:
 *
 *           CUnit : A Unit testing framework for C.
 *           http://cunit.sourceforge.net/
 *
 *       Suite: Suite_1
 *         Test: test of fprintf() ... passed
 *         Test: test of fread() ... passed
 *
 *       --Run Summary: Type      Total     Ran  Passed  Failed
 *                      suites        1       1     n/a       0
 *                      tests         2       2       2       0
 *                      asserts       5       5       5       0
 */

#include <stdio.h>
#include <string.h>
#include "CUnit/Basic.h"
#include <stdlib.h>
#include "lsm9ds1.h"

static lsm9ds1_device_t *lsm9ds1 = NULL;

/* The suite initialization function.
 *
 */
int init_lsm9ds1_suite(void) {

	lsm9ds1 = malloc(sizeof(lsm9ds1_device_t));
	(void)lsm9ds1_init(lsm9ds1, LSM9DS1_SPI_BUS, LSM9DS1_ACCELRANGE_8G, LSM9DS1_MAGGAIN_8GAUSS, LSM9DS1_GYROSCALE_500DPS);

	return 0;
}

/* The suite cleanup function.
 * Returns zero on success, non-zero otherwise.
 */
int clean_lsm9ds1_suite(void) {
	//TODO add close.
	return 0;
}

/* Simple test of fprintf().
 * Writes test data to the temporary file and checks
 * whether the expected number of bytes were written.
 */
void test_lsm9ds1_read_sub_device_accel_gryo(void) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_select_sub_device(lsm9ds1, LSM9DS1_ACCEL_GYRO);
	CU_ASSERT(0 == status);
}

void test_lsm9ds1_read_sub_device_mag(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_select_sub_device(lsm9ds1, LSM9DS1_MAG);
	CU_ASSERT(0 == status);
}

void test_lsm9ds1_read_temp(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1->update_temp(lsm9ds1);

	printf("%d\n", status);

	CU_ASSERT(0 == status);
	printf("Raw Temp: %d\n", lsm9ds1->raw_data.temperature);
	printf("Temp: %f\n", lsm9ds1->converted_data.temperature);
}

void test_lsm9ds1_read_accel(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1->update_accel(lsm9ds1);
	printf("%d\n", status);

	CU_ASSERT(0 == status);
	printf("Raw Accel X: %d\n", lsm9ds1->raw_data.accelerometer.x);
	printf("Raw Accel Y: %d\n", lsm9ds1->raw_data.accelerometer.y);
	printf("Raw Accel Z: %d\n", lsm9ds1->raw_data.accelerometer.z);
	printf("Accel X: %f\n", lsm9ds1->converted_data.accelerometer.x);
	printf("Accel Y: %f\n", lsm9ds1->converted_data.accelerometer.y);
	printf("Accel Z: %f\n", lsm9ds1->converted_data.accelerometer.z);
}

void test_lsm9ds1_read_gyro(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1->update_gyro(lsm9ds1);
	printf("%d\n", status);

	CU_ASSERT(0 == status);
	printf("Raw Gyro X: %d\n", lsm9ds1->raw_data.gyroscope.x);
	printf("Raw Gyro Y: %d\n", lsm9ds1->raw_data.gyroscope.y);
	printf("Raw Gyro Z: %d\n", lsm9ds1->raw_data.gyroscope.z);
	printf("Gyro X: %f\n", lsm9ds1->converted_data.gyroscope.x);
	printf("Gyro Y: %f\n", lsm9ds1->converted_data.gyroscope.y);
	printf("Gyro Z: %f\n", lsm9ds1->converted_data.gyroscope.z);
}

void test_lsm9ds1_read_mag(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1->update_mag(lsm9ds1);
	printf("%d\n", status);

	CU_ASSERT(0 == status);
	printf("Raw Mag X: %d\n", lsm9ds1->raw_data.magnetometer.x);
	printf("Raw Mag Y: %d\n", lsm9ds1->raw_data.magnetometer.y);
	printf("Raw Mag Z: %d\n", lsm9ds1->raw_data.magnetometer.z);
	printf("Mag X: %f\n", lsm9ds1->converted_data.magnetometer.x);
	printf("Mag Y: %f\n", lsm9ds1->converted_data.magnetometer.y);
	printf("Mag Z: %f\n", lsm9ds1->converted_data.magnetometer.z);
}

/* The main() function for setting up and running the tests.
 * Returns a CUE_SUCCESS on successful running, another
 * CUnit error code on failure.
 */
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
	if ((NULL == CU_add_test(pSuite, "test read_sub_device for mag", test_lsm9ds1_read_sub_device_mag))
	        || (NULL == CU_add_test(pSuite, "test read_sub_device for accel and gyro", test_lsm9ds1_read_sub_device_accel_gryo))
	        || (NULL == CU_add_test(pSuite, "test lsm9ds1_read_temp", test_lsm9ds1_read_temp))
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

