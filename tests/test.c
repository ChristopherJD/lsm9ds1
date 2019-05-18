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
#include "lsm9ds1.h"

/* The suite initialization function.
 *
 */
int init_lsm9ds1_suite(void) {
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	status = lsm9ds1_init(LSM9DS1_SPI_BUS, LSM9DS1_ACCELRANGE_8G, LSM9DS1_MAGGAIN_8GAUSS, LSM9DS1_GYROSCALE_500DPS);
	if (status < 0) {
		fprintf(stderr, "Error initializing lsm9ds1!\n");
		return -1;
	}

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
void test_lsm9ds1_read_sub_device(void) {

	lsm9ds1_devices_t found_device = LSM9DS1_UNKNOWN_SUB_DEVICE;
	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
	status = lsm9ds1_read_sub_device(&found_device);

	CU_ASSERT(0 == status);
	CU_ASSERT(LSM9DS1_ACCEL_GYRO == found_device);
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
	pSuite = CU_add_suite("lsm9ds1 suite", init_lsm9ds1_suite, clean_lsm9ds1_suite);
	if (NULL == pSuite) {
		CU_cleanup_registry();
		return CU_get_error();
	}

	/* add the tests to the suite */
	if ((NULL == CU_add_test(pSuite, "test read_sub_device", test_lsm9ds1_read_sub_device))) {
		CU_cleanup_registry();
		return CU_get_error();
	}

	/* Run all tests using the CUnit Basic interface */
	CU_basic_set_mode(CU_BRM_VERBOSE);
	CU_basic_run_tests();
	CU_cleanup_registry();
	return CU_get_error();
}

