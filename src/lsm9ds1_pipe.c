#include <stdio.h>
#include "lsm9ds1.h"

int main() {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
	status = lsm9ds1_init();
	if(status < 0) {
		fprintf(stderr, "Error initializing lsm9ds1!\n");
	}

	accelerometer_converted_data_t accel_data = {0};
	status = get_accel(&accel_data);

	printf("Accelerometer x: %f\n", accel_data.x);
	printf("Accelerometer y: %f\n", accel_data.y);
	printf("Accelerometer z: %f\n", accel_data.z);	

	gyro_converted_data_t gyro_data = {0};
	status = get_gyro(&gyro_data);

	printf("Gyroscope x: %f\n", gyro_data.x);
	printf("Gyroscope y: %f\n", gyro_data.y);
	printf("Gyroscope z: %f\n", gyro_data.z);	

	mag_converted_data_t mag_data = {0};
	status = get_mag(&mag_data);

	printf("Magnetometer x: %f\n", mag_data.x);
	printf("Magnetometer y: %f\n", mag_data.y);
	printf("Magnetometer z: %f\n", mag_data.z);	

	status = lsm9ds1_close();
	if(status < 0) {
		fprintf(stderr, "Error closing lsm9ds1!\n");
	}

	return status;
}