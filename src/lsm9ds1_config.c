#include <cjson/cJSON.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include "lsm9ds1_config.h"
#include "lsm9ds1_error.h"

char *file_to_buf(const char *config_file) {

	char *source = NULL;

    FILE *fp = fopen(LSM9DS1_CONFIG, "r");

	if(fp != NULL) {
		if(fseek(fp, 0L, SEEK_END) == 0) {
			long bufsize = ftell(fp);
			if(bufsize == -1) {

			}

			source = malloc(sizeof(char) * (bufsize +1));

			if(fseek(fp, 0L, SEEK_SET) != 0) {

			}

			size_t new_len = fread(source, sizeof(char), bufsize, fp);
			if(ferror(fp) != 0) {
				//Error reading file
			}
			else {
				source[new_len++] = '\0';
			}
		}

		fclose(fp);
	}
}

lsm9ds1_status_t read_config(char *file_buffer) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Check that the file is there
	if(access(LSM9DS1_CONFIG, F_OK) != -1) {
	    // file exists
	    file_buffer = file_to_buf(LSM9DS1_CONFIG);
	}
	else {
	    // file doesn't exist
	    return LSM9DS1_CONFIG_FILE_NOT_FOUND;
	}

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t parse_json() {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
	char *file_buffer = NULL;

	status = read_config(file_buffer);
	if(status < 0) {return status;}

	free(file_buffer);

	return LSM9DS1_SUCCESS;
}
