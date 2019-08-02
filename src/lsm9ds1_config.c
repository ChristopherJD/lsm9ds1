#include <cjson/cJSON.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "lsm9ds1_config.h"
#include "lsm9ds1_debug.h"

char *file_to_buf(const char *config_file) {

	char *config_buffer = NULL;

    FILE *fp = fopen(LSM9DS1_CONFIG, "r");

	if(fp != NULL) {
		if(fseek(fp, 0L, SEEK_END) == 0) {
			long bufsize = ftell(fp);
			if(bufsize == -1) {
				return NULL;
			}

			config_buffer = malloc(sizeof(char) * (bufsize +1));

			if(fseek(fp, 0L, SEEK_SET) != 0) {
				return NULL;
			}

			size_t new_len = fread(config_buffer, sizeof(char), bufsize, fp);
			if(ferror(fp) != 0) {
				return NULL;
			}
			else {
				config_buffer[new_len++] = '\0';
			}
		}

		fclose(fp);
	}

	return config_buffer;
}

static char *read_config() {

	char *file_buffer = NULL;

	// Check that the file is there
	if(access(LSM9DS1_CONFIG, F_OK) != -1) {
	    // file exists
	    file_buffer = file_to_buf(LSM9DS1_CONFIG);
	    if(NULL == file_buffer) {return NULL;}
	}
	else {
	    // file doesn't exist
	    DEBUG_PRINT("File %s not found!\n");
	    return NULL;
	}

	return file_buffer;
}

static lsm9ds1_status_t parse_sub_device_spi(const cJSON *json, const char *sub_device_name, struct lsm9ds1_sub_device *bus) {
	const cJSON *sub_device = NULL;	
	const cJSON *spi = NULL;
	const cJSON *device_name = NULL;
	const cJSON *speed = NULL;
	size_t config_string_size = 0;

	sub_device = cJSON_GetObjectItemCaseSensitive(sub_device, sub_device_name);
	spi = cJSON_GetObjectItemCaseSensitive(sub_device, "spi");
	device_name = cJSON_GetObjectItemCaseSensitive(spi, "device");

	if(cJSON_IsString(device_name) && (device_name->valuestring != NULL))
    {

    	config_string_size = 0;
    	config_string_size = strlen(device_name->valuestring) + 1;
    	if(config_string_size > LSM9DS1_MAX_STR_SIZE) {
    		return LSM9DS1_UNABLE_TO_PARSE_JSON;
    	}
	    strcpy(bus->spi.device, device_name->valuestring);
    }

	speed = cJSON_GetObjectItemCaseSensitive(spi, "speed");
	if(cJSON_IsNumber(speed))
    {
    	bus->spi.speed = speed->valueint;
    }

    return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t parse_json(lsm9ds1_config_t *lsm9ds1_config) {

	const cJSON *name = NULL;
	const cJSON *sub_device = NULL;
	size_t config_string_size = 0;

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
	char *config = NULL;

	config = read_config();
	if(NULL == config) {return status;}

	cJSON *config_json = cJSON_Parse(config);
	if (config_json == NULL) {
		return LSM9DS1_UNABLE_TO_PARSE_JSON;
	}

	free(config);

	DEBUG_PRINT("Parsed configuration...\n");

	name = cJSON_GetObjectItemCaseSensitive(config_json, "name");
	if(cJSON_IsString(name) && (name->valuestring != NULL))
    {
    	config_string_size = 0;
    	config_string_size = strlen(name->valuestring) + 1;
    	if(config_string_size > LSM9DS1_MAX_STR_SIZE) {
    		return LSM9DS1_UNABLE_TO_PARSE_JSON;
    	}
    	strcpy(lsm9ds1_config->name, name->valuestring);
    }

	sub_device = cJSON_GetObjectItemCaseSensitive(config_json, "sub_device");
	parse_sub_device_spi(sub_device, "accelerometer", &(lsm9ds1_config->sub_device.accelerometer));
	parse_sub_device_spi(sub_device, "magnetometer", &(lsm9ds1_config->sub_device.magnetometer));

    cJSON_Delete(config_json);

	return LSM9DS1_SUCCESS;
}
