#include <cjson/cJSON.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "lsm9ds1_config.h"

static lsm9ds1_status_t file_to_buf(const char *config_file, char *config_buffer) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

    FILE *fp = fopen(LSM9DS1_CONFIG, "r");

	if(fp != NULL) {
		if(fseek(fp, 0L, SEEK_END) == 0) {
			long bufsize = ftell(fp);
			if(bufsize == -1) {
				return LSM9DS1_CONFIG_FILE_SIZE_UKNOWN;
			}

			config_buffer = malloc(sizeof(char) * (bufsize +1));

			if(fseek(fp, 0L, SEEK_SET) != 0) {
				return LSM9DS1_CONFIG_FILE_SIZE_UKNOWN;
			}

			size_t new_len = fread(config_buffer, sizeof(char), bufsize, fp);
			if(ferror(fp) != 0) {
				return LSM9DS1_UNABLE_TO_READ_CONFIG;
			}
			else {
				config_buffer[new_len++] = '\0';
			}
		}

		fclose(fp);
	}
}

static lsm9ds1_status_t read_config(char *file_buffer) {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	// Check that the file is there
	if(access(LSM9DS1_CONFIG, F_OK) != -1) {
	    // file exists
	    status = file_to_buf(LSM9DS1_CONFIG, file_buffer);
	    if(status < 0) {return status;}
	}
	else {
	    // file doesn't exist
	    return LSM9DS1_CONFIG_FILE_NOT_FOUND;
	}

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t parse_json(lsm9ds1_config_t *lsm9ds1_config) {

	const cJSON *name = NULL;
	const cJSON *bus = NULL;
	const cJSON *device = NULL;
	const cJSON *spi = NULL;
	const cJSON *spi_settings = NULL;
	const cJSON *spi_speed = NULL;
	size_t config_string_size = 0;

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
	char *config = NULL;

	status = read_config(config);
	if(status < 0) {return status;}
	free(config);

	cJSON *config_json = cJSON_Parse(config);
	if (config_json == NULL) {
		return LSM9DS1_UNABLE_TO_PARSE_JSON;
	}

	name = cJSON_GetObjectItemCaseSensitive(config_json, "name");
	if(cJSON_IsString(name) && (name->valuestring != NULL))
    {
    	config_string_size = 0;
    	config_string_size = sizeof(name->valuestring);
    	if(config_string_size > LSM9DS1_MAX_STR_SIZE) {
    		return LSM9DS1_UNABLE_TO_PARSE_JSON;
    	}
    	strncpy(lsm9ds1_config->name, name->valuestring, config_string_size);
    }

	bus = cJSON_GetObjectItemCaseSensitive(config_json, "bus");
	device = cJSON_GetObjectItemCaseSensitive(bus, "device");
	if(cJSON_IsString(device) && (device->valuestring != NULL))
    {
    	config_string_size = 0;
    	config_string_size = sizeof(device->valuestring);
    	if(config_string_size > LSM9DS1_MAX_STR_SIZE) {
    		return LSM9DS1_UNABLE_TO_PARSE_JSON;
    	}
    	strncpy(lsm9ds1_config->bus.device, device->valuestring, config_string_size);
    }

	spi = cJSON_GetObjectItemCaseSensitive(bus, "spi");
	spi_settings = cJSON_GetObjectItemCaseSensitive(spi, "settings");
	spi_speed = cJSON_GetObjectItemCaseSensitive(spi_settings, "speed");
	if(cJSON_IsNumber(spi_speed))
    {
    	lsm9ds1_config->bus.spi.settings.speed = spi_speed->valueint;
    }

    cJSON_Delete(config_json);

	return LSM9DS1_SUCCESS;
}
