#include <cjson/cJSON.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "lsm9ds1_config.h"
#include "lsm9ds1_debug.h"

// Default Sensor Settings if not otherwise set.
static const lsm9ds1_config_t gdefault_config = {
	.name = "lsm9ds1",
	.xfer_bus = LSM9DS1_SPI_BUS,
	.sub_device = {
		.accelerometer = {
			.bus = {
				.spi = {
					.name = "/dev/spidev0.0",
					.settings = {
						.speed = 15000000,
					}
				},
			}
		},
		.magnetometer = {
			.bus = {
				.spi = {
					.name = "/dev/spidev0.1",
					.settings = {
						.speed = 15000000,
					}
				},
			}
		}
	}
};

lsm9ds1_config_t glsm9ds1_config = {0};

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

static lsm9ds1_status_t parse_sub_device_spi(const cJSON *json, const char *sub_device_name, lsm9ds1_bus_t *bus) {
	const cJSON *sub_device = NULL;	
	const cJSON *spi = NULL;
	const cJSON *device_name = NULL;
	const cJSON *speed = NULL;
	size_t config_string_size = 0;

	sub_device = cJSON_GetObjectItemCaseSensitive(json, sub_device_name);
	if(!cJSON_IsObject(sub_device)) {
		return LSM9DS1_SUCCESS;	
	}

	spi = cJSON_GetObjectItemCaseSensitive(sub_device, "spi");
	if(!cJSON_IsObject(spi)) {
		return LSM9DS1_SUCCESS;	
	}

	device_name = cJSON_GetObjectItemCaseSensitive(spi, "device");
	if(cJSON_IsString(device_name) && (device_name->valuestring != NULL))
    {
    	config_string_size = 0;
    	config_string_size = strlen(device_name->valuestring) + 1;
    	if(config_string_size > LSM9DS1_MAX_STR_SIZE) {
			return LSM9DS1_UNABLE_TO_PARSE_JSON;
    	}
	    strcpy(bus->spi.name, device_name->valuestring);
	    DEBUG_PRINT("%s configured for %s.\n", sub_device_name, bus->spi.name);
    }

	speed = cJSON_GetObjectItemCaseSensitive(spi, "speed");
	if(cJSON_IsNumber(speed))
    {
    	if(speed->valueint > MAX_SPI_SPEED) {
    		//Keep the default value. Return early.
    		return LSM9DS1_INVALID_SETTING;
    	}
    	bus->spi.settings.speed = speed->valueint;
	    DEBUG_PRINT("%s configured spi speed at %d.\n", sub_device_name, bus->spi.settings.speed);
    }

    return LSM9DS1_SUCCESS;
}

static lsm9ds1_status_t parse_json(lsm9ds1_config_t *lsm9ds1_config) {

	const cJSON *name = NULL;
	const cJSON *xfer_bus = NULL;
	const cJSON *sub_device = NULL;
	size_t config_string_size = 0;

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
	char *config = NULL;

	config = read_config();
	if(NULL == config) {return LSM9DS1_UNABLE_TO_PARSE_JSON;}

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

	xfer_bus = cJSON_GetObjectItemCaseSensitive(config_json, "xfer_bus");
	if(cJSON_IsNumber(xfer_bus))
    {
    	lsm9ds1_config->xfer_bus = xfer_bus->valueint;
	    DEBUG_PRINT("lsm9ds1 bus: %d\n", xfer_bus->valueint);
    }

	sub_device = cJSON_GetObjectItemCaseSensitive(config_json, "sub_device");
	status = parse_sub_device_spi(sub_device, "accelerometer", &(lsm9ds1_config->sub_device.accelerometer.bus));
	if(status < 0) { return status;}

	status = parse_sub_device_spi(sub_device, "magnetometer", &(lsm9ds1_config->sub_device.magnetometer.bus));
	if(status < 0) { return status;}

    cJSON_Delete(config_json);

	return LSM9DS1_SUCCESS;
}

lsm9ds1_status_t init_config() {

	lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;

	//Copy defaults
	glsm9ds1_config = gdefault_config;

	DEBUG_PRINT("Opened config file: %s\n", LSM9DS1_CONFIG);
	DEBUG_PRINT("Configuration for %s\n", glsm9ds1_config.name);

	// Fall back to default settings if we fail to read the config for some
	// reason. If an invalid settings is found, just use the default.
	status = parse_json(&glsm9ds1_config);
	if((status < 0) && (status != LSM9DS1_INVALID_SETTING)) {
		DEBUG_PRINT("Parse config status: %d\n", status);
		glsm9ds1_config = gdefault_config;
		return status;
	}

	return LSM9DS1_SUCCESS;
}
