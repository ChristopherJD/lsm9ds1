# LSM9DS1 C/C++ Library

![lsm9ds1_axes.png](https://github.com/ChristopherJD/lsm9ds1/blob/master/docs/lsm9ds1_axes.png)

## About

Library for the LSM9DS1 from ST Microelectronics. Library makes use of the Linux platform device driver "spidev". The library was originally written for the RaspberryPi B+.

The LSM9DS1 provides 9 degrees of freedom (9-DOF), a 3 axis accelerometer, gyroscope and magnetometer. The accelerometer supports ± 2, 4, 8, or 16 g, the gyroscope supports ± 245, 500, and 2000 °/s, and the magnetometer has full-scale ranges of ± 2, 4, 12, or 16 gauss. It is equipped with a digital interface supporting both I2C and SPI.

## Install

Please see the [releases](https://github.com/ChristopherJD/lsm9ds1/releases) to obtain prebuilt versions for
the raspberrypi 3B+. Always use the latest version as only alpha version are released.

**Install using RPM**

Remember to replace v0.4.0-alpha with the appropriate version.

```sh
rpm -ivh liblsm9ds1-v0.4.0-alpha-raspberrypi3.rpm
```

## Uninstall

**Uninstall using RPM**

Remember to replace v0.4.0-alpha-1 with the appropriate version.

```sh
rpm -e liblsm9ds1-v0.4.0_alpha-1
```

## API Documentation

You can find the documentation [here](https://christopherjd.github.io/lsm9ds1/html/index.html).

The API is simple to use. Here is a quick example of reading from the accelerometer.

### C API

```c
#include <lsm9ds1.h>
#include <stdio.h>
#include <stdlib.h>

int main() {

    lsm9ds1_status_t status = LSM9DS1_UNKNOWN_ERROR;
    lsm9ds1_device_t lsm9ds1 = {0};

    status = lsm9ds1_init(&lsm9ds1, LSM9DS1_SPI_BUS, LSM9DS1_ACCELRANGE_8G, LSM9DS1_MAGGAIN_8GAUSS, LSM9DS1_GYROSCALE_500DPS);
    if(status < 0) {
        fprinf(stderr, "Error initializing lsm9ds1!\n");
        exit(EXIT_FAILURE);
    }
    status = lsm9ds1.update_accel(lsm9ds1);
    if(status < 0) {
   		fprintf(stderr, "Error reading accelerometer!\n");
        exit(EXIT_FAILURE);
    }
    
    printf("Accelerometer x: %f\n", lsm9ds1.converted_data.accelerometer.x);
    printf("Accelerometer y: %f\n", lsm9ds1.converted_data.accelerometer.y);
    printf("Accelerometer z: %f\n", lsm9ds1.converted_data.accelerometer.z);
    
    return EXIT_SUCCESS;
}
```

## Dependencies

This library depends on the [cjson](https://github.com/DaveGamble/cJSON) library by Dave Gamble. Please follow the installation steps to build for your system.

## Building

There are 2 options when building the LSM9DS1 Library.

* Build for release
    When building for release you will build the source code, doxygen documentation, CUnit tests, and package for the RPM format.
* Build for debug
    When building for debug purposes you will build the source code with debugging symbols and additional print statements as well as the CUnit tests.

1. You MUST have the SDK sourced to create a cross-compiled build for the raspberrypi system. (If you don't intend to build for this system you can skip this step.)

    ```bash
    source /opt/poky/2.6.2/environment-setup-cortexa7t2hf-neon-vfpv4-poky-linux-gnueabi
    ```

1. Use the build shell script. This will use cmake and make to build for your system. 

    If building for release.

    ```bash
    ./build.sh release
    ```

    If building for debugging purposes.

    ```bash
    ./build.sh debug
    ```
    
## Testing

This project uses the the CUnit Testing Framwork. Tests can be run by running the lms9ds1_test program installed in the `/usr/bin` directory.

## Debugging

You can remotely debug the application on the raspberry pi using gdbserver. A nice application to aid in debugging is gdbgui. You can install from the instructions foun [here](https://www.gdbgui.com/installation/). In ubuntu also make sure you install gdb-multiarch if using ubuntu.

```bash
sudo apt install gdb-multiarch
```

1. On the remote system, start the server.

    ```bash
    gdbserver localhost:5000 lsm9ds1_test
    ```

1. On the local machine start gdbgui.

    ```bash
    gdbgui -r -g gdb-multiarch
    ```

## RaspBerryPi 3 B + Schematic

![rpi_wiring_schematic.png](https://github.com/ChristopherJD/lsm9ds1/blob/master/docs/rpi_wiring_schematic.png)

### LSM9DS1 Pinout

| Pin Label | Pin Function                                            | Notes                                                                                                                                                                                                        |
|-----------|---------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| GND       | Ground                                                  | 0V voltage supply                                                                                                                                                                                            |
| VDD       | Power Supply                                            | Supply voltage to the chip. Should be regulated between 2.4V and 3.6V.                                                                                                                                       |
| SDA       | SPI: MOSI I2C: Serial Data                              | SPI: Device data in (MOSI) I2C: Serial data (bi-directional)                                                                                                                                                 |
| SCL       | Serial Clock                                            | I2C and SPI serial clock.                                                                                                                                                                                    |
| DEN       | Gyroscope Data Enable                                   | Mostly unknown. The LSM9DS1 datasheet doesn't have much to say about this pin.                                                                                                                               |
| INT2      | Accel/Gyro Interrupt 2                                  | INT1 and INT2 are programmable interrupts for the accelerometer and gyroscope. They can be set to alert on over/under thresholds, data ready, or FIFO overruns.                                              |
| INT1      | Accel/Gyro Interrupt 1                                  |                                                                                                                                                                                                              |
| INTM      | Magnetometer Interrupt                                  | A programmable interrupt for the magnetometer. Can be set to alert on over-under thresholds.                                                                                                                 |
| RDY       | Magnetometer Data Ready                                 | An interrupt indicating new magnetometer data is available. Non-programmable.                                                                                                                                |
| CS M      | Magnetometer Chip Select                                | This pin selects between I2C and SPI on the magnetometer. Keep it HIGH for I2C, or use it as an (active-low) chip select for SPI. HIGH (1): SPI idle mode / I2C enabled LOW (0): SPI enabled / I2C disabled. |
| CS AG     | Accel/Gyro Chip Select                                  | This pin selects between I2C and SPI on the accel/gyro. Keep it HIGH for I2C, or use it as an (active-low) chip select for SPI. HIGH (1): SPI idle mode / I2C enabled LOW (0): SPI enabled / I2C disabled.   |
| SDO M     | SPI: Magnetometer MISO I2C: Magnetometer Address Select | In SPI mode, this is the magnetometer data output (SDO_M). In I2C mode, this selects the LSb of the I2C address (SA0_M)                                                                                      |
| SDO AG    | SPI: Accel/Gyro MISO I2C: Accel/Gryo Address Select     | In SPI mode, this is the accel/gryo data output (SDO_AG). In I2C mode, this selects the LSb of the I2C address (SA0_AG)                                                                                      |

