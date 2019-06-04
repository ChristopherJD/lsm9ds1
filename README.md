# lsm9ds1
C and Python extension library for the LSM9DS1

# Building

1. You MUST have the SDK sourced to create a cross-compiled build for the raspberrypi system. (If you don't indend to build for this system you can skip this step.)

```bash
source /opt/poky/2.6.2/environment-setup-cortexa7t2hf-neon-vfpv4-poky-linux-gnueabi
```

2. Use the build shell script. This will use cmake and make to build for your system.
```bash
./build.sh
```
