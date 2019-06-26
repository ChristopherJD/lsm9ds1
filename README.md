# lsm9ds1
C and Python extension library for the LSM9DS1

# API Documentation

You can find the documentation [here](https://christopherjd.github.io/lsm9ds1/html/index.html).

# Building

1. You MUST have the SDK sourced to create a cross-compiled build for the raspberrypi system. (If you don't intend to build for this system you can skip this step.)

```bash
source /opt/poky/2.6.2/environment-setup-cortexa7t2hf-neon-vfpv4-poky-linux-gnueabi
```

2. Use the build shell script. This will use cmake and make to build for your system.

If building for release.

```bash
./build.sh release
```

If building for debugging purposes.

```bash
./build.sh debug
```

# Debugging

You can remotely debug the application on the raspberry pi using gdbserver. A nice application to aid in debugging is gdbgui. You can install from the instructions foun [here](https://www.gdbgui.com/installation/). In ubuntu also make sure you install gdb-multiarch if using ubuntu.

```bash
sudo apt install gdb-multiarch
```

1. On the remote system, start the server.

```bash
gdbserver localhost:5000 lsm9ds1_test
```

2. On the local machine start gdbgui.

```bash
gdbgui -r -g gdb-multiarch
```

