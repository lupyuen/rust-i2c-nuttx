# Rust talks I2C to BME280 on Apache NuttX RTOS

(Tested on Pine64 PineCone BL602)

This repo depends on...

-   [lupyuen/rust-nuttx](https://github.com/lupyuen/rust-nuttx)

# Install App

To add this repo to your NuttX project...

```bash
## TODO: Change this to the path of our "incubator-nuttx-apps" folder
cd nuttx/apps/examples
git submodule add https://github.com/lupyuen/rust-i2c-nuttx rust_i2c
```

Then update the NuttX Build Config...

```bash
## TODO: Change this to the path of our "incubator-nuttx" folder
cd nuttx/nuttx

## Preserve the Build Config
cp .config ../config

## Erase the Build Config
make distclean

## For BL602: Configure the build for BL602
./tools/configure.sh bl602evb:nsh

## For ESP32: Configure the build for ESP32.
## TODO: Change "esp32-devkitc" to our ESP32 board.
./tools/configure.sh esp32-devkitc:nsh

## Restore the Build Config
cp ../config .config

## Edit the Build Config
make menuconfig 
```

In menuconfig, enable the Rust I2C App under "Application Configuration" → "Examples".

In NuttX Shell, enter this to run the app...

```bash
rust_i2c
```
