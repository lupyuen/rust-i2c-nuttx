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

# From C to Rust

TODO

https://github.com/lupyuen/bme280-nuttx/blob/main/driver.c#L155-L183

```c
static int bme280_reg_read(const struct device *priv,
    uint8_t start, uint8_t *buf, int size)
{
  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(buf != NULL);
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;

#ifdef CONFIG_BL602_I2C0
  //  For BL602: Register ID must be passed as I2C Sub Address
  msg[0].flags     = I2C_M_NOSTOP;
#else
  //  Otherwise pass Register ID as I2C Data
  msg[0].flags     = 0;
#endif  //  CONFIG_BL602_I2C0

  msg[0].buffer    = &start;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = size;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
```

How do we call __I2C_TRANSFER__ from a NuttX App? Thanks to the I2C Demo App we have the answer...

https://github.com/lupyuen/incubator-nuttx-apps/blob/rusti2c/system/i2c/i2c_get.c#L158-L206

```c
int i2ctool_get(FAR struct i2ctool_s *i2ctool, int fd, uint8_t regaddr,
                FAR uint16_t *result)
{
  struct i2c_msg_s msg[2];
  ...
  int ret = i2cdev_transfer(fd, msg, 2);
```

__i2cdev_transfer__ is defined as...

https://github.com/lupyuen/incubator-nuttx-apps/blob/rusti2c/system/i2c/i2c_devif.c#L117-L129

```c
int i2cdev_transfer(int fd, FAR struct i2c_msg_s *msgv, int msgc)
{
  struct i2c_transfer_s xfer;

  /* Set up the IOCTL argument */

  xfer.msgv = msgv;
  xfer.msgc = msgc;

  /* Perform the IOCTL */

  return ioctl(fd, I2CIOC_TRANSFER, (unsigned long)((uintptr_t)&xfer));
}
```

Let's port this to Rust.

# C Types and Constants

Earlier we've seen __i2c_msg_s__ and __i2c_transfer_s__. They are defined as...

https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L208-L215

```c
struct i2c_msg_s
{
  uint32_t frequency;         /* I2C frequency */
  uint16_t addr;              /* Slave address (7- or 10-bit) */
  uint16_t flags;             /* See I2C_M_* definitions */
  FAR uint8_t *buffer;        /* Buffer to be transferred */
  ssize_t length;             /* Length of the buffer in bytes */
};
```

https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L231-L235

```c
struct i2c_transfer_s
{
  FAR struct i2c_msg_s *msgv; /* Array of I2C messages for the transfer */
  size_t msgc;                /* Number of messages in the array. */
};
```

__I2CIOC_TRANSFER__ is defined as...

https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L113-L119

```c
#define I2CIOC_TRANSFER      _I2CIOC(0x0001)
```

___I2CIOC__ is defined as...

https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/fs/ioctl.h#L467-L468

```c
#define _I2CIOC(nr)       _IOC(_I2CBASE,nr)
```

___IOC__ and ___I2CBASE__ are defined as...

https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/fs/ioctl.h#L107

```c
#define _IOC(type,nr)   ((type)|(nr))
```

https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/fs/ioctl.h#L73

```c
#define _I2CBASE        (0x2100) /* I2C driver commands */
```

We'll port these C Types and Constants to Rust too.
