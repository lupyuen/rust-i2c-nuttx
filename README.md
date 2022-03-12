# Rust talks I2C to BME280 on Apache NuttX RTOS

(Tested on Pine64 PineCone BL602)

Will Rust talk I2C on Apache NuttX RTOS for BL602? Let's find out!

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

This is how we read an I2C Register in C...

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

[(Source)](https://github.com/lupyuen/bme280-nuttx/blob/main/driver.c#L155-L183)

How do we call __I2C_TRANSFER__ from a NuttX App? Thanks to the I2C Demo App we have the answer...

```c
int i2ctool_get(FAR struct i2ctool_s *i2ctool, int fd, uint8_t regaddr,
                FAR uint16_t *result)
{
  struct i2c_msg_s msg[2];
  ...
  int ret = i2cdev_transfer(fd, msg, 2);
```

[(Source)](https://github.com/lupyuen/incubator-nuttx-apps/blob/rusti2c/system/i2c/i2c_get.c#L158-L206)

__i2cdev_transfer__ is defined as...

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

[(Source)](https://github.com/lupyuen/incubator-nuttx-apps/blob/rusti2c/system/i2c/i2c_devif.c#L117-L129)

Let's port this to Rust.

# C Types and Constants

Earlier we've seen __i2c_msg_s__ and __i2c_transfer_s__. They are defined as...

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

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L208-L215)

```c
struct i2c_transfer_s
{
  FAR struct i2c_msg_s *msgv; /* Array of I2C messages for the transfer */
  size_t msgc;                /* Number of messages in the array. */
};
```

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L231-L235)

__I2CIOC_TRANSFER__ is defined as...

```c
#define I2CIOC_TRANSFER      _I2CIOC(0x0001)
```

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L105-L129)

___I2CIOC__ is defined as...

```c
#define _I2CIOC(nr)       _IOC(_I2CBASE,nr)
```

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/fs/ioctl.h#L467-L468)

___IOC__ and ___I2CBASE__ are defined as...

```c
#define _IOC(type,nr)   ((type)|(nr))
```

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/fs/ioctl.h#L107)

```c
#define _I2CBASE        (0x2100) /* I2C driver commands */
```

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/fs/ioctl.h#L73)

We'll port these C Types and Constants to Rust as well.

# Read I2C Register in Rust

Here's how we read an I2C Register in Rust, ported from the above C code...

```rust
/// Test the I2C Port by reading an I2C Register
pub fn test_i2c() {
    println!("test_i2c");

    //  Open I2C Port
    let i2c = unsafe { 
        open(b"/dev/i2c0\0".as_ptr(), O_RDWR) 
    };
    assert!(i2c > 0);

    //  Read one I2C Register, starting at Device ID
    let mut start = [BME280_REG_ID ; 1];
    let mut buf   = [0u8 ; 1];

    //  Compose I2C Transfer
    let msg: [i2c_msg_s ; 2] = [
        //  First I2C Message: Send Register ID
        i2c_msg_s {
            frequency: BME280_FREQ,   //  I2C Frequency
            addr:      BME280_ADDR,   //  I2C Address
            buffer:    start.as_mut_ptr(),      //  Buffer to be sent
            length:    start.len() as ssize_t,  //  Length of the buffer in bytes

            //  For BL602: Register ID must be passed as I2C Sub Address
            #[cfg(target_arch = "riscv32")]  //  If architecture is RISC-V 32-bit...
            flags:     I2C_M_NOSTOP,  //  I2C Flags: Send I2C Sub Address
            
            //  Otherwise pass Register ID as I2C Data
            #[cfg(not(target_arch = "riscv32"))]  //  If architecture is not RISC-V 32-bit...
            flags:     0,  //  I2C Flags: None

            //  TODO: Check for BL602 specifically, not just RISC-V 32-bit
        },
        //  Second I2C Message: Receive Register Value
        i2c_msg_s {
            frequency: BME280_FREQ,  //  I2C Frequency
            addr:      BME280_ADDR,  //  I2C Address
            buffer:    buf.as_mut_ptr(),      //  Buffer to be received
            length:    buf.len() as ssize_t,  //  Length of the buffer in bytes
            flags:     I2C_M_READ,   //  I2C Flags: Read from I2C Device
        },
    ];

    //  Compose ioctl Argument
    let xfer = i2c_transfer_s {
        msgv: msg.as_ptr(),         //  Array of I2C messages for the transfer
        msgc: msg.len() as size_t,  //  Number of messages in the array
    };

    //  Execute I2C Transfer
    let ret = unsafe { 
        ioctl(
            i2c,
            I2CIOC_TRANSFER,
            &xfer
        )
    };
    assert!(ret >= 0);

    //  Show the received Register Value
    println!(
        "test_i2c: Register 0x{:02x} is 0x{:02x}",
        BME280_REG_ID,  //  Register ID (0xD0)
        buf[0]          //  Register Value (0x60)
    );

    //  Register Value must be BME280 Device ID (0x60)
    assert!(buf[0] == BME280_CHIP_ID);
     
    //  Close the I2C Port
    unsafe { close(i2c); }

    //  Sleep 5 seconds
    unsafe { sleep(5); }
}
```

[(Source)](src/test.rs)

# Test I2C Port

Yep our Rust app reads the I2C Register correctly!

```text
NuttShell (NSH) NuttX-10.2.0-RC0
nsh> rust_i2c
Hello from Rust!
test_i2c
i2cdrvr_ioctl: cmd=2101 arg=4201c378
bl602_i2c_transfer: subflag=1, subaddr=0xd0, sublen=1
bl602_i2c_recvdata: count=1, temp=0x60
bl602_i2c_transfer: i2c transfer success
test_i2c: Register 0xd0 is 0x60
Done!
nsh>
```
