# Rust talks I2C to Bosch BME280 Sensor on Apache NuttX RTOS

(Tested on Pine64 PineCone BL602)

Read the articles...

-   ["Rust talks I2C on Apache NuttX RTOS"](https://lupyuen.github.io/articles/rusti2c)

-   ["Rust on Apache NuttX OS"](https://lupyuen.github.io/articles/rust2)

This repo depends on...

-   [lupyuen/rust-nuttx](https://github.com/lupyuen/rust-nuttx)

# Install App

To add this repo to your NuttX project...

```bash
## TODO: Change this to the path of our "incubator-nuttx-apps" folder
pushd nuttx/apps/examples
git submodule add https://github.com/lupyuen/rust-i2c-nuttx rust_i2c
popd
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

To build the NuttX + Rust project...

```bash
cd nuttx/apps/examples/rust_i2c
./run.sh
```

In NuttX Shell, enter this to run the app...

```bash
rust_i2c
```

Here's the output...

```text
NuttShell (NSH) NuttX-10.2.0-RC0
nsh> rust_i2c
Hello from Rust!
test_i2c
test_i2c: Register 0xd0 is 0x60

test_hal_read
test_hal_read: Register 0xd0 is 0x60

test_hal_write
test_hal_write: Write 0xA0 to register
test_hal_write: Register value is 0xa0
test_hal_write: Write 0x00 to register
test_hal_write: Register value is 0x00

read_bme280
Relative Humidity = 89.284164%
Temperature = 29.942907 deg C
Pressure = 100483.04 pascals
Done!
nsh>
```

# Rust Embedded HAL for NuttX

The Rust Embedded HAL for NuttX has been published at crates.io...

https://crates.io/crates/nuttx-embedded-hal

To use it in your project, add this to your [Cargo.toml](rust/Cargo.toml):

```text
# External Rust libraries used by this module.  See crates.io.
[dependencies]
nuttx-embedded-hal = "1.0.6"  # Rust Embedded HAL for NuttX: https://crates.io/crates/nuttx-embedded-hal
```

[(Always use the latest version of __nuttx-embedded-hal__)](https://crates.io/crates/nuttx-embedded-hal)

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

[(Source)](rust/src/test.rs)

The NuttX Types are ported from C to Rust like so...

```rust
/// I2C Message Struct: I2C transaction segment beginning with a START. A number of these can
/// be transferred together to form an arbitrary sequence of write/read
/// transfer to an I2C device.
/// TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L208-L215
#[repr(C)]
pub struct i2c_msg_s {
    /// I2C Frequency
    pub frequency: u32,
    /// I2C Address
    pub addr: u16,
    /// I2C Flags (I2C_M_*)
    pub flags: u16,
    /// Buffer to be transferred
    pub buffer: *mut u8,
    /// Length of the buffer in bytes
    pub length: ssize_t,
}

/// I2C Transfer Struct: This structure is used to communicate with the I2C character driver in
/// order to perform IOCTL transfers.
/// TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L231-L235
#[repr(C)]
pub struct i2c_transfer_s {
    /// Array of I2C messages for the transfer
    pub msgv: *const i2c_msg_s,
    /// Number of messages in the array
    pub msgc: size_t,
}
```

[(Source)](https://github.com/lupyuen/nuttx-embedded-hal/blob/main/src/lib.rs#L125-L153)

# Test I2C Port

To build the NuttX + Rust project...

```bash
cd nuttx/apps/examples/rust_i2c
./run.sh
```

In NuttX Shell, enter this to run our Rust app...

```bash
rust_i2c
```

Our Rust app reads BME280 Register 0xD0 (Device ID), which should contain 0x60...

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

Yep our Rust app reads the BME280 I2C Register correctly!

# Rust Embedded HAL

Rust Embedded HAL defines a standard API for I2C Operations. Let's wrap the NuttX I2C ioctl() Commands and expose as Rust Embedded HAL interfaces...

```rust
/// NuttX Implementation of I2C Read
impl i2c::Read for I2c {
    ...
    /// TODO: Read I2C data
    fn read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Self::Error> { ... }
}

/// NuttX Implementation of I2C Write
impl i2c::Write for I2c {
    ...
    /// TODO: Write I2C data
    fn write(&mut self, addr: u8, buf: &[u8]) -> Result<(), Self::Error> { ... }
}

/// NuttX Implementation of I2C WriteRead
impl i2c::WriteRead for I2c {
    ...
    /// TODO: Write and read I2C data
    fn write_read(&mut self, addr: u8, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> { ... }
}
```

[(Source)](https://github.com/lupyuen/nuttx-embedded-hal/blob/main/src/hal.rs#L20-L160)

# Read I2C Register

Here's how we implement the Rust Embedded HAL to read an I2C Register...

```rust
/// NuttX Implementation of I2C WriteRead
impl i2c::WriteRead for I2c {
    /// Error Type
    type Error = i32;

    /// Write `wbuf` to I2C Port and read `rbuf` from I2C Port.
    /// We assume this is a Read I2C Register operation, with Register ID at `wbuf[0]`.
    /// TODO: Handle other kinds of I2C operations
    fn write_read(&mut self, addr: u8, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        //  We assume this is a Read I2C Register operation, with Register ID at wbuf[0]
        assert_eq!(wbuf.len(), 1);
        let reg_id = wbuf[0];

        //  Read I2C Registers, starting at Register ID
        let mut start = [reg_id ; 1];

        //  Compose I2C Transfer
        let msg = [
            //  First I2C Message: Send Register ID
            i2c_msg_s {
                frequency: self.frequency,  //  I2C Frequency
                addr:      addr as u16,     //  I2C Address
                buffer:    start.as_mut_ptr(),      //  Buffer to be sent
                length:    start.len() as ssize_t,  //  Number of bytes to send

                //  For BL602: Register ID must be passed as I2C Sub Address
                #[cfg(target_arch = "riscv32")]  //  If architecture is RISC-V 32-bit...
                flags:     crate::I2C_M_NOSTOP,  //  I2C Flags: Send I2C Sub Address
                
                //  Otherwise pass Register ID as I2C Data
                #[cfg(not(target_arch = "riscv32"))]  //  If architecture is not RISC-V 32-bit...
                flags:     0,  //  I2C Flags: None

                //  TODO: Check for BL602 specifically (by target_abi?), not just RISC-V 32-bit
            },
            //  Second I2C Message: Receive Register Values
            i2c_msg_s {
                frequency: self.frequency,  //  I2C Frequency
                addr:      addr as u16,     //  I2C Address
                buffer:    rbuf.as_mut_ptr(),      //  Buffer to be received
                length:    rbuf.len() as ssize_t,  //  Number of bytes to receive
                flags:     I2C_M_READ,  //  I2C Flags: Read I2C Data
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
                self.fd,
                I2CIOC_TRANSFER,
                &xfer
            )
        };
        assert!(ret >= 0);   
        Ok(())
    }
}
```

[(Source)](https://github.com/lupyuen/nuttx-embedded-hal/blob/main/src/hal.rs#L98-L160)

To read an I2C Register, we call the Rust Embedded HAL like so...

```rust
/// Test the I2C HAL by reading an I2C Register
pub fn test_hal_read() {

    //  Open I2C Port
    let mut i2c = nuttx_embedded_hal::I2c::new(
        "/dev/i2c0",  //  I2C Port
        BME280_FREQ,  //  I2C Frequency
    ).expect("open failed");

    //  Buffer for received I2C data
    let mut buf = [0 ; 1];

    //  Read one I2C Register, starting at Device ID
    i2c.write_read(
        BME280_ADDR as u8,  //  I2C Address
        &[BME280_REG_ID],   //  Register ID (0x60)
        &mut buf            //  Buffer to be received
    ).expect("read register failed");

    //  Show the received Register Value
    println!(
        "test_hal_read: Register 0x{:02x} is 0x{:02x}",
        BME280_REG_ID,  //  Register ID (0xD0)
        buf[0]          //  Register Value (0x60)
    );

    //  Register Value must be BME280 Device ID (0x60)
    assert_eq!(buf[0], BME280_CHIP_ID);
}
```

[(Source)](rust/src/test.rs)

# Test I2C HAL

Rust Embedded HAL works OK for reading an I2C Register!

```text
NuttShell NSH NuttX-10.2.0-RC0
nsh> rust_i2c
Hello from Rust!
...
test_hal_read
i2cdrvr_ioctl: cmd=2101 arg=4201c360
bl602_i2c_transfer: subflag=1, subaddr=0xd0, sublen=1
bl602_i2c_recvdata: count=1, temp=0x60
bl602_i2c_transfer: i2c transfer success
test_hal_read: Register 0xd0 is 0x60
Done!
nsh>
```

# Write I2C Register

This code calls the Rust Embedded HAL to write the value 0xA0 to the I2C Register 0xF5....

```rust
/// Test the I2C HAL by writing an I2C Register
pub fn test_hal_write() {

    //  Open I2C Port
    let mut i2c = nuttx_embedded_hal::I2c::new(
        "/dev/i2c0",  //  I2C Port
        BME280_FREQ,  //  I2C Frequency
    ).expect("open failed");

    //  Write 0xA0 to register 0xF5
    i2c.write(
        BME280_ADDR as u8,          //  I2C Address
        &[BME280_REG_CONFIG, 0xA0]  //  Register ID and value
    ).expect("write register failed");
```

[(Source)](rust/src/test.rs)

But the Logic Analyser shows that BL602 is writing to I2C the value 0x00 instead of 0xA0...

```text
Setup Write to [0xEE] + ACK
0xF5 + ACK
0x00 + ACK
```

![BL602 is writing to I2C the value 0x00 instead of 0xA0](https://lupyuen.github.io/images/rusti2c-logic1.png)

Let's fix this. Here's the log for the I2C write...

```text
nsh> rust_i2c
Hello from Rust!
test_hal_write
i2cdrvr_ioctl: cmd=2101 arg=4201c370
bl602_i2c_transfer: subflag=1, subaddr=0xf5, sublen=1
bl602_i2c_send_data: count=1, temp=0xa0
bl602_i2c_transfer: i2c transfer success
test_hal_write: Write 0xA0 to register
```

# Fix I2C Write

BL602 has a peculiar I2C Port that uses I2C Sub Addresses ... Let's make it work with Rust Embedded HAL

["Quirks in BL602 I2C Driver"](https://lupyuen.github.io/articles/bme280#appendix-quirks-in-bl602-nuttx-i2c-driver)

We tried all sequences of I2C Read / Write / Sub Address. Only this strange sequence works for writing to I2C Registers...

1.  Write I2C Register ID and I2C Data together as I2C Sub Address

1.  Followed by Read I2C Data

```rust
/// NuttX Implementation of I2C Write
impl i2c::Write for I2c {
    /// Error Type
    type Error = i32;

    /// Write `buf` to I2C Port.
    /// We assume this is a Write I2C Register operation, with Register ID at `buf[0]`.
    /// TODO: Handle other kinds of I2C operations
    fn write(&mut self, addr: u8, buf: &[u8]) -> Result<(), Self::Error> {
        //  Copy to local buffer because we need a mutable reference
        let mut buf2 = [0 ; 64];
        assert!(buf.len() <= buf2.len());
        buf2[..buf.len()].copy_from_slice(buf);

        //  Buffer for received I2C data
        let mut rbuf = [0 ; 1];

        //  Compose I2C Transfer
        let msg = [
            //  First I2C Message: Send Register ID and I2C Data as I2C Sub Address
            i2c_msg_s {
                frequency: self.frequency,  //  I2C Frequency
                addr:      addr as u16,     //  I2C Address
                buffer:    buf2.as_mut_ptr(),     //  Buffer to be sent
                length:    buf.len() as ssize_t,  //  Number of bytes to send

                //  For BL602: Register ID must be passed as I2C Sub Address
                #[cfg(target_arch = "riscv32")]  //  If architecture is RISC-V 32-bit...
                flags:     crate::I2C_M_NOSTOP,  //  I2C Flags: Send I2C Sub Address
                
                //  Otherwise pass Register ID as I2C Data
                #[cfg(not(target_arch = "riscv32"))]  //  If architecture is not RISC-V 32-bit...
                flags:     0,  //  I2C Flags: None

                //  TODO: Check for BL602 specifically (by target_abi?), not just RISC-V 32-bit
            },
            //  Second I2C Message: Read I2C Data, because this forces BL602 to send the first message correctly
            i2c_msg_s {
                frequency: self.frequency,  //  I2C Frequency
                addr:      addr as u16,     //  I2C Address
                buffer:    rbuf.as_mut_ptr(),      //  Buffer to be received
                length:    rbuf.len() as ssize_t,  //  Number of bytes to receive
                flags:     I2C_M_READ,  //  I2C Flags: Read I2C Data
            },
        ];
        
        //  Compose ioctl Argument to write I2C Registers
        let xfer = i2c_transfer_s {
            msgv: msg.as_ptr(),         //  Array of I2C messages for the transfer
            msgc: msg.len() as size_t,  //  Number of messages in the array
        };

        //  Execute I2C Transfer to write I2C Registers
        let ret = unsafe { 
            ioctl(
                self.fd,          //  I2C Port
                I2CIOC_TRANSFER,  //  I2C Transfer
                &xfer             //  I2C Messages for the transfer
            )
        };
        assert!(ret >= 0);   
        Ok(())
    }
}
```

[(Source)](https://github.com/lupyuen/nuttx-embedded-hal/blob/main/src/hal.rs#L33-L96)

After fixing, the Logic Analyser shows that BL602 writes correctly to the I2C Register! (With a harmless I2C Read at the end)

```text
Setup Write to [0xEE] + ACK
0xF5 + ACK
0xA0 + ACK
Setup Read to [0xEF] + ACK
0xA0 + NAK
```

![BL602 writes correctly to the I2C Register! With a harmless I2C Read at the end](https://lupyuen.github.io/images/rusti2c-logic3a.png)

Here's the log...

```text
nsh> rust_i2c
Hello from Rust!
test_hal_write
i2cdrvr_ioctl: cmd=2101 arg=4201c358
bl602_i2c_transfer: subflag=1, subaddr=0xa0f5, sublen=2
bl602_i2c_recvdata: count=1, temp=0xa0
bl602_i2c_transfer: i2c transfer success
test_hal_write: Write 0xA0 to register

i2cdrvr_ioctl: cmd=2101 arg=4201c370
bl602_i2c_transfer: subflag=1, subaddr=0xf5, sublen=1
bl602_i2c_recvdata: count=1, temp=0xa0
bl602_i2c_transfer: i2c transfer success
test_hal_write: Register value is 0xa0

i2cdrvr_ioctl: cmd=2101 arg=4201c358
bl602_i2c_transfer: subflag=1, subaddr=0xf5, sublen=2
bl602_i2c_recvdata: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
test_hal_write: Write 0x00 to register

i2cdrvr_ioctl: cmd=2101 arg=4201c370
bl602_i2c_transfer: subflag=1, subaddr=0xf5, sublen=1
bl602_i2c_recvdata: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
test_hal_write: Register value is 0x00
Done!
nsh>
```

_What if we write to the I2C Register without reading?_

The I2C Address is sent incorrectly (`0x02`) and the I2C Write gets truncated...

```text
Setup Write to [0x02] + NAK
```

![Write to I2C Register without reading](https://lupyuen.github.io/images/rusti2c-noread.png)

_What if we send the Register ID and Register Value as I2C Data (flags = 0) instead of I2C Sub Address?_

The Register ID and value are sent incorrectly as `0x00 0x00`...

```text
Setup Write to [0xEE] + ACK
0x00 + ACK
0x00 + ACK
(...600 microseconds later...)
Setup Read to [0xEF] + ACK
0x00 + NAK
```

![Send the Register ID and Register Value as I2C Data instead of I2C Sub Address](https://lupyuen.github.io/images/rusti2c-nosubaddr.png)

# Rust Embedded Driver for BME280

Now that our Rust Embedded HAL is working on NuttX, let's test the Rust Embedded Driver for BME280...

https://crates.io/crates/bme280

We add the BME280 Driver to [Cargo.toml](rust/Cargo.toml)...

```text
# External Rust libraries used by this module.  See crates.io.
[dependencies]
bme280 = "0.2.1"  # BME280 Driver: https://crates.io/crates/bme280
```

Here's how we read the Temperature, Humidity and Pressure from the BME280 Driver...

```rust
/// Read Temperature, Pressure and Humidity from BME280 Sensor over I2C
pub fn read_bme280() {
    println!("read_bme280");

    //  Open I2C Port
    let i2c = nuttx_embedded_hal::I2c::new(
        "/dev/i2c0",  //  I2C Port
        400000,       //  I2C Frequency: 400 kHz
    ).expect("open failed");
    
    //  Init the BME280 Driver
    let mut bme280 = bme280::BME280::new(
        i2c,   //  I2C Port
        0x77,  //  I2C Address of BME280
        nuttx_embedded_hal::Delay  //  Delay Interface
    );

    //  Init the BME280 Senor
    bme280.init()
        .expect("init failed");

    //  Measure Temperature, Pressure and Humidity
    let measurements = bme280.measure()
        .expect("measure failed");

    //  Print the measurements
    println!("Relative Humidity = {}%", measurements.humidity);
    println!("Temperature = {} deg C",  measurements.temperature);
    println!("Pressure = {} pascals",   measurements.pressure);
}
```

[(Source)](rust/src/bme280.rs)

# Test Rust Driver for BME280

Rust Driver for BME280 works OK on NuttX!

```text
nsh> rust_i2c
Hello from Rust!
read_bme280
i2cdrvr_ioctl: cmd=2101 arg=4201c340
bl602_i2c_transfer: subflag=1, subaddr=0xd0, sublen=1
bl602_i2c_recvdata: count=1, temp=0x60
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c328
bl602_i2c_transfer: subflag=1, subaddr=0xb6e0, sublen=2
bl602_i2c_recvdata: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c350
bl602_i2c_transfer: subflag=1, subaddr=0x88, sublen=1
bl602_i2c_recvdata: count=26, temp=0x65e66e97
bl602_i2c_recvdata: count=22, temp=0x8f990032
bl602_i2c_recvdata: count=18, temp=0xbd0d581
bl602_i2c_recvdata: count=14, temp=0xffdb1e71
bl602_i2c_recvdata: count=10, temp=0x26acfff9
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c350
bl602_i2c_transfer: subflag=1, subaddr=0xe1, sublen=1
bl602_i2c_recvdata: count=7, temp=0x14000165
bl602_i2c_recvdata: count=3, temp=0x141e000b
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c340
bl602_i2c_transfer: subflag=1, subaddr=0xf4, sublen=1
bl602_i2c_recvdata: count=1, temp=0x141e0000
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c338
bl602_i2c_transfer: subflag=1, subaddr=0x1f2, sublen=2
bl602_i2c_recvdata: count=1, temp=0x141e0001
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c340
bl602_i2c_transfer: subflag=1, subaddr=0xf4, sublen=1
bl602_i2c_recvdata: count=1, temp=0x141e0000
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c338
bl602_i2c_transfer: subflag=1, subaddr=0xf4, sublen=2
bl602_i2c_recvdata: count=1, temp=0x141e0000
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c340
bl602_i2c_transfer: subflag=1, subaddr=0xf4, sublen=1
bl602_i2c_recvdata: count=1, temp=0x141e0000
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c338
bl602_i2c_transfer: subflag=1, subaddr=0x54f4, sublen=2
bl602_i2c_recvdata: count=1, temp=0x141e0054
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c340
bl602_i2c_transfer: subflag=1, subaddr=0xf5, sublen=1
bl602_i2c_recvdata: count=1, temp=0x141e0000
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c338
bl602_i2c_transfer: subflag=1, subaddr=0x10f5, sublen=2
bl602_i2c_recvdata: count=1, temp=0x141e0010
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c370
bl602_i2c_transfer: subflag=1, subaddr=0xf4, sublen=1
bl602_i2c_recvdata: count=1, temp=0x141e0054
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c370
bl602_i2c_transfer: subflag=1, subaddr=0xf4, sublen=1
bl602_i2c_recvdata: count=1, temp=0x141e0054
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c368
bl602_i2c_transfer: subflag=1, subaddr=0x55f4, sublen=2
bl602_i2c_recvdata: count=1, temp=0x141e0055
bl602_i2c_transfer: i2c transfer success

i2cdrvr_ioctl: cmd=2101 arg=4201c380
bl602_i2c_transfer: subflag=1, subaddr=0xf7, sublen=1
bl602_i2c_recvdata: count=8, temp=0x86f0b752
bl602_i2c_recvdata: count=4, temp=0x7b8f806b
bl602_i2c_transfer: i2c transfer success

Relative Humidity = 87.667625%
Temperature = 30.358515 deg C
Pressure = 100967.46 pascals
Done!
```
