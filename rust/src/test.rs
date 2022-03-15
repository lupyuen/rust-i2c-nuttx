//! Test NuttX I2C Port

//  Import Libraries
use embedded_hal::blocking::i2c::{  //  Rust Embedded HAL for I2C
    Write,     //  Write I2C Data
    WriteRead  //  Write and Read I2C Data
};
use crate::{      //  Local Library
    nuttx_hal,    //  NuttX Embedded HAL
    close, ioctl, open, sleep,                          //  NuttX Functions
    i2c_msg_s, i2c_transfer_s, size_t, ssize_t,         //  NuttX Types
    I2CIOC_TRANSFER, I2C_M_NOSTOP, I2C_M_READ, O_RDWR,  //  NuttX Constants
};

/// I2C Address of BME280
const BME280_ADDR: u16 = 0x77;

/// I2C Frequency in Hz
const BME280_FREQ: u32 = 400000;

/// I2C Register that contains the BME280 Device ID
const BME280_REG_ID: u8 = 0xD0;

/// I2C Register that controls the BME280 Power Mode
const BME280_REG_CTRL_MEASL: u8 = 0xF4;

/// I2C Register that configures the BME280 Standby Interval
const BME280_REG_CONFIG: u8 = 0xF5;

/// Device ID of BME280
const BME280_CHIP_ID: u8 = 0x60;

/// Test the I2C HAL by reading an I2C Register
pub fn test_hal_read() {
    println!("test_hal_read");

    //  Open I2C Port
    let mut i2c = nuttx_hal::I2c::new(
        "/dev/i2c0",  //  I2C Port
        BME280_FREQ,  //  I2C Frequency
    );

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

    //  Sleep 2 seconds
    unsafe { sleep(2); }
}

/// Test the I2C HAL by writing an I2C Register
pub fn test_hal_write() {
    println!("test_hal_write");

    //  Open I2C Port
    let mut i2c = nuttx_hal::I2c::new(
        "/dev/i2c0",  //  I2C Port
        BME280_FREQ,  //  I2C Frequency
    );

    //  Buffer for received I2C data
    let mut buf = [0 ; 1];

    /*
    //  Write 0x00 to register 0xF4 to enter Sleep Mode.
    //  BME280 must be in sleep mode for register 0xF5 to be written correctly.
    i2c.write(
        BME280_ADDR as u8,              //  I2C Address
        &[BME280_REG_CTRL_MEASL, 0x00]  //  Register ID and value
    ).expect("write register failed");
    println!("test_hal_write: Enter sleep mode");

    //  Sleep 1 second
    unsafe { sleep(1); }
        
    //  Read from register 0xF5
    i2c.write_read(
        BME280_ADDR as u8,     //  I2C Address
        &[BME280_REG_CONFIG],  //  Register ID
        &mut buf  //  Buffer to be received
    ).expect("read register failed");
    println!("test_hal_write: Register value is 0x{:02x}", buf[0]);

    //  Sleep 1 second
    unsafe { sleep(1); }
    */

    //  Write 0xA0 to register 0xF5
    i2c.write(
        BME280_ADDR as u8,          //  I2C Address
        &[BME280_REG_CONFIG, 0xA0]  //  Register ID and value
    ).expect("write register failed");
    println!("test_hal_write: Write 0xA0 to register");

    /*
    //  Sleep 1 second
    unsafe { sleep(1); }

    //  Read from register 0xF5
    i2c.write_read(
        BME280_ADDR as u8,     //  I2C Address
        &[BME280_REG_CONFIG],  //  Register ID
        &mut buf  //  Buffer to be received
    ).expect("read register failed");
    println!("test_hal_write: Register value is 0x{:02x}", buf[0]);
    assert_eq!(buf[0], 0xA0);

    //  Sleep 1 second
    unsafe { sleep(1); }

    //  Write 0x00 to register 0xF5
    i2c.write(
        BME280_ADDR as u8,          //  I2C Address
        &[BME280_REG_CONFIG, 0x00]  //  Register ID and value
    ).expect("write register failed");
    println!("test_hal_write: Write 0x00 to register");

    //  Sleep 1 second
    unsafe { sleep(1); }

    //  Read from register 0xF5
    i2c.write_read(
        BME280_ADDR as u8,     //  I2C Address
        &[BME280_REG_CONFIG],  //  Register ID
        &mut buf  //  Buffer to be received
    ).expect("read register failed");
    println!("test_hal_write: Register value is 0x{:02x}", buf[0]);
    assert_eq!(buf[0], 0x00);

    //  Sleep 2 seconds
    unsafe { sleep(2); }
    */
}

/// Test the I2C Port by reading an I2C Register through ioctl
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
    let msg = [
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

            //  TODO: Check for BL602 specifically (by target_abi?), not just RISC-V 32-bit
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
            i2c,          //  I2C Port
            I2CIOC_TRANSFER,  //  I2C Transfer
            &xfer             //  I2C Messages for the transfer
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
    assert_eq!(buf[0], BME280_CHIP_ID);
     
    //  Close the I2C Port
    unsafe { close(i2c); }

    //  Sleep 2 seconds
    unsafe { sleep(2); }
}
