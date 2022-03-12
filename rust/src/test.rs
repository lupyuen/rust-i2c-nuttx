//! Test NuttX I2C Port

//  Import Libraries
use crate::{      //  Local Library
    close, ioctl, open, sleep,  //  NuttX Functions
    size_t, ssize_t,            //  NuttX Types
    I2CIOC_TRANSFER, I2C_M_NOSTOP, I2C_M_READ, O_RDWR,  //  NuttX Constants
};

/// I2C Address of BME280
const BME280_ADDR: u16 = 0x77;

/// I2C Frequency in Hz
const BME280_FREQ: u32 = 400000;

/// I2C Register that contains the Device ID
const BME280_REG_ID:  u8 = 0xD0;

/// Device ID of BME280
const BME280_CHIP_ID: u8 = 0x60;

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

/// I2C transaction segment beginning with a START. A number of these can
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

/// This structure is used to communicate with the I2C character driver in
/// order to perform IOCTL transfers.
/// TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rusti2c/include/nuttx/i2c/i2c_master.h#L231-L235
#[repr(C)]
pub struct i2c_transfer_s {
    /// Array of I2C messages for the transfer
    pub msgv: *const i2c_msg_s,
    /// Number of messages in the array
    pub msgc: size_t,
}
