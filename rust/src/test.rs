//! Test I2C Port

//  Import Libraries
use crate::{      //  Local Library
    nuttx_hal,    //  NuttX Embedded HAL
    close, ioctl, open, read, sleep, write, O_RDWR, //  NuttX Input / Output
};

/// Test the I2C Port by reading an I2C Register
pub fn test_i2c() {
    println!("test_i2c");

    //  Open GPIO Input for SX1262 Busy Pin
    let busy = unsafe { 
        open(b"/dev/gpio0\0".as_ptr(), O_RDWR) 
    };
    assert!(busy > 0);

    //  Open GPIO Output for SX1262 Chip Select
    let cs = unsafe { 
        open(b"/dev/gpio1\0".as_ptr(), O_RDWR) 
    };
    assert!(cs > 0);  

    //  Open GPIO Interrupt for SX1262 DIO1 Pin
    let dio1 = unsafe { 
        open(b"/dev/gpio2\0".as_ptr(), O_RDWR) 
    };
    assert!(dio1 > 0);

    //  Open SPI Bus for SX1262
    let spi = unsafe { 
        open(b"/dev/spitest0\0".as_ptr(), O_RDWR) 
    };
    assert!(spi >= 0);

    //  Read SX1262 Register twice
    for _i in 0..2 {

        //  Set SX1262 Chip Select to Low
        let ret = unsafe { 
            0  //  ioctl(cs, GPIOC_WRITE, 0) 
        };
        assert!(ret >= 0);

        //  Transmit command to SX1262: Read Register 8
        const READ_REG: &[u8] = &[ 0x1d, 0x00, 0x08, 0x00, 0x00 ];
        let bytes_written = unsafe { 
            write(spi, READ_REG.as_ptr(), READ_REG.len() as u32) 
        };
        assert!(bytes_written == READ_REG.len() as i32);

        //  Read response from SX1262
        let mut rx_data: [ u8; 16 ] = [ 0; 16 ];
        let bytes_read = unsafe { 
            read(spi, rx_data.as_mut_ptr(), rx_data.len() as u32) 
        };
        assert!(bytes_read == READ_REG.len() as i32);

        //  Set SX1262 Chip Select to High
        let ret = unsafe { 
            0  //  ioctl(cs, GPIOC_WRITE, 1) 
        };
        assert!(ret >= 0);

        //  Show the received register value
        println!("test_spi: received");
        for i in 0..bytes_read {
            println!("  {:02x}", rx_data[i as usize])
        }
        println!("test_spi: SX1262 Register 8 is 0x{:02x}", rx_data[4]);

        //  Sleep 5 seconds
        unsafe { sleep(5); }
    }

    //  Close the GPIO and SPI ports
    unsafe {
        close(busy);
        close(cs);
        close(dio1);
        close(spi);    
    }
}
