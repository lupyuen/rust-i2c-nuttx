#![no_std]  //  Use the Rust Core Library instead of the Rust Standard Library, which is not compatible with embedded systems

//  Import Test Module
mod test;

//  Import BME280 Module
mod bme280;

//  Import Libraries
use core::{            //  Rust Core Library
    panic::PanicInfo,  //  Panic Handler
};
use nuttx_embedded_hal::{  //  NuttX Embedded HAL
    exit, println,         //  NuttX Functions
};

#[no_mangle]                 //  Don't mangle the function name
extern "C" fn rust_main() {  //  Declare `extern "C"` because it will be called by NuttX

    //  Print a message to the serial console
    println!("Hello from Rust!");    

    //  Test the I2C Port by reading an I2C Register
    test::test_i2c();

    //  Test the I2C HAL by reading an I2C Register
    test::test_hal_read();

    //  Test the I2C HAL by writing an I2C Register
    test::test_hal_write();

    //  Read Temperature, Pressure and Humidity from BME280 Sensor over I2C
    bme280::read_bme280();
}

/// This function is called on panic, like an assertion failure
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {  //  `!` means that panic handler will never return
    //  Display the filename and line number
    println!("*** Rust Panic:");
    if let Some(location) = info.location() {
        println!("File: {}", location.file());
        println!("Line: {}", location.line());
    } else {
        println!("Unknown location");
    }

    //  Set to true if we are already in the panic handler
    static mut IN_PANIC: bool = false;

    //  Display the payload
    if unsafe { !IN_PANIC } {  //  Prevent panic loop while displaying the payload
        unsafe { IN_PANIC = true };
        if let Some(payload) = info.payload().downcast_ref::<&str>() {
            println!("Payload: {}", payload);
        }
    }

    //  Terminate the app
    unsafe { exit(1); }
}