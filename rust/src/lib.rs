#![no_std]  //  Use the Rust Core Library instead of the Rust Standard Library, which is not compatible with embedded systems

//  Import Macros for NuttX
#[macro_use]
mod macros;

//  Import NuttX HAL
mod nuttx_hal;

//  Import Test Module
mod test;

//  Import Libraries
use core::{            //  Rust Core Library
    fmt,               //  String Formatting    
    panic::PanicInfo,  //  Panic Handler
    str::FromStr,      //  For converting `str` to `String`
};
use embedded_hal::{           //  Rust Embedded HAL
    digital::v2::OutputPin,   //  GPIO Output
    blocking::{               //  Blocking I/O
        delay::DelayMs,       //  Delay Interface
        spi::Transfer,        //  SPI Transfer
    },
};

#[no_mangle]                 //  Don't mangle the function name
extern "C" fn rust_main() {  //  Declare `extern "C"` because it will be called by NuttX

    //  Print a message to the serial console
    println!("Hello from Rust!");    

    //  Test the I2C Port by reading an I2C Register
    test::test_i2c();
}

/// Print a message to the serial console.
/// TODO: Auto-generate this wrapper with `bindgen` from the C declaration
pub fn puts(s: &str) -> i32 {  //  `&str` is a reference to a string slice, similar to `const char *` in C

    extern "C" {  //  Import C Function
        /// Print a message to the serial console (from C stdio library)
        fn puts(s: *const u8) -> i32;
    }

    //  Convert `str` to `String`, which similar to `char [64]` in C
    //  TODO: Increase the buffer size if we're sure we won't overflow the stack
    let mut s_with_null = String::from_str(s)  //  `mut` because we will modify it
        .expect("puts conversion failed");     //  If it exceeds 64 chars, halt with an error
    
    //  Terminate the string with null, since we will be passing to C
    s_with_null.push('\0')
        .expect("puts overflow");  //  If we exceed 64 chars, halt with an error

    //  Convert the null-terminated string to a pointer
    let p = s_with_null.as_str().as_ptr();

    //  Call the C function
    unsafe {  //  Flag this code as unsafe because we're calling a C function
        puts(p)
    }

    //  No semicolon `;` here, so the value returned by the C function will be passed to our caller
}

/// Print a formatted message to the serial console. Called by println! macro.
pub fn puts_format(args: fmt::Arguments<'_>) {
    //  Allocate a 64-byte buffer.
    //  TODO: Increase the buffer size if we're sure we won't overflow the stack
    let mut buf = String::new();

    //  Format the message into the buffer
    fmt::write(&mut buf, args)
        .expect("puts_format overflow");

    //  Print the buffer
    puts(&buf);
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

/// Limit Strings to 64 chars, similar to `char[64]` in C
pub type String = heapless::String::<64>;

extern "C" {  //  Import POSIX Functions. TODO: Import with bindgen
    pub fn open(path: *const u8, oflag: i32, ...) -> i32;
    pub fn read(fd: i32, buf: *mut u8, count: u32) -> i32;
    pub fn write(fd: i32, buf: *const u8, count: u32) -> i32;
    pub fn close(fd: i32) -> i32;
    pub fn ioctl(fd: i32, request: i32, ...) -> i32;  //  On NuttX: request is i32, not u64 like Linux
    pub fn sleep(secs: u32) -> u32;
    pub fn usleep(usec: u32) -> u32;
    pub fn exit(status: u32) -> !;
}

/// TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rust/include/nuttx/ioexpander/gpio.h
pub const GPIOC_WRITE:      i32 = _GPIOBASE | 1;  //  _GPIOC(1)
pub const GPIOC_READ:       i32 = _GPIOBASE | 2;  //  _GPIOC(2)
pub const GPIOC_PINTYPE:    i32 = _GPIOBASE | 3;  //  _GPIOC(3)
pub const GPIOC_REGISTER:   i32 = _GPIOBASE | 4;  //  _GPIOC(4)
pub const GPIOC_UNREGISTER: i32 = _GPIOBASE | 5;  //  _GPIOC(5)
pub const GPIOC_SETPINTYPE: i32 = _GPIOBASE | 6;  //  _GPIOC(6)

/// TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rust/include/fcntl.h
pub const _GPIOBASE: i32 = 0x2300; /* GPIO driver commands */
//  #define _GPIOC(nr)       _IOC(_GPIOBASE,nr)
//  #define _IOC(type,nr)    ((type)|(nr))

/// TODO: Import with bindgen from https://github.com/lupyuen/incubator-nuttx/blob/rust/include/fcntl.h
pub const O_RDONLY: i32 = 1 << 0;        /* Open for read access (only) */
pub const O_RDOK:   i32 = O_RDONLY;      /* Read access is permitted (non-standard) */
pub const O_WRONLY: i32 = 1 << 1;        /* Open for write access (only) */
pub const O_WROK:   i32 = O_WRONLY;      /* Write access is permitted (non-standard) */
pub const O_RDWR:   i32 = O_RDOK|O_WROK; /* Open for both read & write access */
