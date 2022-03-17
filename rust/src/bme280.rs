//! Read BME280 Sensor over I2C

//  Import Libraries
use nuttx_embedded_hal::{  //  NuttX Embedded HAL
    println,
};

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