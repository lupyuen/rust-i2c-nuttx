//! Read BME280 Sensor over I2C

//  Import Libraries
use crate::{      //  Local Library
    nuttx_hal,    //  NuttX Embedded HAL
};

/// Read Temperature, Pressure and Humidity from BME280 Sensor over I2C
pub fn read_bme280() {
    println!("read_bme280");

    //  Open I2C Port
    let i2c = nuttx_hal::I2c::new(
        "/dev/i2c0",  //  I2C Port
        400000,       //  I2C Frequency: 400 kHz
    );
    
    /*
    //  Init the BME280 Driver
    let mut bme280 = bme280::BME280::new(
        i2c,   //  I2C Port
        0x77,  //  I2C Address of BME280
        nuttx_hal::Delay  //  Delay Interface
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
    */

    const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
        config: bme280_multibus::Config::reset()
            .set_standby_time(bme280_multibus::Standby::Millis1000)
            .set_filter(bme280_multibus::Filter::X16),
        ctrl_meas: bme280_multibus::CtrlMeas::reset()
            .set_osrs_t(bme280_multibus::Oversampling::X8)
            .set_osrs_p(bme280_multibus::Oversampling::X8)
            .set_mode(bme280_multibus::Mode::Normal),
        ctrl_hum: bme280_multibus::Oversampling::X8,
    };
    //  SdoGnd = 0x76, SdoVddio = 0x77
    let mut bme: bme280_multibus::Bme280<_> = bme280_multibus::Bme280::from_i2c(
        i2c, 
        bme280_multibus::i2c::Address::SdoVddio
    ).expect("init failed");
    bme.settings(&SETTINGS)
        .expect("settings failed");
    let sample: bme280_multibus::Sample = bme.sample()
        .expect("sample failed");

    //  Print the measurements
    println!("Relative Humidity = {}%", sample.humidity);
    println!("Temperature = {} deg C",  sample.temperature);
    println!("Pressure = {} pascals",   sample.pressure);
}