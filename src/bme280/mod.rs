extern crate i2cdev;
// (register) addresses
const ADDRESS: u8 = 0x76;

const REG_DIG_T1: u8 = 0x88;
const REG_DIG_T2: u8 = 0x8A;
const REG_DIG_T3: u8 = 0x8C;
const REG_DIG_P1: u8 = 0x8E;
const REG_DIG_P2: u8 = 0x90;
const REG_DIG_P3: u8 = 0x92;
const REG_DIG_P4: u8 = 0x94;
const REG_DIG_P5: u8 = 0x96;
const REG_DIG_P6: u8 = 0x98;
const REG_DIG_P7: u8 = 0x9A;
const REG_DIG_P8: u8 = 0x9C;
const REG_DIG_P9: u8 = 0x9E;
const REG_DIG_H1: u8 = 0xA1;
const REG_DIG_H2: u8 = 0xE1;
const REG_DIG_H3: u8 = 0xE3;
const REG_DIG_H4: u8 = 0xE4;
const REG_DIG_H5: u8 = 0xE5;
const REG_DIG_H6: u8 = 0xE7;
const REG_CHIPID: u8 = 0xD0;
const REG_VERSION: u8 = 0xD1;
const REG_SOFTRST: u8 = 0xE0;
const RST: u8 = 0xB6;
const REG_CAL26: u8 = 0xE1;
const REG_CTRL_HUMID: u8 = 0xF2;
const REG_CTRL: u8 = 0xF4;
const REG_CONF: u8 = 0xF5;
const REG_PRESS: u8 = 0xF7;
const REG_TEMP: u8 = 0xFA;
const REG_HUMID: u8 = 0xFD;
const REG_MEAS: u8 = REG_PRESS;
// power modes
const SLEEP_MODE: u8 = 0x00;
const FORCED_MODE: u8 = 0x01;
const NORMAL_MODE: u8 = 0x03;
// standby times
const STANDBY_TIME_1_MS: u8 = 0x00;
const STANDBY_TIME_10_MS: u8 = 0x06;
const STANDBY_TIME_20_MS: u8 = 0x07;
const STANDBY_TIME_63_MS: u8 = 0x01;
const STANDBY_TIME_125_MS: u8 = 0x02;
const STANDBY_TIME_250_MS: u8 = 0x03;
const STANDBY_TIME_500_MS: u8 = 0x04;
const STANDBY_TIME_1000_MS: u8 = 0x05;
// oversampling ratios
const OVERSAMP_SKIPPED: u8 = 0x00;
const OVERSAMP_1X: u8 = 0x01;
const OVERSAMP_2X: u8 = 0x02;
const OVERSAMP_4X: u8 = 0x03;
const OVERSAMP_8X: u8 = 0x04;
const OVERSAMP_16X: u8 = 0x05;
// filter coefficients
const FILTER_COEFF_OFF: u8 = 0x00;
const FILTER_COEFF_2: u8 = 0x01;
const FILTER_COEFF_4: u8 = 0x02;
const FILTER_COEFF_8: u8 = 0x03;
const FILTER_COEFF_16: u8 = 0x04;

#[derive(Default)]
struct CalibTable {
    dig_T1: u16,
    dig_T2: i16,
    dig_T3: i16,
    dig_P1: u16,
    dig_P2: i16,
    dig_P3: i16,
    dig_P4: i16,
    dig_P5: i16,
    dig_P6: i16,
    dig_P7: i16,
    dig_P8: i16,
    dig_P9: i16,
    dig_H1: u8,
    dig_H2: i16,
    dig_H3: u8,
    dig_H4: i16,
    dig_H5: i16,
    dig_H6: i8,
}

#[derive(Default)]
struct UncompMeas {
    pmsb: u8,
    plsb: u8,
    pxsb: u8,
    tmsb: u8,
    tlsb: u8,
    txsb: u8,
    hmsb: u8,
    hlsb: u8,
    temperature: i32,
    pressure: i32,
    humidity: i32,
    t_fine: i64,
}

pub struct Bme280 {
    dev: i2cdev::linux::LinuxI2CDevice,
    buffer: [u8; 32],
    p_calib: CalibTable,
    p_uncomp_meas: UncompMeas,
    power_mode: u8,
    filter: u8,
    standby_durn: u8,
    ovrsmpl_temp: u8,
    ovrsmpl_press: u8,
    ovrsmpl_humid: u8,
    comp_temp: f64,
    comp_press: f64,
    comp_humid: f64,
    ctrl_humid_reg: u8,
    ctrl_meas_reg: u8,
    config_reg: u8,
}

impl Bme280 {
    pub fn new() -> Result<Bme280, i32> {
        let bme = Bme280 {
            dev: i2cdev::linux::LinuxI2CDevice::new("/dev/i2c", ADDRESS.into()).unwrap(),
            buffer: [0; 32],
            p_calib: Default::default(),
            p_uncomp_meas: Default::default(),
            power_mode: 0,
            filter: 0,
            standby_durn: 0,
            ovrsmpl_temp: 0,
            ovrsmpl_press: 0,
            ovrsmpl_humid: 0,
            comp_temp: 0.0,
            comp_press: 0.0,
            comp_humid: 0.0,
            ctrl_humid_reg: 0,
            ctrl_meas_reg: 0,
            config_reg: 0,
        };
        if !bme.readCalib() {
            return Err(1);
        }
        Ok(bme)
    }

    fn readCalib(&self) -> bool {
        true
    }

    fn readBurstTPH(&self) -> bool {
        true
    }
}
