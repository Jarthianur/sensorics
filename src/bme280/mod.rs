extern crate i2cdev;

use i2cdev::core::*;

pub const ADDRESS: u8 = 0x76;
const RST: u8 = 0xB6;

// (register) addresses
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
const REG_CAL26: u8 = 0xE1;
const REG_CTRL_HUMID: u8 = 0xF2;
const REG_CTRL: u8 = 0xF4;
const REG_CONF: u8 = 0xF5;
const REG_PRESS: u8 = 0xF7;
const REG_TEMP: u8 = 0xFA;
const REG_HUMID: u8 = 0xFD;
const REG_MEAS: u8 = REG_PRESS;

// power modes
const PWR_SLEEP: u8 = 0x00;
const PWR_FORCED: u8 = 0x01;
const PWR_NORMAL: u8 = 0x03;

#[derive(Debug)]
#[repr(u8)]
enum PowerModes {
    SLEEP = PWR_SLEEP,
    FORCED = PWR_FORCED,
    NORMAL = PWR_NORMAL,
}

impl From<u8> for PowerModes {
    fn from(item: u8) -> Self {
        match item {
            PWR_SLEEP => PowerModes::SLEEP,
            PWR_FORCED => PowerModes::FORCED,
            PWR_NORMAL => PowerModes::NORMAL,
            _ => panic!("{} is not a valid PowerMode", item),
        }
    }
}

#[derive(Debug)]
#[repr(u8)]
enum StandbyTimes {
    Ms1 = 0x00,
    Ms10 = 0x06,
    Ms20 = 0x07,
    Ms64 = 0x01,
    Ms125 = 0x02,
    Ms250 = 0x03,
    Ms500 = 0x04,
    Ms1000 = 0x05,
}

#[derive(Debug)]
#[repr(u8)]
enum OversamplingRatios {
    SKIPPED = 0x00,
    X1 = 0x01,
    X2 = 0x02,
    X4 = 0x03,
    X8 = 0x04,
    X16 = 0x05,
}

#[derive(Debug)]
#[repr(u8)]
enum FilterCoefficients {
    OFF = 0x00,
    X2 = 0x01,
    X4 = 0x02,
    X8 = 0x03,
    X16 = 0x04,
}

#[derive(Debug)]
pub enum Bme280Error<E> {
    Error(E),
}

impl<E: std::error::Error> std::fmt::Display for Bme280Error<E> {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match *self {
            Bme280Error::Error(ref e) => std::fmt::Display::fmt(&e, f),
        }
    }
}

impl<E: std::error::Error + 'static> std::error::Error for Bme280Error<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match *self {
            Bme280Error::Error(ref e) => Some(e),
        }
    }
}

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
struct Measurements<T> {
    temperature: T,
    pressure: T,
    humidity: T,
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
    meas: Measurements<i32>,
    t_fine: i64,
}

#[derive(Default)]
struct Oversampling {
    ovrsmpl_temp: u8,
    ovrsmpl_press: u8,
    ovrsmpl_humid: u8,
}

pub struct Bme280<T: I2CDevice> {
    device: T,
    calib: CalibTable,
    uncomp_meas: UncompMeas,
    power_mode: PowerModes,
    filter: FilterCoefficients,
    standby_durn: u8,
    ovrsmpl: Oversampling,
    comp_meas: Measurements<f64>,
    ctrl_humid_reg: u8,
    ctrl_meas_reg: u8,
    config_reg: u8,
}

macro_rules! bit_slice {
    ($regvar: ident, $mask:literal, $pos:literal, $val:ident) => {
        ($regvar & !$mask) | (($val << $pos) & $mask)
    };
}

macro_rules! get_bit_slice {
    ( $regvar:ident,  $mask:literal,  $pos:literal) => {
        ($regvar & $mask) >> $pos
    };
}

impl<T> Bme280<T>
where
    T: I2CDevice,
{
    pub fn new(dev: T) -> Result<Bme280<T>, i32> {
        let mut bme = Bme280 {
            device: dev,
            calib: Default::default(),
            uncomp_meas: Default::default(),
            power_mode: PowerModes::NORMAL,
            filter: FilterCoefficients::OFF,
            standby_durn: 0,
            ovrsmpl: Default::default(),
            comp_meas: Default::default(),
            ctrl_humid_reg: 0,
            ctrl_meas_reg: 0,
            config_reg: 0,
        };
        if !bme.read_calib() {
            return Err(1);
        }
        Ok(bme)
    }

    fn read_calib(&mut self) -> bool {
        let data = match self.device.smbus_read_i2c_block_data(REG_DIG_T1, 26) {
            Ok(n) => n,
            Err(_) => return false,
        };

        let ref mut calib = self.calib;
        calib.dig_T1 = ((data[1] as u16) << 8) | data[0] as u16;
        calib.dig_T2 = ((data[3] as i16) << 8) | data[2] as i16;
        calib.dig_T3 = ((data[5] as i16) << 8) | data[4] as i16;
        calib.dig_P1 = ((data[7] as u16) << 8) | data[6] as u16;
        calib.dig_P2 = ((data[9] as i16) << 8) | data[8] as i16;
        calib.dig_P3 = ((data[11] as i16) << 8) | data[10] as i16;
        calib.dig_P4 = ((data[13] as i16) << 8) | data[12] as i16;
        calib.dig_P5 = ((data[15] as i16) << 8) | data[14] as i16;
        calib.dig_P6 = ((data[17] as i16) << 8) | data[16] as i16;
        calib.dig_P7 = ((data[19] as i16) << 8) | data[18] as i16;
        calib.dig_P8 = ((data[21] as i16) << 8) | data[20] as i16;
        calib.dig_P9 = ((data[23] as i16) << 8) | data[22] as i16;
        calib.dig_H1 = data[25];

        let data = match self.device.smbus_read_i2c_block_data(REG_DIG_H2, 7) {
            Ok(n) => n,
            Err(_) => return false,
        };
        calib.dig_H2 = ((data[1] as i16) << 8) | data[0] as i16;
        calib.dig_H3 = data[2];
        calib.dig_H4 = ((data[3] as i16) << 4) | (data[4] & 0x0F) as i16;
        calib.dig_H5 = ((data[5] as i16) << 4) | (data[4] >> 4) as i16;
        calib.dig_H6 = data[6] as i8;

        true
    }

    fn compensate_temp(&mut self) {
        let adc_t = self.uncomp_meas.meas.temperature as f64;
        let ref calib = self.calib;
        let var1 = (adc_t / 16384.0 - (calib.dig_T1 as f64) / 1024.0) * calib.dig_T2 as f64;
        let var2 = ((adc_t / 131_072.0 - (calib.dig_T1 as f64) / 8192.0)
            * (adc_t / 131_072.0 - (calib.dig_T1 as f64) / 8192.0))
            * calib.dig_T3 as f64;
        self.uncomp_meas.t_fine = (var1 + var2) as i64;
        self.comp_meas.temperature = (var1 + var2) / 5120.0;
    }

    fn compensate_press(&mut self) {
        let adc_p = self.uncomp_meas.meas.pressure as f64;
        let ref calib = self.calib;
        let mut var1 = self.uncomp_meas.t_fine as f64 / 2.0 - 64000.0;
        let mut var2 = (((var1 * var1 * calib.dig_P6 as f64 / 32768.0)
            + var1 * calib.dig_P5 as f64 * 2.0)
            / 4.0)
            + calib.dig_P4 as f64 * 65536.0;
        var1 = (calib.dig_P3 as f64 * var1 * var1 / 524_288.0 + calib.dig_P2 as f64 * var1)
            / 524_288.0;
        var1 = (1.0 + var1 / 32768.0) * calib.dig_P1 as f64;
        if var1 != 0.0 {
            let mut p = (1_048_576.0 - adc_p - (var2 / 4096.0)) * 6250.0 / var1;
            var1 = calib.dig_P9 as f64 * p * p / 2_147_483_648.0;
            var2 = p * calib.dig_P8 as f64 / 32768.0;
            p = p + (var1 + var2 + calib.dig_P7 as f64) / 16.0;
            self.comp_meas.pressure = p / 100.0;
        }
    }

    fn compensate_humid(&mut self) {
        let adc_h = self.uncomp_meas.meas.humidity as f64;
        let ref calib = self.calib;
        let mut var_h = self.uncomp_meas.t_fine as f64 - 76800.0;
        var_h = (adc_h - (calib.dig_H4 as f64 * 64.0 + calib.dig_H5 as f64 / 16384.0 * var_h))
            * (calib.dig_H2 as f64 / 65536.0
                * (1.0
                    + calib.dig_H6 as f64 / 67_108_864.0
                        * var_h
                        * (1.0 + calib.dig_H3 as f64 / 67_108_864.0 * var_h)));
        var_h = var_h * (1.0 - calib.dig_H1 as f64 * var_h / 524_288.0);
        if var_h > 100.0 {
            var_h = 100.0;
        } else if var_h < 0.0 {
            var_h = 0.0;
        }
        self.comp_meas.humidity = var_h;
    }

    fn unpack_pressure(&mut self) {
        let ref mut meas = self.uncomp_meas;
        meas.meas.pressure = (((meas.pmsb as u32) << 12)
            | ((meas.plsb as u32) << 4)
            | (meas.pxsb as u32 >> 4)) as i32;
    }

    fn unpack_temperature(&mut self) {
        let ref mut meas = self.uncomp_meas;
        meas.meas.temperature = (((meas.tmsb as u32) << 12)
            | ((meas.tlsb as u32) << 4)
            | (meas.txsb as u32 >> 4)) as i32;
    }

    fn unpack_humidity(&mut self) {
        let ref mut meas = self.uncomp_meas;
        meas.meas.humidity = (((meas.hmsb as u32) << 8) | meas.hlsb as u32) as i32;
    }

    fn read_temp(&mut self) -> bool {
        let ref mut meas = self.uncomp_meas;
        let data = match self.device.smbus_read_i2c_block_data(REG_TEMP, 3) {
            Ok(n) => n,
            Err(_) => return false,
        };
        meas.tmsb = data[0];
        meas.tlsb = data[1];
        meas.txsb = data[2];
        self.unpack_temperature();

        true
    }

    fn read_press(&mut self) -> bool {
        let ref mut meas = self.uncomp_meas;
        let data = match self.device.smbus_read_i2c_block_data(REG_PRESS, 3) {
            Ok(n) => n,
            Err(_) => return false,
        };
        meas.pmsb = data[0];
        meas.plsb = data[1];
        meas.pxsb = data[2];
        self.unpack_pressure();

        true
    }

    fn read_humid(&mut self) -> bool {
        let ref mut meas = self.uncomp_meas;
        let data = match self.device.smbus_read_i2c_block_data(REG_HUMID, 2) {
            Ok(n) => n,
            Err(_) => return false,
        };
        meas.hmsb = data[0];
        meas.hlsb = data[1];
        self.unpack_humidity();

        true
    }

    fn readBurstTPH(&mut self) -> bool {
        let ref mut meas = self.uncomp_meas;
        let data = match self.device.smbus_read_i2c_block_data(REG_MEAS, 8) {
            Ok(n) => n,
            Err(_) => return false,
        };
        meas.pmsb = data[0];
        meas.plsb = data[1];
        meas.pxsb = data[2];
        meas.tmsb = data[3];
        meas.tlsb = data[4];
        meas.txsb = data[5];
        meas.hmsb = data[6];
        meas.hlsb = data[7];
        self.unpack_pressure();
        self.unpack_temperature();
        self.unpack_humidity();
        self.compensate_press();
        self.compensate_temp();
        self.compensate_humid();

        true
    }

    /*
        u8_t BME280_set_powermode(bme280* inst, u8_t mode)
    {
        u8_t ctrl_val     = 0;
        u8_t prev_mode    = 0;
        u8_t ctrl_hum_pre = 0;
        u8_t conf_pre     = 0;
        u8_t common       = 0;

        if (mode <= BME280_NORMAL_MODE)
        {
            ctrl_val  = inst->ctrl_meas_reg;
            ctrl_val  = BME280_bit_slice(ctrl_val, 0x03, 0, mode);
            prev_mode = BME280_get_powermode(inst);

            if (prev_mode != BME280_SLEEP_MODE)
            {
                BME280_soft_rst(inst);
                BME280_delay(3);
                conf_pre = inst->config_reg;
                BUS_write_reg(inst->fd, BME280_REG_CONF, conf_pre);
                ctrl_hum_pre = inst->ctrl_humid_reg;
                BUS_write_reg(inst->fd, BME280_REG_CTRL_HUMID, ctrl_hum_pre);
                BUS_write_reg(inst->fd, BME280_REG_CTRL, ctrl_val);
            }
            else
            {
                BUS_write_reg(inst->fd, BME280_REG_CTRL, ctrl_val);
            }
            BUS_read_reg(inst->fd, BME280_REG_CTRL, &common);
            inst->ctrl_meas_reg = common;
            BUS_read_reg(inst->fd, BME280_REG_CTRL_HUMID, &common);
            inst->ctrl_humid_reg = common;
            BUS_read_reg(inst->fd, BME280_REG_CONF, &common);
            inst->config_reg = common;
        }
        else
        {
            return BME280_ERROR;
        }
        return BME280_SUCCESS;
    }
    */
    fn get_powermode(&mut self) -> PowerModes {
        let data = match self.device.smbus_read_byte_data(REG_CTRL) {
            Ok(b) => b,
            Err(_) => 0xFF,
        };
        self.power_mode = get_bit_slice!(data, 0x03, 0).into();

        self.power_mode
    }
}
