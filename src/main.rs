mod bme280;

fn main() {
    let bme = bme280::Bme280::new(
        i2cdev::linux::LinuxI2CDevice::new("/dev/i2c", bme280::ADDRESS.into()).unwrap(),
    );
}
