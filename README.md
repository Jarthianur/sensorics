# sensorics

This project contains a bunch of utilities to read and provide sensoric measurement data.
For that purpose there are a few modules implemented, which allow different usages of sensors like **Bosch BME280**.
The target platform is **Linux** only, commonly a **Raspberry Pi**.
Currently implemented are a NMEA-server and a SQLite-exporter for the BME280 sensor communicating over an I2C bus.

## installation

The build is done with CMake and GNU make. As an example for how to build the *bmeserver*.
After that you can find the executable under *build/bin*.

```bash
cd build
cmake ..
make -j$(nproc) bmeserver
```

## requirements

There are some required packages which need to be installed before, depending on what you want to build.
Note that the package names may differe on the platform of your choice. Following are all packages by their name on Raspbian (Jessie).

+ libapr1
+ libapr1-dev
+ libaprutil1
+ libaprutil1-dev
+ i2c-tools
+ libi2c-dev
+ sqlite3
+ libsqlite3-dev

## development and contribution

Of course contributions and ideas are always welcome. I would be proud, if you make use of this project in your own ones.
As an information for developers, these included modules a also compiled to shared libraries during the make process.
