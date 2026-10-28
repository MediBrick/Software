# Software

### 🔌 **Firmware** & 📋 **Test Software

- ESP32
    - [📋 📂 ESP32 Hardware Test](/ESP32/Adafruit_FeatherS3_test/Adafruit_FeatherS3_test.ino)
    - [📋 📂 I2C Scan](/ESP32/MediBrick_I2C_Scan/MediBrick_I2C_Scan.ino)
    - [📋 📂 Display, Battery & Deep Sleep Test](/ESP32/MediBrick_Display_DeepSleep_Test/)
    - [📋 📂 BlueTooth Serial Test](/ESP32/BLESerial_comprehensive/BLESerial_comprehensive.ino)
- ECG & BIOZ
    - [📂 Current Firmware with BLESerial, Display & Sleep](/MediBrick_ECG_BIOZ/MAX30001G_Firmware/MAX30001G_Firmware.ino)
- Spirometer
    - Need to write
- Pulse Oximeter
    - [📂 Current Firmware](/MediBrick_ECG_BIOZ/MAX30001G_Firmware/MAX30001G_Firmware.ino)
- Stethoscope
    - Need to write
- Activity v1
    - 📋 individual test programs in [folder](/MediBrick_Activity/v1/)
- Activity v2
    - Need to write
- Temperature
    - [📂 Current Firmware](/MediBrick_Temperature/MediBrick_Temperature_Test/)
- Environment
    - 📋 individual test programs
    - Need to write
- DC PowerBrick
    - [📂 Current Firmware](/MediBrick_Power/DRV8704_Firmware/DRV8704_Firmware.ino)

## 🧰 Libraries

These libraries are imported and needed to create the Firmware for the MediBricks.

You can install them with these automated scripts
- [Unix Shell](/install_libraries.sh)
- [Windows Power Shell](/install_libraries.ps1)

and then execute:

```sh
./install_libraries.sh           # install missing, update outdated
```
or on Windows
```powershell
powershell -ExecutionPolicy Bypass -File install_libraries.ps1
```

Linux / macOS (installs into `~/Arduino/libraries`), Windows (installs into `%USERPROFILE%\Arduino\libraries`)

You can find the libraries original repository here:

- General for all MediBrick Modules
    - [↗Git 📂 BlueTooth Driver](https://github.com/h2zero/NimBLE-Arduino)
    - [↗Git 📂 **BLESerial**](https://github.com/uutzinger/Arduino_BLESerial)
    - [↗Git 📂 **RingBuffer**](https://github.com/uutzinger/Arduino_RingBuffer)
    - [↗Git 📂 **Savitzky Golay**](https://github.com/uutzinger/SavitzkyGolayFilter)
    - [↗Git 📂 **logger**](https://github.com/uutzinger/logger)
    - [↗Git 📂 Display SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
    - [↗Git 📂 Battery MAX1704X](https://github.com/sparkfun/SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library)
    - [↗Git 📂 Battery LC709203F](https://github.com/adafruit/Adafruit_LC709203F)
- ECG & BIOZ
    - [↗Git 📂 **MAX30001G**](https://github.com/uutzinger/Arduino_MAX30001G)
- Spirometer
    - [↗Git 📂 Flow Sensirion SFM3505](https://github.com/Sensirion/arduino-i2c-sfm3505)
    - [↗Git 📂 Flow Sensirion SFM3300](https://github.com/TronixLab/SFM3300)
    - Pressure, Amphenol ELVH-M100D-HRND-I-N2A4, create your own simple driver
- Pulse Oximeter
    - [↗Git 📂 **AFE44XX**](https://github.com/uutzinger/Arduino_AFE44XX)
- Stethoscope
    - [↗Git 📂 AudioTools](https://github.com/pschatzmann/arduino-audio-tools)
    - [↗Git 📂 Pressure Honeywell MPR](https://github.com/sparkfun/SparkFun_MicroPressure_Arduino_Library)
- Activity v1
    - [↗Git 📂 IMU ICM 20948](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)
    - [↗Git 📂 Barometric Pressure BMP581](https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library)
- Activity v2
    - IMU Fusion, build Arduino version based on https://github.com/xioTechnologies/Fusion
    - [↗Git 📂 IMU Bosch Sensortec BMI323](https://github.com/7semi-solutions/7Semi-BMI323)
    - [↗Git 📂 Magnetometer Bosch Sensortec BMM350](https://github.com/CancianMatteo/BMM350-Magnetometer)
    - [↗Git 📂 Barometric Pressure BMP581](https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library)
    - [↗Git 📂 GNSS u-blox MAX-M10S-00B](https://github.com/sparkfun/SparkFun_u-blox_MAX-M10S)
    - [↗Git 📂 GNSS RTK u-blox ZED-F9P-05B](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library)
- Temperature
    - [↗Git 📂 AudioTools](https://github.com/pschatzmann/arduino-audio-tools)
- Environment
    - [↗Git 📂 Neopixel](https://github.com/adafruit/Adafruit_NeoPixel)
    - [↗Git 📂 Barometric Pressure BMP581](https://github.com/sparkfun/SparkFun_BMP581_Arduino_Library)
    - [↗Git 📂 Sensirion Core](https://github.com/Sensirion/arduino-core/)
    - [↗Git 📂 Particulate Matter SEN50](https://github.com/Sensirion/arduino-i2c-sen5x)
    - [↗Git 📂 CO2 SCD40 ](https://github.com/Sensirion/arduino-i2c-scd4x)
    - [↗Git 📂 Humidity, Temperature, SHT45](https://github.com/sensirion/arduino-sht)  
    - [↗Git 📂 eVOC, eNOX, SGP41](https://github.com/Sensirion/arduino-i2c-sgp41)
    - [↗Git 📂 Gas Index Algorithm for SGP41](https://github.com/Sensirion/gas-index-algorithm)
    - [↗Git 📂 CO, NOX, Amonia, MiCS-6814 MEMS](https://github.com/eNBeWe/MiCS6814-I2C-Library) 
- DC PowerBrick
    - [↗Git 📂 **DRV8704**](https://github.com/uutzinger/Arduino_DRV8704)

Items listed in bold are libraries created by the author.

