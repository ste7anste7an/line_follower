# Line follower

## ch32
This line follower is based on the chear WCH CH32v203 MCU. This MCU can be programmed in Arduino and can use up to 10 ADC pins. Internally, these pins are multiplexed over 2 ADC units.

The CH32 acts as an I2C slave. An I2C master can read all ADC values in one I2C data transfer. Furthermore, the implementation supports different modes. In raw mode, the raw ADC values (scaled down to 8 bit) can be read. After using the calibration mode, the calibrated values can be read.

## Connecting the CH32

| GPIO | function |
|------|----------|
| PB6   | SDC      |
| PB7   | SDA    |
| PA9   | UART TX |
| PA10 | UART RX |
| PA0   | ADC0  |
| PA1   | ADC1  |
| PA2   | ADC2  |
| PA3   | ADC3  |
| PA4   | ADC4  |
| PA5   | ADC5  |
| PA6   | ADC6  |
| PA7   | ADC7  |
| PB0   | ADC8  |
| PB1   | ADC9  |

## MicroBlocks library

## I2C commands

### I2C Write

| First byte|                  | command                                        |
|-----------|------------------|------------------------------------------------|
| 0x00      | MODE_VAL_RAW     | Switch to Raw Mode; raw ADC values can be read |  
| 0x01      | MODE_VAL_CAL     | Switch to Calinbrated Mode; Calibrated values can be read |  
| 0x02      | MODE_VAL_DIG     | Switch to Digital Mode; only 0 or 1; works only after calibration |  
| 0x03      | MODE_CALIBRATE   | Start Calibration mode        |  
| 0x04      | MODE_GET_MIN     | Read Minimum calibrated values                 |  
| 0x05      | MODE_GET_MAX    | Read Maximum calibrated values                 |  
| 0x06      | MODE_GET_AVG     | Read Averaged calibrated values                 |  
| 0x07      | * MODE_IS_CALIBRATED     | Returns 1 when clibration has run                 |  
| 0x08      | MODE_PRINT_CAL     | Show calibration values in Serial Port                 |  
| 0x09      | * MODE_SAVE_CAL    | Saves calibrated values in EEPROM/FLASH                 |  
| 0x0A      | * MODE_LOAD_CAL     | Load calibrated values from EEPROM/FLASH                 |  
| 0x0B      | MODE_VERSION     | Returns version in 2 bytes (major, minor)                 |  
| 0x0C      | MODE_DEBUG     | Shows extra debug output to Serial Port                 |  
| 0x0D      | * MODE_POSITION     | Returns weighted position of line                 |  
| 0x0E      | * MODE_INVERT     | Inverts ADC values                 |  

* is not yet implemented
* 
### I2C Read
In the MODE_VAL_ modes, the master can read continously a number of bytes (maximum 10 corresponding with the number of sensors used.



``` C
typedef enum {
    MODE_VAL_RAW = 0,
    MODE_VAL_CAL, //1
    MODE_VAL_DIG, //2
    MODE_CALIBRATE, //3
    MODE_GET_MIN,// 4
    MODE_GET_MAX,// 5
    MODE_GET_AVG,//6
    MODE_IS_CALIBRATED, //7 
    MODE_PRINT_CAL, //8
    MODE_SAVE_CAL, //9
    MODE_LOAD_CAL, // 10
    MODE_VERSION, //11
    MODE_DEBUG, //12
    MODE_POSITION, //13
    MODE_INVERT //14
} Mode;
```
