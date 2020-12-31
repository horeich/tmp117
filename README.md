# Texas Instrument TMP117 (in progress, first release soon)

Library for Texas Instrument TMP117 using mbedOS

![Main](https://github.com/horeich/tmp117/blob/master/content/TMP117_main.jpg)

### Links to devices:
- [NUCLEO-L476RG](https://os.mbed.com/platforms/ST-Nucleo-L476RG/) - a simple STM development board
- [Sparkfun-TMP117](https://www.sparkfun.com/products/15805) - a high precision temperature sensor breakout board

## Folder content
in the making

## Registers

The registers of the devices are 16bit wide
- After power-down the first call to a register takes time -> wait until busy flag is reset;
## Writing to a register
It's worth noting that programming certain functions of the TMP117 (e.g. alert mode/ therm mode) can be quite slow (typically 7ms). Therefore, we have to wait until the device returns the "not busy" flag which signals that data is successfully written to the EEPROM.

Note: See Figure 20 on datasheet p.18 for how to program the EEPROM.

## Resetting the device
The reset sequence ```soft_reset``` takes around 1.5ms and restores the default parameters. Note that the EEPROM is locked by default so call ```unlock_registers()``` before writing to the registers. See section 7.5 for further info

## Temperature conversion
- ```read_temperature()```: Note that we have to use the ```short``` data type to also cover negative temperature values (do not use ```unsigned short``` aka ```uint16_t```)
- The maximum possible value is 0b0111 1111 1111 1111 (255.9912F); the minimum possible value is -256.0f


## Use the library

## Recommendations (typical application)
- If possible use one-shot mode to minimize self-heating of the device
- In continuous mode it is recommended to use >1 second conversion cycle time and only 8 samples to calculate the average (see table in ```tmp116.hpp```)
- Use the minimal acceptable power supply voltage (1.8V)
- Use the hightest available communication speed (400kHz)

## Tests
### Integration tests
Integration tests try to cover every functionality of the TMP117 device
- therm mode, alert mode
- callback data ready
- normal data ready
- all average mode
- no average mode

## Examples
### tmp117_oneshot.cpp
- Configure the the pins in the
- comment out/enable #define TMP117_CALLBACK to switch between callback (recommended!) and polling
TODO: alert mode and therm mode examples