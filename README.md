# Texas Instrument TMP117 (in progress, first release soon)

Driver library for Texas Instrument TMP117 using mbedOS. The devices uses an I2C communication interface

![Main](https://github.com/horeich/tmp117/blob/master/assets/TMP117_main.jpg)

## Overview
### Required hardware
For quick prototyping the following hardware is recommended:
- [NUCLEO-L476RG](https://os.mbed.com/platforms/ST-Nucleo-L476RG/) - a simple STM32 development board
- [Sparkfun-TMP117](https://www.sparkfun.com/products/15805) - a high precision temperature sensor breakout board

The library works with any board supported by mbedOS

### Library features
- [x] Full support of every device functionality
- [x] Easy to use API - write easily readable code within minutes ([see examples](https://github.com/horeich/tmp117/tree/master/examples))
- [x] Fully documented functions
- [ ] Examples for each functionality

## Hardware design
- External or internal pull-up/pull-down resistor can be set on alert pin line to MCU. Note that the device has an internal pull-push mechanism which "overrules" any pull-up. So low/high mode on alert pin will work in any case. When using an external resistor use ```PinMode::PullNone``` in ```set_output_pin_interrupt(...)```.
## Software design
The registers of the tmp117 device are 16bit wide

### Writing to registers
It is worth noting that programming certain functions of the TMP117 (e.g. alert mode/ therm mode) can be quite slow (up to 7ms). Therefore, we have to wait until the device returns the "not busy" flag which signals that data is successfully written to the EEPROM.

Note: See Figure 20 on datasheet p.18 for details on how to program the EEPROM.

### Resetting the device
The reset sequence ```soft_reset``` takes around 1.5ms and restores the default parameters. Note that the EEPROM is locked by default so call ```unlock_registers()``` before writing to the registers. See section 7.5 for further info

### Temperature conversion
- ```read_temperature()```: Note that we have to use the ```short``` data type to also cover negative temperature values (do not use ```unsigned short``` aka ```uint16_t```)
- The maximum possible value is 0b0111 1111 1111 1111 (255.9912F); the minimum possible value is -256.0f
### Recommendations (typical application)
- If possible use the one-shot mode to minimize self-heating of the device
- Therefore, in continuous mode it is recommended to use >1 second conversion cycle time and only 8 samples to calculate the average (see table in ```tmp116.hpp```)
- Use the minimal acceptable power supply voltage (1.8V)
- Use the hightest available communication speed (400kHz)

## Tests
### Unit tests
TODO
### Integration tests
Basic integration test for each function; does not include larger tests with temperature read-outs yet

## Examples
To run an example, include the respective header file into your main program and call the ```Run()``` method from there
Modify the mbed_lib.json file for the correct pins the device is connected to

### Run Examples
- 
### tmp117_oneshot.cpp
This example shows how to use of the one-shot mode
- Configure the the pins in the
- comment out/enable #define TMP117_CALLBACK to switch between callback (recommended!) and polling
TODO: alert mode and therm mode examples

### tmp117_continuous.cpp
This example shows how to use the continuous mode
- Averaging mode is stopped

### tmp117_write_config.cpp
TODO

### Known issues
- The I2C reset via 0x00 address pointer does not properly work (test fails)