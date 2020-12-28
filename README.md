# Texas Instrument TMP117

Library for mbed devices

## Folder content
in the making

## Registers

The registers of the devices are 16bit wide

## Writing to a register
It's worth noting that programming certain functions of the TMP117 (e.g. alert mode/ therm mode) can be quite slow (typically 7ms). Therefore, we have to wait until the device returns the "not busy" flag which signals that data is successfully written to the EEPROM.

Note: See Figure 20 on datasheet p.18 for how to program the EEPROM.

## Resetting the device
The reset sequence ```soft_reset``` takes around 1.5ms and restores the default parameters. Note that the EEPROM is locked by default so call ```unlock_registers()``` before writing to the registers. See section 7.5 for further info

## Temperature conversion