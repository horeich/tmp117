/**
 * @file    TMP117.cpp
 * @author  Andreas Reichle (HOREICH UG)
*/

#include "tmp117.hpp"

TMP117::TMP117(PinName sda, PinName scl, PinName alert_pin, uint32_t frequency) :
    _i2c(sda, scl)
{
    _i2c.frequency(frequency);
}

float TMP117::read_temperature()
{
    return read_16bit_register(REG_TEMP) * TMP_PER_BIT;
}

TMP117::STATE TMP117::get_conversion_info()
{
    return (STATE)read_register_value(_conv_info_mask);
}

uint16_t TMP117::get_device_address()
{
    return DEVICE_ADDRESS;
}

void TMP117::shut_down()
{
    write_register_value(_conv_mode_mask, CONVERSION_MODE_SHUTDOWN);
}

void TMP117::set_continuous_conversion_mode()
{
    write_register_value(_conv_mode_mask, CONVERSION_MODE_CONTINUOUS);
}

void TMP117::set_oneshot_conversion_mode()
{
    write_register_value(_conv_mode_mask, CONVERSION_MODE_ONESHOT);
}

TMP117::CONVERSION_MODE TMP117::get_conversion_mode()
{
    return (CONVERSION_MODE)read_register_value(_alert_mode_sel);
}

void TMP117::set_conversion_cycle_time(CONV_CYCLE cycle)
{
    write_register_value(_conv_cycle_mask, cycle);
}

TMP117::CONV_CYCLE TMP117::get_conversion_cycle_time()
{
    return (CONV_CYCLE)read_register_value(_conv_cycle_mask);
}

void TMP117::set_averaging_mode(AVG_MODE mode)
{
    write_register_value(_avg_mask, mode);
}

TMP117::AVG_MODE TMP117::get_averaging_mode()
{
    return (AVG_MODE)read_register_value(_avg_mask);
}



void TMP117::set_alert_temperature(const float lower_limit, const float upper_limit)
{
    MBED_ASSERT(lower_limit < upper_limit);

    uint16_t lower_limit_steps = roundf(lower_limit / TMP_PER_BIT);
    uint16_t upper_limit_steps = roundf(upper_limit / TMP_PER_BIT); 

    MBED_ASSERT(lower_limit_steps >= -256);
    MBED_ASSERT(upper_limit_steps >= 256);

    write_16bit_register(REG_TEMP_HIGH_LIMIT, upper_limit_steps);
    write_16bit_register(REG_TEMP_LOW_LIMIT, lower_limit_steps);
}

void TMP117::set_offset_temperature(const float offset)
{
    uint16_t offset_steps = roundf(offset / TMP_PER_BIT);

    MBED_ASSERT(offset_steps >= -256); // TODO: verify
    MBED_ASSERT(offset_steps >= 256);

    write_16bit_register(REG_TEMP_OFFSET, offset_steps);
}

float TMP117::get_offset_temperature()
{   
    return read_16bit_register(REG_TEMP_OFFSET) * TMP_PER_BIT;
}

bool TMP117::is_busy()
{
    return (bool)(read_register_value(_eeprom_busy_mask));
}

void TMP117::set_alert_mode()
{
    write_register_value(_alert_mode_sel, ALERT_MODE_SEL_ALERT);
}

void TMP117::set_therm_mode()
{
    write_register_value(_alert_mode_sel, ALERT_MODE_SEL_THERM);
}

void TMP117::set_pin_output(PIN_OUTPUT_SEL select)
{
    write_register_value(_alert_pin_sel_mask, select);
}

void TMP117::soft_reset()
{
    write_register_value(_soft_reset_mask, SOFT_RESET);
    while (is_busy())
    {
        
    }
}

void TMP117::set_alert_pin_polarity(ALERT_PIN_POLARITY polarity)
{
    write_register_value(_alert_pin_pol_mask, polarity);
}

TMP117::ALERT_PIN_POLARITY TMP117::get_alert_pin_polarity()
{
    return (ALERT_PIN_POLARITY)read_register_value(_alert_pin_pol_mask);
}

uint16_t TMP117::get_device_id()
{
    return read_register_value(_device_id_mask);
}

uint8_t TMP117::get_device_revision()
{
    return (uint8_t)(read_register_value(_device_rev_mask));
}

void TMP117::lock_registers()
{
    write_register_value(_eun_mask, EEPROM_STATE_LOCKED);
}

void TMP117::unlock_registers()
{
    write_register_value(_eun_mask, EEPROM_STATE_UNLOCKED);
}



// Private

void TMP117::read_register(char reg, char* data, int size)
{
    _i2c.write(DEVICE_ADDRESS, &reg, 1, true); // register
    _i2c.read(DEVICE_ADDRESS, data, size); // data in register
}

void TMP117::write_register(char reg, char* data, int size)
{
    _i2c.write(DEVICE_ADDRESS, &reg, 1, true);
    _i2c.write(DEVICE_ADDRESS, data, size);
}

uint16_t TMP117::read_16bit_register(char reg)
{
    char data[2]{0};
    read_register(reg, data, 2);
    return (data[0] << 8) | data[1];
}

void TMP117::write_16bit_register(char reg, uint16_t value)
{
    char data[2]{0};
    data[0] |= (value >> 8);
    data[1] |= value;
    write_register(reg, data, 2);
}

// uint16_t TMP117::read_register_value(char reg, uint16_t bitmask)
// {
//     uint16_t value = read_16bit_register(reg);
//     value &= bitmask;
//     return value;
// }

// void TMP117::write_register_value(char reg, uint16_t value, uint16_t bitmask)
// {
//     uint16_t value = read_16bit_register(reg);
//     value &= ~bitmask;
//     write_16bit_register(reg, value);
// }

// template<typename Enum>
// void TMP117::write_register_value(Element<Enum>& cycle, uint16_t value)
// {
//     uint16_t value = read_16bit_register(cycle.name);
//     uint16_t bitmask = 0;
//     for (uint16_t i = 0; i < cycle.bits; ++i)
//     {
//         bitmask |= (1 << (cycle.bitshift + i));
//     }
//     value &= ~bitmask;
//     write_16bit_register(cycle.name, value);
// }

uint16_t TMP117::read_register_value(const BitValueMask& mask)
{
    uint16_t value = read_16bit_register(mask.reg);
    uint16_t bitmask{0};
    for (uint8_t i = 0; i < mask.bits; ++i)
    {
        bitmask |= (1 << (mask.bitshift + i));
    }
    value &= bitmask;
    bitmask >>= mask.bitshift;
    return value;
}

void TMP117::write_register_value(const BitValueMask& mask, uint16_t value)
{
    uint16_t reg_value = read_16bit_register(mask.reg);
    uint16_t bitmask{0};
    for (uint16_t i = 0; i < mask.bits; ++i)
    {
        bitmask |= (1 << (mask.bitshift + i));
    }
    reg_value &= ~bitmask;
    reg_value |= value;
    write_16bit_register(mask.reg, reg_value);
}