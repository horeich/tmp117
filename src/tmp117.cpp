/**
 * @file    TMP117.cpp
 * @author  Andreas Reichle (HOREICH UG)
*/

#include "tmp117.hpp"

TMP117::TMP117(PinName sda, PinName scl, uint32_t frequency) :
    _i2c(sda, scl)
{
    _i2c.frequency(frequency);
}

void TMP117::set_output_pin_interrupt(
    OUTPUT_PIN_MODE mode, 
    OUTPUT_PIN_POLARITY polarity, 
    mbed::Callback<void()>& cb, 
    PinName output_pin)
{
    set_output_pin_mode(mode);
    set_output_pin_polarity(polarity);
    if (output_pin != PinName::NC)
    {
        _isr = std::make_unique<mbed::InterruptIn>(output_pin, PinMode::PullNone);
        if (polarity == OUTPUT_PIN_POL_ACTIVE_HIGH)
        {
            _isr->rise(cb);
        }
        else
        {
            _isr->fall(cb);
        }
    }
    else
    {
        _isr.reset();
    }
}

float TMP117::read_temperature()
{
    return read_16bit_register(REG_TEMP) * TMP_PER_BIT;
}

TMP117::STATE TMP117::get_conversion_info()
{
    return (STATE)read_value(_conv_info_mask);
}

uint16_t TMP117::get_device_address()
{
    return DEVICE_ADDRESS;
}

void TMP117::shut_down()
{
    write_value(_conv_mode_mask, CONVERSION_MODE_SHUTDOWN);
}

void TMP117::set_continuous_conversion_mode()
{
    write_value(_conv_mode_mask, CONVERSION_MODE_CONTINUOUS);
}

void TMP117::set_oneshot_conversion_mode()
{
    write_value(_conv_mode_mask, CONVERSION_MODE_ONESHOT);
}

TMP117::CONVERSION_MODE TMP117::get_conversion_mode()
{
    return (CONVERSION_MODE)read_value(_alert_mode_sel);
}

void TMP117::set_conversion_cycle_time(CONV_CYCLE cycle)
{
    write_value(_conv_cycle_mask, cycle);
}

TMP117::CONV_CYCLE TMP117::get_conversion_cycle_time()
{
    return (CONV_CYCLE)read_value(_conv_cycle_mask);
}

void TMP117::set_averaging_mode(AVG_MODE mode)
{
    write_value(_avg_mask, mode);
}

TMP117::AVG_MODE TMP117::get_averaging_mode()
{
    return (AVG_MODE)read_value(_avg_mask);
}

void TMP117::set_alert_limits(const float lower_limit, const float upper_limit)
{
    MBED_ASSERT(lower_limit < upper_limit);
    MBED_ASSERT(lower_limit >= -256.0f);
    MBED_ASSERT(upper_limit <= 256.0f); // TODO: correct value?

    uint16_t lower_limit_steps = roundf(lower_limit / TMP_PER_BIT);
    uint16_t upper_limit_steps = roundf(upper_limit / TMP_PER_BIT); 

    printf("lower steps: %d, upper steps: %d\n", lower_limit_steps, upper_limit_steps);

    write_16bit_register(REG_TEMP_LOW_LIMIT, lower_limit_steps);
    wait_ready();
    write_16bit_register(REG_TEMP_HIGH_LIMIT, upper_limit_steps);
    wait_ready();
}

void TMP117::get_alert_limits(float& lower_limit, float& upper_limit)
{
    lower_limit = read_16bit_register(REG_TEMP_LOW_LIMIT) * TMP_PER_BIT;
    upper_limit = read_16bit_register(REG_TEMP_HIGH_LIMIT) * TMP_PER_BIT;
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

void TMP117::wait_ready()
{
    while(is_busy()) {}
}

bool TMP117::is_busy()
{
    return static_cast<bool>(read_value(_eeprom_busy_mask));
}

void TMP117::set_alert_mode()
{
    write_value(_alert_mode_sel, ALERT_MODE_ALERT);
    while(is_busy())
    {
        printf("waiting...\n");
    }
}

void TMP117::set_therm_mode()
{
    write_value(_alert_mode_sel, ALERT_MODE_THERM);
    while (is_busy())
    {
        printf(".");
    }
    printf("\n");
}

TMP117::ALERT_MODE TMP117::get_alert_mode()
{
    return (ALERT_MODE)read_value(_alert_mode_sel);
}

void TMP117::set_output_pin_mode(OUTPUT_PIN_MODE mode)
{
    write_value(_alert_pin_sel_mask, mode);
}

void TMP117::soft_reset()
{
    write_value(_soft_reset_mask, SOFT_RESET);
    wait_ready(); // may take up to 2ms
}

void TMP117::set_output_pin_polarity(OUTPUT_PIN_POLARITY polarity)
{
    write_value(_alert_pin_pol_mask, polarity);
}

TMP117::OUTPUT_PIN_POLARITY TMP117::get_alert_pin_polarity()
{
    return (OUTPUT_PIN_POLARITY)read_value(_alert_pin_pol_mask);
}

uint16_t TMP117::get_device_id()
{
    return read_value(_device_id_mask);
}

uint8_t TMP117::get_device_revision()
{
    return (uint8_t)(read_value(_device_rev_mask));
}

void TMP117::lock_registers()
{
    write_value(_eun_mask, EEPROM_STATE_LOCKED);
}

uint16_t TMP117::unlock_registers()
{
    return write_value(_eun_mask, EEPROM_STATE_UNLOCKED);
}

bool TMP117::registers_unlocked()
{
    return static_cast<bool>(read_value(_eun_mask));
}

// Private

void TMP117::read_register(char reg, char* data, int size)
{
    int rc = _i2c.write(DEVICE_ADDRESS, &reg, 1, true); // register
    if (rc != MBED_SUCCESS)
    {
        MBED_WARNING1(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_WRITE_FAILED), "TMP117", rc);
    }
    rc = _i2c.read(DEVICE_ADDRESS, data, size); // data in register
    if (rc != MBED_SUCCESS)
    {
        MBED_WARNING1(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_READ_FAILED), "TMP117", rc);
    }
}

void TMP117::write_register(char reg, char* data, int size)
{
    char* data_tmp = (char*)malloc(sizeof(char) * size + 1);
    data_tmp[0] = reg;
    if (size > 0) 
    {
        memcpy(&data_tmp[1], data, size);
    }
    int rc = _i2c.write(DEVICE_ADDRESS, data_tmp, size + 1);
    if (rc != MBED_SUCCESS)
    {
        MBED_WARNING1(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_WRITE_FAILED), "TMP117", rc);
    }
    free(data_tmp);
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

uint16_t TMP117::read_value(const BitValueMask& mask)
{
    uint16_t reg_value = read_16bit_register(mask.reg);

    printf("##Read register value = 0b " BYTE_PLACEHOLDER" " BYTE_PLACEHOLDER"", 
            BYTE_TO_BIN(reg_value>>8), BYTE_TO_BIN(reg_value));
    printf("\n");

    uint16_t bitmask{0};
    for (uint8_t i = 0; i < mask.bits; ++i)
    {
        bitmask |= (1 << (mask.bitshift + i));
    }
    reg_value &= bitmask;
    reg_value >>= mask.bitshift;

    printf("Return value = %d\n\n", reg_value);

    return reg_value;
}

uint16_t TMP117::write_value(const BitValueMask& mask, uint16_t value)
{
    uint16_t reg_value = read_16bit_register(mask.reg);
    uint16_t bitmask{0};
    for (uint8_t i = 0; i < mask.bits; ++i)
    {
        bitmask |= (1 << (mask.bitshift + i));
    }
    value <<= mask.bitshift;
    reg_value &= ~bitmask;
    reg_value |= value;

    printf("##Write register value = 0b " BYTE_PLACEHOLDER" " BYTE_PLACEHOLDER"", 
            BYTE_TO_BIN(reg_value>>8), BYTE_TO_BIN(reg_value));
    printf("\n");

    write_16bit_register(mask.reg, reg_value);
    return value;
}