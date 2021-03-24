/**
 * @file    TMP117.cpp
 * @author  Andreas Reichle (HOREICH UG)
 * TODO:
 * - interrupt pin pullup/pulldown fix?
 * - Add locks to i2c connection
 */

#include "tmp117.hpp"

#if MBED_CONF_TMP117_ENABLE_DEBUG_MODE == 1
#define BYTE_PLACEHOLDER "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BIN(byte) \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

#define debug_print(...)      printf(__VA_ARGS__)
#else
#define debug_print(...)   
#endif

TMP117::TMP117(PinName sda, PinName scl, uint32_t frequency) :
    _sda_pin(sda),
    _scl_pin(scl),
    _frequency(frequency),
    _i2c(nullptr),
    _sda(nullptr),
    _scl(nullptr),
    _isr(nullptr),
    _isr_idle(nullptr)
{

}

void TMP117::set_output_pin_interrupt(
    mbed::Callback<void()> callback, 
    OUTPUT_PIN_MODE output_mode, 
    OUTPUT_PIN_POLARITY polarity, 
    PinMode pin_mode,
    PinName output_pin)
{
    debug_print("TMP117::%s\n", __func__);
    if (callback)
    {
        set_output_pin_mode(output_mode);
        set_output_pin_polarity(polarity);
        _isr_idle.reset();
        _isr = std::make_unique<mbed::InterruptIn>(output_pin, pin_mode);
        if (polarity == OUTPUT_PIN_POL_ACTIVE_HIGH)
        {
            _isr->rise(callback);
        }
        else
        {
            _isr->fall(callback);
        }
    }
    else
    {
        _isr.reset();
        _isr_idle = std::make_unique<mbed::DigitalInOut>(output_pin, PIN_INPUT, PinMode::PullDown, 0);
        // TODO: low power pin mode analog in?
    }
}

void TMP117::enable()
{
    if (!_i2c)
    {
        _sda.reset();
        _scl.reset();
        _i2c = std::make_unique<mbed::I2C>(_sda_pin, _scl_pin);
        _i2c->frequency(_frequency);
    }
}

void TMP117::disable()
{
    if(_i2c)
    {
        _i2c.reset();
        _sda = std::make_unique<mbed::DigitalInOut>(_sda_pin, PIN_INPUT, PinMode::PullDown, 0);
        _scl = std::make_unique<mbed::DigitalInOut>(_scl_pin, PIN_INPUT, PinMode::PullDown, 0); 
    }
}

float TMP117::read_temperature()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return static_cast<short>(read_16bit_register(REG_TEMP)) * TMP_PER_BIT;
}

TMP117::STATE TMP117::get_conversion_state()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return (STATE)read_value(_conv_state_mask);
}

uint16_t TMP117::get_device_address()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return DEVICE_ADDRESS;
}

void TMP117::shut_down()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_conv_mode_mask, CONVERSION_MODE_SHUTDOWN);
}

void TMP117::set_continuous_conversion_mode()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_conv_mode_mask, CONVERSION_MODE_CONTINUOUS);
    wait_ready();
}

void TMP117::set_oneshot_conversion_mode()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_conv_mode_mask, CONVERSION_MODE_ONESHOT);
    //wait_ready();
}

TMP117::CONVERSION_MODE TMP117::get_conversion_mode()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return static_cast<CONVERSION_MODE>(read_value(_conv_mode_mask));
}

void TMP117::set_conversion_cycle_time(CONV_CYCLE cycle)
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_conv_cycle_mask, cycle);
    wait_ready();
}

TMP117::CONV_CYCLE TMP117::get_conversion_cycle_time()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return static_cast<CONV_CYCLE>(read_value(_conv_cycle_mask));
}

void TMP117::set_averaging_mode(AVG_MODE mode)
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_avg_mask, mode);
    wait_ready();
}

TMP117::AVG_MODE TMP117::get_averaging_mode()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return static_cast<AVG_MODE>(read_value(_avg_mask));
}

void TMP117::set_alert_limits(const float lower_limit, const float upper_limit)
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    MBED_ASSERT(lower_limit < upper_limit);
    MBED_ASSERT(lower_limit >= MIN_TEMPERATURE);
    MBED_ASSERT(upper_limit <= MAX_TEMPERATURE);

    short lower_limit_steps = roundf(lower_limit / TMP_PER_BIT);
    short upper_limit_steps = roundf(upper_limit / TMP_PER_BIT); 

    write_16bit_register(REG_TEMP_LOW_LIMIT, lower_limit_steps);
    wait_ready();
    write_16bit_register(REG_TEMP_HIGH_LIMIT, upper_limit_steps);
    wait_ready();
}

void TMP117::get_alert_limits(float& lower_limit, float& upper_limit)
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    lower_limit = static_cast<short>(read_16bit_register(REG_TEMP_LOW_LIMIT)) * TMP_PER_BIT;
    upper_limit = static_cast<short>(read_16bit_register(REG_TEMP_HIGH_LIMIT)) * TMP_PER_BIT;
}

void TMP117::set_offset_temperature(const float offset)
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    MBED_ASSERT(offset >= MIN_TEMPERATURE); // TODO: verify
    MBED_ASSERT(offset <= MAX_TEMPERATURE);

    short offset_steps = roundf(offset / TMP_PER_BIT);

    write_16bit_register(REG_TEMP_OFFSET, offset_steps);
    wait_ready();
}

float TMP117::get_offset_temperature()
{   
    debug_print("TMP117::%s\n", __func__);
    enable();

    return read_16bit_register(REG_TEMP_OFFSET) * TMP_PER_BIT;
}

void TMP117::set_alert_mode()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_alert_mode_sel, ALERT_MODE_ALERT);
    wait_ready();
}

void TMP117::set_therm_mode()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_alert_mode_sel, ALERT_MODE_THERM);
    wait_ready();
}

TMP117::ALERT_MODE TMP117::get_alert_mode()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return (ALERT_MODE)read_value(_alert_mode_sel);
}

void TMP117::soft_reset()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_soft_reset_mask, SOFT_RESET);
    wait_ready(); // may take up to 2ms
}

void TMP117::i2c_reset()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    char reset = I2C_RESET;
    //_i2c.write(0x00, &reset, 1);
    write_register(0x00, &reset, 1);
}

TMP117::OUTPUT_PIN_POLARITY TMP117::get_alert_pin_polarity()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return (OUTPUT_PIN_POLARITY)read_value(_alert_pin_pol_mask);
}

uint16_t TMP117::get_device_id()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return read_value(_device_id_mask);
}

uint8_t TMP117::get_device_revision()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return static_cast<uint8_t>(read_value(_device_rev_mask));
}

void TMP117::lock_registers()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_eun_mask, EEPROM_STATE_LOCKED);
    wait_ready();
}

void TMP117::unlock_registers()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    write_value(_eun_mask, EEPROM_STATE_UNLOCKED);
    wait_ready();
}

bool TMP117::registers_unlocked()
{
    debug_print("TMP117::%s\n", __func__);
    enable();

    return static_cast<bool>(read_value(_eun_mask));
}

/************************************************************************************
 * PRIVATE MEMBERS
 ***********************************************************************************/

void TMP117::set_output_pin_mode(OUTPUT_PIN_MODE mode)
{
    write_value(_alert_pin_sel_mask, mode);
    wait_ready();
}

void TMP117::set_output_pin_polarity(OUTPUT_PIN_POLARITY polarity)
{
    write_value(_alert_pin_pol_mask, polarity);
    wait_ready();
}

void TMP117::wait_ready()
{
    debug_print("TMP117::%s\n", __func__);
    while(is_busy()) {}
}

bool TMP117::is_busy()
{
    return static_cast<bool>(read_value(_eeprom_busy_mask));
}

void TMP117::read_register(char reg, char* data, int size)
{
    int rc = _i2c->write(DEVICE_ADDRESS, &reg, 1, true); // register
    if (rc != MBED_SUCCESS)
    {
        MBED_WARNING1(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_WRITE_FAILED), "TMP117", rc);
    }
    rc = _i2c->read(DEVICE_ADDRESS, data, size); // data in register
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
    int rc = _i2c->write(DEVICE_ADDRESS, data_tmp, size + 1);
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

uint16_t TMP117::read_value(const BitValueMask& mask)
{
    uint16_t value = read_16bit_register(mask.reg);

    debug_print("##Read 0b " BYTE_PLACEHOLDER" " BYTE_PLACEHOLDER"", 
            BYTE_TO_BIN(value>>8), BYTE_TO_BIN(value));
    debug_print("\n");

    uint16_t bitmask{0};
    for (uint8_t i = 0; i < mask.bits; ++i)
    {
        bitmask |= (1 << (mask.bitshift + i));
    }
    value &= bitmask;
    value >>= mask.bitshift;

    debug_print("Value = %d\n\n", value);

    return value;
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

    write_16bit_register(mask.reg, reg_value);
    
    debug_print("##Wrote 0b " BYTE_PLACEHOLDER" " BYTE_PLACEHOLDER"", 
            BYTE_TO_BIN(reg_value>>8), BYTE_TO_BIN(reg_value));
    debug_print("\n");

    return value;
}