/**
 * @file    TMP117.hpp
 * @author  Andreas Reichle (HOREICH UG)
 * @TODO:   
 * - Check i2c locks
 * - 8bit bitshift value instead of 16bit
 */

#ifndef TMP117_HPP
#define TMP117_HPP

#include "drivers/I2C.h"
#include "drivers/InterruptIn.h"
#include "drivers/DigitalInOut.h"
#include "platform/Callback.h"
#include "mbed_error.h"
#include <math.h>
#include <memory>

// Enable for debug print output
#define MBED_CONF_TMP117_ENABLE_DEBUG_MODE 0

class TMP117
{

public:

    /*  Conversion Cycle Time in CC Mode
              AVG       0       1       2       3
      CONV  averaging  (0)     (8)     (32)   (64)
        0             15.5ms  125ms   500ms    1s     CONV_CYCLE_15_5_MS
        1             125ms   125ms   500ms    1s     CONV_CYCLE_125_MS
        2             250ms   250ms   500ms    1s     CONV_CYCLE_250_MS
        3             500ms   500ms   500ms    1s     CONV_CYCLE_500_MS
        4             1s      1s      1s       1s     CONV_CYLCE_1_S
        5             4s      4s      4s       4s     CONV_CYCLE_4_S
        6             8s      8s      8s       8s     CONV_CYCLE_8_S
        7             16s     16s     16s      16s    CONV_CYCLE_16_S
    */

    public: enum STATE
    { 
        STATE_DATA_READY                = 0x01,
        STATE_ALERT_LOW                 = 0x02,
        STATE_ALERT_HIGH                = 0x04,
    };

    public: enum CONVERSION_MODE
    {
        CONVERSION_MODE_CONTINUOUS      = 0x00,
        CONVERSION_MODE_SHUTDOWN        = 0x01,
        CONVERSION_MODE_ONESHOT         = 0x03,
        
    };

    enum CONV_CYCLE
    {
        CONV_CYCLE_15_5_MS              = 0x00,
        CONV_CYCLE_125_MS               = 0x01,
        CONV_CYCLE_250_MS               = 0x02,
        CONV_CYCLE_500_MS               = 0x03,
        CONV_CYCLE_1_S                  = 0x04,
        CONV_CYCLE_4_S                  = 0x05,
        CONV_CYCLE_8_S                  = 0x06,
        CONV_CYCLE_16_S                 = 0x07,
    };

    public: enum AVG_MODE
    {
        AVG_MODE_NO_AVG                 = 0x00,
        AVG_MODE_8_SAMPLES              = 0x01,
        AVG_MODE_32_SAMPLES             = 0x02,
        AVG_MODE_64_SAMPLES             = 0x03,
    };

    public: enum ALERT_MODE
    {
        ALERT_MODE_ALERT                = 0x00,
        ALERT_MODE_THERM                = 0x01,
    }; 

    public: enum OUTPUT_PIN_POLARITY
    {
        OUTPUT_PIN_POL_ACTIVE_LOW        = 0x00,
        OUTPUT_PIN_POL_ACTIVE_HIGH      = 0x01,   
    };

    public: enum OUTPUT_PIN_MODE
    {
        OUTPUT_PIN_MODE_ALERT            = 0x00,
        OUTPUT_PIN_MODE_DATA_READY       = 0x01,
    };

    private: enum EEPROM_STATE
    {
        EEPROM_STATE_LOCKED               = 0x00,
        EEPROM_STATE_UNLOCKED             = 0x01,
    };

private:

    struct BitValueMask
    {
        BitValueMask(uint16_t r, uint16_t b, uint16_t bs) : reg(r), bits(b), bitshift(bs) {};    
        uint16_t reg;
        uint16_t bits;
        uint16_t bitshift;
    };

public:

    TMP117(
        PinName sda = MBED_CONF_TMP117_SDA_PIN, 
        PinName scl = MBED_CONF_TMP117_SCL_PIN,
        uint32_t frequency = MBED_CONF_TMP117_FREQUENCY);
    ~TMP117() = default;

    /**
     * @brief                   Configures the output alert pin and callback
     * 
     * @param callback          Callback to be called in case of falling or rising edge 
     * @param output_pin        Pin number at MCU 
     * @param output_mode       Pin represents either therm oder alert mode
     * @param polarity          Pin either signals low oder high 
     * @param pin_mode          Use internal pull-up/ pull-down or none
     */
    void set_output_pin_interrupt(
        mbed::Callback<void()> callback, 
        OUTPUT_PIN_MODE output_mode = OUTPUT_PIN_MODE_DATA_READY, 
        OUTPUT_PIN_POLARITY polarity = OUTPUT_PIN_POL_ACTIVE_LOW, 
        PinMode pin_mode = PinMode::PullNone,
        PinName output_pin = MBED_CONF_TMP117_OUTPUT_PIN);

    void enable();

    void disable();

    /**
     * @brief           In therm mode the alert flag is cleared when the temp drops below the low
     *                  temp limit after exceeding the high limit once (functions as a hysterese)
     * @return          void
     */
    void set_therm_mode(void);

    /**
     * @brief           configures the device into alert mode
     *                  The alert flag is set when the temp goes above or below the given limits and 
     *                  is cleared when the conversion result or low/high alert status is read
     * @return          void              
     */
    void set_alert_mode(void);

    /**
     * 
     */
    ALERT_MODE get_alert_mode(void);

    /**
     * @brief           Reads out high and low alert flags als well as data ready flag
     * @return          states of the flags during last conversion cycle
     */
    STATE get_conversion_state(void);

    

    /**
     * @brief           During reset default values are written to all registers (may take up to 1.5ms);
     *                  after reset all registeres are locked
     * @return          void
     */
    void soft_reset(void);

    void i2c_reset();


    /**
     * @brief           Sets the offset temperature for custom system calibration
     * @param offset    Offset temperature
     * @return          void
     */
    void set_offset_temperature(float offset);

    /**
     * @brief           Returns the current polarity of the alert/output pin
     * @return          Polarity of the pin (0/1)
     */
    OUTPUT_PIN_POLARITY get_alert_pin_polarity(void);

    /**
     * @brief           Gets the offset temperture
     * @return          Offset temperature
     */
    float get_offset_temperature(void);

    /**
     * @brief           Returns the device address
     * @return          I2C address of the device
     */
    uint16_t get_device_address(void);

    /**
     * @brief           Reads the last temperature measurement; also clears the data_ready flag in the config register
     * @return          Current temperature measurement
     */
    float read_temperature(void);

    /**
     * @brief           Reads the device id
     * @return          Device id as 12 bit value
     */
    uint16_t get_device_id(void);

    /**
     * @brief           Reads the device revision
     * @return          Device revision as 3 bit value
     */
    uint8_t get_device_revision(void);



   

    /**
     * @brief           Forces the device into shut down mode (250nA QC); it is the default mode
     * @return          void
     */
    void shut_down(void);

    /**
     * @brief           Makes the device to perform continuous measurements with short sleep cycles in between;
     *                  the duration of the measurement cycle depends on the selected averaging mode
     * @return          void
     */
    void set_continuous_conversion_mode(void);

    /**
     * @brief           In one-shot conversion mode the device shuts down after performing a single conversion
     * @return          void
     */
    void set_oneshot_conversion_mode(void);
    
    /**
     * @brief           Gets the current conversion mode
     * @note            CONF READ is deleted
     * @return          conversion mode
     */
    CONVERSION_MODE get_conversion_mode(void);

    /**
     * @brief           Sets the averaging mode
     * @param mode      The averaging mode to be set (AVG_MODE)
     * @return          void
     */
    void set_averaging_mode(AVG_MODE mode);

    /**
     * @brief           Returns the averaging mode
     * @return          Average mode
     */
    AVG_MODE get_averaging_mode(void);

    /**
     * @brief           Sets the standby time between conversion cycles (see table)
     * @param cycle     Conversion cycle time
     * @return          void
     *                  
     */
    void set_conversion_cycle_time(CONV_CYCLE cylce);

    /**
     * @brief           Returns the current conversion cycle time
     * @return          Conversion cycle time as enum
     */
    CONV_CYCLE get_conversion_cycle_time();

    /**
     * @brief               Sets the upper and lower alert limit
     * @param lower_limit   The lower alert limit in °C
     * @param upper_limit   The upper alert limit in °C
     * @return              void
     */
    void set_alert_limits(const float lower_limit, const float upper_limit);

    /**
     * 
     */
    void get_alert_limits(float& lower_limit, float& upper_limit);

    /**
     * @brief           Locks the EEPROM so data cannot be programmed to the EEPROM
     * @return          void
     */
    void lock_registers(void);

    /**
     * @brief           Unlocks the EEPROM so data can be programmed to the EEPROM
     * @return          void
     */
    void unlock_registers(void);

    /**
     * @brief           Returns whether EEPROM is unlocked or not
     * @return          true if unlocked, false if locked
     */
    bool registers_unlocked();

private:

    /**
     * @brief           Selects the ALERT pin to either function as data ready or alert output
     * @param mode      OUTPUT_PIN_MODE: pin reflects status of alert flag
     *                  or pin reflects status of data ready flag
     * @return          void          
     */
    void set_output_pin_mode(OUTPUT_PIN_MODE mode);

    /**
     * @brief           Polarity (high or low) of alert/output pin
     * @param polarity  high or low polarity
     * @return          void
     */
    void set_output_pin_polarity(OUTPUT_PIN_POLARITY polarity);

    void wait_ready();

    bool is_busy(void);

    /**
     * @brief           Reads from connected device via I2C protocol
     * @param reg       The start address of the registers to be read
     * @param data      char pointer to the read data
     * @param size      Number of bytes to be read   
     * @return          void
     */
    void read_register(char reg, char* data, int size);
    void write_register(char reg, char* data, int size);

    void write_16bit_register(char reg, uint16_t value);
    uint16_t read_16bit_register(char reg);

    uint16_t write_value(const BitValueMask& mask, uint16_t value);
    uint16_t read_value(const BitValueMask& mask);

private:

    PinName _sda_pin;
    PinName _scl_pin;
    uint32_t _frequency;
    std::unique_ptr<mbed::I2C> _i2c;                                    // <I2C interface>
    std::unique_ptr<mbed::DigitalInOut> _sda;
    std::unique_ptr<mbed::DigitalInOut> _scl;
    std::unique_ptr<mbed::InterruptIn> _isr;                            // <alert interrupt pin>
    std::unique_ptr<mbed::DigitalInOut> _isr_idle;

    static constexpr float TMP_PER_BIT                 = 0.0078125f;    // <temperature step in K per bit (256.0 / 32768)>
    static constexpr float MAX_TEMPERATURE             = 255.9921f;     // <maximum temperature (0b0111 1111 1111 1111 * 0.0078125f)>
    static constexpr float MIN_TEMPERATURE             = -256.0f;       // <minimum temperature (0x1000 0000 0000 0000 * 0.0078125f)>
    static constexpr uint16_t DEVICE_ADDRESS           = 0x90;          // <device I2C address>

    static constexpr uint16_t EEPROM_STATE_BUSY        = 0x01;
    static constexpr uint16_t SOFT_RESET               = 0x01;
    static constexpr uint8_t I2C_RESET                 = 0x06;

    static constexpr uint16_t REG_TEMP                 = 0x00;
    static constexpr uint16_t REG_CONFIG               = 0x01;
    static constexpr uint16_t REG_TEMP_HIGH_LIMIT      = 0x02;
    static constexpr uint16_t REG_TEMP_LOW_LIMIT       = 0x03;
    static constexpr uint16_t REG_EEPROM_UNLOCK        = 0x04; // <default: 0x00>
    static constexpr uint16_t REG_EEPROM1              = 0x05;
    static constexpr uint16_t REG_EEPROM2              = 0x06;
    static constexpr uint16_t REG_EEPROM3              = 0x08;
    static constexpr uint16_t REG_TEMP_OFFSET          = 0x07;
    static constexpr uint16_t REG_DEVICE_ID            = 0x0F;
    
    // CONFIGURATION REGISTER (0x01)
    const BitValueMask _conv_state_mask     {REG_CONFIG, 3, 13};
    const BitValueMask _conv_mode_mask      {REG_CONFIG, 2, 10};
    const BitValueMask _conv_cycle_mask     {REG_CONFIG, 3, 7 };
    const BitValueMask _avg_mask            {REG_CONFIG, 2, 5 };
    const BitValueMask _alert_mode_sel      {REG_CONFIG, 1, 4 };
    const BitValueMask _alert_pin_pol_mask  {REG_CONFIG, 1, 3 };
    const BitValueMask _alert_pin_sel_mask  {REG_CONFIG, 1, 2 };
    const BitValueMask _soft_reset_mask     {REG_CONFIG, 1, 1 };

    // EEPROM UNLOCK REGISTER (0x04)
    const BitValueMask _eun_mask            {REG_EEPROM_UNLOCK, 1, 15};
    const BitValueMask _eeprom_busy_mask    {REG_EEPROM_UNLOCK, 1, 14};

    // DEVICE ID REGISTER (0x0F)
    const BitValueMask _device_rev_mask     {REG_DEVICE_ID, 3, 12};
    const BitValueMask _device_id_mask      {REG_DEVICE_ID, 15, 0};

};

#endif // TMP117_HPP