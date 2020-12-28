/**
 * @file    TMP117.hpp
 * @author  Andreas Reichle (HOREICH UG)
 * 
 * ToDo: Mutex
 */

#ifndef TMP117_HPP
#define TMP117_HPP

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

#include "drivers/I2C.h"
#include "drivers/InterruptIn.h"
#include "platform/Callback.h"
#include "mbed_error.h"
#include <math.h>
#include <memory>

#ifdef MBED_CONF_TMP117_ENABLE_DEBUG_MODE
#endif

class TMP117
{

public:

    enum CONVERSION_TIME
    {
        CONVERSION_TIME
    };

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
        7             16s     16s     16s      16s    CONV_CYCLE_BITMASK
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
        CONV_CYLCE_1_S                  = 0x04,
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
        ALERT_PIN_POL_ACTIVE_LOW        = 0x00,
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

    //template<typename Enum>
    class BitValueMask
    {
        public:

        //template<typename Enum>
        BitValueMask(uint16_t r, uint16_t b, uint16_t bs) : reg(r), bits(b), bitshift(bs) {};
        
        uint16_t reg;
        uint16_t bits;
        uint16_t bitshift;
        //Enum values;
    };

    // Element<CONV_CYCLE>* _conversionCycle = new Element<CONV_CYCLE>(convCycle, TMP117_REG_CONFIG, 3, 9);
    // Element<CONVERSION_MODE>* _conversionMode = new Element<CONVERSION_MODE>(convMode, TMP117_REG_CONFIG, 2, 10);

public:

    TMP117(
        PinName sda = MBED_CONF_TMP117_SDA_PIN, 
        PinName scl = MBED_CONF_TMP117_SCL_PIN,
        uint32_t frequency = MBED_CONF_TMP117_FREQUENCY);
    ~TMP117() = default;

    /**
     * @brief           
     */
    void set_output_pin_interrupt(
        OUTPUT_PIN_MODE mode,
        OUTPUT_PIN_POLARITY polarity,
        mbed::Callback<void()>& cb,
        PinName alert_pin = MBED_CONF_TMP117_OUTPUT_PIN);

    /**
     * @brief           configures the device into therm mode
     *                  The alert flag is cleared when the temp drops below low temp limit after exceeding high limit once (hysterese mode)
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
    STATE get_conversion_info(void);

    /**
     * @brief           Selects the ALERT pin to either function as data ready or alert output
     * @param mode      PIN_SEL_ALERT_OUTPUT: pin reflects status of alert flag
     *                  PIN_SEL_DATA_READY_OUTPUT: pin reflects status of data ready flag
     * @return          void          
     */
    void set_output_pin_mode(OUTPUT_PIN_MODE mode);

    /**
     * @brief           Runs the power-on reset sequence
     *                  During reset default values are written to all the registers (may take up to 1.5ms)
     * @return          void
     */
    void soft_reset(void);

    /**
     * @brief           Sets the offset temperature for custom system calibration
     * @param offset    Offset temperature
     * @return          void
     */
    void set_offset_temperature(float offset);

    /**
     * @brief           Polarity (high or low) of alert/output pin
     * @param polarity  high or low polarity
     * @return          void
     */
    void set_output_pin_polarity(OUTPUT_PIN_POLARITY polarity);

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
     * @brief           Reads the last temperature measurement
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
     * @brief           Locks the eeprom so data can be programmed to the EEPROM
     * @return          void
     */
    void lock_registers();

    /**
     * @brief           Unlocks the eeprom so data can be programmed to the EEPROM
     * @return          void
     */
    uint16_t unlock_registers(void);

    bool registers_unlocked();

    /**
     * @brief           Forces the device into shut down mode (250nA QC)
     * @return          void
     */
    void shut_down(void);

    /**
     * @brief           Sets the continouous conversion mode
     *                  Device performs continuous measurements and goes to sleep in between
     *                  Conversion cycle depends on averaging mode
     * @return          void
     */
    void set_continuous_conversion_mode(void);

    /**
     * @brief           Sets the one-shot conversion mode
     *                  Device shuts down after one conversion
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
     * @param mode      The averaging mode to be set
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
     * @param           Conversion cycle time
     * @return          void
     *                  
     */
    void set_conversion_cycle_time(CONV_CYCLE cylce);

    
    CONV_CYCLE get_conversion_cycle_time();

    /**
     * @brief               Sets the upper and lower alert limit
     * @param lower_limit   The lower alert limit in °C
     * @param upper_limit   The upper alert limit in °C
     * @return              void
     */
    void set_alert_limits(const float lower_limit, const float upper_limit);

    void get_alert_limits(float& lower_limit, float& upper_limit);

    // TODO: hard_reset

private:

    /**
     * @brief TODO timer
     */
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

    // Deprecated
    // void write_register_value(char reg, uint16_t value, uint16_t bitmask);
    // uint16_t read_register_value(char reg, uint16_t bitmask);

    // Deprecated
    //template<class Enum>
    //void write_register_value(Element<Enum>& cycle, uint16_t value);

    uint16_t write_value(const BitValueMask& mask, uint16_t value);
    uint16_t read_value(const BitValueMask& mask);

private:

    mbed::I2C _i2c;                                     // <I2C interface>
    std::unique_ptr<mbed::InterruptIn> _isr;

    static constexpr float TMP_PER_BIT = 0.0078125f;    // <Temperature in K per bit>
    static constexpr uint16_t DEVICE_ADDRESS = 0x90;    // <Device I2C address>

    static constexpr uint16_t EEPROM_STATE_BUSY        = 0x01;
    static constexpr uint16_t SOFT_RESET               = 0x01;

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
    const BitValueMask _conv_info_mask      {REG_CONFIG, 3, 13};
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