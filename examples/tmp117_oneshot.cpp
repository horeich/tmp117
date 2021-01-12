/**
 * @file    tmp117_oneshot.cpp
 * @author  Andreas Reichle (HOREICH UG)
 */

#include "tmp117_oneshot.hpp"

using namespace std::chrono_literals;
using namespace mbed;

DigitalOut TMP117_OneShot::_led(PA_5, 0); // configured for NUCLEO-L476RG

#ifdef TMP117_CALLBACK
EventFlags TMP117_OneShot::_data_ready;
#endif

#ifdef TMP117_CALLBACK
void TMP117_OneShot::SignalDataReady()
{
    _led = !_led;
    _data_ready.set(1 << 0);
}
#endif

void TMP117_OneShot::Run()
{
    printf("### One-shot mode example ###\n");

    // Configure pins in mbed_lib.json
    TMP117 tmp117;

    // Reset standard config
    tmp117.soft_reset();

    #ifdef TMP117_CALLBACK
    // Configure as data ready pin with high on alert
    tmp117.set_output_pin_interrupt(
        TMP117::OUTPUT_PIN_MODE_DATA_READY,
        TMP117::OUTPUT_PIN_POL_ACTIVE_HIGH,
        mbed::callback(SignalDataReady)
    );
    #endif

    // Setup conversion cycle time and averaging mode
    tmp117.set_conversion_cycle_time(TMP117::CONV_CYCLE_1_S);
    tmp117.set_averaging_mode(TMP117::AVG_MODE_64_SAMPLES);

    for (uint8_t i = 0; i < 20; ++i)
    {
        // Initiate a one-shot conversion
        tmp117.set_oneshot_conversion_mode();

        #ifdef TMP117_CALLBACK

        // Wait for callback to signal data ready
        _data_ready.wait_all(1 << 0);
        #else

        // Poll for conversion results
        while (!(tmp117.get_conversion_state() & TMP117::STATE_DATA_READY)) {}
        #endif

        // Read conversion result
        printf("Temperature [°C] = %.4f\n", tmp117.read_temperature());

        // Sleep for a while
        rtos::ThisThread::sleep_for(3s);
    }

    tmp117.shut_down();
}