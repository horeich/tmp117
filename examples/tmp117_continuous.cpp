/**
 * @file    tmp117_continuous.cpp
 * @author  Andreas Reichle (HOREICH UG)
 */

#include "tmp117_continuous.hpp"

using namespace std::chrono_literals;
using namespace mbed;

DigitalOut TMP117_Continuous::Led(PA_5, 0); // configured for NUCLEO-L476RG

#ifdef TMP117_CALLBACK
EventFlags TMP117_Continuous::DataReady;
#endif

#ifdef TMP117_CALLBACK
void TMP117_Continuous::SignalDataReady()
{
    Led = !Led;
    DataReady.set(1 << 0);
}
#endif

void TMP117_Continuous::Run()
{
    printf("### One-shot mode example ###\n");

    TMP117 tmp117;

    // Reset standard config
    tmp117.soft_reset();
    
    #ifdef TMP117_CALLBACK
    // Configure as data ready pin with low on alert
    tmp117.set_output_pin_interrupt(
        TMP117::OUTPUT_PIN_MODE_DATA_READY,
        TMP117::OUTPUT_PIN_POL_ACTIVE_LOW,
        mbed::callback(SignalDataReady)
    );
    #endif

    // Setup conversion cycle time and no averaging mode
    tmp117.set_continuous_conversion_mode();
    tmp117.set_conversion_cycle_time(TMP117::CONV_CYCLE_4_S);
    tmp117.set_averaging_mode(TMP117::AVG_MODE_NO_AVG);

    for (uint8_t i = 0; i < 40; ++i)
    {
        #ifdef TMP117_CALLBACK

        // Wait for callback to signal data ready
        DataReady.wait_all(1 << 0);
        #else

        // Poll for conversion results
        while (!(tmp117.get_conversion_state() & TMP117::STATE_DATA_READY)) {}
        #endif

        // Read conversion result
        printf("Temperature [°C] = %.4f\n", tmp117.read_temperature());
    }

    tmp117.shut_down();
}