/**
 * @file    tmp117_oneshot.hpp
 * @author  Andreas Reichle (HOREICH UG)
 */

#include "../src/tmp117.hpp"
#include "mbed.h"
#include "rtos.h"

class TMP117_OneShot
{
public:
    TMP117_OneShot() = default;
    ~TMP117_OneShot() = default;

    static void Run();

private:

    static void SignalDataReady();

private:

    static mbed::DigitalOut _led;
    static rtos::EventFlags _data_ready;
};