/**
 * @file    tmp117_continuous.hpp
 * @author  Andreas Reichle (HOREICH UG)
 */

#include "../src/tmp117.hpp"
#include "mbed.h"
#include "rtos.h"

class TMP117_Continuous
{
public:
    TMP117_Continuous() = default;
    ~TMP117_Continuous() = default;

    static void Run();

private:

    static void SignalDataReady();

private:

    static mbed::DigitalOut Led;
    static rtos::EventFlags DataReady;
};