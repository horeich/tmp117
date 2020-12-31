#include "itest_tmp117.hpp"
#include "drivers/DigitalOut.h"

#include "rtos/ThisThread.h"

TMP117* _device = new TMP117(PB_9, PB_8, 400000);;
mbed::DigitalOut led(PA_5, 0);

void data_ready_cb()
{
    led = !led;
}

void setUp(void)
{
    // printf("-----------------------START SETUP---------------------------\n");
    
    // _device->soft_reset();
    // printf("------------------------END SETUP----------------------------\n");
}

void tearDown(void)
{
    // printf("---------------------START TEARDOWN--------------------------\n");
    // _device->shut_down();
    // delete _device;
    // printf("----------------------END TEARDOWN---------------------------\n");
}


// Config register

void test_set_one_shot_conversion_mode()
{
    _device->set_oneshot_conversion_mode();
    TEST_ASSERT_EQUAL_UINT16(TMP117::CONVERSION_MODE_ONESHOT, _device->get_conversion_mode());
}

void test_set_continuous_conversion_mode()
{
    _device->set_continuous_conversion_mode();
    TEST_ASSERT_EQUAL_UINT16(TMP117::CONVERSION_MODE_CONTINUOUS, _device->get_conversion_mode());
}

void test_shutdown()
{
    _device->shut_down();
    TEST_ASSERT_EQUAL_UINT16(TMP117::CONVERSION_MODE_SHUTDOWN, _device->get_conversion_mode());
}

void test_set_averaging_mode()
{
    _device->set_averaging_mode(TMP117::AVG_MODE_64_SAMPLES); 
    TEST_ASSERT_EQUAL_UINT16(TMP117::AVG_MODE_64_SAMPLES, _device->get_averaging_mode());
}

void test_set_conversion_cycle_time()
{
    _device->set_conversion_cycle_time(TMP117::CONV_CYCLE_4_S);
    TEST_ASSERT_EQUAL_UINT16(TMP117::CONV_CYCLE_4_S, _device->get_conversion_cycle_time());
}

void test_set_therm_mode()
{
    _device->set_therm_mode();
    TEST_ASSERT_EQUAL_UINT16(TMP117::ALERT_MODE_THERM, _device->get_alert_mode());
}

void test_set_alert_mode()
{
    _device->set_alert_mode();
    TEST_ASSERT_EQUAL_UINT16(TMP117::ALERT_MODE_ALERT, _device->get_alert_mode());
}

void test_soft_reset()
{
    _device->unlock_registers();
    _device->set_therm_mode();
    _device->lock_registers();
    _device->set_alert_mode();
    _device->soft_reset();
    TEST_ASSERT_EQUAL_UINT16(TMP117::ALERT_MODE_THERM, _device->get_alert_mode());
}

void test_i2c_reset()
{
    _device->unlock_registers();
    _device->set_therm_mode();
    _device->lock_registers();
    _device->set_alert_mode();
    _device->i2c_reset();
    TEST_ASSERT_EQUAL_UINT16(TMP117::ALERT_MODE_THERM, _device->get_alert_mode());
}

void test_set_output_pin_interrupt()
{
    _device->set_output_pin_interrupt(
        TMP117::OUTPUT_PIN_MODE_DATA_READY,
        TMP117::OUTPUT_PIN_POL_ACTIVE_HIGH,
        mbed::callback(data_ready_cb),
        PC_9
    );
}



void test_set_limits()
{
    _device->set_alert_limits(-256.0f, 25.46f);
    float upper_limit, lower_limit;
    _device->get_alert_limits(lower_limit, upper_limit);
    //TEST_ASSERT_FLOAT_WITHIN(lower_limit,);
    printf("lower: %f, upper: %f", lower_limit, upper_limit);
}



void test_read_temperature()
{
    int info = _device->get_conversion_state();
    if (info & TMP117::STATE_DATA_READY)
    {
        float value = _device->read_temperature();
        printf("value = %f", value);
        TEST_ASSERT_FLOAT_WITHIN(256.0f, 0.0f, value);
    }
    else
    {
        TEST_ASSERT_TRUE(true);
    }
}

void run_itest_tmp117()
{
    printf("##Run TMP117 test##\n");

    UNITY_BEGIN();

    // Run lock device

    // Test basic functions

    RUN_TEST(test_set_continuous_conversion_mode);

    RUN_TEST(test_set_one_shot_conversion_mode);

    RUN_TEST(test_shutdown);

    RUN_TEST(test_set_conversion_cycle_time);

    RUN_TEST(test_set_averaging_mode);

    RUN_TEST(test_set_therm_mode);

    RUN_TEST(test_set_alert_mode);

    RUN_TEST(test_soft_reset);

    RUN_TEST(test_i2c_reset);
    


    //RUN_TEST(test_soft_reset);
    // Test run according to datasheet p.18

    // RUN_TEST(test_lock_registers);

    // RUN_TEST(test_unlock_registers);

    // RUN_TEST(test_set_therm_mode);

    // RUN_TEST(test_set_alert_mode);

    // RUN_TEST(test_set_limits);

    // RUN_TEST(test_set_output_pin_interrupt);

    // RUN_TEST(test_set_continuous_conversion_mode);

    // //RUN_TEST(read_temperature_iterative)

    // for (int i = 0; i < 20; ++i)
    // {
        
    //     RUN_TEST(test_read_temperature);
    //     rtos::ThisThread::sleep_for(500);
    // }

    UNITY_END();
}