#include "itest_tmp117.hpp"
#include "drivers/DigitalOut.h"

#include "rtos/ThisThread.h"

TMP117* _device;
mbed::DigitalOut led(PA_5, 0);

void data_ready_cb()
{
    led = !led;
}

void setUp(void)
{
    printf("-----------------------START SETUP---------------------------\n");
    _device = new TMP117();
    _device->soft_reset();
    printf("------------------------END SETUP----------------------------\n");
}

void tearDown(void)
{
    printf("---------------------START TEARDOWN--------------------------\n");
    _device->shut_down();
    delete _device;
    printf("----------------------END TEARDOWN---------------------------\n");
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

void test_lock_register()
{
    _device->lock_registers();
    TEST_ASSERT_FALSE(_device->registers_unlocked());
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
        mbed::callback(data_ready_cb)
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

    RUN_TEST(test_lock_register);

    

    UNITY_END();
}