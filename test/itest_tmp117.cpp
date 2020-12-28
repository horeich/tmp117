#include "itest_tmp117.hpp"

TMP117 _tmp117(PB_9, PB_8, 100000);

void data_ready_cb()
{
    printf("Data is ready\n");
}

void setUp(void)
{
}

void tearDown(void)
{
}

void test_soft_reset()
{
    _tmp117.soft_reset();
    _tmp117.soft_reset();
}

void test_lock_registers()
{
    _tmp117.lock_registers();
    TEST_ASSERT_FALSE(_tmp117.registers_unlocked());
}

void test_unlock_registers()
{
    _tmp117.unlock_registers();
    TEST_ASSERT_TRUE(_tmp117.registers_unlocked());
}

void test_set_therm_mode()
{
    _tmp117.set_therm_mode();
    TEST_ASSERT_EQUAL(TMP117::ALERT_MODE_THERM, _tmp117.get_alert_mode());
}

void test_set_alert_mode()
{
    _tmp117.set_alert_mode();
    TEST_ASSERT_EQUAL(TMP117::ALERT_MODE_ALERT, _tmp117.get_alert_mode());
}

void test_set_limits()
{
    _tmp117.set_alert_limits(10.32f, 25.46f);
    float upper_limit, lower_limit;
    _tmp117.get_alert_limits(lower_limit, upper_limit);
    //TEST_ASSERT_FLOAT_WITHIN(lower_limit,);
    printf("lower: %f, upper: %f", lower_limit, upper_limit);
}

void test_read_temperature()
{
    float value = _tmp117.read_temperature();
    printf("value = %f", value);
    TEST_ASSERT_FLOAT_WITHIN(256.0f, 0.0f, value);
}

void test_set_output_pin_interrupt_low()
{
    // _tmp117.set_output_pin_interrupt(
    //     TMP117::OUTPUT_PIN_MODE_DATA_READY,
    //     TMP117::OUTPUT_PIN_POL_ACTIVE_LOW,
    //     mbed::callback(data_ready_cb),
    //     PC_9);

}

void run_itest_tmp117()
{
    printf("##Run TMP117 test##\n");

    UNITY_BEGIN();


    RUN_TEST(test_soft_reset);
    // Test run according to datasheet p.18

    RUN_TEST(test_lock_registers);

    RUN_TEST(test_unlock_registers);

    RUN_TEST(test_set_therm_mode);

    RUN_TEST(test_set_alert_mode);

    RUN_TEST(test_set_limits);

    for (int i = 0; i < 200; ++i)
    {
        RUN_TEST(test_read_temperature);
        rtos::ThisThread::sleep_for(2000);
    }

    UNITY_END();
}