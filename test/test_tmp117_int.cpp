
#include "unity.h"

/**
 * Dependencies
 */
#include "mbed.h"
#include "rtos.h"
#include "trace_helper.h"

/**
 * File to be tested
 */
#define private public
#define protected public
#include "../src/tmp117.hpp"


/**
 * Access mbed namespace
 */
using namespace mbed;

/**
 * Declare variables 
 */
const char* _fileName = "config.json";
const char* _testStream = "{\"endpoint-ipv4\": \"134.134.134.134\", \"endpoint-port\": 1234}";

TMP117 _tempSensor;

/**
 * SetUp
 */
void setUp(void) 
{
}

void tearDown(void) 
{
}

void create_config_file()
{
    TEST_ASSERT_EQUAL(true, _configFile->create(nullptr));
}

void open_config_file_success()
{
    TEST_ASSERT_TRUE(_configFile->Open());
}

void close_config_file_success()
{
    TEST_ASSERT_TRUE(_configFile->Close());
}

void read_config_file_success()
{
    TEST_ASSERT_TRUE(_configFile->ParseConfig(_config))
}

void read_config_file_blocked()
{
    TEST_ASSERT_FALSE(_configFile->ParseConfig(_config));
}

void suspend_config_file_success()
{
    TEST_ASSERT_TRUE(_configFile->Suspend());
}

void suspend_fail()
{
    TEST_ASSERT_FALSE(_configFile->Suspend())
}

void resume_success()
{
    TEST_ASSERT_TRUE(_configFile->Resume());
}

void resume_fail()
{
    TEST_ASSERT_FALSE(_configFile->Resume());
}


void test_get_device_id()
{
    TEST_ASSERT_UINT16_WITHIN(0x3FFF/2, 0x3FFF/2, 4); // Must be 14 bit integer
}

void test_get_device_revision()
{
    TEST_ASSERT_UINT8_WITHIN(0x03/2, 0x03/2, 0x01);
}


int main()
{
    // If system doesn't support software reset via Serial.DTR/RTS
    ThisThread::sleep_for(std::chrono::milliseconds(3000));
    setup_trace();

    UNITY_BEGIN();

    // Test bit


    // Test configuration register
    RUN_TEST()




    RUN_TEST(test_get_device_id);


    for (int i = 0; i < 100; ++i)
    {
        RUN_TEST(create_config_file);
        //ThisThread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Open - close alternatingly
    for (int i = 0; i < 100; ++i)
    {
        RUN_TEST(open_config_file_success);
        RUN_TEST(close_config_file_success);
    }

    // Open - close randomly
    for (int i = 0; i < 100; ++i)
    {
        RUN_TEST(open_config_file_success);
        RUN_TEST(open_config_file_success);
        RUN_TEST(close_config_file_success);
        RUN_TEST(close_config_file_success);
        RUN_TEST(open_config_file_success);
        RUN_TEST(open_config_file_success);
        RUN_TEST(close_config_file_success);
        RUN_TEST(close_config_file_success);
    }

    // Parse empty config file (open automatically)
    for (int i = 0; i < 100; ++i)
    {
        RUN_TEST(read_config_file_success);
    }

    // Parse empty config file (close after every read)
    for (int i = 0; i < 100; ++i)
    {
        RUN_TEST(read_config_file_success);
        RUN_TEST(close_config_file_success);
        RUN_TEST(close_config_file_success);
    }
    
    _configFile->Open();
    _configFile->write(_testStream, strlen(_testStream));
    _configFile->Close();

    // Parse full config
    for (int i = 0; i < 100; ++i)
    {
        RUN_TEST(read_config_file_success);
    }

    // Suspend and Resume with parsing in between
    for (int i = 0; i < 100; ++i)
    {
        RUN_TEST(suspend_config_file_success); 
        RUN_TEST(read_config_file_blocked); 
        RUN_TEST(suspend_fail);
        RUN_TEST(resume_success);
        RUN_TEST(resume_fail);
        RUN_TEST(read_config_file_success);
    }

    UNITY_END(); 

}


/**
 * Test cases
 */






