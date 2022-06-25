/*
 * test_app.cpp
 *
 *  Created on: Mar 18, 2022
 *      Author: Dash
 */


#include "testCore.h"
#include "TestDebug.h"
#include "TestConfig.h"

static Serial* testSerialLogger;

#ifdef RUN_GPIO_TEST

static Gpio* testGpioPin;

#endif

void test_setup(void)
{
	testSerialLogger = Serial::getInstance(SerialInstancesTypeDef::SERIAL_2);

	Serial::settings_t settings= {
        .baudrate= 115200,
        .wordLength= SerialWordLengthTypeDef::EIGHT_BITS_LENGTH,
        .stopBits = SerialStopBitTypeDef::ONE_STOP_BIT,
        .parity= SerialParityTypeDef::NONE
    };

	if (STATUS_OK != testSerialLogger->begin(settings))
		while(1){}


	if (STATUS_OK != TestDebug::begin(testSerialLogger))
		while(1){}

#ifdef RUN_GPIO_TEST
	testGpioPin = new Gpio(TEST_GPIO_PORT, TEST_GPIO_PIN, TEST_GPIO_MODE);
#endif
}

void test_loop(void)
{
#ifdef RUN_GPIO_TEST
	TestDebug::run_gpio_test(testGpioPin);
#endif

	HAL_Delay(20000);
}
