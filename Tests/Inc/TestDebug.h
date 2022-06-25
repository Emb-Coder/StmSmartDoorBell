/*
 * TestDebug.h
 *
 *  Created on: Mar 18, 2022
 *      Author: Dash
 */

#ifndef INC_TESTDEBUG_H_
#define INC_TESTDEBUG_H_

#include "Serial.h"
#include "Gpio.h"

class TestDebug {
public:
	TestDebug() = delete;
    TestDebug(TestDebug const&) = delete;
    TestDebug& operator=(TestDebug const&) = delete;

    static DriversExecStatus begin(Serial* testLoggerObj);

	static void run_gpio_test(Gpio* gpioPin);
	static void run_serial_test(Serial* serialInst);
private:

};

#endif /* INC_TESTDEBUG_H_ */
