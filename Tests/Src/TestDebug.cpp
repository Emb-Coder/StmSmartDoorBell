/*
 * TestDebug.cpp
 *
 *  Created on: Mar 18, 2022
 *      Author: Dash
 */

#include <TestDebug.h>

static Serial* testLogger = nullptr;

DriversExecStatus TestDebug::begin(Serial* testLoggerObj)
{
	testLogger = testLoggerObj;

	if (testLogger == nullptr)
	{
		return (STATUS_ERROR);
	}

	if (! testLogger->valid())
	{
	    Serial::settings_t settings= {
	        .baudrate= 115200,
	        .wordLength= SerialWordLengthTypeDef::EIGHT_BITS_LENGTH,
	        .stopBits = SerialStopBitTypeDef::ONE_STOP_BIT,
	        .parity= SerialParityTypeDef::NONE
	    };

		testLogger->begin(settings);
	}

	return (STATUS_OK);
}

void TestDebug::run_gpio_test(Gpio *gpioPin)
{
	testLogger->println("*-------------GPIO pin begin TEST-------------*");

	if (!gpioPin->isInitialized())
	{
		testLogger->println("GPIO not initialized..");
		testLogger->println("initializing the Gpio pin..");

		if (STATUS_OK != gpioPin->begin())
		{
			testLogger->println("GPIO pin initialization error");
			testLogger->println("!!!!!!!!!!!////////TEST FAILED////////!!!!!!!!!!!");
			return;
		}
		else
		{
			testLogger->println("GPIO pin initialization OK");
		}
	}

	testLogger->println("testing the pin..");

	if (gpioPin->getMode() == GpioModeTypeDef::OUTPUT)
	{
		testLogger->println("testing output pin.....");
		testLogger->println("turning on the pin..");

		testLogger->println("writing the pin in HIGH state..");
		gpioPin->digitalWrite(HIGH);

		HAL_Delay(3000);

		testLogger->println("Checking the pin....");
		if (gpioPin->digitalRead() != HIGH)
		{
			testLogger->println("!!!!!!!!!!!////////TEST FAILED////////!!!!!!!!!!!");
			return;
		}
		else
		{
			testLogger->println("Pin is HIGH, OK");
		}

		testLogger->println("toggling the pin..");

		DigitalStateTypeDef state1 = gpioPin->digitalRead();
		gpioPin->digitalToggle();

		HAL_Delay(3000);

		testLogger->println("Checking if the pins is toggled");
		DigitalStateTypeDef state2 = gpioPin->digitalRead();
		if (state1 == state2)
		{
			testLogger->println("!!!!!!!!!!!////////TEST FAILED////////!!!!!!!!!!!");
			return;
		}
		else
		{
			testLogger->println("the pins is toggled, OK");
		}
	}
	else if (gpioPin->getMode() == GpioModeTypeDef::INPUT)
	{
		testLogger->println("testing input pin..");
		testLogger->println("checking pin state..");

		testLogger->println("please indicate input state (HIGH/LOW ?)..");

		char inputState[5] = "\0";

		testLogger->readStringUntil(inputState, "\r\n", 20000);
		uint8_t j = 0;
		while (inputState[j]) {
			inputState[j] = toupper(inputState[j]);
			j++;
		}

		if (strcmp(inputState, "HIGH") != 0 && strcmp(inputState, "LOW") != 0)
		{
			while (strcmp(inputState, "HIGH") != 0 && strcmp(inputState, "LOW") != 0)
			{
				testLogger->println("please chose state between (HIGH/LOW) only..");
				testLogger->println("lets retry");

				testLogger->readStringUntil(inputState, "\r\n", 20000);
				uint8_t i = 0;
				while (inputState[i]) {
					inputState[i] = toupper(inputState[i]);
					i++;
				}

				if (strcmp(inputState, "HIGH") != 0 && strcmp(inputState, "LOW") != 0)
					memset(inputState, '\0', sizeof(inputState));
			}
		}

		if (strcmp(inputState, "HIGH") == 0)
		{

			testLogger->println("You've chosen HIGH lets read the pin");
			DigitalStateTypeDef state = gpioPin->digitalRead();
			if (state != HIGH)
			{
				testLogger->println("!!!!!!!!!!!////////TEST FAILED////////!!!!!!!!!!!");
				return;
			}
			else
			{
				testLogger->println("The pin is HIGH, OK");
			}

		}
		else
		{

			testLogger->println("You've chosen LOW lets read the pin");
			DigitalStateTypeDef state = gpioPin->digitalRead();
			if (state != LOW)
			{
				testLogger->println("!!!!!!!!!!!////////TEST FAILED////////!!!!!!!!!!!");
				return;
			}
			else
			{
				testLogger->println("The pin is LOW, OK");
			}
		}
	}
	else
	{
		testLogger->println("cannot test this mode :(");
		return;
	}

	testLogger->println("////////TEST SUCCESS :) ////////");
	testLogger->println("GPIO test has been successfully tested");

	testLogger->println("*-------------GPIO pin end TEST-------------*");
}

void TestDebug::run_serial_test(Serial *serialInst)
{
}
