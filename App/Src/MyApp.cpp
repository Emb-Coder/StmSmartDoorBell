/*
 * MyMain.cpp
 *
 *  Created on: Mar 10, 2022
 *      Author: larbi
 */

#include "MyApp.h"

#include "Gpio.h"
#include "Serial.h"
#include "LowPower.h"
#include "Cpu.h"

#include "Sim800l.h"
#include "Pn532.h"

#include "PinsConfig.h"

#include <algorithm>

#define SERIAL_DEBUG_BAUDRATE	115200

#define GSM_NUMBER				"+213556565622"
#define APP_SMS_READ_TIMEOUT	30000   		// 30 seconds

const uint8_t authorized_uids[][7] = {
		{ 0, 1, 2, 3, 4, 5, 6 },
		{ 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0, 0 }
};


/* Objects */
Serial* serialDebug;

Gpio doorBellButton(doorBellBtnPin.first, doorBellBtnPin.second, GpioModeTypeDef::INPUT);
Gpio doorBellModeSwitch(modeSwitchPin.first, modeSwitchPin.second, GpioModeTypeDef::INPUT);
Gpio doorBellRelay(relayPin.first, relayPin.second, GpioModeTypeDef::OUTPUT, GpioPullTypeDef::PULL_DOWN);
Gpio esp32shutDown(esp32ShutDownPin.first, esp32ShutDownPin.second, GpioModeTypeDef::OUTPUT, GpioPullTypeDef::PULL_DOWN);
Gpio esp32Ack(esp32AckPin.first, esp32AckPin.second, GpioModeTypeDef::INPUT, GpioPullTypeDef::PULL_DOWN);
Gpio statusLed(statusLedPin.first, statusLedPin.second, GpioModeTypeDef::OUTPUT);

Sim800l gsm(SerialInstancesTypeDef::SERIAL_1);
Pn532 nfcModule(TwiInstancesTypeDef::I2C_3, pn532IrqPin, pn532RstPin);


/* global vars */

volatile bool button_pressed_flag = false;

bool gsm_is_sleeped = false;
const uint8_t authorized_uids_size = sizeof(authorized_uids) / sizeof(authorized_uids[0]);

/* function declaration */
void user_button_isr(void);
static void myapp_gpios_init(void);
static void myapp_gsm_init(void);
static void myapp_nfc_init(void);
static void myapp_low_power_init(void);
static void serialDebugInit(void);
static void myapp_mode1_routine(void);
static void myapp_mode2_routine(void);
static void button_routine(void);
static void nfc_read_routine(void);
static void error_handle(void);


void myapp_setup()
{
	serialDebugInit();

	myapp_gpios_init();

	myapp_gsm_init();

	myapp_nfc_init();

	myapp_low_power_init();
}

void myapp_loop()
{
	LowPower::enable();
	LowPower::disable();

	clockInit();
	myapp_setup();

	if (button_pressed_flag)
		button_routine();
	else
		nfc_read_routine();
}


static void button_routine()
{
	button_pressed_flag = false;

	if (doorBellModeSwitch.digitalRead() == DigitalStateTypeDef::LOW)
		myapp_mode1_routine();
	else
		myapp_mode2_routine();
}

static void nfc_read_routine()
{
	bool success;
	uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };	// Buffer to store the returned UID
	uint8_t uidLength;				// Length of the UID (4 or 7 bytes depending on ISO14443A card type)

	// Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
	// if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
	success = nfcModule.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);

	if (success)
	{
		bool open_door = false;
		for (uint8_t i = 0; i < authorized_uids_size; ++i)
		{
			if (std::equal(std::begin(uid), std::end(uid), std::begin(authorized_uids[i])))
			{
				open_door = true;
				break;
			}
		}

		if (open_door)
		{
			doorBellRelay.digitalWrite(DigitalStateTypeDef::HIGH);
			HAL_Delay(2000);
			doorBellRelay.digitalWrite(DigitalStateTypeDef::LOW);
		}
	}
}
static void myapp_mode1_routine()
{
	// wakeup GSM
	if (gsm_is_sleeped)
		gsm.wakeup();

	gsm.callNumber(GSM_NUMBER);
	HAL_Delay(15000);
	gsm.hangoffCall();

	// read timeout = 30 Seconds
	std::string RxSms = gsm.readSms(APP_SMS_READ_TIMEOUT);

	serialDebug->print("read sms =");
	serialDebug->println(RxSms);

	std::transform(RxSms.begin(), RxSms.end(), RxSms.begin(), ::toupper);

	if (RxSms == "OPEN")
	{
		doorBellRelay.digitalWrite(DigitalStateTypeDef::HIGH);
		HAL_Delay(2000);
		doorBellRelay.digitalWrite(DigitalStateTypeDef::LOW);
	}
}

static void myapp_mode2_routine()
{
	if (!gsm_is_sleeped)
	{
		gsm.sleep();
		gsm_is_sleeped = true;
	}

	esp32shutDown.digitalWrite(DigitalStateTypeDef::HIGH);

	while (esp32Ack.digitalRead() == DigitalStateTypeDef::LOW);
}

static void serialDebugInit(void)
{
	serialDebug = Serial::getInstance(SerialInstancesTypeDef::SERIAL_2);

	Serial::settings_t settings= {
			.baudrate= SERIAL_DEBUG_BAUDRATE,
			.wordLength= SerialWordLengthTypeDef::EIGHT_BITS_LENGTH,
			.stopBits = SerialStopBitTypeDef::ONE_STOP_BIT,
			.parity= SerialParityTypeDef::NONE
	};

	if (STATUS_OK != serialDebug->begin(settings))
	{
		serialDebug->println("serial debug initialization error!!!");
		error_handle();
	}
}

static void myapp_gpios_init()
{
	/* --------------- Door bell button initialization ---------------*/
	if (STATUS_OK != doorBellButton.begin())
	{
		serialDebug->println("door bell button initialization error!!!");
		error_handle();
	}

	doorBellButton.attachInterrupt(user_button_isr, GpioTriggerModeTypeDef::CHANGE);

	/* --------------- Door bell switch initialization ---------------*/
	if (STATUS_OK != doorBellModeSwitch.begin())
	{
		serialDebug->println("door bell switch initialization error!!!");
		error_handle();
	}

	/* --------------- Door bell relay initialization ---------------*/
	if (STATUS_OK != doorBellRelay.begin())
	{
		serialDebug->println("door bell relay initialization error!!!");
		error_handle();
	}

	doorBellRelay.digitalWrite(DigitalStateTypeDef::LOW);

	/* --------------- Esp32 shutdown initialization ---------------*/
	if (STATUS_OK != esp32shutDown.begin())
	{
		serialDebug->println("esp32 shutdown initialization error!!!");
		error_handle();
	}

	esp32shutDown.digitalWrite(DigitalStateTypeDef::LOW);

	/* --------------- Esp32 shutdown initialization ---------------*/
	if (STATUS_OK != esp32Ack.begin())
	{
		serialDebug->println("esp32 ack initialization error!!!");
		error_handle();
	}

	/* --------------- status LED initialization ---------------*/
	if (STATUS_OK != statusLed.begin())
	{
		serialDebug->println("status LED initialization error!!!");
		error_handle();
	}
}

static void myapp_gsm_init()
{
	if (STATUS_OK != gsm.begin())
	{
		serialDebug->println("GSM initialization error!!!");
		error_handle();
	}

	gsm.setDebugger(serialDebug);
}

static void myapp_nfc_init()
{
	if (STATUS_OK != nfcModule.begin())
	{
		serialDebug->println("NFC initialization error!!!");
		error_handle();
	}
	nfcModule.setDebugger(serialDebug);

	uint32_t versiondata = nfcModule.getFirmwareVersion();

	if (! versiondata) {
		serialDebug->print("Didn't find PN53x board");
		while (1); // halt
	}
	// Got ok data, print it out!
	serialDebug->print("Found chip PN5");
	serialDebug->println((versiondata>>24) & 0xFF, HEX);
	serialDebug->print("Firmware ver. ");
	serialDebug->print((versiondata>>16) & 0xFF, DEC);
	serialDebug->print('.');
	serialDebug->println((versiondata>>8) & 0xFF, DEC);

	// Set the max number of retry attempts to read from a card
	// This prevents us from waiting forever for a card, which is
	// the default behaviour of the PN532.
	nfcModule.setPassiveActivationRetries(0xFF);

	// configure board to read RFID tags
	nfcModule.SAMConfig();

	serialDebug->println("Waiting for an ISO14443A card");
}

static void myapp_low_power_init()
{
	LowPower::LPmode = LowPowerModesTypeDef::stop;
	LowPower::sleepTime = UINT16_MAX;
	LowPower::init();

	LowPower::exceptedGpioLowPower.insert(std::make_pair(pn532IrqPin.first, pn532IrqPin.second));
	LowPower::exceptedGpioLowPower.insert(std::make_pair(doorBellBtnPin.first, doorBellBtnPin.second));
}

static void error_handle()
{
	while (1)
	{
		statusLed.digitalToggle();
		HAL_Delay(700);
	}
}

/* --------------------------- ISRs -----------------------------*/

void user_button_isr(void)
{
	button_pressed_flag = true;
}
