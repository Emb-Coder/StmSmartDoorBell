/*
 * MyMain.cpp
 *
 *  Created on: Mar 10, 2022
 *      Author: larbi
 */

#include "Cpu.h"
#include "Gpio.h"
#include "Serial.h"
#include "Pwm.h"
#include "Twi.h"
#include "Spi.h"
#include "Rtc.h"
#include "LowPower.h"
#include "AnalogInput.h"

#include "Sim800l.h"
#include "Pn532.h"

#include "AHT10.h"

#include "MyApp.h"

/*! @warning : Chose one test Only */
//#define GPIO_TEST_EXAMPLE
//#define SERIAL_TEST_EXAMPLE
//#define TIMER_TEST_EXAMPLE
//#define TWI_TEST_EXAMPLE
//#define SPI_TEST_EXAMPLE
//#define RTC_TEST_EXAMPLE
//#define LOW_POWER_TEST_EXAMPLE
#define ANALOG_INPUT_TEST_EXAMPLE

//#define AHT10_SENSOR_TEST_EXAMPLE

//#define NETWORK_SIM800L_TEST_EXAMPLE
//#define NETWORK_PN532_TEST_EXAMPLE

#define SERIAL_DEBUG_BAUDRATE	115200


Serial* serialDebug;


void serialDebugInit(void)
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
		while(1){}
	}
}


#ifdef GPIO_TEST_EXAMPLE
Gpio* built_in_led;

volatile bool user_button_pressed = false;
volatile bool board_button_pressed = false;

void user_button_isr(void);
void board_button_isr(void);


// setup function
void myapp_setup(void)
{
	built_in_led = new Gpio(GpioPortsTypeDef::PA, 5, GpioModeTypeDef::OUTPUT);

	Gpio user_button(GpioPortsTypeDef::PC, 0, GpioModeTypeDef::INPUT);
	Gpio board_button(GpioPortsTypeDef::PC, 13, GpioModeTypeDef::INPUT);

	built_in_led->begin();
	user_button.begin();
	board_button.begin();

	user_button.attachInterrupt(user_button_isr, GpioTriggerModeTypeDef::CHANGE);
	board_button.attachInterrupt(board_button_isr, GpioTriggerModeTypeDef::FALLING_EDGE);
}

// loop function
void myapp_loop(void)
{
	if (user_button_pressed)
	{
		user_button_pressed = false;
		for (int i = 0; i < 3; ++i)
		{
			built_in_led->digitalWrite(HIGH);
			HAL_Delay(200);
			built_in_led->digitalWrite(LOW);
			HAL_Delay(200);
		}
	}

	if (board_button_pressed)
	{
		board_button_pressed = false;
		for (int i = 0; i < 3; ++i)
		{
			built_in_led->digitalToggle();
			HAL_Delay(1000);
		}
	}
}

void user_button_isr(void)
{
	user_button_pressed = true;
}

void board_button_isr(void)
{
	board_button_pressed = true;
}

#elif defined (SERIAL_TEST_EXAMPLE)


Serial* stlinkSerial;

// setup function
void myapp_setup(void)
{
	stlinkSerial = Serial::getInstance(SerialInstancesTypeDef::SERIAL_2);

    Serial::settings_t settings= {
        .baudrate= 115200,
        .wordLength= SerialWordLengthTypeDef::EIGHT_BITS_LENGTH,
        .stopBits = SerialStopBitTypeDef::ONE_STOP_BIT,
        .parity= SerialParityTypeDef::NONE
    };

	if (STATUS_OK != stlinkSerial->begin(settings))
	{
		while(1){}
	}
}

// loop function
void myapp_loop(void)
{
	stlinkSerial->println("--------------------------------------------------------");

	stlinkSerial->println("hello, this is the first test of serial class ");

	stlinkSerial->println("--------------------------------------------------------");

	stlinkSerial->println("Let's print the number 255 in multiple formats ..... ");

	stlinkSerial->print("255 in hex = ");
	stlinkSerial->println(255, HEX);

	stlinkSerial->print("255 in DEC = ");
	stlinkSerial->println(255, DEC);

	stlinkSerial->print("255 in OCT = ");
	stlinkSerial->println(255, OCT);

	stlinkSerial->print("255 in BIN = ");
	stlinkSerial->println(255, BIN);

	stlinkSerial->println("--------------------------------------------------------");

	stlinkSerial->println("Let's print the number 1.123456789 ..... ");

	stlinkSerial->print("1.123456789 with 3 decimal point = ");
	stlinkSerial->println(1.123456789, 3);

	stlinkSerial->println("--------------------------------------------------------");

	stlinkSerial->println("Let's test the printf method");

	stlinkSerial->printf("A[%d] = %02X\r\n", 10, 255);

	stlinkSerial->println("--------------------------------------------------------");

	stlinkSerial->println("Let's test the read method");

	uint8_t dataBuf[255];
	memset(dataBuf, 0, 255);

	uint16_t dataBufLen = stlinkSerial->read(dataBuf);

	if (dataBufLen > 0)
	{
		stlinkSerial->print("Received data = ");
		stlinkSerial->write(dataBuf, dataBufLen);
		stlinkSerial->println();
	}
	else
	{
		stlinkSerial->print("read error");

	}

	stlinkSerial->println("--------------------------------------------------------");

	stlinkSerial->println("Let's test the read byte method");

	uint8_t byt = 0;

	if (STATUS_OK == stlinkSerial->readByte(&byt))
	{
		stlinkSerial->print("Received byte = ");
		stlinkSerial->println(byt, HEX);
	}
	else
	{
		stlinkSerial->println("Error occurred while receiving the byte");
	}
	stlinkSerial->println("--------------------------------------------------------");

	stlinkSerial->println("Let's test the read data method (size = 5)");

	uint8_t byts[5] = {0, 0, 0, 0, 0};

	if (STATUS_OK == stlinkSerial->readData(byts, 5))
	{
		stlinkSerial->print("Received data = ");
		stlinkSerial->write(byts, 5);
		stlinkSerial->println();
	}
	else
	{
		stlinkSerial->println("Error occurred while receiving data");
	}

	stlinkSerial->println("--------------------------------------------------------");

	stlinkSerial->println("Let's test the read Bytes Until method (size = 5, delimiter =  0x40, 0x12)");

	const uint8_t bytesDelim[] = {0x40, 0x12};
	uint8_t receivedDelimBytes[5] = {0, 0, 0, 0, 0};

	if (STATUS_OK == stlinkSerial->readBytesUntil(receivedDelimBytes, bytesDelim, sizeof(bytesDelim), 65535))
	{
		stlinkSerial->print("Received data = ");
		stlinkSerial->write(receivedDelimBytes, 5);
		stlinkSerial->println();
	}
	else
	{
		stlinkSerial->println("Error occurred while receiving delim data");
	}

	stlinkSerial->println("--------------------------------------------------------");

	stlinkSerial->println("Let's test the read string Until method (delimiter =  OK)");

	char receivedDelimString[50] = "\0";

	if (STATUS_OK == stlinkSerial->readStringUntil(receivedDelimString, "OK", 65535))
	{
		stlinkSerial->print("Received data = ");
		stlinkSerial->println(receivedDelimString);
	}
	else
	{
		stlinkSerial->println("Error occurred while receiving delim string");
	}


	stlinkSerial->println("--------------------------------------------------------");


	HAL_Delay(10000);
}
#elif defined (TIMER_TEST_EXAMPLE)

//#define PWM_EXAMPLE
#define BASE_EXAMPLE

Timer* ticker;
Timer* timerIT;
Gpio* built_in_led;

Pwm* pwm_led;
uint16_t dutyCycle = 1;

void timerCallback();

// setup function
void myapp_setup(void)
{
#ifdef PWM_EXAMPLE
	/*  PA8     ------> TIM1_CH1 */
	pwm_led = new Pwm(TimerInstancesTypeDef::TIMER_1, TimerChannelTypeDef::CHANNEL_1);

	uint16_t SignalPsc = 65535;
	uint32_t cpuFreq = pwm_led->getCpuFrequency();

	// 5 seconds period
	if (STATUS_OK != pwm_led->begin(SignalPsc, 6409))
		while(1){}

	if (STATUS_OK != pwm_led->generatePwm())
		while(1){}

#elif defined (BASE_EXAMPLE)

	timerIT = Timer::getInstance(TimerInstancesTypeDef::TIMER_3);
	ticker = Timer::getInstance(TimerInstancesTypeDef::TIMER_2);

	tim_settings base_settings;
	tim_settings baseIT_settings;

	built_in_led = new Gpio(GpioPortsTypeDef::PA, 5, GpioModeTypeDef::OUTPUT);

	if ( STATUS_OK != built_in_led->begin())
		while(1){}

	base_settings.mode[0] = TimerModeTypeDef::BASE;
	base_settings.mode[1] = TimerModeTypeDef::BASE;
	base_settings.mode[2] = TimerModeTypeDef::BASE;
	base_settings.mode[3] = TimerModeTypeDef::BASE;

	base_settings.ticksPeriod = TimerTicksPeriodTypeDef::MICROSECOND_TICK;
	base_settings.useInterrupt = false;

	baseIT_settings.mode[0] = TimerModeTypeDef::BASE;
	baseIT_settings.mode[1] = TimerModeTypeDef::BASE;
	baseIT_settings.mode[2] = TimerModeTypeDef::BASE;
	baseIT_settings.mode[3] = TimerModeTypeDef::BASE;

	baseIT_settings.prescValue = 65535;
	baseIT_settings.arrValue = 7691;

	baseIT_settings.ticksPeriod = TimerTicksPeriodTypeDef::CUSTOM_TICK;
	baseIT_settings.useInterrupt = true;

	if ( STATUS_OK != ticker->begin(base_settings))
		while(1){}

	if ( STATUS_OK != timerIT->begin(baseIT_settings))
		while(1){}

	ticker->start();
	timerIT->triggerIt(timerCallback);

#endif /* PWM_EXAMPLE */

}

// loop function
void myapp_loop(void)
{

#ifdef PWM_EXAMPLE
	dutyCycle++;
	if (dutyCycle > 90)
		dutyCycle = 1;

	if (STATUS_OK != pwm_led->setDutyCycle(dutyCycle))
		while(1){}

	HAL_Delay(1000);

#elif defined (BASE_EXAMPLE)

	for (uint16_t j = 0 ; j < 1000; ++j)
		ticker->delayUs(10000);

	HAL_Delay(1000);

#endif /* PWM_EXAMPLE */

}

void timerCallback()
{
	built_in_led->digitalToggle();
}

#elif defined (TWI_TEST_EXAMPLE)

Twi* aht10Sensor;

#define AHT10_SLAVE_ADDRESS 	(0x38 << 1)

#define SOFT_RESET				(0xBA)
#define TRIGGER_MEASUREMENT_CMD (0xAC)

// setup function
void myapp_setup(void)
{
	aht10Sensor = Twi::getInstance(TwiInstancesTypeDef::I2C_1);

    Twi::twi_settings_t settings= {
        .clockSpeed= TwiSpeedModeTypeDef::STANDARD_MODE,
        .slaveAddLength= 7,
        .slaveAddress = 0
    };


    if (STATUS_OK != aht10Sensor->begin(settings))
    {
    	while(1) {};
    }
}

// loop function
void myapp_loop(void)
{
	// software reset
	aht10Sensor->write(AHT10_SLAVE_ADDRESS, SOFT_RESET);
	HAL_Delay(1000);

	// initialization
	uint8_t initData[3] = {TRIGGER_MEASUREMENT_CMD, 0x33, 0x00};
	aht10Sensor->write(AHT10_SLAVE_ADDRESS, initData, 3);
	HAL_Delay(300);

	uint8_t tempHumidityData[6];
	memset(tempHumidityData, 0, 6);

	// read data
	aht10Sensor->read(AHT10_SLAVE_ADDRESS | 0x01, tempHumidityData, 6);

	uint32_t S_humidity = 0;
	uint32_t S_temperature = 0;

	S_humidity =  (((uint32_t)tempHumidityData[1] << 16) | ((uint16_t)tempHumidityData[2] << 8) | (tempHumidityData[3])) >> 4;
	S_temperature = ((uint32_t)(tempHumidityData[3] & 0x0F) << 16) | ((uint16_t)tempHumidityData[4] << 8) | tempHumidityData[5];

	float humidity = S_humidity * 0.000095;
	float temperature = S_temperature * 0.000191  - 50;

	HAL_Delay(5000);
}

#elif defined (SPI_TEST_EXAMPLE)

Spi* spiDevice;
Gpio* spiDeviceSS;

// setup function
void myapp_setup(void)
{
	spiDeviceSS = new Gpio(GpioPortsTypeDef::PA, 5, GpioModeTypeDef::OUTPUT);

	if (STATUS_OK != spiDeviceSS->begin())
		while(1){};

	spiDevice = Spi::getInstance(SpiInstancesTypeDef::SPI_INSTANCE_1);

	Spi::spi_settings_t settings = {
		.dataSize = SpiDataSizeTypeDef::EIGHT_BITS,
		.firstBit = SpiFirstBitTypeDef::MSB,
		.clockPrescaler = SpiClockPrescalerTypeDef::PSC_2,
		.clockMode = SpiClockModeTypeDef::MODE_0
	};


    if (STATUS_OK != spiDevice->begin(settings, spiDeviceSS))
		while(1){};

}

// loop function
void myapp_loop(void)
{

}
#elif defined (NETWORK_SIM800L_TEST_EXAMPLE)

Sim800l* gsm;

#define GSM_NUMBER	"+213556565622"

// setup function
void myapp_setup(void)
{
	serialDebugInit();

	gsm = new Sim800l(SerialInstancesTypeDef::SERIAL_1);

	if (STATUS_OK != gsm->begin())
	{
		while(1) {};
	}

	gsm->setDebugger(serialDebug);
}

// loop function
void myapp_loop(void)
{
	/*------------------ call ------------------ */

	gsm->callNumber(GSM_NUMBER);
	HAL_Delay(15000);
	gsm->hangoffCall();

	/*------------------ send sms ------------------ */

	char text[] = "Testing Sms";  //text for the message.
	bool error = gsm->sendSms(GSM_NUMBER, text);

	if (error)
		serialDebug->println("send sms OK");
	else
		serialDebug->println("send sms error");

	/*------------------ read sms ------------------ */

	// read timeout = 30 Seconds
	std::string RxSms = gsm->readSms(30000);

	serialDebug->print("read sms =");
	serialDebug->println(RxSms);

	HAL_Delay(10000);
}

#elif defined (NETWORK_PN532_TEST_EXAMPLE)

Pn532* nfcModule;

// setup function
void myapp_setup(void)
{
	serialDebugInit();

	std::pair<GpioPortsTypeDef, uint16_t> pn532IrqPin = {GpioPortsTypeDef::PB, 10};
	std::pair<GpioPortsTypeDef, uint16_t> pn532RstPin = {GpioPortsTypeDef::PB, 4};

	nfcModule = new Pn532(TwiInstancesTypeDef::I2C_3, pn532IrqPin, pn532RstPin);

	if (STATUS_OK != nfcModule->begin())
		while(1) {};

	nfcModule->setDebugger(serialDebug);

	uint32_t versiondata = nfcModule->getFirmwareVersion();

	if (! versiondata) {
		serialDebug->print("Didn't find PN53x board");
 		while (1); // halt
	}

	// Got ok data, print it out!
	serialDebug->print("Found chip PN5"); serialDebug->println((versiondata>>24) & 0xFF, HEX);
	serialDebug->print("Firmware ver. "); serialDebug->print((versiondata>>16) & 0xFF, DEC);
	serialDebug->print('.'); serialDebug->println((versiondata>>8) & 0xFF, DEC);

	// Set the max number of retry attempts to read from a card
	// This prevents us from waiting forever for a card, which is
	// the default behaviour of the PN532.
	nfcModule->setPassiveActivationRetries(0xFF);

	// configure board to read RFID tags
	nfcModule->SAMConfig();

	serialDebug->println("Waiting for an ISO14443A card");
}

// loop function
void myapp_loop(void)
{
	bool success;
	uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };	// Buffer to store the returned UID
	uint8_t uidLength;				// Length of the UID (4 or 7 bytes depending on ISO14443A card type)

	// Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
	// 'uid' will be populated with the UID, and uidLength will indicate
	// if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
	success = nfcModule->readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);

	if (success) {
		serialDebug->println("Found a card!");
		serialDebug->print("UID Length: ");serialDebug->print(uidLength, DEC);serialDebug->println(" bytes");
		serialDebug->print("UID Value: ");
		for (uint8_t i=0; i < uidLength; i++)
		{
			serialDebug->print(" 0x");serialDebug->print(uid[i], HEX);
		}
		serialDebug->println("");
		// Wait 1 second before continuing
		HAL_Delay(1000);
	}
	else
	{
		// PN532 probably timed out waiting for a card
		serialDebug->println("Timed out waiting for a card");
	}
}

#elif defined (RTC_TEST_EXAMPLE)

Rtc* realTimeClk;

void alarmACallback(void);
void rtcWakeupCallback(void);

// setup function
void myapp_setup(void)
{
	Rtc::rtcTimeDate rtc_settings = {
			.hour = 13,
			.minute = 49,
			.seconds = 0,

			.weekday = RtcWeekDayTypeDef::WEDNESDAY,
			.day = 22,
			.month = 6,
			.year = 22
	};

	serialDebugInit();

	realTimeClk = Rtc::getInstance();

	realTimeClk->setInitFlagValue(0x2509);

	if (STATUS_OK != realTimeClk->begin(RtcClockSourceTypeDef::LSE, rtc_settings))
		while(1) {};

	if (STATUS_OK != realTimeClk->setAlarm(RtcAlarmTypesTypeDef::A, (time_t) 1655765726, alarmACallback)) // 22 - 55
		while(1) {};

	if (STATUS_OK != realTimeClk->setWakeupTimer(20000, rtcWakeupCallback) )
		while(1) {};
}

// loop function
void myapp_loop(void)
{
	time_t epochNow = realTimeClk->getTimeStamps();

	serialDebug->print("epochTime = ");
	serialDebug->println(epochNow, DEC);

	HAL_Delay(10000);
}

void rtcWakeupCallback(void)
{
	serialDebug->println("Wakeup callback triggered");
}

void alarmACallback(void)
{
	serialDebug->println("Alarm A callback triggered");
}

#elif defined (LOW_POWER_TEST_EXAMPLE)

Rtc* realTimeClk;

// setup function
void myapp_setup(void)
{
	Rtc::rtcTimeDate rtc_settings = {
			.hour = 12,
			.minute = 35,
			.seconds = 0,

			.weekday = RtcWeekDayTypeDef::WEDNESDAY,
			.day = 22,
			.month = 6,
			.year = 22
	};

	serialDebugInit();

	realTimeClk = Rtc::getInstance();

	realTimeClk->setInitFlagValue(0x2509);

	if (STATUS_OK != realTimeClk->begin(RtcClockSourceTypeDef::LSE, rtc_settings))
		while(1) {};

	LowPower::LPmode = LowPowerModesTypeDef::stop;

	LowPower::debugMode = true;
	LowPower::secondsSleep = true;
	LowPower::sleepTime = 120;
	LowPower::lpRtc = realTimeClk;

	LowPower::init();
}

// loop function
void myapp_loop(void)
{
	serialDebug->println("entering stop mode");

	serialDebug->end();

	LowPower::enable();

	LowPower::disable();

	clockInit();
	serialDebugInit();

	serialDebug->println("disable stop mode");
}

#elif defined (ANALOG_INPUT_TEST_EXAMPLE)

AnalogInput* internalAdc;

// setup function
void myapp_setup(void)
{
	serialDebugInit();

	internalAdc = AnalogInput::getInstance();

	std::set<analogInputChannelsTypeDef> adcChannels;

	adcChannels.insert(analogInputChannelsTypeDef::CH0);
	adcChannels.insert(analogInputChannelsTypeDef::CH1);
	adcChannels.insert(analogInputChannelsTypeDef::CH4);

	if (STATUS_ERROR == internalAdc->begin(adcChannels))
		while(1) {};
}

// loop function
void myapp_loop(void)
{
	uint16_t raw_datas[3];
//	uint16_t ch4_data = 0;

	if (STATUS_ERROR == internalAdc->startRead())
		while(1) {};

	/* read all */
	if (STATUS_ERROR == internalAdc->read(raw_datas))
		while(1) {};

	/* read only one adc */
//	ch4_data = internalAdc->read(analogInputChannelsTypeDef::CH4);

	if (STATUS_ERROR == internalAdc->stopRead())
		while(1) {};

//	serialDebug->print("CH4 read = ");
//	serialDebug->println(ch4_data, DEC);

	serialDebug->printf("CH1 = %d\tCH2 = %d\tCH3 = %d\r\n", raw_datas[0], raw_datas[1], raw_datas[2]);

	HAL_Delay(500);
}

#elif defined (AHT10_SENSOR_TEST_EXAMPLE)

AHT10 *roomSensor;

// setup function
void myapp_setup(void)
{
	roomSensor = new AHT10(TwiInstancesTypeDef::I2C_1);

    if (STATUS_OK != roomSensor->begin())
    {
    	while(1) {};
    }

}

// loop function
void myapp_loop(void)
{
	float humidity = roomSensor->readHumidity();
	float temperature = roomSensor->readTemperature();

	serialDebug->print("temperature = ");
	serialDebug->println(temperature);

	serialDebug->print("humidity = ");
	serialDebug->println(humidity);

	HAL_Delay(5000);
}

#endif /*  TEST_EXAMPLE */
