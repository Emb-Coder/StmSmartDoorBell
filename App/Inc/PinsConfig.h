/*
 * PinsConfig.h
 *
 *  Created on: 26 juin 2022
 *      Author: larbi
 */

#ifndef INC_PINSCONFIG_H_
#define INC_PINSCONFIG_H_

#include "ChipsUtil.h"

const std::pair<GpioPortsTypeDef, uint16_t> doorBellBtnPin = { GpioPortsTypeDef::PA, 5 };
const std::pair<GpioPortsTypeDef, uint16_t> modeSwitchPin = { GpioPortsTypeDef::PA, 1 };
const std::pair<GpioPortsTypeDef, uint16_t> relayPin = { GpioPortsTypeDef::PA, 2 };

const std::pair<GpioPortsTypeDef, uint16_t> pn532IrqPin = {GpioPortsTypeDef::PB, 10};
const std::pair<GpioPortsTypeDef, uint16_t> pn532RstPin = {GpioPortsTypeDef::PB, 4};

const std::pair<GpioPortsTypeDef, uint16_t> esp32ShutDownPin = {GpioPortsTypeDef::PB, 5};
const std::pair<GpioPortsTypeDef, uint16_t> esp32AckPin = {GpioPortsTypeDef::PB, 0};

const std::pair<GpioPortsTypeDef, uint16_t> statusLedPin = {GpioPortsTypeDef::PB, 7};


#endif /* INC_PINSCONFIG_H_ */
