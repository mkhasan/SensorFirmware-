#ifndef _CRC_H
#define _CRC_H

#include "stm32f1xx_hal.h"

#define POLYNOM4 11001
#define POLYNOM8 110101001

int crcInit(uint32_t _plynom4, uint32_t _polynom8);
uint8_t crcCalc(uint32_t const message, int const msgLen, uint32_t polynom);
#endif