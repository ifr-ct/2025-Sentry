#ifndef __IFR_CRC_H_
#define __IFR_CRC_H_

#include "main.h"

uint8_t IFR_Get_CRC8_Check(uint8_t *pchMessage,uint32_t DataLength,uint8_t crc8_init);
uint16_t IFR_Get_CRC16_Check(uint8_t *pchMessage,uint32_t DataLength,uint16_t crc16_init);

#endif //#ifndef __IFR_CRC_H_

