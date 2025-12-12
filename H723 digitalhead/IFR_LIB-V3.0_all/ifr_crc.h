#ifndef __IFR_CRC_H_
#define __IFR_CRC_H_

#include "main.h"
#ifdef __cplusplus
 extern "C" {
#endif

#define CRC8_INIT 0xff
#define CRC16_INIT 0xffff
uint8_t IFR_Get_CRC8_Check(uint8_t *pchMessage,uint32_t DataLength,uint8_t crc8_init);
uint8_t IFR_Create_CRC8_Table(unsigned char *ptr, unsigned char len); //?ио?и╣
void IFR_Append_CRC8_Check(unsigned char *pchMessage, unsigned int dwLength);
uint16_t IFR_Get_CRC16_Check(uint8_t *pchMessage, uint32_t dwLength, uint16_t uCRC); //?a??
uint16_t IFR_Create_CRC16_Table(unsigned char *ptr, unsigned char len); //?ио?и╣
void IFR_Append_CRC16_Check(uint8_t * pchMessage,uint32_t dwLength); 

#ifdef __cplusplus
 }
#endif
#endif //#ifndef __IFR_CRC_H_

