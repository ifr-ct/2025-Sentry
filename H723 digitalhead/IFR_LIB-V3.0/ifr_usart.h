/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2024, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_usart.h
  * Version		: v2.3
  * Author		: LiuHao Lijiawei Albert
  * Date		: 2023-07-30
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_USART_H_
#define __IFR_USART_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#ifdef HAL_UART_MODULE_ENABLED
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#ifndef IFR_UDBRX_Len_Max
#define IFR_UDBRX_Len_Max 50
#endif

#define __USART_DMA_ENABLE(__HANDLE__)          ((__HANDLE__)->Instance->CR3 |=  USART_CR3_DMAR)

class IFR_UDBRx_ClassDef
{
	public:
		void UDB_Recevice_Init(UART_HandleTypeDef *huart,void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len));
		void Uart_DoubleBuffer_Recevice(uint16_t len);
	  void Uart_Recevice_AnalysisFunc(void){if(AnalysisFunc != NULL) (*AnalysisFunc)(Memory[!Buffer_Num],Data_Length);_updata_systick = HAL_GetTick();}
		void Uart_Start_DMA_Receive(void);
		void Uart_Restart(void);
		HAL_StatusTypeDef Get_UsartRxState(void){return UsartRxState;}
		uint32_t Get_updata_systick(void){return _updata_systick;}
	private:
		void (*AnalysisFunc)(uint8_t* pData,uint8_t len);

		uint8_t Memory[2][IFR_UDBRX_Len_Max];
		UART_HandleTypeDef *_huart;
		uint8_t Buffer_Num;
		uint8_t Data_Length;
		HAL_StatusTypeDef UsartRxState;
		uint32_t _updata_systick;
};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void IFR_Uart_DoubleBuffer_Recevice_Callback(UART_HandleTypeDef *huart,uint16_t len);
static uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif
#endif
#endif

