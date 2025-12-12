/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2024, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_usart.cpp
  * Version		: v2.3
  * Author		: LiuHao Lijiawei Albert
  * Date		: 2023-07-31
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_usart.h"

/* Private variables -------------------------------------------------*/
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
//如果底下是虚的说明你没使能Register Callback USART && UART
#ifdef HAL_UART_MODULE_ENABLED	//如果底下是虚的说明你没使用任何串口
IFR_UDBRx_ClassDef* UDB_Pointer[9];//串口类指针，用于定向查找解析函数
/**
  * @概述	串口DMA双缓存接收初始化。
  * @参数1	串口句柄
  * @参数2	解析函数指针
  * @返回值 void
  */
void IFR_UDBRx_ClassDef::UDB_Recevice_Init(UART_HandleTypeDef *huart,void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len))
{
	_huart = huart;UDB_Pointer[IFR_Uart_ID_Get(huart)] = this;

	if (UART_Analysis_Function != NULL) AnalysisFunc = UART_Analysis_Function;
	HAL_UART_RegisterRxEventCallback(huart,IFR_Uart_DoubleBuffer_Recevice_Callback);

	UsartRxState = HAL_UARTEx_ReceiveToIdle_DMA(huart,Memory[Buffer_Num],IFR_UDBRX_Len_Max);
	__HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
}
 /**
  * @概述	串口双缓存接收处理。
  * @参数1	数据长度
  * @返回值 void
  */
void IFR_UDBRx_ClassDef::Uart_DoubleBuffer_Recevice(uint16_t len)
{
	Buffer_Num =!Buffer_Num;  Data_Length = len;
	UsartRxState = HAL_UARTEx_ReceiveToIdle_DMA(_huart,Memory[Buffer_Num],IFR_UDBRX_Len_Max);
	__HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
	if(AnalysisFunc != NULL) (*AnalysisFunc)(Memory[!Buffer_Num],len);
	_updata_systick = HAL_GetTick();
}
/**
  * @概述	串口错误重新连接。
  * @参数		void
  * @返回值 void
  */
void IFR_UDBRx_ClassDef::Uart_Restart(void)
{
	_huart->ErrorCode = HAL_UART_ERROR_NONE;
	__HAL_UNLOCK(_huart);
	HAL_UART_MspDeInit(_huart);
	HAL_UART_MspInit(_huart);

	HAL_UART_RegisterRxEventCallback(_huart,IFR_Uart_DoubleBuffer_Recevice_Callback);

	HAL_UARTEx_ReceiveToIdle_DMA(_huart,Memory[Buffer_Num],IFR_UDBRX_Len_Max);
	__HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);

}
//中断回调函数，禁止改写和调用!!!
void IFR_Uart_DoubleBuffer_Recevice_Callback(UART_HandleTypeDef *huart,uint16_t len)
{
	IFR_UDBRx_ClassDef* pUDB_Class = UDB_Pointer[IFR_Uart_ID_Get(huart)];
	pUDB_Class->Uart_DoubleBuffer_Recevice(len);
}

void IFR_UDBRx_ClassDef::Uart_Start_DMA_Receive(void)
{
	UsartRxState = HAL_UARTEx_ReceiveToIdle_DMA(_huart,Memory[Buffer_Num],IFR_UDBRX_Len_Max);
	__HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
}
//断线重连,禁止调用
uint32_t CountError[10]={0};//重连次数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uint8_t UartID = IFR_Uart_ID_Get(huart);

	UDB_Pointer[UartID]->Uart_Restart();

	CountError[UartID]++;
}

//static函数外部无法调用，禁止改写!!!
static uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) 			return 1;
	else if(huart->Instance == USART2) 	return 2;
	else if(huart->Instance == USART3) 	return 3;
	else if(huart->Instance == UART4)  	return 4;
	else if(huart->Instance == UART5)  	return 5;
	else if(huart->Instance == USART6) 	return 6;
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx) ||defined(STM32F469xx) || defined(STM32F479xx)
	else if(huart->Instance == UART7)  	return 7;
	else if(huart->Instance == UART8) 	return 8;
#endif
	else return 0;
}
#endif
#endif
