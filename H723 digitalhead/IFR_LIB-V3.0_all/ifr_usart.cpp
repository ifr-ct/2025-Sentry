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
#include <string.h>
/* Private variables -------------------------------------------------*/
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
//如果底下是虚的说明你没使能Register Callback USART && UART
#ifdef HAL_UART_MODULE_ENABLED	//如果底下是虚的说明你没使用任何串口
IFR_UDBRx_ClassDef* UDB_Pointer[9];//串口类指针，用于定向查找解析函数
//static osThreadId IFR_UartRx_TaskHandle = NULL;
char uart_task_name[] = "IFR_UartRx_task";
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
  * @概述	适配FreeRTOS的串口DMA双缓存接收初始化。
  * @参数1	串口句柄
  * @参数2	解析函数指针
  * @参数3	任务句柄(串口消息过来通知的任务线程)
  * @返回值 void
  */
//void IFR_UDBRx_ClassDef::UDB_Recevice_Init(UART_HandleTypeDef *huart,\
//										   void(*UART_Analysis_Function)(uint8_t *pData,uint8_t len),osThreadId *TaskHandle)
//{
//	_huart = huart;
//	UDB_Pointer[IFR_Uart_ID_Get(huart)] = this;

//	if (UART_Analysis_Function != NULL) AnalysisFunc = UART_Analysis_Function;
//	HAL_UART_RegisterRxEventCallback(huart,IFR_FreeRTOS_Uart_DoubleBuffer_Recevice_Callback);

//	if (IFR_UartRx_TaskHandle == NULL)
//	{
//		const osThreadDef_t os_thread_def_IFR_UartRx_task = { uart_task_name, IFR_UartRx_Task, osPriorityHigh, 0, 128};
//		//osThreadDef(IFR_UartRx_task, IFR_UartRx_Task, osPriorityHigh, 0, 128);
//		IFR_UartRx_TaskHandle = osThreadCreate(osThread(IFR_UartRx_task), NULL);
//	}

//	_TaskHandle = &IFR_UartRx_TaskHandle;
//	if (TaskHandle != NULL) *TaskHandle = *_TaskHandle;

//	UsartRxState = HAL_UARTEx_ReceiveToIdle_DMA(huart,Memory[Buffer_Num],IFR_UDBRX_Len_Max);
//	__HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
//}
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
  * @概述	FreeRTOS串口双缓存接收处理。
  * @参数1	数据长度
  * @返回值 void
  */
void IFR_UDBRx_ClassDef::Uart_DoubleBuffer_Recevice_FreeRTOS(uint16_t len)
{
	Buffer_Num =!Buffer_Num;  Data_Length = len;
	UsartRxState = HAL_UARTEx_ReceiveToIdle_DMA(_huart,Memory[Buffer_Num],IFR_UDBRX_Len_Max);
	__HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);

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

//	HAL_UART_RegisterRxEventCallback(_huart,IFR_FreeRTOS_Uart_DoubleBuffer_Recevice_Callback);

	UsartRxState = HAL_UARTEx_ReceiveToIdle_DMA(_huart,Memory[Buffer_Num],IFR_UDBRX_Len_Max);
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
//中断回调函数，禁止改写和调用!!!
//void IFR_FreeRTOS_Uart_DoubleBuffer_Recevice_Callback(UART_HandleTypeDef *huart,uint16_t len)
//{
//	uint8_t UDB_ID = IFR_Uart_ID_Get(huart);
//	IFR_UDBRx_ClassDef* pUDB_Class = UDB_Pointer[UDB_ID];
//	pUDB_Class->Uart_DoubleBuffer_Recevice_FreeRTOS(len);
//	switch(UDB_ID)
//	{
//		case 1:
//			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart1Flag);
//			break;
//		case 2:
//			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart2Flag);
//			break;
//		case 3:
//			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart3Flag);
//			break;
//		case 4:
//			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart4Flag);
//			break;
//		case 5:
//			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart5Flag);
//			break;
//		case 6:
//			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart6Flag);
//			break;
//		case 7:
//			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart7Flag);
//			break;
//		case 8:
//			osSignalSet(*(pUDB_Class->_TaskHandle),IFR_Uart8Flag);
//			break;
//	}
//}

//适用于FreeRTOS的串口接收任务,禁止调用！！！
//void IFR_UartRx_Task(void const * argument)
//{
//	osEvent event;

//	while (1)
//	{
//		event = osSignalWait(IFR_UartAllFlag, osWaitForever);
//		if (event.status == osEventSignal)
//		{
//			for (uint8_t i=2; i<10; i++)
//			{
//				if ((event.value.signals & (1<<i)) == (1<<i))
//				{
//					UDB_Pointer[i-1]->Uart_Recevice_AnalysisFunc();
//					event.value.signals &= (~(1<<i));
//					if (event.value.signals == 0) break;
//				}

//			}
//		}
//	}
//}

/**
  * @brief  Sends an amount of data in DMA mode.If you are going to use this function, make sure you use FreeRTOS.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  pData Pointer to data buffer (u8 or u16 data elements).
  * @param  Size  Amount of data elements (u8 or u16) to be sent
  * @retval HAL status
  */

/**
	* @brief	以DMA模式发送大量数据,并将自己暂时提升至最高优先级。如果你要使用这个函数，请确保你使用FreeRTOS。
	* @note		当不启用UART校验(PCE = 0)，并且字长配置为9位(M1-M0 = 01)时，
	*					发送的数据作为u16的集合处理。在这种情况下，
	*					Size必须指出通过pData提供的u16的数目。
	* @param 	pData 指向数据缓冲区(u8或u16数据元素)。
	* @param 	Size 	要发送的数据元素数量(u8或u16)
	* @retval HAL状态
	*/
//HAL_StatusTypeDef IFR_UDBRx_ClassDef::IFR_Uart_Transmint_DMA_FreeRTOS(uint8_t* pData, uint16_t Size)
//{
//	HAL_StatusTypeDef status = HAL_OK;
//	taskENTER_CRITICAL();
//	status = HAL_UART_Transmit_DMA(_huart, pData, Size);
//	taskEXIT_CRITICAL();
//	return status;
//}
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
