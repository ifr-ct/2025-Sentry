/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_can.h
  * Version		: v2.1
  * Author		: LiuHao Lijiawei
  * Date		: 2022-09-27
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_CAN_H_
#define __IFR_CAN_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#ifdef HAL_CAN_MODULE_ENABLED
#include "ifr_motor.h"
//#include "cmsis_os.h"

#include "ifr_dmmotor.h"
 
typedef struct
{
	CAN_RxHeaderTypeDef CAN_RxHeaderStru;
	uint8_t pRxdata[8];
}CanRxTypedef;
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

class IFR_CAN_ClassDef
{
	public:
		void CAN_Init(CAN_HandleTypeDef *hcan);
		void CAN_Init(CAN_HandleTypeDef *hcan,IFR_DJI_Motor *DJI_Motor);
		//void CAN_Init(CAN_HandleTypeDef *hcan,IFR_DJI_Motor *DJI_Motor,osThreadId *TaskHandle);
		void CAN_Transmit(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData);
		void CAN_MultiMessage_Transmit(void);
		void CAN_Recevice(void);
		void CAN_TransmitForMotor(void);
		//osThreadId _TaskHandle;
		void (*CAN_Analysis_Function_std)(uint8_t *Data,uint32_t stdid);
		void (*CAN_Analysis_Function_exd)(uint8_t *Data,uint32_t stdid);
		IFR_DJI_Motor** Get_DJI_MotorClass(void){return DJI_MotorClass;}
	  uint8_t Get_Class_Num(void){return Class_Num;}
		
		void CAN_Base_Init(CAN_HandleTypeDef *hcan);
		void CAN_Base_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef* sFilterConfig);

#if USE_MF9015_MOTOR //9015     ز  ֣     ʹ      ifr_motor.h н  ˺   Ϊ1

		void CAN_Init(CAN_HandleTypeDef *hcan,MF_MotorClassDef *MF_Motor);
		void CAN_ReadFrom_MF_Motor(uint8_t MF_Command);
		void CAN_TransmitFor_MF_Motor(void);
		void CAN_TransmitFor_MF_Motor(uint16_t MotorId, uint8_t MF_Command, uint32_t Data);
		void CAN_TransmitFor_MF_Motor(uint16_t MotorId, uint8_t* pData);

		MF_MotorClassDef** Get_MF_MotorClass(void){return MF_MotorClass;}
		uint8_t Get_MFClass_Num(void){return MF_Class_Num;}

#endif //#if USE_MF9015_MOTOR

#if USE_DM_MOTOR
		void CAN_Init(CAN_HandleTypeDef *hcan,IFR_DM_Motor *DM_Motor);
		//void CAN_Init(CAN_HandleTypeDef *hcan,IFR_DM_Motor *DM_Motor,osThreadId *TaskHandle);
		void CAN_Transmit_DMMotor_Command(IFR_DM_Motor *DM_Motor, DM_Command_t Command);
		void CAN_TransmitFor_DM_Motor(void);
		void CAN_TransmitFor_Single_DM_Motor(uint16_t Motor_id);
		void CAN_TransmitFor_Single_DM_Motor(IFR_DM_Motor *pDM_Moror);
		void CAN_DM_Motor_All_Enable(void);
		
		IFR_DM_Motor** Get_DM_MotorClass(void){return DM_MotorClass;}
		uint8_t Get_DMClass_Num(void){return DM_Class_Num;}
#endif //#if USE_DM_MOTOR
		
	private:
		CAN_HandleTypeDef *_hcan;
		IFR_DJI_Motor* DJI_MotorClass[8];
		uint8_t Class_Num;
		uint8_t RxData[8];
		CAN_RxHeaderTypeDef RXHeader;

		uint32_t Transmit_Flag;
		uint32_t Transmit_Finish_Flag;
		uint8_t MotorData[3][8];
		uint8_t TxData[32][8];
		CAN_TxHeaderTypeDef TxHeader[32];

#if USE_MF9015_MOTOR
		uint8_t MF_Class_Num;
		MF_MotorClassDef* MF_MotorClass[8];
#endif //#if USE_MF9015_MOTOR

#if USE_DM_MOTOR
		uint8_t DM_Class_Num;
		IFR_DM_Motor* DM_MotorClass[8];
#endif
};
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void IFR_CAN_Recevice_Callback(CAN_HandleTypeDef *hcan);
void IFR_FreeRTOS_CAN_Recevice_Callback(CAN_HandleTypeDef *_hcan);
void IFR_CAN_Transmit_Callback(CAN_HandleTypeDef *hcan);
static uint8_t IFR_Can_ID_Get(CAN_HandleTypeDef *hcan);
void Can1Rx_Task(void const * argument);
void Can2Rx_Task(void const * argument);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif
#endif

