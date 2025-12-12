/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_fdcan.h
  * Version		: v2.1
  * Author		: LiuHao Lijiawei
  * Date		: 2022-09-27
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_FDCAN_H_
#define __IFR_FDCAN_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#ifdef HAL_FDCAN_MODULE_ENABLED
#include "ifr_motor.h"
#include "ifr_dmmotor.h"
#include "ifr_robstride.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

class IFR_FDCAN_ClassDef
{
	public:
		void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan);
		void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, IFR_DJI_Motor *DJI_Motor);
		void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, IFR_LZ_Motor *LZ_Motor);
		uint8_t FDCAN_Transmit(FDCAN_TxHeaderTypeDef *pHeader, uint8_t *pData);
		void FDCAN_MultiMessage_Transmit(void);
		void FDCAN_Recevice(void);
		void FDCAN_TransmitForMotor(void);
		void FDCAN_All_Transmit();
		void (*FDCAN_Analysis_Function)(uint8_t *Data, uint32_t stdid);
		IFR_DJI_Motor** Get_DJI_MotorClass(void){return DJI_MotorClass;}
	  uint8_t Get_Class_Num(void){return Class_Num;}
		uint8_t num1;

#if USE_MF9015_MOTOR //9015     ?  ?     ?      ifr_motor.h §ß  ?   ?1

		void FDCAN_Init(FDCAN_HandleTypeDef *fdcan,MF_MotorClassDef *MF_Motor);
		void FDCAN_ReadFrom_MF_Motor(uint8_t MF_Command);
		void FDCAN_TransmitFor_MF_Motor(void);
		void FDCAN_TransmitFor_MF_Motor(uint16_t MotorId, uint8_t MF_Command, uint32_t Data);
		void FDCAN_TransmitFor_MF_Motor(uint16_t MotorId, uint8_t* pData);

		MF_MotorClassDef** Get_MF_MotorClass(void){return MF_MotorClass;}
		uint8_t Get_MFClass_Num(void){return MF_Class_Num;}

#endif //#if USE_MF9015_MOTOR
#if USE_DM_MOTOR
		void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, IFR_DM_Motor *DM_Motor);
		//void CAN_Init(CAN_HandleTypeDef *hcan,IFR_DM_Motor *DM_Motor,osThreadId *TaskHandle);
		void FDCAN_Transmit_DMMotor_Command(IFR_DM_Motor *DM_Motor, DM_Command_t Command);
		void FDCAN_TransmitFor_DM_Motor(void);
		void FDCAN_TransmitFor_Single_DM_Motor(uint16_t Motor_id);
		void FDCAN_TransmitFor_Single_DM_Motor(IFR_DM_Motor *pDM_Moror);
		void FDCAN_DM_Motor_All_Enable(void);
		
		IFR_DM_Motor** Get_DM_MotorClass(void){return DM_MotorClass;}
		uint8_t Get_DMClass_Num(void){return DM_Class_Num;}
		void FDCAN_Robstride_Motor_All_Enable(void);
#endif //#if USE_DM_MOTOR
	private:
		FDCAN_HandleTypeDef *_hfdcan;
		uint8_t LZ_Class_Num;
		IFR_LZ_Motor* LZ_MotorClass[8];
		IFR_DJI_Motor* DJI_MotorClass[8];
		uint8_t Class_Num;
		uint8_t RxData[8];
		FDCAN_RxHeaderTypeDef RXHeader;

		uint32_t Transmit_Flag;
		uint32_t Transmit_Finish_Flag;
		uint8_t MotorData[3][8];
		uint8_t TxData[48][8];
		FDCAN_TxHeaderTypeDef TxHeader[48];

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

void IFR_FDCAN_Transmit_Callback(FDCAN_HandleTypeDef *_fdcan);
void IFR_FDCAN_Recevice_Callback(FDCAN_HandleTypeDef *_fdcan, uint32_t RxFifo0ITs);
static uint8_t IFR_FDCAN_ID_Get(FDCAN_HandleTypeDef *hfdcan);
extern IFR_FDCAN_ClassDef *FDCAN_Pointer[4];
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#endif
#endif

