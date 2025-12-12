/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_can.cpp
  * Version		: v2.2
  * Author		: LiuHao Lijiawei Albert
  * Date			: 2022-09-27
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_can.h"
#include "string.h"
/* Private variables -------------------------------------------------*/
#ifdef HAL_CAN_MODULE_ENABLED	//如果底下是虚的说明你没使用任何CAN口
#if USE_HAL_CAN_REGISTER_CALLBACKS	//如果底下是虚的说明你没使能Register Callback CAN
IFR_CAN_ClassDef *CAN_Pointer[3] = {0};//CAN通信类指针，用于定向查找解析函数

//osMailQId CanRx_MailQid[3] = {NULL};//CAN接收OS邮箱。
char task1_name[12] = "can1Rx_Task";//CAN1句柄名
char task2_name[12] = "can2Rx_Task";//CAN2句柄名
/* Private function prototypes -----------------------------------------------*/
/**
  * @概述	CAN通信初始化,包括过滤器的配置。
  * @参数1	CAN句柄
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan)
{
	_hcan = hcan;

	CAN_Base_Init(hcan);
}

/**
  * @概述	CAN通信初始化。
  * @参数1	CAN句柄
  * @参数2	大疆电机对象指针
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan,IFR_DJI_Motor *DJI_Motor)
{
	_hcan = hcan;
	DJI_MotorClass[Class_Num] = DJI_Motor;			Class_Num++;

	CAN_Base_Init(hcan);
}

 /**
  * @概述	CAN通信初始化，并创建接收Task，适用于使用了FreeRTOS的情况。如果使用需确保调用的部分至少有额外128*32bit空间。
  * @参数1	CAN句柄
  * @参数2	大疆电机对象指针
  * @参数3	FreeRTOS任务句柄指针，用来告知外部任务句柄。
  * @返回值 void
  */
//void IFR_CAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan,IFR_DJI_Motor *DJI_Motor,osThreadId *TaskHandle)
//{
//	_hcan = hcan;
//	
//	DJI_MotorClass[Class_Num] = DJI_Motor;			Class_Num++;
//	
//	if (CanRx_MailQid[IFR_Can_ID_Get(_hcan)] == NULL)//第一次初始化
//	{
//		if(hcan->Instance == CAN1)//创建邮箱
//		{
//			osMailQDef(Can1_RxMail, 16, CanRxTypedef);
//			CanRx_MailQid[1] = osMailCreate(osMailQ(Can1_RxMail), NULL);
//			if (CanRx_MailQid[1] == NULL) Error_Handler();

////			const osThreadDef_t os_thread_def_can1Rx_Task = { task1_name, Can1Rx_Task, osPriorityRealtime, 0, 128};

//			osThreadDef(can1Rx_Task, Can1Rx_Task, osPriorityRealtime, 0, 128);
//			_TaskHandle = osThreadCreate(osThread(can1Rx_Task), NULL);

//		}
//		else if(hcan->Instance == CAN2)
//		{
//			osMailQDef(Can2_RxMail, 16, CanRxTypedef);
//			while(1)
//			{
//				CanRx_MailQid[2] = osMailCreate(osMailQ(Can2_RxMail), NULL);
//				if (CanRx_MailQid[2] != NULL) break;

//			}

////			const osThreadDef_t os_thread_def_can2Rx_Task = { task2_name, Can2Rx_Task, osPriorityHigh, 0, 128};
//			osThreadDef(can2Rx_Task, Can2Rx_Task, osPriorityHigh, 0, 128);
//			_TaskHandle = osThreadCreate(osThread(can2Rx_Task), NULL);
//		}
//	}
//	
//	CAN_Base_Init(hcan);
//	if (TaskHandle != NULL && *TaskHandle == NULL)
//		*TaskHandle = _TaskHandle;//传进一个空的有实例的句柄，防止传入
//	
//}

 /**
  * @概述	CAN发送，同一时间多次发送消息不会丢除。
  * @参数1	CAN发送协议句柄
  * @参数2	发送数据头指针
  * @返回值 void
  */
	int j = 0;
void IFR_CAN_ClassDef::CAN_Transmit(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
	uint32_t TxMailbox;
	j = HAL_CAN_GetTxMailboxesFreeLevel(_hcan);
	if(HAL_CAN_GetTxMailboxesFreeLevel(_hcan)>0 && Transmit_Flag == Transmit_Finish_Flag)
	{
		HAL_CAN_AddTxMessage(_hcan,pHeader,pData,&TxMailbox);
	}
	else
	{
		TxHeader[Transmit_Flag%32]=*pHeader;
		for(int i=0;i<pHeader->DLC;i++) TxData[Transmit_Flag%32][i]=pData[i];
		if(Transmit_Flag-Transmit_Finish_Flag>32)
		{
			Error_Handler();
		}
		else if (Transmit_Flag-Transmit_Finish_Flag > 31)
			Transmit_Flag++;
		HAL_CAN_ActivateNotification(_hcan,CAN_IT_TX_MAILBOX_EMPTY);
	}
}
uint8_t test_data[8];
 /**
  * @概述	CAN接收处理函数，只针对大疆电机设计。
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_Recevice(void)
{
	uint8_t i;
	if(HAL_CAN_GetRxMessage(_hcan,CAN_RX_FIFO0,&RXHeader,RxData) == HAL_OK)
	{
		if (RXHeader.IDE == CAN_ID_STD)
		{
			for(i=0;i<Class_Num;i++)
			{
				if(DJI_MotorClass[i]!=NULL)
				{
					(DJI_MotorClass[i]->DJI_Motor_Analysis)(RxData,RXHeader.StdId);
				}
			}
			#if USE_MF9015_MOTOR
				for(i=0;i<MF_Class_Num;i++)
				{
					if(MF_MotorClass[i]!=NULL)
					{
						(MF_MotorClass[i]->MF_Motor_Analysis)(RxData,RXHeader.StdId);
					}
				}
			#endif //#if USE_MF9015_MOTOR
			#if USE_DM_MOTOR
				for(i=0;i<DM_Class_Num;i++)
				{
					if(DM_MotorClass[i]!=NULL)
					{
						(DM_MotorClass[i]->DM_Motor_Analysis)(RxData,RXHeader.StdId);
					}
				}
			#endif //#if USE_DM_MOTOR
			if(CAN_Analysis_Function_std != NULL) 
			{
				(*CAN_Analysis_Function_std)(RxData,RXHeader.StdId);
			}
		}
		else 
		{
			if(CAN_Analysis_Function_exd != NULL) 
			{
				(*CAN_Analysis_Function_exd)(RxData,RXHeader.ExtId);
			}
		}
	}
}
 /**
  * @概述	CAN发送处理同一时间多调消息，为CAN发送中断自动调用，一般外部不要调用。
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_MultiMessage_Transmit(void)
{
	uint32_t TxMailbox;
	if(Transmit_Flag>Transmit_Finish_Flag)
	{
		HAL_CAN_AddTxMessage(_hcan,&TxHeader[Transmit_Finish_Flag%32],TxData[Transmit_Finish_Flag%32],&TxMailbox);
		Transmit_Finish_Flag++;
	}
	else HAL_CAN_DeactivateNotification(_hcan,CAN_IT_TX_MAILBOX_EMPTY);
}
//中断回调函数，禁止改写和调用!!!
void IFR_CAN_Recevice_Callback(CAN_HandleTypeDef *_hcan)
{
	IFR_CAN_ClassDef* pCAN_Receive = CAN_Pointer[IFR_Can_ID_Get(_hcan)];
	pCAN_Receive->CAN_Recevice();
}
 //中断回调函数，禁止改写和调用!!!
//void IFR_FreeRTOS_CAN_Recevice_Callback(CAN_HandleTypeDef *hcan)
//{
//	uint8_t CAN_ID = IFR_Can_ID_Get(hcan);
//	CanRxTypedef* CanRx = (CanRxTypedef*)osMailAlloc(CanRx_MailQid[CAN_ID], 0);
//	if (CanRx == NULL)
//	{
//		CAN_Pointer[CAN_ID]->CAN_Recevice();
//		return;
//	}
//	if (HAL_OK == HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRx->CAN_RxHeaderStru, CanRx->pRxdata))
//	{

//		if (osMailPut(CanRx_MailQid[CAN_ID], CanRx) != osOK)
//		{
//			Error_Handler();
//		}
//	}
//}
//中断回调函数，禁止改写和调用!!!
void IFR_CAN_Transmit_Callback(CAN_HandleTypeDef *_hcan)
{
	IFR_CAN_ClassDef* pCAN_Transmit = CAN_Pointer[IFR_Can_ID_Get(_hcan)];
	pCAN_Transmit->CAN_MultiMessage_Transmit();
}
 /**
  * @概述	CAN发送，针对大疆电机自动的CAN消息处理与发送。
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_TransmitForMotor(void)
{
	uint8_t i,_Flag_[3]={0};
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	for(i=0;i<Class_Num;i++)
	{
		if(DJI_MotorClass[i]!=NULL)
		{
			switch(DJI_MotorClass[i]->Get_DJIMotor_Num())
			{
				case 0x201:
					_Flag_[0]++;
					MotorData[0][0]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[0][1]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x205:
					_Flag_[1]++;
					MotorData[1][0]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[1][1]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x209:
					_Flag_[2]++;
					MotorData[2][0]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[2][1]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x202:
					_Flag_[0]++;
					MotorData[0][2]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[0][3]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x206:
					_Flag_[1]++;
					MotorData[1][2]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[1][3]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x20A:
					_Flag_[2]++;
					MotorData[2][2]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[2][3]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x203:
					_Flag_[0]++;
					MotorData[0][4]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[0][5]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x207:
					_Flag_[1]++;
					MotorData[1][4]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[1][5]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x20B:
					_Flag_[2]++;
					MotorData[2][4]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[2][5]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x204:
					_Flag_[0]++;
					MotorData[0][6]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[0][7]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x208:
					_Flag_[1]++;
					MotorData[1][6]= (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[1][7]= (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
			}
		}
	}
	if(_Flag_[0] !=0)
	{
		TxMessage.StdId = 0x200;
		CAN_Transmit(&TxMessage,MotorData[0]);
	}
	if(_Flag_[1] !=0)
	{
		TxMessage.StdId = 0x1ff;
		CAN_Transmit(&TxMessage,MotorData[1]);
	}
	if(_Flag_[2] !=0)
	{
		TxMessage.StdId = 0x2ff;
		CAN_Transmit(&TxMessage,MotorData[2]);
	}
}

/**
  * @概述	CAN基本初始化，包括中断启动，can启动。
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_Base_Init(CAN_HandleTypeDef *hcan)
{
	if (CAN_Pointer[IFR_Can_ID_Get(hcan)] != NULL)
	{
		if (CAN_Pointer[IFR_Can_ID_Get(hcan)] == this) return;
		else Error_Handler();//有重复的can对象
	}
		

	CAN_Pointer[IFR_Can_ID_Get(hcan)] = this;
	
	CAN_FilterTypeDef sFilterConfig;
	if(hcan->Instance == CAN1) sFilterConfig.FilterBank=0,sFilterConfig.SlaveStartFilterBank=14;
	else sFilterConfig.FilterBank=14,sFilterConfig.SlaveStartFilterBank=14;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	
#ifdef _CMSIS_OS_H
	if (_TaskHandle != NULL) HAL_CAN_RegisterCallback(hcan,HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, IFR_FreeRTOS_CAN_Recevice_Callback);
	else HAL_CAN_RegisterCallback(hcan,HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,IFR_CAN_Recevice_Callback);
	
#else
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,IFR_CAN_Recevice_Callback);
#endif

  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(hcan);
}

/**
  * @概述	CAN基本初始化，包括中断启动，can启动，配置过滤器。
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_Base_Init(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef* sFilterConfig)
{
	if (CAN_Pointer[IFR_Can_ID_Get(hcan)] != NULL)
	{
		if (CAN_Pointer[IFR_Can_ID_Get(hcan)] == this) 
		{
			if(hcan->Instance == CAN1) sFilterConfig->FilterBank=0,sFilterConfig->SlaveStartFilterBank=14;
			else sFilterConfig->FilterBank=14,sFilterConfig->SlaveStartFilterBank=14;
			
			if (sFilterConfig->FilterFIFOAssignment == CAN_RX_FIFO0)	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
			else Error_Handler();//当前IFR库版本暂时不支持FIFO1的使用，请自己写can的各个功能
			
			HAL_CAN_Start(hcan);
			return;
		}
		else Error_Handler();//有重复的can对象
	}

	CAN_Pointer[IFR_Can_ID_Get(hcan)] = this;

	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);

	HAL_CAN_RegisterCallback(hcan,HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,IFR_CAN_Recevice_Callback);
	
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(hcan);
}

//static函数外部无法调用，禁止改写!!!
static uint8_t IFR_Can_ID_Get(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1) 			return 1;
	else if(hcan->Instance == CAN2) return 2;
	else return 0;
}
//FreeRTOS的CAN1接收任务，不可调用！！！
//void Can1Rx_Task(void const * argument)
//{
//	CanRxTypedef CanRx;
//	IFR_CAN_ClassDef* pCAN_Receive = NULL;

//	pCAN_Receive = CAN_Pointer[1];
//	while(pCAN_Receive == NULL)
//	{
//		osDelay(1);
//		pCAN_Receive = CAN_Pointer[1];
//	} 

//	for(;;)
//	{
//		osEvent event = osMailGet(CanRx_MailQid[1], osWaitForever);
//		if (event.status == osEventMail && event.value.p != NULL)
//		{
//			CanRx = *(CanRxTypedef*)(event.value.p);
//			osMailFree(CanRx_MailQid[1], event.value.p);
//			
//			
//			for(uint8_t i=0;i<pCAN_Receive->Get_Class_Num();i++)
//				if(pCAN_Receive->Get_DJI_MotorClass()[i]!=NULL)
//					(pCAN_Receive->Get_DJI_MotorClass()[i]->DJI_Motor_Analysis)(CanRx.pRxdata,CanRx.CAN_RxHeaderStru.StdId);

//#if USE_MF9015_MOTOR
//			for(uint8_t i=0;i<pCAN_Receive->Get_MFClass_Num();i++)
//				if(pCAN_Receive->Get_MF_MotorClass()[i]!=NULL)
//					(pCAN_Receive->Get_MF_MotorClass()[i]->MF_Motor_Analysis)(CanRx.pRxdata,CanRx.CAN_RxHeaderStru.StdId);
//#endif //#if USE_MF9015_MOTOR

//#if USE_DM_MOTOR
//			for(uint8_t i=0;i<pCAN_Receive->Get_DMClass_Num();i++)
//				if(pCAN_Receive->Get_DM_MotorClass()[i]!=NULL)
//					(pCAN_Receive->Get_DM_MotorClass()[i]->DM_Motor_Analysis)(CanRx.pRxdata,CanRx.CAN_RxHeaderStru.StdId);
//#endif //#if USE_DM_MOTOR
//			
//			if(pCAN_Receive->CAN_Analysis_Function != NULL) (*pCAN_Receive->CAN_Analysis_Function)(CanRx.pRxdata,CanRx.CAN_RxHeaderStru.StdId);
//		}

//	}

//}
//FreeRTOS的CAN2接收任务，不可调用！！！
//void Can2Rx_Task(void const * argument)
//{
//	CanRxTypedef CanRx;
//	IFR_CAN_ClassDef* pCAN_Receive = NULL;
//	while (CanRx_MailQid[2] == NULL) osDelay(1);
//	do
//	{
//		osDelay(1);
//		pCAN_Receive = CAN_Pointer[2];

//	} while(pCAN_Receive == NULL );

//	for(;;)
//	{
//		osEvent event = osMailGet(CanRx_MailQid[2], osWaitForever);
//		if (event.status == osEventMail && event.value.p != NULL)
//		{
//			CanRx = *((CanRxTypedef*)(event.value.p));

//			for(uint8_t i=0;i<pCAN_Receive->Get_Class_Num();i++)
//				if(pCAN_Receive->Get_DJI_MotorClass()[i]!=NULL)
//					(pCAN_Receive->Get_DJI_MotorClass()[i]->DJI_Motor_Analysis)(CanRx.pRxdata,CanRx.CAN_RxHeaderStru.StdId);

//#if USE_MF9015_MOTOR
//			for(uint8_t i=0;i<pCAN_Receive->Get_MFClass_Num();i++)
//				if(pCAN_Receive->Get_MF_MotorClass()[i]!=NULL)
//					(pCAN_Receive->Get_MF_MotorClass()[i]->MF_Motor_Analysis)(CanRx.pRxdata,CanRx.CAN_RxHeaderStru.StdId);
//#endif //#if USE_MF9015_MOTOR

//#if USE_DM_MOTOR
//		for(uint8_t i=0;i<pCAN_Receive->Get_DMClass_Num();i++)
//			if(pCAN_Receive->Get_DM_MotorClass()[i]!=NULL)
//				(pCAN_Receive->Get_DM_MotorClass()[i]->DM_Motor_Analysis)(CanRx.pRxdata,CanRx.CAN_RxHeaderStru.StdId);
//#endif //#if USE_DM_MOTOR
//		if(pCAN_Receive->CAN_Analysis_Function != NULL)
//			(*pCAN_Receive->CAN_Analysis_Function)(CanRx.pRxdata,CanRx.CAN_RxHeaderStru.StdId);
//			osMailFree(CanRx_MailQid[2], event.value.p);
//		}

//	}
//}
#if USE_MF9015_MOTOR

/************************/
/**
  * @概述	CAN通信初始化。
  * @参数1	CAN句柄
  * @参数2	MF电机对象指针
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan,MF_MotorClassDef *MF_Motor)
{
	_hcan = hcan;
	CAN_Pointer[IFR_Can_ID_Get(hcan)] = this;

	MF_MotorClass[MF_Class_Num] = MF_Motor;			MF_Class_Num++;

  CAN_FilterTypeDef sFilterConfig;
	if(hcan->Instance == CAN1) sFilterConfig.FilterBank=0,sFilterConfig.SlaveStartFilterBank=14;
	else sFilterConfig.FilterBank=14,sFilterConfig.SlaveStartFilterBank=14;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan, &sFilterConfig);

	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(hcan,HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID,IFR_CAN_Transmit_Callback);

	HAL_CAN_RegisterCallback(hcan,HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID,IFR_CAN_Recevice_Callback);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_Start(hcan);
}
 /**
  * @概述		从MF9015读取数据。
  * @参数1	MF9015电机读取命令 @rel MF_COMMAND_R_PID
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_ReadFrom_MF_Motor(uint8_t MF_Command)
{
	uint8_t pTxData[8] = {MF_Command, 0,0,0,0,0,0,0},i;
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	for(i=0;i<MF_Class_Num;i++)
	{
		if(MF_MotorClass[i]!=NULL)
		{
			TxMessage.StdId = MF_MotorClass[i]->Get_MotorId();
			CAN_Transmit(&TxMessage,pTxData);
		}
	}
}

 /**
  * @概述		发送多电机指令，进行多电机控制。
  * @参数1	void
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_TransmitFor_MF_Motor(void)
{
	uint8_t pTxData[8] = {0},i;
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.StdId = 0x280;
	for(i=0;i<MF_Class_Num;i++)
	{
		if(MF_MotorClass[i]!=NULL)
		{
			switch (MF_MotorClass[i]->Get_MotorId())
			{
				case 0x141:
					*(int16_t*)(pTxData) = MF_MotorClass[i]->Get_MotorOutput();
					break;
				case 0x142:
					*(int16_t*)(pTxData+2) = MF_MotorClass[i]->Get_MotorOutput();
					break;
				case 0x143:
					*(int16_t*)(pTxData+4) = MF_MotorClass[i]->Get_MotorOutput();
					break;
				case 0x144:
					*(int16_t*)(pTxData+6) = MF_MotorClass[i]->Get_MotorOutput();
					break;
			}
		}
	}
	CAN_Transmit(&TxMessage,pTxData);
}

/**
  * @概述		你猜是干啥用的
  * @参数1	不详
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_TransmitFor_MF_Motor(uint16_t MotorId, uint8_t MF_Command, uint32_t Data)
{
	uint8_t pTxData[8] = {MF_Command, 0,0,0,0,0,0,0};
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.StdId = MotorId;

	*((uint32_t*)(pTxData+4)) = Data;

	CAN_Transmit(&TxMessage,pTxData);
}

/**
  * @概述		请参考说明书，反正肯定有用。
  * @参数1	不详
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_TransmitFor_MF_Motor(uint16_t MotorId, uint8_t* pData)
{
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.StdId = MotorId;
	CAN_Transmit(&TxMessage,pData);
}
#endif //#if USE_MF9015_MOTOR

#if USE_DM_MOTOR
/**
  * @概述	CAN通信初始化。
  * @参数1	CAN句柄
  * @参数2	MF电机对象指针
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan,IFR_DM_Motor *DM_Motor)
{
	_hcan = hcan;

	DM_MotorClass[DM_Class_Num] = DM_Motor;			DM_Class_Num++;

  CAN_Base_Init(hcan);
}

 /**
  * @概述	CAN通信初始化，并创建接收Task，适用于使用了FreeRTOS的情况。如果使用需确保调用的部分至少有额外128*32bit空间。
  * @参数1	CAN句柄
  * @参数2	大疆电机对象指针
  * @参数3	FreeRTOS任务句柄指针，用来告知外部任务句柄。
  * @返回值 void
  */
//void IFR_CAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan,IFR_DM_Motor *DM_Motor,osThreadId *TaskHandle)
//{
//	_hcan = hcan;
//	
//	DM_MotorClass[DM_Class_Num] = DM_Motor;			DM_Class_Num++;
//	
//	if (CanRx_MailQid[IFR_Can_ID_Get(_hcan)] == NULL)//第一次初始化
//	{
//		if(hcan->Instance == CAN1)//创建邮箱
//		{
//			osMailQDef(Can1_RxMail, 8, CanRxTypedef);
//			CanRx_MailQid[1] = osMailCreate(osMailQ(Can1_RxMail), NULL);
//			if (CanRx_MailQid[1] == NULL) Error_Handler();

//			const osThreadDef_t os_thread_def_can1Rx_Task = { task1_name, Can1Rx_Task, osPriorityHigh, 0, 128};

////			osThreadDef(can1Rx_Task, Can1Rx_Task, osPriorityHigh, 0, 128);
//			_TaskHandle = osThreadCreate(osThread(can1Rx_Task), NULL);

//		}
//		else if(hcan->Instance == CAN2)
//		{
//			osMailQDef(Can2_RxMail, 8, CanRxTypedef);
//			while(1)
//			{
//				CanRx_MailQid[2] = osMailCreate(osMailQ(Can2_RxMail), NULL);
//				if (CanRx_MailQid[2] != NULL) break;

//			}

//			const osThreadDef_t os_thread_def_can2Rx_Task = { task2_name, Can1Rx_Task, osPriorityHigh, 0, 128};
////			osThreadDef(can2Rx_Task, Can2Rx_Task, osPriorityHigh, 0, 128);
//			_TaskHandle = osThreadCreate(osThread(can2Rx_Task), NULL);
//		}

//		CAN_Base_Init(hcan);
//	}
//	if (TaskHandle != NULL && *TaskHandle == NULL)
//		*TaskHandle = _TaskHandle;//传进一个空的有实例的句柄，防止传入
//	
//}

/**
	* @概述	CAN发送电机命令。
  * @参数1	IFR_DM_Motor对象指针
  * @参数2	DM电机命令
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_Transmit_DMMotor_Command(IFR_DM_Motor *DM_Motor, DM_Command_t Command)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t pTxData[8];
	
	tx_header.DLC = 8;
	tx_header.ExtId = 0;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.StdId = DM_Motor->Get_DMMotor_Id();
	tx_header.TransmitGlobalTime = DISABLE;
	
	for (uint8_t i=0;i<7;i++) pTxData[i] = 0xFF;
	pTxData[7] = (uint8_t)Command;
//	DM_Motor->DM_Motor_Command(Command);
	CAN_Transmit(&tx_header, pTxData);
}
/**
	* @概述	CAN发送DM电机的输出。
  * @参数1	IFR_DM_Motor对象指针
  * @参数2	DM电机命令
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_TransmitFor_DM_Motor(void)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t pTxData[8] = {0x7F,0xFF,0x7F,0xF0,0x00,0x00,0x07,0xFF};
	uint16_t tar_f = 0x07FF;
	
	tx_header.DLC = 8;
	tx_header.ExtId = 0;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	
	tx_header.TransmitGlobalTime = DISABLE;
	
	for (uint8_t i=0;i<DM_Class_Num;i++)
		if (DM_MotorClass[i] != NULL)
		{
			tx_header.StdId = DM_MotorClass[i]->Get_DMMotor_Id();
			tar_f = DM_MotorClass[i]->get_t_ff();
			
			pTxData[6] = (tar_f>>8)&0x0F;
  		pTxData[7] = (uint8_t)tar_f;
			
			CAN_Transmit(&tx_header, pTxData);
		}
	
}

/**
	* @概述	CAN发送单一DM电机的输出。
  * @参数1	IFR_DM_Motor对象指针
  * @参数2	DM电机命令
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_TransmitFor_Single_DM_Motor(uint16_t Motor_id)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t pTxData[8] = {0x7F,0xFF,0x7F,0xF0,0x00,0x00,0x07,0xFF};
	uint16_t tar_f = 0x07FF;
	
	tx_header.DLC = 8;
	tx_header.ExtId = 0;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.TransmitGlobalTime = DISABLE;
	
	for (uint8_t i=0;i<DM_Class_Num;i++)
		if (DM_MotorClass[i] != NULL && DM_MotorClass[i]->Get_DMMotor_Id() == Motor_id)
		{
			tx_header.StdId = DM_MotorClass[i]->Get_DMMotor_Id();
			tar_f = DM_MotorClass[i]->get_t_ff();
			
			pTxData[6] = (tar_f>>8)&0x0F;
  		pTxData[7] = (uint8_t)tar_f;
			
			CAN_Transmit(&tx_header, pTxData);
			break;
		}
	
}

/**
	* @概述	CAN发送单一DM电机的输出。
  * @参数1	IFR_DM_Motor对象指针
  * @参数2	DM电机命令
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_TransmitFor_Single_DM_Motor(IFR_DM_Motor *pDM_Moror)
{
	if (pDM_Moror == NULL) return;
	
	CAN_TxHeaderTypeDef tx_header;
	uint8_t pTxData[8] = {0x7F,0xFF,0x7F,0xF0,0x00,0x00,0x07,0xFF};
	uint16_t tar_f = 0x07FF;
	
	tx_header.DLC = 8;
	tx_header.ExtId = 0;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.TransmitGlobalTime = DISABLE;
	
	tx_header.StdId = pDM_Moror->Get_DMMotor_Id();
	tar_f = pDM_Moror->get_t_ff();
	
	pTxData[6] = (tar_f>>8)&0x0F;
	pTxData[7] = (uint8_t)tar_f;
	
	CAN_Transmit(&tx_header, pTxData);
	
}

/**
	* @概述		检测所有达妙电机状态，移除达妙异常状态。
  * @参数1	void
  * @返回值 void
  */
void IFR_CAN_ClassDef::CAN_DM_Motor_All_Enable(void)
{	
	for (uint8_t i=0;i<DM_Class_Num;i++)
	{
		if (DM_MotorClass[i] != NULL)
		{
			if (DM_MotorClass[i]->DM_Info.DM_State != DMState_Enable)
			{
				if (DM_MotorClass[i]->DM_Info.DM_State != DMState_Disable) CAN_Transmit_DMMotor_Command(DM_MotorClass[i], DM_ClearError);
				CAN_Transmit_DMMotor_Command(DM_MotorClass[i], DM_Enable);
			}
		}
	}
}
#endif //#if USE_DM_MOTOR

#endif
#endif

