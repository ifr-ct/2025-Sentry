/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: IFR_FDCAN.cpp
  * Version		: v2.2
  * Author		: LiuHao Lijiawei Albert
  * Date			: 2022-09-27
  * Description	:

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_fdcan.h"
/* Private variables -------------------------------------------------*/
#ifdef HAL_FDCAN_MODULE_ENABLED	//如果底下是虚的说明你没使用任何FDCAN口
#if USE_HAL_FDCAN_REGISTER_CALLBACKS	//如果底下是虚的说明你没使能Register Callback FDCAN
IFR_FDCAN_ClassDef *FDCAN_Pointer[4] = {0};//FDCAN通信类指针，用于定向查找解析函数

/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
* @功能     		: FDCAN通信初始化
* @参数         : FDCAN句柄
* @返回值 			: void
* @概述  				: FDCAN通信初始化,包括过滤器的配置。
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
	_hfdcan = hfdcan;
	
	if (FDCAN_Pointer[IFR_FDCAN_ID_Get(_hfdcan)] != NULL && FDCAN_Pointer[IFR_FDCAN_ID_Get(_hfdcan)] == this)
		return;

	FDCAN_Pointer[IFR_FDCAN_ID_Get(_hfdcan)] = this;
	
	// 配置过滤器
	FDCAN_FilterTypeDef fdcan_filter = {};
	fdcan_filter.FilterIndex = IFR_FDCAN_ID_Get(hfdcan) - 1; 				// 滤波器索引
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       // 标准ID
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                  // 掩码过滤器
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;          // 过滤器到 RX FIFO0
	fdcan_filter.FilterID1 = 0x0000;                              // ID
	fdcan_filter.FilterID2 = 0x0000;                              // 掩码
  HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);			
	
	//设置全局是否拒绝：不拒绝匹配不成功的标准ID和扩展ID，进入fif0，不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);		
	
		//将IFR_FDCAN_Transmit_Callback 注册为发送空闲fifo中断
	HAL_FDCAN_RegisterCallback(hfdcan, HAL_FDCAN_TX_FIFO_EMPTY_CB_ID, IFR_FDCAN_Transmit_Callback);		

	//将IFR_FDCAN_Recevice_Callback 注册为接收fifo0中断
	HAL_FDCAN_RegisterRxFifo0Callback(hfdcan, IFR_FDCAN_Recevice_Callback);				
	
	// 激活接收中断
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	
	//启动fdcan
  HAL_FDCAN_Start(hfdcan);
	
}


/*******************************************************************************
* @功能     		: FDCAN通信初始化
* @参数1        : FDCAN句柄
* @参数2        : 大疆电机对象指针
* @返回值 			: void
* @概述  				: FDCAN通信初始化,包括过滤器的配置，只针对大疆电机设计。
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, IFR_DJI_Motor *DJI_Motor)
{
	_hfdcan = hfdcan;
	DJI_MotorClass[Class_Num] = DJI_Motor;			Class_Num++;
	FDCAN_Init(hfdcan);
}
/**
  * @概述	CAN通信初始化，只针对灵足电机设计。
  * @参数1	CAN句柄
  * @参数2	灵足电机电机对象指针
  * @返回值 void
  */
void IFR_FDCAN_ClassDef::FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, IFR_LZ_Motor *LZ_Motor)
{
	LZ_MotorClass[LZ_Class_Num] = LZ_Motor;			LZ_Class_Num++;
  FDCAN_Init(hfdcan);
	LZ_Motor->Can_Motor = IFR_FDCAN_ID_Get(hfdcan);
}
/*******************************************************************************
* @功能     		: FDCAN发送
* @参数1        : FDCAN发送句柄
* @参数2        : 发送数据头指针
* @返回值 			: void
* @概述  				: FDCAN发送，同一时间多次发送消息不会丢除
*******************************************************************************/
uint8_t IFR_FDCAN_ClassDef::FDCAN_Transmit(FDCAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
	if(HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan)>0)
	{
		if (HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, pHeader, pData) != HAL_OK)
			return 1;	//发送成功
		return 0;		//发送错误
	}
	else
	{
		TxHeader[Transmit_Flag % 48] =* pHeader;
		for(int i = 0; i < pHeader->DataLength; i++) TxData[Transmit_Flag % 48][i] = pData[i];
		
		if(Transmit_Flag - Transmit_Finish_Flag > 48)
		{
			Error_Handler();	//说明库中的缓存区满·111111
		}
		else if (Transmit_Flag - Transmit_Finish_Flag < 48)
		{
			Transmit_Flag++;
		}
		HAL_FDCAN_ActivateNotification(_hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0);
		return 2;
	}
}

/*******************************************************************************
* @功能     		: FDCAN接收处理函数（大疆）
* @参数         : None
* @返回值 			: void
* @概述  				: FDCAN接收处理函数，只针对大疆电机设计
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_Recevice(void)
{
	uint8_t i;
	if(HAL_FDCAN_GetRxMessage(_hfdcan, FDCAN_RX_FIFO0, &RXHeader, RxData) == HAL_OK)
	{
		if (RXHeader.IdType == FDCAN_STANDARD_ID)
		{
			for(i = 0; i < Class_Num; i++)
				if(DJI_MotorClass[i] != NULL && RXHeader.Identifier == DJI_MotorClass[i]->Get_DJIMotor_Num())
					(DJI_MotorClass[i]->DJI_Motor_Analysis)(RxData, RXHeader.Identifier);

#if USE_MF9015_MOTOR
					for(i = 0;i < MF_Class_Num; i++)
						if(MF_MotorClass[i] != NULL)
							(MF_MotorClass[i]->MF_Motor_Analysis)(RxData, RXHeader.Identifier);
#endif //#if USE_MF9015_MOTOR
#if USE_DM_MOTOR
		for(i=0;i<DM_Class_Num;i++)
			if(DM_MotorClass[i]!=NULL)
				(DM_MotorClass[i]->DM_Motor_Analysis)(RxData,RXHeader.Identifier);
#endif //#if USE_DM_MOTOR
		}
	else if (RXHeader.IdType == FDCAN_EXTENDED_ID)
	{
		for(i = 0; i < LZ_Class_Num; i++)
			if(LZ_MotorClass[i] != NULL)
				(LZ_MotorClass[i]->LZ_Motor_Analysis)(RxData, RXHeader.Identifier);
		
	}
	if(FDCAN_Analysis_Function != NULL) (*FDCAN_Analysis_Function)(RxData, RXHeader.Identifier);

	}
}

/*******************************************************************************
* @功能     		: FDCAN发送处理同一时间多调消息
* @参数         : None
* @返回值 			: void
* @概述  				: FDCAN发送处理同一时间多调消息，为FDCAN发送中断自动调用，一般外部不要调用！
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_MultiMessage_Transmit(void)
{
	if(Transmit_Flag>Transmit_Finish_Flag)
	{
		if(HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan)>0)
		{
			HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &TxHeader[Transmit_Finish_Flag % 8], TxData[Transmit_Finish_Flag % 8]);
			Transmit_Finish_Flag++;
		}
	}
	else HAL_FDCAN_DeactivateNotification(_hfdcan, FDCAN_IT_TX_FIFO_EMPTY);
}

//中断回调函数，禁止改写和调用!!!
void IFR_FDCAN_Recevice_Callback(FDCAN_HandleTypeDef *_hfdcan, uint32_t RxFifo0ITs)
{
	IFR_FDCAN_ClassDef* pCAN_Receive = FDCAN_Pointer[IFR_FDCAN_ID_Get(_hfdcan)];
	pCAN_Receive->FDCAN_Recevice();
}

//中断回调函数，禁止改写和调用!!!
void IFR_FDCAN_Transmit_Callback(FDCAN_HandleTypeDef *_hfdcan)
{
	IFR_FDCAN_ClassDef* pCAN_Transmit = FDCAN_Pointer[IFR_FDCAN_ID_Get(_hfdcan)];
	pCAN_Transmit->FDCAN_MultiMessage_Transmit();
}

/*******************************************************************************
* @功能     		: FDCAN发送，针对大疆电机自动的FDCAN消息处理与发送。
* @参数         : None
* @返回值 			: void
* @概述  				: FDCAN发送，针对大疆电机自动的FDCAN消息处理与发送。
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_TransmitForMotor(void)
{
	uint8_t i,_Flag_[3]={0};
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_STANDARD_ID;								//标准帧，11 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	for(i = 0; i < Class_Num; i++)
	{
		if(DJI_MotorClass[i] != NULL)
		{
			switch(DJI_MotorClass[i]->Get_DJIMotor_Num())
			{
				case 0x201:
					_Flag_[0]++;
					MotorData[0][0] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[0][1] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x205:
					_Flag_[1]++;
					MotorData[1][0] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[1][1] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x209:
					_Flag_[2]++;
					MotorData[2][0] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[2][1] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x202:
					_Flag_[0]++;
					MotorData[0][2] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[0][3] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x206:
					_Flag_[1]++;
					MotorData[1][2] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[1][3] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x20A:
					_Flag_[2]++;
					MotorData[2][2] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[2][3] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x203:
					_Flag_[0]++;
					MotorData[0][4] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[0][5] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x207:
					_Flag_[1]++;
					MotorData[1][4] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[1][5] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x20B:
					_Flag_[2]++;
					MotorData[2][4] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[2][5] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x204:
					_Flag_[0]++;
					MotorData[0][6] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[0][7] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
				case 0x208:
					_Flag_[1]++;
					MotorData[1][6] = (DJI_MotorClass[i]->Get_DJIMotor_Output())>>8;
					MotorData[1][7] = (DJI_MotorClass[i]->Get_DJIMotor_Output());
					break;
			}
		}
	}
	if(_Flag_[0] !=0)
	{
		TxMessage.Identifier = 0x200;
		FDCAN_Transmit(&TxMessage, MotorData[0]);
	}
	if(_Flag_[1] !=0)
	{
		TxMessage.Identifier = 0x1ff;
		FDCAN_Transmit(&TxMessage, MotorData[1]);
	}
	if(_Flag_[2] !=0)
	{
		TxMessage.Identifier = 0x2ff;
		FDCAN_Transmit(&TxMessage, MotorData[2]);
	}
}
//用于计算FDCAN-ID
//static函数外部无法调用，禁止改写!!!
static uint8_t IFR_FDCAN_ID_Get(FDCAN_HandleTypeDef *fdcan)
{
	if(fdcan->Instance == FDCAN1) 			return 1;
	else if(fdcan->Instance == FDCAN2) 	return 2;
	else if(fdcan->Instance == FDCAN3)	return 3;
	else return 0;
}
#if USE_MF9015_MOTOR

/************************/
/**
  * @概述	FDCAN通信初始化。
  * @参数1	FDCAN句柄
  * @参数2	MF电机对象指针
  * @返回值 void
  */
void IFR_FDCAN_ClassDef::CAN_Init(FDCAN_HandleTypeDef *fdcan,MF_MotorClassDef *MF_Motor)
{
	_fdcan = fdcan;
	FDCAN_Pointer[IFR_FDCAN_ID_Get(fdcan)] = this;

	MF_MotorClass[MF_Class_Num] = MF_Motor;			MF_Class_Num++;

  FDCAN_Init(fdcan);
}
 /**
  * @概述		从MF9015读取数据。
  * @参数1	MF9015电机读取命令 @rel MF_COMMAND_R_PID
  * @返回值 void
  */
void IFR_FDCAN_ClassDef::FDCAN_ReadFrom_MF_Motor(uint8_t MF_Command)
{
	uint8_t pTxData[8] = {MF_Command, 0,0,0,0,0,0,0},i;
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_STANDARD_ID;								//标准帧，11 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	for(i=0;i<MF_Class_Num;i++)
	{
		if(MF_MotorClass[i]!=NULL)
		{
			TxMessage.Identifier = MF_MotorClass[i]->Get_MotorId();
			FDCAN_Transmit(&TxMessage, pTxData);
		}
	}
}

 /**
  * @概述		发送多电机指令，进行多电机控制。
  * @参数1	void
  * @返回值 void
  */
void IFR_FDCAN_ClassDef::FDCAN_TransmitFor_MF_Motor(void)
{
	uint8_t pTxData[8] = {0},i;
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_STANDARD_ID;								//标准帧，11 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = 0x280;
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
	FDCAN_Transmit(&TxMessage, pTxData);
}

/**
  * @概述		你猜是干啥用的
  * @参数1	不详
  * @返回值 void
  */
void IFR_FDCAN_ClassDef::FDCAN_TransmitFor_MF_Motor(uint16_t MotorId, uint8_t MF_Command, uint32_t Data)
{
	uint8_t pTxData[8] = {MF_Command, 0,0,0,0,0,0,0};
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DataLength = 8;
	TxMessage.StdId = MotorId;

	*((uint32_t*)(pTxData+4)) = Data;

	FDCAN_Transmit(&TxMessage,pTxData);
}

/**
  * @概述		请参考说明书，反正肯定有用。
  * @参数1	不详
  * @返回值 void
  */
void IFR_FDCAN_ClassDef::FDCAN_TransmitFor_MF_Motor(uint16_t MotorId, uint8_t* pData)
{
	CAN_TxHeaderTypeDef TxMessage;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DataLength = 8;
	TxMessage.StdId = MotorId;
	FDCAN_Transmit(&TxMessage,pData);
}
#endif //#if USE_MF9015_MOTOR

#if USE_DM_MOTOR
/*******************************************************************************
* @功能     		: FDCAN通信初始化
* @参数1        : FDCAN句柄
* @参数2        : MF电机对象指针
* @返回值 			: void
* @概述  				: FDCAN通信初始化，只针对达妙电机设计
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_Init(FDCAN_HandleTypeDef *hcan, IFR_DM_Motor *DM_Motor)
{
	_hfdcan = hcan;

	DM_MotorClass[DM_Class_Num] = DM_Motor;			DM_Class_Num++;

  FDCAN_Init(hcan);
}

// /**
//  * @概述	CAN通信初始化，并创建接收Task，适用于使用了FreeRTOS的情况。如果使用需确保调用的部分至少有额外128*32bit空间。
//  * @参数1	CAN句柄
//  * @参数2	大疆电机对象指针
//  * @参数3	FreeRTOS任务句柄指针，用来告知外部任务句柄。
//  * @返回值 void
//  */
//void IFR_FDCAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan,IFR_DM_Motor *DM_Motor,osThreadId *TaskHandle)
//{
//	_hcan = hcan;
//	
//	DM_MotorClass[DM_Class_Num] = DM_Motor;			DM_Class_Num++;
//	
//	if (CanRx_MailQid[IFR_Can_ID_Get(_hcan)] == NULL)//????γ????
//	{
//		if(hcan->Instance == CAN1)//????????
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
//		*TaskHandle = _TaskHandle;//????????????????????????????
//	
//}


/*******************************************************************************
* @功能     		: FDCAN发送电机命令
* @参数1        : IFR_DM_Motor对象指针
* @参数2        : DM电机命令
* @返回值 			: void
* @概述  				: FDCAN发送电机命令
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_Transmit_DMMotor_Command(IFR_DM_Motor *DM_Motor, DM_Command_t Command)
{
	uint8_t pTxData[8];
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_STANDARD_ID;								//标准帧，11 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	TxMessage.Identifier = DM_Motor->Get_DMMotor_Id();
	
	for (uint8_t i=0;i<7;i++) pTxData[i] = 0xFF;
	pTxData[7] = (uint8_t)Command;
//	DM_Motor->DM_Motor_Command(Command);
	FDCAN_Transmit(&TxMessage, pTxData);
}
/**
	* @概述	CAN发送DM电机的输出。
  * @参数1	IFR_DM_Motor对象指针
  * @参数2	DM电机命令
  * @返回值 void
  */
/*******************************************************************************
* @功能     		: FDCAN发送此FDCAN上DM电机的输出
* @参数         : None
* @返回值 			: void
* @概述  				: FDCAN发送此FDCAN上DM电机的输出
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_TransmitFor_DM_Motor(void)
{
	uint8_t pTxData[8] = {0x7F,0xFF,0x7F,0xF0,0x00,0x00,0x07,0xFF};
	uint16_t tar_f = 0x07FF;
	
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_STANDARD_ID;								//标准帧，11 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	
	for (uint8_t i=0;i<DM_Class_Num;i++)
		if (DM_MotorClass[i] != NULL)
		{
			TxMessage.Identifier = DM_MotorClass[i]->Get_DMMotor_Id();
			tar_f = DM_MotorClass[i]->get_t_ff();
			
			pTxData[6] = (tar_f>>8)&0x0F;
  		pTxData[7] = (uint8_t)tar_f;
			
			FDCAN_Transmit(&TxMessage, pTxData);
		}
	
}

/*******************************************************************************
* @功能     		: FDCAN发送单一DM电机的输出
* @参数         : 电机CANid
* @返回值 			: void
* @概述  				: FDCAN发送单一DM电机的输出
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_TransmitFor_Single_DM_Motor(uint16_t Motor_id)
{
	uint8_t pTxData[8] = {0x7F,0xFF,0x7F,0xF0,0x00,0x00,0x07,0xFF};
	uint16_t tar_f = 0x07FF;
	
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_STANDARD_ID;								//标准帧，11 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	
	for (uint8_t i=0;i<DM_Class_Num;i++)
		if (DM_MotorClass[i] != NULL && DM_MotorClass[i]->Get_DMMotor_Id() == Motor_id)
		{
			TxMessage.Identifier = DM_MotorClass[i]->Get_DMMotor_Id();
			tar_f = DM_MotorClass[i]->get_t_ff();
			
			pTxData[6] = (tar_f>>8)&0x0F;
  		pTxData[7] = (uint8_t)tar_f;
			
			FDCAN_Transmit(&TxMessage, pTxData);
			break;
		}
	
}

/*******************************************************************************
* @功能     		: FDCAN发送单一DM电机的输出
* @参数         : IFR_DM_Motor对象指针
* @返回值 			: void
* @概述  				: FDCAN发送单一DM电机的输出
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_TransmitFor_Single_DM_Motor(IFR_DM_Motor *pDM_Moror)
{
	if (pDM_Moror == NULL) return;
	
	uint8_t pTxData[8] = {0x7F,0xFF,0x7F,0xF0,0x00,0x00,0x07,0xFF};
	uint16_t tar_f = 0x07FF;
	
	FDCAN_TxHeaderTypeDef TxMessage;
	TxMessage.IdType = FDCAN_STANDARD_ID;								//标准帧，11 位 ID
	TxMessage.TxFrameType = FDCAN_DATA_FRAME;						//数据帧
	TxMessage.DataLength = FDCAN_DLC_BYTES_8;						//8 字节数据长度
	TxMessage.ErrorStateIndicator = FDCAN_ESI_ACTIVE;		//活动错误状态
	TxMessage.BitRateSwitch = FDCAN_BRS_OFF;						//关闭位速率切换
	TxMessage.FDFormat = FDCAN_CLASSIC_CAN;							//经典 CAN 格式
	TxMessage.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//不记录发送事件
	TxMessage.MessageMarker = 0;	//标记此消息，在 TX Event FIFO 中区分不同消息
	
	TxMessage.Identifier = pDM_Moror->Get_DMMotor_Id();
	tar_f = pDM_Moror->get_t_ff();
	
	pTxData[6] = (tar_f>>8)&0x0F;
	pTxData[7] = (uint8_t)tar_f;
	
	FDCAN_Transmit(&TxMessage, pTxData);
	
}


/*******************************************************************************
* @功能     		: 检测所有达妙电机状态，未使能的使能，移除达妙异常状态。
* @参数         : None
* @返回值 			: void
* @概述  				: 检测所有达妙电机状态，未使能的使能，移除达妙异常状态。
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_DM_Motor_All_Enable(void)
{	
	for (uint8_t i=0;i<DM_Class_Num;i++)
	{
		if (DM_MotorClass[i] != NULL)
		{
			if (DM_MotorClass[i]->DM_Info.DM_State != DMState_Enable)
			{
				if (DM_MotorClass[i]->DM_Info.DM_State != DMState_Disable) FDCAN_Transmit_DMMotor_Command(DM_MotorClass[i], DM_ClearError);
				FDCAN_Transmit_DMMotor_Command(DM_MotorClass[i], DM_Enable);
			}
		}
	}
}

/*******************************************************************************
* @功能     		: 检测所有灵足电机状态，未使能的使能，移除灵足异常状态。
* @参数         : None
* @返回值 				: void
* @概述  				: 检测所有灵足电机状态，未使能的使能，移除灵足异常状态。
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_Robstride_Motor_All_Enable(void)
{	
	for (uint8_t i  =0; i < LZ_Class_Num; i++)
	{
		if (LZ_MotorClass[i] != NULL)
		{
			if (LZ_MotorClass[i]->LZ_Pos_Info.Error_Code != 0) 
				LZ_MotorClass[i]->Disable_Motor(1);
			else if (LZ_MotorClass[i]->LZ_Pos_Info.Pattern != 2)
			{
							
//				LZ_MotorClass[i]->Set_LZ_Motor_parameter(0X7005, 0, 'P'); // 设置电机模式运控
				LZ_MotorClass[i]->Enable_Motor();
			}
		}
	}
}

#endif //#if USE_DM_MOTOR

#endif
#endif

