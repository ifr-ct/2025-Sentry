/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2024, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_remote.c
  * Version		: v2.3
  * Author		: LiuHao Lijiawei Albert
  * Date		: 2023-07-26
  * Description	: 各种遥控器的解析函数

  *********************************************************************
  */

/* Includes ---------------------------------------------------------*/
#include "ifr_remote.h"

/* Private variables -------------------------------------------------*/
DJI_REMOTE_TypeDef DJI_Remote = {1024,1024,1024,1024,RC_SWITCH_UP,RC_SWITCH_UP,1024};
KEYBOARD_TypeDef Keyboard_Data[KEY_NUMS]={0};

DJI_CilentRobotCommand_Typedef CilentRobotCommand = {0};

ET10_RemoteTypedef ET10_Remote;
/**
  * @概述	DJI遥控器解析函数，无键盘数据解析。
  * @参数1	解析数据包头指针
  * @参数2  数据包长度
  * @返回值 void
  */
void IFR_DJI_Remote_Analysis(uint8_t *pData,uint8_t len)
{
	if(pData == 0)
	{
		return;
	}

	DJI_Remote.Chx_Right = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	DJI_Remote.Chy_Right = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	DJI_Remote.Chx_Left = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	DJI_Remote.Chy_Left = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	DJI_Remote.Switch_Left = ((pData[5] >> 4) & 0x000C) >> 2;
	DJI_Remote.Switch_Right = ((pData[5] >> 4) & 0x0003);

	DJI_Remote.Chz_Left = ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;

	DJI_Remote.Updata = 1;
}
/**
  * @概述	DJI遥控器解析函数，含键盘数据解析。
  * @参数1	解析数据包头指针
  * @参数2  数据包长度
  * @返回值 void
  */
void IFR_DJI_RemoteAndKeyboard_Analysis(uint8_t *pData,uint8_t len)
{
	if(pData == 0)
	{
		return;
	}

	DJI_Remote.Chx_Right = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	DJI_Remote.Chy_Right = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	DJI_Remote.Chx_Left = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	DJI_Remote.Chy_Left = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	DJI_Remote.Switch_Left = ((pData[5] >> 4) & 0x000C) >> 2;
	DJI_Remote.Switch_Right = ((pData[5] >> 4) & 0x0003);

	DJI_Remote.Chz_Left = ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;

	DJI_Remote.Mouse.X = pData[6] | (pData[7] << 8);   //!< Mouse X axis
	DJI_Remote.Mouse.Y = pData[8] | (pData[9] << 8);   //!< Mouse Y axis
	DJI_Remote.Mouse.Z = pData[10] | (pData[11] << 8); //!< Mouse Z axis
	DJI_Remote.Mouse.Press_L = pData[12]; //!< Mouse Left  Is Press ?
	DJI_Remote.Mouse.Press_R = pData[13]; //!< Mouse Right Is Press ?

	DJI_Remote.Key.V = (pData[14]|pData[15] << 8);

	DJIRemoteKey_Analysis();

	DJI_Remote.Updata = 1;
}
/**
  * @概述	DJI遥控器键盘按键状态遍历，需放在定时器里循环遍历。
  * @返回值 void
  */
void IFR_DJIRemoteButton_Statu_Traversal(void)
{
		for(int keyid=0;keyid<KEY_NUMS;keyid++)
		{
			DJIRemoteButton_Statu_Get(&Keyboard_Data[keyid]);
		}
}
/**
  * @概述	DJI遥控器键盘按键处理函数。
  */
static void DJIRemoteButton_Statu_Get(KEYBOARD_TypeDef * Key)
{
	if(Key->Last==1)
	{
		Key->Count++;
	}
	else
	{
		Key->Count=0;
	}

	if(Key->Count>10)
	{
		if(Key->Count<100)	//100ms
		{
			if(Key->Last==1&&Key->Value==1)
			{
				Key->Statu=1;
				if(Key->Count == 15) Key->Short_Press = 1;
			}
			else
			{
				Key->Statu=0;
			}
		}
		else
		{
			if(Key->Last==1&&Key->Value==1)
			{
				Key->Statu=2;
			}
			else
			{
				Key->Statu=0;
			}
		}
	}

	Key->Last=Key->Value;
}
/**
  * @概述	DJI遥控器键鼠数据解析函数。
  */
static void DJIRemoteKey_Analysis(void)
{
	Keyboard_Data[KEY_W].Value= DJI_Remote.Key.V&0x01;
	Keyboard_Data[KEY_S].Value=(DJI_Remote.Key.V&0x02)>>1;
	Keyboard_Data[KEY_A].Value=(DJI_Remote.Key.V&0x04)>>2;
	Keyboard_Data[KEY_D].Value=(DJI_Remote.Key.V&0x08)>>3;

	Keyboard_Data[KEY_SHIFT].Value=(DJI_Remote.Key.V&0x10)>>4;
	Keyboard_Data[KEY_CTRL].Value=(DJI_Remote.Key.V&0x20)>>5;
	Keyboard_Data[KEY_Q].Value=(DJI_Remote.Key.V&0x40)>>6;
	Keyboard_Data[KEY_E].Value=(DJI_Remote.Key.V&0x80)>>7;

	Keyboard_Data[KEY_R].Value=(DJI_Remote.Key.V&0x0100)>>8;
	Keyboard_Data[KEY_F].Value=(DJI_Remote.Key.V&0x0200)>>9;
	Keyboard_Data[KEY_G].Value=(DJI_Remote.Key.V&0x0400)>>10;
	Keyboard_Data[KEY_Z].Value=(DJI_Remote.Key.V&0x0800)>>11;
	Keyboard_Data[KEY_X].Value=(DJI_Remote.Key.V&0x1000)>>12;
	Keyboard_Data[KEY_C].Value=(DJI_Remote.Key.V&0x2000)>>13;
	Keyboard_Data[KEY_V].Value=(DJI_Remote.Key.V&0x4000)>>14;
	Keyboard_Data[KEY_B].Value=(DJI_Remote.Key.V&0x8000)>>15;
}

/**
  * @概述	DJI键鼠数据解析函数。
  */
static void DJIKeyData_Analysis(KEYBOARD_TypeDef *Keyboard_Data, uint16_t KeyData)
{
	Keyboard_Data[KEY_W].Value= KeyData&0x01;
	Keyboard_Data[KEY_S].Value=(KeyData&0x02)>>1;
	Keyboard_Data[KEY_A].Value=(KeyData&0x04)>>2;
	Keyboard_Data[KEY_D].Value=(KeyData&0x08)>>3;

	Keyboard_Data[KEY_SHIFT].Value=(KeyData&0x10)>>4;
	Keyboard_Data[KEY_CTRL].Value=(KeyData&0x20)>>5;
	Keyboard_Data[KEY_Q].Value=(KeyData&0x40)>>6;
	Keyboard_Data[KEY_E].Value=(KeyData&0x80)>>7;

	Keyboard_Data[KEY_R].Value=(KeyData&0x0100)>>8;
	Keyboard_Data[KEY_F].Value=(KeyData&0x0200)>>9;
	Keyboard_Data[KEY_G].Value=(KeyData&0x0400)>>10;
	Keyboard_Data[KEY_Z].Value=(KeyData&0x0800)>>11;
	Keyboard_Data[KEY_X].Value=(KeyData&0x1000)>>12;
	Keyboard_Data[KEY_C].Value=(KeyData&0x2000)>>13;
	Keyboard_Data[KEY_V].Value=(KeyData&0x4000)>>14;
	Keyboard_Data[KEY_B].Value=(KeyData&0x8000)>>15;
}
/**
  * @概述	DJI图传客户端按键数据解析函数。
  * @参数1	解析数据包头指针
  * @参数2  数据包长度
  * @返回值 void
  */
void CilentData_Analysis(uint8_t *pData, uint8_t len)
{
	uint16_t data_length 			= 0;
	uint16_t CRC16 						= 0;
	uint16_t Cmd_ID 					= 0;

	if (pData == NULL || pData[0] != 0xA5)
	{
		return;
	}

	if(IFR_Get_CRC8_Check(pData,4,0xFF)== pData[4])
	{
		data_length = (uint16_t)((pData[2] <<8)| pData[1]);
		Cmd_ID = (uint16_t)((pData[6]<<8) | pData[5]);
		if (Cmd_ID != 0x0304) return;
		CRC16 = (uint16_t)((((uint16_t)(pData[5+2+data_length+1]))<<8) | pData[5+2+data_length]);
		if(IFR_Get_CRC16_Check(pData,+5+2+data_length,0xFFFF) == CRC16)
		{
			CilentRobotCommand.cilent_robot_command_realdata = *((cilent_robot_command_realdata_typedef*)(pData+7));
			DJIKeyData_Analysis(CilentRobotCommand.Keyboard_Data_UI, CilentRobotCommand.cilent_robot_command_realdata.keyboard_value);

			CilentRobotCommand.UpData_Flag = 1;
		}
		else CilentRobotCommand.UpData_Flag |= 2;
	}
	else CilentRobotCommand.UpData_Flag |= 4;

}

/**
  * @概述	DJI客户端图传链路键盘按键状态遍历，需放在定时器里循环遍历。
  * @返回值 void
  */
void IFR_DJICilentButton_Statu_Traversal(void)
{
		for(uint8_t keyid=0;keyid<KEY_NUMS;keyid++)
		{
			DJIRemoteButton_Statu_Get(&CilentRobotCommand.Keyboard_Data_UI[keyid]);
		}
}

void IFR_ET10_Remote_Analysis(uint8_t *pData,uint8_t len)
{
	if(pData == 0)
	{
		return;
	}
	if(pData[0] != 0x0f)
	{
		return;
	}
	pData++;
	ET10_Remote.Chx_Right = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	ET10_Remote.Chy_Left = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	ET10_Remote.Chy_Right = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	ET10_Remote.Chx_Left = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	ET10_Remote.Switch_B = (((int16_t)pData[5] >> 4) | ((int16_t)pData[6]<<4)) & 0x07FF;
	ET10_Remote.V1			= (((int16_t)pData[6] >> 7) | ((int16_t)pData[7]<<1) | ((int16_t)pData[8] << 9)) & 0x07FF;
	
	ET10_Remote.Switch_E = (((int16_t)pData[8] >> 2) | ((int16_t)pData[9]<<6)) & 0x07FF;
	ET10_Remote.Switch_C = (((int16_t)pData[9] >> 5) | ((int16_t)pData[10]<<3)) & 0x07FF;
	
	ET10_Remote.Switch_D = ((int16_t)pData[11] | ((int16_t)pData[12] << 8)) & 0x07FF;
	ET10_Remote.Switch_A = (((int16_t)pData[12] >> 3) | ((int16_t)pData[13] << 5)) & 0x07FF;
	ET10_Remote.bu = (((int16_t)pData[22]) | ((int16_t)pData[23]<<8)) & 0x07FF;

	
}
