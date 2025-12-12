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
#include "string.h"
#include "stdbool.h"
/* Private variables -------------------------------------------------*/
DJI_REMOTE_TypeDef DJI_Remote = {1024,1024,1024,1024,RC_SWITCH_UP,RC_SWITCH_UP,1024};
KEYBOARD_TypeDef Keyboard_Data[KEY_NUMS]={0};
remote_control_keyboard_mouset_t New_DJI_Remote;
remote_control_keyboard_mouset_t New_Keyboard;
DJI_CilentRobotCommand_Typedef CilentRobotCommand = {0};
/**
  * @概述	DJI遥控器解析函数，无键盘数据解析。
  * @参数1	解析数据包头指针
  * @参数2  数据包长度
  * @返回值 void
  */
uint8_t Tx_can_data[8];//遥控器数据数组
void IFR_DJI_Remote_Analysis(uint8_t *pData,uint8_t len)
{
	if(pData == 0)
	{
		return;
	}
    int16_t temp; 
    // 计算 Chx_Right 的值并判断 
    temp = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    if (temp < 364 || temp > 1684) 
		{ 
       return; // 返回 1 表示有值不满足范围 
    } 
    // 计算 Chy_Right 的值并判断 
    temp = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF; 
    if (temp < 364 || temp > 1684) 
		{ 
       return; // 返回 1 表示有值不满足范围 
    } 
    // 计算 Chx_Left 的值并判断 
    temp = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF; 
    if (temp < 364 || temp > 1684) 
		{ 
       return; // 返回 1 表示有值不满足范围 
    } 
    // 计算 Chy_Left 的值并判断 
    temp = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF; 
    if (temp < 364 || temp > 1684) 
		{ 
       return; // 返回 1 表示有值不满足范围 
    } 
		temp = ((pData[5] >> 4) & 0x000C) >> 2;
		if(temp < 1 || temp > 3)
		{
			return;
		}
		temp = ((pData[5] >> 4) & 0x0003);
		if(temp < 1 || temp > 3)
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
	memcpy(Tx_can_data, pData, 8);//复制遥控器数据
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

typedef __packed struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}remote_data_t;

static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);

static uint16_t crc16_init = 0xffff;
static const uint16_t crc16_tab[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief Get the crc16 checksum
 *
 * @param p_msg Data to check
 * @param lenData length
 * @param crc16 Crc16 initialized checksum
 * @return crc16 Crc16 checksum
 */
static uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
    uint8_t data;

    if(p_msg == NULL)
    {
        return 0xffff;
    }

    while(len--)
    {
        data = *p_msg++;
        (crc16) = ((uint16_t)(crc16) >> 8) ^ crc16_tab[((uint16_t)(crc16) ^ (uint16_t)(data)) & 0x00ff];
    }

    return crc16;
}

/**
 * @brief crc16 verify function
 *
 * @param p_msg Data to verify
 * @param len Stream length=data+checksum
 * @return bool Crc16 check result
 */
bool verify_crc16_check_sum(uint8_t *p_msg, uint16_t len)
{
    uint16_t w_expected = 0;

    if((p_msg == NULL) || (len <= 2))
    {
        return false;
    }
    w_expected = get_crc16_check_sum(p_msg, len - 2, crc16_init);

    return ((w_expected & 0xff) == p_msg[len - 2] && ((w_expected >> 8) & 0xff) == p_msg[len - 1]);
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

void NewDJIRemote_Analysis(uint8_t *pData, uint8_t len)
{
    if (pData == NULL) 
		{
        return;
    }
    if (len != sizeof(remote_control_keyboard_mouset_t) - 1) 
		{
        return;
    }
 		if(pData[0] == 0xA9 && pData[1] == 0x53)
		{
			bool is_valid = verify_crc16_check_sum(pData, len);
			if(is_valid == 1)
			{
				
				memcpy(&New_DJI_Remote, pData, len);
				New_DJI_Remote.Updata = 1;
//				New_DJI_Remote.Updata = 0;//保险，不论如何新遥控器一定断连
				memcpy(&New_Keyboard, (uint16_t*)&New_DJI_Remote.key, sizeof(New_DJI_Remote.key));
			}	
		}
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

