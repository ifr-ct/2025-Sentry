/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2024, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_remote.h
  * Version		: v2.3
  * Author		: LiuHao Lijiawei Albert
  * Date		: 2023-09-26
  * Description	:

  *********************************************************************
  */

#ifndef __IFR_REMOTE_H_
#define __IFR_REMOTE_H_

/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ifr_crc.h"
/* USER CODE BEGIN Includes */

extern uint8_t Tx_can_data[8];
/* USER CODE END Includes */

typedef struct
{
	uint16_t Chx_Left;
	uint16_t Chy_Left;
	uint16_t Chx_Right;
	uint16_t Chy_Right;
	uint8_t  Switch_Left;
	uint8_t  Switch_Right;

	uint16_t Chz_Left;
	struct
	{
		int16_t X;
		int16_t Y;
		int16_t Z;
		uint8_t Press_L;
		uint8_t Press_R;
	}Mouse;

 struct
 {
		uint16_t V;
 } Key;

 uint8_t Updata;

}DJI_REMOTE_TypeDef;

typedef struct
{
	uint16_t Count;
	uint8_t  Value;
	uint8_t  Last;
	uint8_t  Statu;
	uint8_t  Short_Press;
}KEYBOARD_TypeDef;

#define RC_SWITCH_UP        1
#define RC_SWITCH_MIDDLE    3
#define RC_SWITCH_DOWN      2

enum KEYBOARDID
{
	KEY_W,
	KEY_S,
	KEY_A,
	KEY_D,
	KEY_SHIFT,
	KEY_CTRL,
	KEY_Q,
	KEY_E,
	KEY_R,
	KEY_F,
	KEY_G,
	KEY_Z,
	KEY_X,
	KEY_C,
	KEY_V,
	KEY_B,
	KEY_NUMS,
};

typedef __packed struct 
{ 
	uint8_t soft_1; 
	uint8_t soft_2; 
	uint64_t Chx_Right:11; 
	uint64_t Chy_Right:11; 
	uint64_t Chy_Left:11; 
	uint64_t Chx_Left:11; 
	uint64_t mode_sw:2; 
	uint64_t go_home:1; 
	uint64_t fn:1; 
	uint64_t button:1; 
	uint64_t wheel:11; 
	uint64_t shutter:1; 
	int16_t mouse_x; 
	int16_t mouse_y; 
	int16_t mouse_z; 
	uint8_t mouse_left:2; 
	uint8_t mouse_right:2; 
	uint8_t mouse_middle:2; 
	uint16_t key; 
	uint16_t crc16; 
	uint8_t Updata;
}remote_control_keyboard_mouset_t; 

extern remote_control_keyboard_mouset_t New_DJI_Remote;
extern remote_control_keyboard_mouset_t New_Keyboard;
/*
字节偏移量 大小
0 					2 				目标机器人 ID当 x,y 超出界限时则不显示。
2 					4 				目标 x 位置坐标，单位 m 当 x,y 超出界限时则不显示。
6 					4					目标 y 位置坐标，单位 m 当 x,y 超出界限时则不显示。
*/

typedef __packed struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_left;
	uint8_t mouse_right;
	uint16_t keyboard_value;
	uint16_t reserved;
}cilent_robot_command_realdata_typedef;

typedef struct
{
	cilent_robot_command_realdata_typedef cilent_robot_command_realdata;
	KEYBOARD_TypeDef Keyboard_Data_UI[KEY_NUMS];
	uint8_t UpData_Flag;
}DJI_CilentRobotCommand_Typedef;

extern DJI_CilentRobotCommand_Typedef CilentRobotCommand;
extern DJI_REMOTE_TypeDef DJI_Remote;
extern KEYBOARD_TypeDef Keyboard_Data[KEY_NUMS];
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void IFR_DJI_Remote_Analysis(uint8_t *pData,uint8_t len);
void IFR_DJI_RemoteAndKeyboard_Analysis(uint8_t *pData,uint8_t len);
void IFR_DJIRemoteButton_Statu_Traversal(void);
void DJIRemoteButton_Statu_Get(KEYBOARD_TypeDef * Key);
void DJIRemoteKey_Analysis(void);
void NewDJIRemote_Analysis(uint8_t *pData, uint8_t len);

void IFR_DJICilentButton_Statu_Traversal(void);
void CilentData_Analysis(uint8_t *pData, uint8_t len);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
 }
#endif

#endif
