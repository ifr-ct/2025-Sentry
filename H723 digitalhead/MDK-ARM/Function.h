#include "fdcan.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bmi088.h"
#include "BMI088reg.h"
#include "string.h"

#include "sense_analysis.h"
#include "judge.h"
#include "IMU_DM.h"
#include "ifr_lib.h"
//#include "forecast.h"
typedef struct
{
	float Motor_yaw;
	float Motor_little_yaw;
	float Motor_pitch;
	float Motor_zhao;
	float Motor_xing;
	float Motor_jie;
	float Motor_gongdan_ang;
	float Motor_gongdan_speed;

	float tar_x;
	float tar_y;
	int16_t tar_w;
}Robot;

typedef struct
{
	uint16_t Right_x;//11
	uint16_t Right_y;//11
	uint16_t Left_x;//11
	uint16_t Left_y;//11
	uint8_t S1;//2
	uint8_t S2;//2
	uint16_t W;
	//uint8_t IMU_gyro_z;//角速度计
	//uint8_t IMU_quateternion_yaw;//角加速度计
}Tx_data;

typedef __packed struct
{
	frame_header Frame_Header;//根据感知解包协议的自定义帧头
	uint16_t CmdId;//裁判系统发区分每一个包	uint32_t sentry_cmd;
	uint16_t Sub_content;
	uint16_t Send_id;
	uint16_t receive_id;
	uint32_t sentry_cmd;
	uint16_t crc16;
}Sentry_tx_cmd; 

typedef __packed struct
{
	uint16_t Addr;//2

	__packed struct
	{
		uint8_t left_W:1;
		uint8_t left_A:1;
		uint8_t left_S:1;
		uint8_t left_D:1;
		
		uint8_t right_A:1;
		uint8_t right_S:1;
		uint8_t right_D:1;
		uint8_t right_W:1;
	}Key;
	
	__packed struct
	{
		uint8_t Slider_0:2;
		uint8_t Slider_1:2;
		uint8_t Slider_2:2;
		uint8_t Slider_3:2;//1
	}Slider;//从左到右
	
	uint32_t Key_top_1:1;
	uint32_t Key_top_2:1;
	uint32_t Key_top_3:1;
	uint32_t Key_top_4:1;
	uint32_t Chx_Left:11;
	uint32_t Chy_Left:11;
	uint32_t Chx_Right:11;
	uint32_t Chy_Right:11;
	
	__packed struct
	{
		uint8_t Nex_Fre_chx:7;
		uint8_t callback_request:1;
		uint8_t packed_serial_number;
	}Remote_Status;
uint8_t Updata;
}Remote_RxDataTypedef;


float killG(float Motor_tar);
float turn(float now_turn);
void ALL_init();
void Heat_control();
void Target_yuntai_init();//云台目标值初始化
float Judge_heat_control();//热量限制

