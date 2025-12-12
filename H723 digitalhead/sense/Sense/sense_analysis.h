#ifndef __SENSE_ANALYSIS_H_
#define __SENSE_ANALYSIS_H_
#include "Judge.h"
#include "ifr_lib.h"
#include "IMU_DM.h"
#include "stdio.h"
#include "tf.h"
#include "forecast.h"
#include "ifr_lib.h"
typedef __packed struct
{
	uint8_t head;//固定包头0x5A
	uint16_t move_y;//前后y（364-1684），正/负 660 控制前/后速度
	uint16_t move_x;//左右x（364-1684），正/负 660 控制右/左速度
	int16_t camera_yaw;//云台水平方向位置(弧度制，扩大10^4传输)
	int16_t camera_pitch;//云台垂直方向位置(弧度制，，扩大10^4传输)
	uint16_t count_time;//从图像准备完毕到发送前的延时(微秒)
	uint16_t distance;//基于gimbal link的目标距离(扩大10^4传输)
	uint8_t is_scan : 1;//是否进入扫描模式，如果进入扫描模式忽略gun值
	uint8_t gyro : 1;//是否开启陀螺
	uint8_t shoot : 1;//是否允许开启发射
	uint8_t has_target : 1;
	uint8_t is_gyro : 1;
	uint8_t reserve : 3;//保留
	uint8_t crc;//CRC8校验
}Leida;
extern Leida leida;//回传雷达自瞄值

typedef __packed struct 
{
uint8_t head;//帧头0x5C
int16_t yaw;//Yaw轴角度数据（绕z轴转动）*100,-180~180
int16_t pitch;//Pitch轴角度数据（绕x轴转动）*100
int16_t roll;//Roll轴角度数据（绕y轴转动）*100
uint8_t is_red : 1;//是否是红方
uint8_t use_forecast : 1;//是否开启预测
uint8_t imu_type : 2;//陀螺仪坐标系类型
uint8_t aim_type : 1;//自瞄类型
uint8_t need_flash : 1;//是否需要刷新
uint8_t reserve : 2;//保留
int8_t crc;//校验
}IMU_DATA;

	
typedef __packed struct
{
	uint8_t head;   					//帧头0x5C
	int16_t Yaw_info;  				//Yaw轴角度数据（绕z轴转动）*100,-180~180
	int16_t Pitch_info; 			//Pitch轴角度数据（绕x轴转动）* 100
	int16_t Roll_info;  			//Roll轴角度数据（绕y轴转动）* 100
	uint8_t is_red : 1;      	//是否是红方
	uint8_t use_forecast : 1;	//是否开启预测
  uint8_t imu_type : 2; 		//陀螺仪坐标系类型	
	uint8_t reserve:4;				//保留
	int32_t imu_a_yaw;
	int32_t raw_a_yaw;
	int32_t target_a_yaw;
	int32_t imu_av_yaw;
	int32_t target_av_yaw;
	int32_t Motor_little_yaw;
	int32_t imu_a_pitch;
	int32_t raw_a_pitch;
	int32_t target_a_pitch;
	int32_t imu_av_pitch;
	int32_t target_av_pitch;
	int32_t Motor_pitch;
	int32_t big_max_yaw;
	int32_t big_min_yaw;
	int32_t max_yaw;
	int32_t min_yaw;
	int32_t lst_max_yaw;
	int32_t lst_min_yaw;
	int8_t CRC8;    					//校验
}IMU_TxDataTypedef;
extern IMU_TxDataTypedef IFR_Sense_Send_Data;
void leida_handle(uint8_t *pData, uint8_t len);
void Auto_TransmitTask(int32_t little_yaw, int32_t pitch);
void Sense_send_data(IMU_DM_TypeDef IMU_DM_Info,uint8_t is_red ,uint8_t use_forecast, uint8_t imu_type, float pitch_err, int32_t little_yaw, int32_t pitch);//自瞄通讯数据发送
\
#endif
//IFR_Get_CRC8_Check
