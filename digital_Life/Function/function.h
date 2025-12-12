#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bmi088.h"
#include "BMI088reg.h"
#include "string.h"
#include "ifr_lib.hpp"
#define LF 0
#define RF 1
#define RB 2
#define LB 3
#define SPEED 0
#define DIRECTION 1
typedef struct
{
	float Motor[4][2];    
	float Motor_dir[4][2];
	float tar_x;
	float tar_y;
	float tar_w;
}Robot;//赋值用结构体

typedef struct
{
  float L;//车长
  float D;//车宽
  float Angle;//车体运动方向与3点钟方向的夹角，逆时针增（0-6.28）
  float V;//车体运动速度
  float R;//车体中心与圆周运动圆心距离
  float R1;//左前轮离圆周运动圆心距离
  float R2;//右前轮离圆周运动圆心距离
  float R3;//右后轮离圆周运动圆心距离
  float R4;//左后轮离圆周运动圆心距离
  float V1;//左前轮速度
  float V2;//右前轮速度
  float V3;//右后轮速度
  float V4;//左后轮速度
}Vehicle;//解算用结构体

typedef struct
{
	uint8_t Modle_s1;//S1,S2,2+2
	uint8_t Modle_s2;
	uint16_t Left_x;
	uint16_t Left_y;
	uint16_t Right_x;
	uint16_t Right_y;
	uint16_t Buffer_energy;
	int16_t W;
	uint8_t home;
	uint8_t old;
}Rx_can_data;

typedef struct
{
  float Speed;
  float Angle;
  float Temperature;
  float Electric;
	float Torque;
}Yaw;

typedef __packed struct
{
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 

}Judge_DataTypedef;

//extern Rx_can_data rx_can_data;
void ALL_init();
