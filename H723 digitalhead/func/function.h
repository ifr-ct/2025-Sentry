#include "fdcan.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bmi088.h"
#include "BMI088reg.h"
#include "string.h"
#include "ifr_lib.h"
//#include "ifr_fdcan.h"
typedef struct
{
	float Motor_yaw;
	float Motor_small_yaw;
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


typedef struct
{
	uint8_t head;//固定包头
	uint16_t move_y;//前后y（364-1684），正/负 660 控制前/后速度
	uint16_t move_x;//左右x（364-1684），正/负 660 控制右/左速度
	int16_t gun_yaw;//云台垂直方向误差（弧度制，扩大10^4传输）
	int16_t gun_pitch;//云台水平方向误差（弧度制，扩大10^4传输）
	bool is_scan;//是否进入扫描模式，如果进入扫描模式忽略gun值
	bool gyro;//是否开启陀螺
	bool shoot;//是否允许开启发射
	uint8_t reserve;//保留
	uint8_t crc;//CRC8校验
}Leida;

void ALL_init();
