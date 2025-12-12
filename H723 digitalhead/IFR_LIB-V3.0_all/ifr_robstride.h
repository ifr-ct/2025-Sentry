#include "ifr_pid.h"
#include "ifr_motor.h"
#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif
#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -33.0f 
#define V_MAX 33.0f 
#define KP_MIN 0.0f 
#define KP_MAX 500.0f 
#define KD_MIN 0.0f 
#define KD_MAX 5.0f 
#define T_MIN -14.0f 
#define T_MAX 14.0f 


// 各种控制模式
#define move_control_mode 0	 // 运控模式
#define Pos_control_mode 1	 // 位置模式
#define Speed_control_mode 2 // 速度模式
#define Elect_control_mode 3 // 电流模式
#define Set_Zero_mode 4		 // 零点模式

// 通信协议类型说明
#define Communication_Type_Get_ID 0x00			   // 获取设备ID (通信类型0)获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01	   // 运控模式电机控制指令 （通信类型1）用来向电机发送控制指令
#define Communication_Type_MotorRequest 0x02	   // 电机反馈数据 （通信类型2） 用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03		   // 电机使能运行 （通信类型3）
#define Communication_Type_MotorStop 0x04		   // 电机停止运行 （通信类型4）
#define Communication_Type_SetPosZero 0x06		   // 设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
#define Communication_Type_Can_ID 0x07			   // 设置电机CAN_ID（通信类型7）更改当前电机CAN_ID , 立即生效。
#define Communication_Type_Control_Mode 0x12	   // 设置电机模式
#define Communication_Type_GetSingleParameter 0x11 // 单个参数读取（通信类型17）
#define Communication_Type_SetSingleParameter 0x12 // 单个参数写入（通信类型18） （掉电丢失）
#define Communication_Type_ErrorFeedback 0x15	   // 故障反馈帧（通信类型21）


#define Set_Parameter 'P'
#define Set_Value 'V'
typedef struct
{
	uint8_t Yaw_date[8];
}YAW;
extern YAW big_yaw;
// 这是通信17所读取的index参数
typedef struct
{
	float Angle;
	float Abs_Angle;
	float Speed;
	float Torque; // 扭矩
	float Temp;
	int Error_Code;
	int Pattern; // 电机模式（0复位1标定2运行）
} Motor_Pos_LZ_Info;

// 通讯类型17所读取到的信息
typedef struct
{
	uint8_t run_mode;
	float iq_ref;		// 电流模式的iq指令
	float spd_ref;		// 转速模式转速指令
	float limit_torque; // 扭矩限制
	float cur_kp;		// 电流的kp
	float cur_ki;		// 电流的ki
	float curfilt_gain; // 电流滤波系数
	float loc_ref;		// 位置模式角度指令
	float limit_spd;	// 位置模式速度限制
	float limit_cur;	// 速度位置模式电流限制
	float mechPos;		// 负载端计圈机械角度
	float iqf;			// iq滤波值
	float mechVel;		// 负载端转速
	float VBUS;			// 母线电压
	int16_t rotation;	// 圈数
	float loc_kp;		// 位置的kp
	float spd_kp;		// 速度的kp
	float spd_ki;		// 速度的ki
} Read_LZ_Message_From_17;

// 设定值
typedef struct
{
	int set_motor_mode;
	float set_current;
	float set_speed;
	float set_Torque;
	float set_angle;
	float set_limit_cur;
	float set_Kp;
	float set_Ki;
	float set_Kd;
} Motor_Set;

// 灵足电机
class IFR_LZ_Motor
{
public:
	float output;
	int Can_Motor;
	IFR_LZ_Motor(uint8_t CAN_Id);
	IFR_LZ_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id);
	void Get_LZ_Motor_ID(void);
	float Get_LZ_Motor_Output(void);

	Motor_Pos_LZ_Info LZ_Pos_Info;				// 用来查电机信息(通信类型2)
	Read_LZ_Message_From_17 LZ_Message_From_17; // 用来查电机信息(通信类型17)

	void LZ_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd); // 运控模式
	void LZ_Motor_Elect_control(float Elect);												// 电流模式
	void LZ_Motor_Speed_control(float limit_cur, float Tar_Speed);							// 速度模式
	void LZ_Motor_Torque_control(float Torque);
	void LZ_Motor_Zero_Pos_control();														// 零点模式
	void Enable_Motor();																	// 使能模式（通信类型3）
	void Disable_Motor(uint8_t clear_error);												// 失能模式（通信类型4）
	void Set_ZeroPos();																		// CyberGear电机设置电机机械零位 （通信类型6）
	void Set_CAN_ID(uint8_t Set_CAN_ID);													// CyberGear电机设置CAN_ID （通信类型7）
	void Set_LZ_Motor_parameter(uint16_t Index, float Value, char Value_mode);				// CyberGear电机写入参数 （通信类型18）
	void Get_LZ_Motor_parameter(uint16_t Index);
	void LZ_Motor_Set_Zero_control();
	void LZ_Motor_Analysis(uint8_t *LZ_Data, uint32_t ID_ExtId);
	Motor_Set Motor_Set;

protected:
	uint8_t CAN_ID;
	uint8_t CAN_MASTER_ID;

	float (*Motor_Offset_MotoFunc)(float Motor_Tar);
};

class IFR_Speed_LZ_Motor : public IFR_LZ_Motor
{
public:
	IFR_Speed_LZ_Motor(uint8_t CAN_Id);
	void Motor_Speed_Set(float Speed_Tar);
	IFR_PID Speed_PID;

private:
	int16_t Tar_Speed;
};

class IFR_Pos_LZ_Motor : public IFR_LZ_Motor
{
public:
	IFR_Pos_LZ_Motor(uint8_t CAN_Id);
	IFR_Pos_LZ_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t Num);
	void Motor_AbsPos_Set(float Pos_Tar);
	void Motor_Pos_Set(float Pos_Tar);
	int32_t Motor_TarPos_Get(void) { return Tar_Pos; }
	int32_t Get_Tar_Pos(void) { return Tar_Pos; }

	void Motor_Speed_Set(float Speed_Tar);

	IFR_PID Pos_PID;
	IFR_PID Speed_PID;

private:
	int32_t Tar_Pos;
	//float (*Motor_Offset_MotoFunc)(float Motor_Tar);
};

class IFR_GyroControl_LZ_Motor : public IFR_LZ_Motor
{
public:
	IFR_GyroControl_LZ_Motor(uint8_t CAN_Id);
	IFR_GyroControl_LZ_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id);

	void IMU_Data_Get(float IMU_Speed, float IMU_Angle);
	void Motor_Angle_Set(float Angle_Tar);
	void Fliter_Motor_Angle_Set(float Angle_Tar);
	int32_t Motor_Angle_Get(void) { return Tar_Angle; }
	int32_t Get_Tar_Angle(void) { return Tar_Angle; }

	float IMU_Speed;
	float IMU_Angle;

	float Angle_Kd;

	IFR_PID Angle_PID;
	IFR_PID Speed_PID;

private:
	int32_t Tar_Angle;
	//float (*Motor_Offset_MotoFunc)(float Motor_Tar);
};
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
float Byte_to_float(uint8_t *bytedata);
