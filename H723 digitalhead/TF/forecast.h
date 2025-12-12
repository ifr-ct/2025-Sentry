#ifndef FORECAST_H
#define FORECAST_H

#include "stdint.h"
#include "judge.h"
#include "tf.h"
#include "IMU_DM.h"
#include <limits>

//void Motor_target(uint16_t hardware_delay, TF_Vec3 *position, TF_Vec3 *velocity, float *yaw_before, float *pitch_before, IMU_DM_TypeDef IMU_dm_info);
//uint16_t Time_calculation(uint16_t hardware_time, TF_Vec3 *position, TF_Vec3 *velocity);
//void Eye_of_Providence(float *yaw_before, float *pitch_before, TF_Vec3 *velocity, uint16_t all_delay);
//float Ballistic_calculation(TF_Vec3 *final_position);
void err_calculate(float distant, float *yaw_err, float *pitch_err);
double Time_calculation(const TF_Vec3& target_rpd, double dist);
double Motor_calcPitch(double pitch, double dist) ;
double Motor_foreast(double hardware_delay, const TF_Vec3& target_rpd, const TF_Vec3& velocity, float *yaw_before, float *pitch_before);
TF_Vec3 Camera_to_Gun(const TF_Vec3& target_rpd);


#include <algorithm>
#include <cstdint>
#include <cstdlib>
//#include <limits>
const float THRESHOLD_MIN = 0.7f, THRESHOLD_MAX = 1 / THRESHOLD_MIN;
class AntiGyro {
    // 定义序列长度和计算速度用的上一位置(弧度)序列索引
    static const uint8_t SEQ_SIZE = 30;
    static const uint8_t LST_R_CNT = 4, LST_V_CNT = 1;
    // 定义三个序列，分别存储yaw、pitch和time
    float yaw_seq[SEQ_SIZE], pitch_seq[SEQ_SIZE], time_seq[SEQ_SIZE];
    float last_good_vyaw, now_vyaw;
    uint8_t now_i, lst_r_i,
            lst_v_i;// 当前序列索引, 计算速度用的上一位置(弧度)序列索引
    double vyaw_all;
    uint32_t vyaw_cnt;


    uint64_t jump_cnt;
    bool checkJump;

private:
    bool isJump() const;

public:
    float lst_min_yaw, lst_max_yaw;
    float min_yaw, max_yaw;
    float big_max_yaw, big_min_yaw;
	
    AntiGyro()
        : last_good_vyaw(0), now_vyaw(0), now_i(0), vyaw_all(0), vyaw_cnt(0),
          min_yaw(std::numeric_limits<float>::max()),
          max_yaw(-std::numeric_limits<float>::max()), jump_cnt(0), checkJump(true) {
        for (uint8_t i = 0; i < SEQ_SIZE; ++i)
            yaw_seq[i] = pitch_seq[i] = time_seq[i] = 0;
    }

    /// @brief 更新位置并返回当前速度
    /// @param now_yaw 当前偏航角(弧度)
    /// @param now_pitch 当前俯仰角(弧度)
    /// @param now_distance 当前距离(米)
    /// @param now_time 当前时间(秒)
    /// @param vyaw 输出速度(弧度/秒)
    /// @param vpitch 输出速度(弧度/秒)
    void update(float now_yaw, float now_pitch, float now_distance, float now_time, float *vyaw, float *vpitch);
    void predLimit(float *yaw, float *pitch);
};


//void AntiGyro::update(float now_yaw, float now_pitch, float now_distance, float now_time, float *vyaw, float *vpitch);

//void AntiGyro::predLimit(float *yaw, float *pitch);

//bool AntiGyro::isJump();

//class AntiGyro{
//	// 定义序列长度和计算速度用的上一位置(弧度)序列索引
//	static const uint8_t SEQ_SIZE=30;
//	static const uint8_t LST_R_CNT=4,LST_V_CNT=1;
//	// 定义三个序列，分别存储yaw、pitch和time
//	float yaw_seq[SEQ_SIZE], pitch_seq[SEQ_SIZE],time_seq[SEQ_SIZE],vyaw_seq[SEQ_SIZE];
//	uint8_t now_i,lst_r_i,lst_v_i;// 当前序列索引, 计算速度用的上一位置(弧度)序列索引
//  double vyaw_all;
//  uint32_t vyaw_cnt;
//	bool isFirst;

//	uint64_t jump_cnt;
//	bool checkJump,checkLimit;
//	float THRESHOLD_MIN,THRESHOLD_MAX,THRESHOLD_RANGE,THRESHOLD_TIME;
//	public:
//  float lst_min_yaw,lst_max_yaw;
//  float min_yaw,max_yaw;
//	float big_max_yaw,big_min_yaw;			
//		AntiGyro():
//	now_i(0), vyaw_all(0), vyaw_cnt(0), isFirst(true),
//	min_yaw(std::numeric_limits<float>::max()), max_yaw(-std::numeric_limits<float>::max()),
//		jump_cnt(0),checkJump(true),checkLimit(true),
//			THRESHOLD_MIN(0.7f),THRESHOLD_MAX(1/THRESHOLD_MIN),THRESHOLD_RANGE(0.2),THRESHOLD_TIME(0.5){
//		for(uint8_t i=0;i<SEQ_SIZE;++i)yaw_seq[i]=pitch_seq[i]=time_seq[i]=vyaw_seq[0]=0;
//			
//	}
//	
//	void update(float now_yaw,float now_pitch,float now_time,float *vyaw,float *vpitch);
//    void predLimit(float *yaw,float *pitch);  
//  bool isJump() const;
//};
#endif
