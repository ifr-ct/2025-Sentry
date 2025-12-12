#include "forecast.h"
#define Last_time_ms 31
#define G 9.80665f

extern shoot_data_t my_shoot_data;
extern TF_Tree tf_tree;
extern IMU_DM_TypeDef IMU_DM_Info;
float drop_distance = 0;

double shoot_speed()
{
	//if (my_shoot_data.initial_speed>10 && !isnan(my_shoot_data.initial_speed)) return my_shoot_data.initial_speed;
	return 23.5;
}

/// @brief 计算子弹飞行时间
/// @param target_rpd 目标位置(弧度, 米) (yaw,pitch,dist)
/// @return 飞行时间(秒)
double Time_calculation(const TF_Vec3& target_rpd, double dist)
{
	const double X = cos(target_rpd.y) * dist;
	const double Y = sin(target_rpd.y) * dist;
	const double v0 = shoot_speed(); //m/s
	const double g_v02 = G / (v0 * v0);	
  const double a = (target_rpd.y - std::asin((g_v02 * dist * (1 - pow(sin(target_rpd.y), 2))) - sin(target_rpd.y))) / 2.0f;
  const double flight_time = X / (cos(a) * v0);
	return flight_time;
}


double fuck_p,fuck_d,fuck_a;
/// @biref 计算枪口pitch值
/// @param pitch 目标相对pitch弧度
/// @param dist 目标位置(米)
/// @return 枪口pitch值(绝对弧度)
double Motor_calcPitch(double pitch, double dist)  
{/*
	const double Y = -sin(pitch) * dist;
  const double v0 = shoot_speed();
	const double g_v02 = G / (v0 * v0);
	const double a = (pitch - std::asin((g_v02 * dist * (1 - pow(sin(pitch), 2))) - sin(pitch))) / 2.0f;
	*/
	const double theta = -pitch;
	const double v  = shoot_speed();
	const double sin0 = std::sin(theta);
	const double a = (-std::asin((G*dist*(1-(sin0*sin0)))/(v*v)-sin0)+theta)/2;
	
	
	fuck_p=pitch;fuck_d=dist;fuck_a=a;
  return -a;
}

//double Motor_calcPitch(double pitch, double dist) {
//    const double sin0 = std::sin(pitch);
//    const double v0 = shoot_speed();
//    const double g_v02 = G / std::pow(v0, 2);
//    const double a = (std::asin(sin0) - std::asin((g_v02 * dist * (1 - std::pow(sin0, 2))) - sin0)) / 2;
//    return a;
//}
float pitch_err;
TF_Vec3 Gun_target_r(0,0,0);
float err_yaw = 1.48f;
float err_pitch = 5.422f;
float fly_t;
float dd = 1.03f;
/// @brief 极坐标目标预测
/// @param hardware_delay 内部延时(秒) (从相机触发到下位机接收 + 电机响应时间(=0))
/// @param target_rpd 触发时相机位置(弧度, 米) (yaw,pitch,dist) 
/// @param velocity 目标运动速度(弧度) (vyaw,vpitch,0)
/// @param [out]yaw_before yaw相对误差 (弧度)
/// @param [out]pitch_before pitch相对误差 (弧度)
/// @param IMU_dm_info 最新IMU信息
void err_calculate(float distant, float *yaw_err, float *pitch_err)
{
	*yaw_err = 2.218f * pow(distant, -0.855f) - 0.1f;//1.05
	//*pitch_err = 4.898f + 2.590 * sqrt(distant);步兵
	//*pitch_err = -0.00299f * distant * distant + 0.01138f * distant + 5.43618f;
}

double Motor_foreast(double hardware_delay, const TF_Vec3& target_rpd, const TF_Vec3& velocity, float *yaw_before, float *pitch_before)
{
	Gun_target_r = target_rpd;//Camera_to_Gun(target_rpd);//枪管坐标系下极坐标
	double all_delay = hardware_delay + Time_calculation(Gun_target_r + velocity * hardware_delay, Gun_target_r.z);
	fly_t = Time_calculation(Gun_target_r + velocity * hardware_delay, Gun_target_r.z);
	err_calculate(Gun_target_r.z, &err_yaw, &err_pitch);
	if(target_rpd.z <= 1.0f)
	{
		all_delay = 0;//all_delay * (target_rpd.z / 4.f);
	}
	else if(target_rpd.z <= 1.8f && target_rpd.z > 1.0f)
	{
		all_delay = all_delay * ((target_rpd.z / 3.f) + 0.25f);
	}
	*yaw_before = (Gun_target_r.x + velocity.x * all_delay * dd);// * (180.f/3.14159265f) - err_yaw;
	pitch_err = Motor_calcPitch(Gun_target_r.y + velocity.y * all_delay * dd, Gun_target_r.z);
  *pitch_before = Motor_calcPitch(Gun_target_r.y + velocity.y * all_delay * dd, Gun_target_r.z );// + err_pitch;// 57.29f + 5.38f;
	return all_delay;
}

//double Motor_foreast(double hardware_delay, const TF_Vec3& target_rpd, const TF_Vec3& velocity, float *yaw_before, float *pitch_before)
//{
//	Gun_target_r = target_rpd;//Camera_to_Gun(target_rpd);//枪管坐标系下极坐标
//	double all_delay = hardware_delay + Time_calculation(Gun_target_r + velocity * hardware_delay, Gun_target_r.z);
//	fly_t = Time_calculation(Gun_target_r + velocity * hardware_delay, Gun_target_r.z);
//	err_calculate(Gun_target_r.z, &err_yaw, &err_pitch);
//	if(target_rpd.z <= 1.0f)
//	{
//		all_delay = 0;//all_delay * (target_rpd.z / 4.f);
//	}
////	else if(target_rpd.z <= 1.8f && target_rpd.z > 1.0f)
////	{
////		all_delay = all_delay * ((target_rpd.z / 3.f) + 0.25f);
////	}
//	*yaw_before = (Gun_target_r.x + velocity.x * all_delay * dd) * (180.f/3.14159265f) - err_yaw;
//	pitch_err = Motor_calcPitch(Gun_target_r.y + velocity.y * all_delay * dd, Gun_target_r.z);
//  *pitch_before = Motor_calcPitch(Gun_target_r.y + velocity.y * all_delay * dd, Gun_target_r.z ) + err_pitch;// 57.29f + 5.38f;
//	return all_delay;
//}

TF_Vec3 Camera_to_Gun(const TF_Vec3& target_rpd)//转枪管坐标系
{
 	double sp = sin(target_rpd.y);
 	double cp = cos(target_rpd.y);
 	double sy = sin(target_rpd.x);
 	double cy = cos(target_rpd.x);
	float gun_x,gun_y,gun_z;
	gun_x = cy * cp * target_rpd.z - 0.05f;
	gun_y = sy * cp * target_rpd.z - 0.0550713f;
	gun_z = sp * target_rpd.z + 0.072f;
	TF_Vec3 gun_target_p = TF_Vec3(gun_x, gun_y, gun_z);//枪管坐标系下位置
	float dist_g = sqrt(gun_target_p.x * gun_target_p.x + gun_target_p.y * gun_target_p.y + gun_target_p.z * gun_target_p.z);
	float gun_r_yaw = atan2f(gun_target_p.y, gun_target_p.x);
	float gun_r_pitch = atan2f(gun_target_p.z, sqrt(gun_target_p.y * gun_target_p.y + gun_target_p.x * gun_target_p.x));
	TF_Vec3 gun_target_r = TF_Vec3(gun_r_yaw, gun_r_pitch, dist_g);
	return gun_target_r;
}
/*
@概述：进行预测
@参数1：硬件延时时间(相机外触发处理图像时间，相机通信上位机时间，上位机下位机通信时间)(ms)
@参数2：目标绝对位置
@参数3：目标极坐标
@参数4：预测前的水平目标值
@参数5：预测前的垂直目标值
@返回值；无
*/
// void backup_Motor_target(uint16_t hardware_delay, TF_Vec3 *position, TF_Vec3 *velocity, float *yaw_before, float *pitch_before, IMU_DM_TypeDef IMU_dm_info)
// {
// 	uint16_t all_delay = 0;
// 	float extra_pitch = 0;
// 	float cp,sp,cy,sy = 0;
// 	TF_Vec3 final_position(0, 0 , 0);
// 	TF_Vec3 TF_GUN(0, 0, 0);
// 	all_delay = Time_calculation(hardware_delay, position, velocity);
// 	Eye_of_Providence(yaw_before, pitch_before, velocity, all_delay);
// 	cp = cos(*pitch_before);
// 	sp = sin(*pitch_before);
// 	cy = cos(*yaw_before);
// 	sy = sin(*yaw_before);
// 	final_position = TF_Vec3(velocity ->z*cp*cy, velocity ->z*cp*sy, velocity ->z*sp);//目标最终位置右手系
// 	extra_pitch = Ballistic_calculation(&final_position);
// 	tf_tree.updateIMU(IMU_dm_info.Angle.yaw, IMU_dm_info.Angle.roll, IMU_dm_info.Angle.pitch);//更新imu
// 	TF_GUN = tf_tree.transformPoint(final_position, TF_Tree::TF_BASE_GUN);//将相机坐标系下目标转到枪管
// 	*yaw_before = atan2f(TF_GUN.y, TF_GUN.x) + *yaw_before;
// 	*pitch_before = atan2f(TF_GUN.z, TF_GUN.x) + *pitch_before + extra_pitch;
// }

/*
@概述：忽略电机响应到打出弹丸时间，忽略空气阻力，计算获取图像到打到目标位置时间
@参数1：相机开始采集到接到小电脑消息时间
@参数2：目标位置信息
@参数3：目标速度信息
@返回值：近似为相机获取到图像到打到目标位置的时间
*/
// uint16_t backup_Time_calculation(uint16_t hardware_time, TF_Vec3 *position, TF_Vec3 *velocity)
// {
// 	uint16_t flight_time;
// 	uint16_t all_time;
// 	const double X2 = pow(position ->x + velocity ->x * hardware_time, 2) + pow(position ->y + velocity -> y * hardware_time, 2);
//   const double Y = -position ->z + velocity->z * hardware_time;
//   const double l = sqrt(X2 + pow(Y, 2));
//   const double sin0 = Y / l;
//   const double a = (asin(sin0) - asin((G / (my_shoot_data.initial_speed * my_shoot_data.initial_speed) * l * (1 - pow(sin0, 2))) - sin0)) / 2.0f;
//   const double X = sqrt(X2);
//   flight_time = X / (cos(a) * my_shoot_data.initial_speed);
// 	all_time = flight_time + hardware_time;	
// 	return all_time;
// }

/*
@概述：忽略空气阻力，计算目标的最终位置
@参数1：目标在车体坐标下水平角度
@参数2：目标在车体坐标下垂直角度
@参数3：目标极坐标
@参数4：相机开始采集到弹丸打到预测位置的时间
@返回值：目标的预测位置
*/
// void backup_Eye_of_Providence(float *yaw_before, float *pitch_before, TF_Vec3 *velocity, uint16_t all_delay)
// {
// 	*yaw_before = *yaw_before + velocity ->x * all_delay;//计算水平最终位置
// 	*pitch_before = *pitch_before + velocity ->y * all_delay;//计算垂直最终位置
// }

/*
@概述：弹道解算简化为抛物线
@参数1：目标的预测位置
@返回值：返回枪口上抬角度
*/
// float backup_Ballistic_calculation(TF_Vec3 *final_position)
// {
//   const float X2 = powf(final_position ->x, 2) + powf(final_position ->y, 2);
//   const float Y = -final_position ->z;
//   const float l = sqrtf(X2 + powf(Y, 2));
//   const float sin0 = Y / l;
//   const float a = (asinf(sin0) - asinf(G / (my_shoot_data.initial_speed * my_shoot_data.initial_speed) * l * (1 - powf(sin0, 2))) / 2.0f);
// 	return a;
// }

void AntiGyro::update(float now_yaw, float now_pitch, float now_distance, float now_time, float *vyaw, float *vpitch) {

    lst_r_i = (now_i - LST_R_CNT + SEQ_SIZE) % SEQ_SIZE;
    lst_v_i = (now_i - LST_V_CNT + SEQ_SIZE) % SEQ_SIZE;

    yaw_seq[now_i] = now_yaw;
    pitch_seq[now_i] = now_pitch;
    time_seq[now_i] = now_time;

    const float timediff = time_seq[now_i] - time_seq[lst_r_i];
    {
        const float yawdiff = yaw_seq[now_i] - yaw_seq[lst_r_i];
        const float pitchdiff = pitch_seq[now_i] - pitch_seq[lst_r_i];
				*vyaw = yawdiff / timediff;
        now_vyaw = *vyaw;
        *vpitch = pitchdiff / timediff;
    }

    if (isJump()) {
        ++jump_cnt;
        // 速度修复
        *vyaw = last_good_vyaw;
        // 位置更新
        const float old_yaw = yaw_seq[lst_r_i];
        yaw_seq[lst_r_i] = now_yaw - timediff * last_good_vyaw;
        const float yawseqdiff = yaw_seq[lst_r_i] - old_yaw;
        for (uint8_t i = 1; i < LST_R_CNT; ++i) {
            const uint8_t idx = (now_i - i + SEQ_SIZE) % SEQ_SIZE;
            yaw_seq[idx] += yawseqdiff;
        }

        vyaw_cnt = 0;
        vyaw_all = 0;
        lst_min_yaw = min_yaw;
        lst_max_yaw = max_yaw;
        min_yaw = std::numeric_limits<float>::max();
        max_yaw = -std::numeric_limits<float>::max();
    } else {
        if (now_yaw < min_yaw)
            min_yaw = now_yaw;
        if (now_yaw > max_yaw)
            max_yaw = now_yaw;
        big_max_yaw = std::max(min_yaw + (lst_max_yaw - lst_min_yaw), max_yaw);
        big_min_yaw = std::min(max_yaw - (lst_max_yaw - lst_min_yaw), min_yaw);
        last_good_vyaw = now_vyaw;
        vyaw_all += now_vyaw;
        ++vyaw_cnt;
    }

    now_i = (now_i + 1) % SEQ_SIZE;
}

void AntiGyro::predLimit(float *yaw, float *pitch) {

    if (*yaw > (big_max_yaw - ((big_max_yaw - big_min_yaw) * 0.1f)))
        *yaw = min_yaw + (*yaw - (big_max_yaw - ((big_max_yaw - big_min_yaw) * 0.1f)));
    else if (*yaw < (big_min_yaw +((big_max_yaw - big_min_yaw) * 0.1f)))
        *yaw = max_yaw + (*yaw - (big_min_yaw + ((big_max_yaw - big_min_yaw) * 0.1f)));
    else
        return;// 没有超出一个身位, 不对pitch做修正

    // TODO pitch
}

bool AntiGyro::isJump() const {
    if (!checkJump)
        return true;//
    if (vyaw_cnt < 5)
        return false;
    if (std::abs(now_vyaw) < 0.00261799f) return false;
    if ((std::abs(now_vyaw - last_good_vyaw) > 10.f) && ((now_vyaw < 0 && last_good_vyaw > 0) || (now_vyaw > 0 && last_good_vyaw < 0)))//0.5045329251972f
			return true;//0.1745329251972f
    //const float vyaw_avg = float(vyaw_all / vyaw_cnt);
    // if (yaw_seq[now_i] > (big_max_yaw * THRESHOLD_MAX)) {
    //     return true;
    // }
    // if (yaw_seq[now_i] < (big_min_yaw * THRESHOLD_MIN)) {
    //     return true;
    // }

    return false;
}
