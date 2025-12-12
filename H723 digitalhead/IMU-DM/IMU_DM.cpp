#include "IMU_DM.h"
IMU_DM_TypeDef IMU_DM_Info;
uint8_t update_imu = 0;
/**
  * @概述	达妙IMU解析函数
  * @参数1	解析数据包头指针
  * @参数2  数据包长度
  * @返回值 void
  */
void IFR_IMU_DM_Analysis(uint8_t *pData,uint8_t len)
{
	uint8_t i = 0;
	uint16_t crc_calc = 0;
	uint16_t crc_recv = 0;
	memcpy(IMU_DM_Info.imu_data, pData, len);
	IMU_DM_Info.imu_len = len;
	while(i <= len)
	{
		// 检查帧头和帧尾
		if (pData[i] == 0x55 && pData[i + 1] == 0xAA && pData[i + 2] == 0x01 && (pData[i + 18] == 0x0A || pData[i + 22] == 0x0A))//对帧头，标志号，从机ID，帧尾
		{
			if(pData[i + 3] == 0x01)//找到加速度包
			{
				// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 16);
				crc_recv = (uint16_t)(pData[i + 16] | (pData[i + 17] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info.Accel.x, &pData[i + 4], 4);
					memcpy(&IMU_DM_Info.Accel.y, &pData[i + 8], 4);
					memcpy(&IMU_DM_Info.Accel.z, &pData[i + 12], 4);
					IMU_DM_Info.IMU_state = data_ok;
					i += 19;
				} 
				else 
				{
					IMU_DM_Info.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
					i++;
				}
			}
			else if(pData[i + 3] == 0x02)//找到角速度包
			{
				// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 16);
				crc_recv = (uint16_t)(pData[i + 16] | (pData[i + 17] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info.Gyro.x, &pData[i + 4], 4);
					memcpy(&IMU_DM_Info.Gyro.y, &pData[i + 8], 4);
					memcpy(&IMU_DM_Info.Gyro.z, &pData[i + 12], 4);
					IMU_DM_Info.IMU_state = data_ok;
					i += 19;
					update_imu = 1;
				} 
				else 
				{
					IMU_DM_Info.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
					i++;
				}
			}
			else if(pData[i + 3] == 0x03)//找到角度包
			{
				// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 16);
				crc_recv = (uint16_t)(pData[i + 16] | (pData[i + 17] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info.Angle.roll,  &pData[i + 4],  4);  // Roll
					memcpy(&IMU_DM_Info.Angle.pitch, &pData[i + 8],  4); // Pitch
					memcpy(&IMU_DM_Info.Angle.yaw,   &pData[i + 12], 4); // Yaw
					IMU_DM_Info.IMU_state = data_ok;
					i += 19;
				} 
				else 
				{
					IMU_DM_Info.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
					i++;
				}
			}
			else if(pData[i + 3] == 0x04)
			{// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 20);
				crc_recv = (uint16_t)(pData[i + 20] | (pData[i + 21] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info.Quaternion.X, &pData[i + 4],  4);
					memcpy(&IMU_DM_Info.Quaternion.Y, &pData[i + 8],  4);
					memcpy(&IMU_DM_Info.Quaternion.Z, &pData[i + 12], 4);
					memcpy(&IMU_DM_Info.Quaternion.W, &pData[i + 16], 4);
					IMU_DM_Info.IMU_state = data_ok;
					i += 23;
				} 
				else 
				{
					IMU_DM_Info.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
					i++;
				}
			}
			else
			{
				i++;
			}
		}
		else
		{
			IMU_DM_Info.IMU_state = data_frame_error;
			i++;
		}			
	}
	return; 
}

uint16_t Get_CRC16(uint8_t *ptr, uint16_t len)
{
	 uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < len; ++i)
	{
	uint8_t index = (crc >> 8 ^ ptr[i]);
	crc = ((crc << 1) ^ CRC16_table[index]);
	}
	return crc;
}
