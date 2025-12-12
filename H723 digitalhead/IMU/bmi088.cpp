#include "math.h"
#include "bmi088.h"
#include "stm32h7xx_hal_flash.h"
/*******************
* file declaration *
* If you are using FreeRTOS, change USE_FREERTOS to 1!
* Note: To use temperature control, turn on timer 10 channel 1.
* Set psc to 0 and arr to 4999 for timer 10. 
* In addition, it should be noted that the GPIO corresponding to the PWM output is PF6.
* If the gyroscope drift is serious, please erase the flash after the temperature reaches 45 and download the calibration again. 
* The calibration takes about 30s. Do not move during the calibration process!
* The gyroscope data are saved in @ref IMU_Info
* To initialize, use the following functions.

* 如果你使用了FreeRTOS，请将 USE_FREERTOS 改为1！
*	注意：使用温度控制请开启定时器10通道1
* 请将定时器10的psc设置成0，arr设置为4999。
* 此外，需注意PWM输出对应的GPIO为PF6.
* 如果陀螺仪漂移严重，请在温度到45后擦除flash重新下载校准，校准大概需要30s，校准过程请勿移动！
* 陀螺仪数据保存在 @ref IMU_Info.

* 基础配置：
* SPI配置： Mode为FuLL-Duplex Master模式 控制PSC使Baud Rate在10MBit/s以下
* IO配置：	看bmi088.h中 CS1_ACCEL和CS1_GYRO 对应的IO口应设置为Output		INT1_ACCEL和INT1_GRYO 对应的IO口应设置为EXTI中断
* 中断配置：将PG0设置为中断

* 注：若想在除了C板以外的主控板使用BMI088需要修改这四个IO口对应的IO，并且修改PG0为其他中断线

* 以下是使用例程：
* 为了进行初始化，请使用下面的几个函数。


	* @fun						void cali_param_init(void); 
  * @brief          开始校准陀螺仪
  * @param[in]      none
  * @retval         none

	* @fun 						uint8_t BMI088_init(void); 
  * @brief          初始化BMI088。
  * @param[in]      none
  * @retval        	错误码，详见bmi088.h,无错时返回0。 @ref BMI088_NO_ERROR
	
	* @fun 						void Calibrate(void);
  * @brief          校准函数，每1ms间隔调用（建议放在while(1)里）
  * @param[in]      none
  * @retval         none
	* @note						不使用FreeRTOS时。
	
	* @fun 						void calibrate_task(void const *pvParameters);
	* @brief          校准任务，由main函数创建（这个本身就是一个任务函数！！！）
  * @param[in]      pvParameters: 空
  * @retval         none
	* @note						使用FreeRTOS时。
	
	* @pl 例子：
{
...
	MX_xxxxx_Init();
	* USER CODE BEGIN 2 *
	cali_param_init();
	while(BMI088_init())
	{
		HAL_Delay(1);
	}
	
	while (1)
	{
		Calibrate();
		HAL_Delay(1);		//这里很重要，一定是1ms，或者使用一个1ms定时器，不要是其他ms数
	}
}
	
********************/
/*************************/

void cali_param_init(void);//放在 BMI088_init 之前（如果使用了FreeRTOS，这个函数最好不要放进任务（）！！！）。

uint8_t BMI088_init(void);//返回0初始化成功


#if USE_FREERTOS
void calibrate_task(void const *pvParameters);
extern osThreadId INS_TaskHandle;	
#else
void Calibrate(void);
#endif //#if USE_FREERTOS

/*************************/
IMU_InfoTypedef IMU_Info;


//#define sampleFreq	1000.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;


//加速度计低通滤波
static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
__align(32) static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];
/**********flag*********/
uint8_t Accel_flag;
uint8_t Temperate_flag;
uint8_t Gyro_flag;
//bit0:DR(可以数据记录) bit1:SPI（正在传输） bit2：UPDATA(传输完成)
#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2

#define SET_FLAG_BIT(__FLAG__, __BIT__)  	__FLAG__ &= ~(1 << __BIT__)
#define RESET_FLAG_BIT(__FLAG__, __BIT__)	__FLAG__ |=  (1 << __BIT__)
#define READ_FLAG_BIT(__FLAG__, __BIT__)	__FLAG__ & 	 (1 << __BIT__)
/***********************/
static const uint8_t cali_name[CALI_LIST_LENGHT][4] = {"HD", "GM", "GYR", "ACC",}; //"MAG"
cali_sensor_t cali_sensor[CALI_LIST_LENGHT];//gyr,acc

float accel_offset[3];
float accel_cali_offset[3];

float gyro_cali_offset[3];
float gyro_offset[3];


static head_cali_t     head_cali;       //head cali data
static gimbal_cali_t   gimbal_cali;     //gimbal cali data
static imu_cali_t      accel_cali;      //accel cali data
static imu_cali_t      gyro_cali;       //gyro cali data
//static imu_cali_t      mag_cali;        //mag cali data

bmi088_real_data_t bmi088_real_data;
//cali data address
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,};
        //(uint32_t *)&mag_cali};

//*******************************temp ctrl**************************************//


IFR_PID Temp_Pid;

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
//static uint8_t flag_Mem = 0;
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};

float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};

float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};

IMU_UpdataFlagTypedef SPI_UpdataFlag = IMU_Stop;




float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
float INS_accel[3] = {0.0f, 0.0f, 0.0f};
float INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};

/**
  * @brief          初始化BMI088。
  * @param[in]      none
  * @retval        	错误码，详见bmi088.h,无错时返回0 @ref BMI088_NO_ERROR
  */
uint8_t error = BMI088_NO_ERROR;
uint8_t BMI088_init(void)
{
	//
	SPI_UpdataFlag = IMU_Stop;
    
    // GPIO and SPI  Init .
		error |= bmi088_accel_init();
    error |= bmi088_gyro_init();
		
		if (error != 0) return error;
		IMU_Info.UpDataFlag_IMU = 0;
		
		//BMI088_read
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
		float* gyro = bmi088_real_data.gyro;
		float* accel = bmi088_real_data.accel;
		float* temperate = &bmi088_real_data.temp;

		BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
		
		////rotate and zero drift
		
		imu_cali_slove(INS_gyro, INS_accel, &bmi088_real_data);
		
		
		Temp_Pid.PID_Init(TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD, TEMPERATURE_PID_MAX_OUT, 30,TEMPERATURE_PID_MAX_IOUT,0);
		
    AHRS_init(INS_quat, INS_accel);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
		
		
//		hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
		
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
		
		SPI_UpdataFlag = IMU_IDLE;
		//HAL_SPI_TransmitReceive_DMA(&hspi1, gyro_dma_tx_buf, gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

		return error;
}

/**
  * @brief          "head" sensor cali function
  * @param[in][out] cali:the point to head data. when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          "head"设备校准
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static uint8_t cali_head_hook(uint32_t *cali, uint8_t cmd)
{
    head_cali_t *local_cali_t = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
//        memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));

        return 1;
    }
    // self id
    local_cali_t->self_id = SELF_ID;
    //imu control temperature
    local_cali_t->temperature = 45.0f;
    //head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }
    
    local_cali_t->firmware_version = FIRMWARE_VERSION;
    //shenzhen latitude 
    local_cali_t->latitude = 22.0f;

    return 1;
}



/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
{
		sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
		sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4
};
void *cali_hook_fun[CALI_LIST_LENGHT] = {(void *)cali_head_hook, NULL, (void *)cali_gyro_hook, NULL};

void cali_param_init(void)
{
	

	/*
    uint8_t i = 0;
		void *cali_hook_fun[2] = {(void *)cali_gyro_hook, NULL};
    for (i = 0; i < 2; i++)
    {
        cali_sensor[i].flash_len = sizeof(imu_cali_t) / 4;
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (uint8_t(*)(uint32_t *, uint8_t))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < 2; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                //if has been calibrated, set to init 
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
		*/
		uint8_t i = 0;
		
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (uint8_t(*)(uint32_t *, uint8_t))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                //if has been calibrated, set to init 
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}
#if USE_FREERTOS
/**
  * @brief          calibrate task, created by main function
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          校准任务，由main函数创建
  * @param[in]      pvParameters: 空
  * @retval         none
  */

void calibrate_task(void const *pvParameters)
{
    static uint8_t i = 0;
    //osDelay(10);
    //calibrate_RC = get_remote_ctrl_point_cali();

    while (1)
    {
        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {
            if (cali_sensor[i].cali_cmd)
            {
                if (cali_sensor[i].cali_hook != NULL)
                {

                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {
                        //done
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];
                        //set 0x55
                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        //write
                        cali_data_write();
                    }
                }
            }
        }

				osDelay(1);

		}
}


#else

/**
  * @brief          calibrate function, created by main function
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          校准函数，每1ms间隔调用
  * @param[in]      none
  * @retval         none
  */
int count_bmi = 0;
void Calibrate(void) 
{
	static uint8_t i = 0;

	for (i = 0; i < CALI_LIST_LENGHT; i++)
	{
			if (cali_sensor[i].cali_cmd)
			{
					if (cali_sensor[i].cali_hook != NULL)
					{

							if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
							{
									//done
									cali_sensor[i].name[0] = cali_name[i][0];
									cali_sensor[i].name[1] = cali_name[i][1];
									cali_sensor[i].name[2] = cali_name[i][2];
									//set 0x55
									cali_sensor[i].cali_done = CALIED_FLAG;

									cali_sensor[i].cali_cmd = 0;
									//write
									cali_data_write();
									count_bmi++;
							}
					}
			}
	}
				
}


#endif //#if USE_FREERTOS



/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(float temp)
{
    uint16_t tempPWM = 0;
		float pid_out;
		pid_out = Temp_Pid.Positional_PID(GYRO_CONST_MAX_TEMP, temp);		//使用PID进行温度控制
    tempPWM = (uint16_t)pid_out;
		if (pid_out < 0) tempPWM = 0;
    htim3.Instance->CCR4 = (uint16_t)tempPWM;
}
/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          往flash写入校准数据
  * @param[in]      none
  * @retval         none
  */

static void cali_data_write(void) {
    uint8_t i = 0;
    uint32_t offset = 0;

    // 填充校准数据到缓冲区
    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        memcpy(flash_write_buf + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len *4);
        offset += cali_sensor[i].flash_len *4;

        memcpy(flash_write_buf + offset, cali_sensor[i].name, CALI_HEADER_UINT32_LEN *4);
        offset += CALI_HEADER_UINT32_LEN *4;
    }

    // 强制32字节对齐
    uint32_t total_bytes = offset;
    uint32_t remainder = total_bytes % 32;
    if (remainder != 0) {
        uint32_t padding = 32 - remainder;
        total_bytes += padding;
        memset(flash_write_buf + offset, 0xFF, padding);
    }

    // 擦除并写入（确保total_bytes是32的倍数）
    uint32_t sectors_needed = (total_bytes + 0x1FFFF) / 0x20000; // H7每扇区128KB
    flash_erase_address(FLASH_USER_ADDR, sectors_needed);

    // 写入长度必须为8的倍数（32字节=8个uint32_t）
    uint32_t write_len = total_bytes /4;
    if (flash_write_single_address(FLASH_USER_ADDR, (uint32_t*)flash_write_buf, write_len) !=0) 
		{
        //Error_Handler(); // 处理写入错误
    }
}
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(uint32_t*)faddr; 
}
/**
  * @brief          write data to one page of flash
  * @param[in]      start_address: flash address
  * @param[in]      buf: data point
  * @param[in]      len: data num
  * @retval         success 0, fail -1
  */

		uint32_t addrx=0;
		uint32_t endaddr=0;	
/**
  * @brief          向Flash写入数据（自动补全到32字节对齐）(只针对H7)
  * @param[in]      start_address: 起始地址（必须32字节对齐）
  * @param[in]      buf: 数据指针（建议32字节对齐以提升性能）
  * @param[in]      len: 数据长度（支持任意字节数）
  * @retval         成功返回0，失败返回-1
  */
uint32_t b = 0;
uint32_t c = 0;
uint32_t d = 0;
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len)
{
		b = (uintptr_t)buf;
		c = len;
		d = start_address;
    // 参数检查
    if ((start_address % 32) != 0 || (uintptr_t)buf % 32 != 0 || (len * 4) % 32 != 0) {
        return -1;
    }

    HAL_FLASH_Unlock();
    __disable_irq();

    for (uint32_t i = 0; i < len; i += 8) // 每次写入8个uint32_t（32字节）
    {
        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, 
                            start_address + i * 4, 
                            (uint32_t)(buf + i)) != HAL_OK) 
        {
            HAL_FLASH_Lock();
            __enable_irq();
            return -1;
        }
    }

    HAL_FLASH_Lock();
    __enable_irq();
    return 0;
}

/**
  * @brief          erase flash
  * @param[in]      address: flash address
  * @param[in]      len: page num
  * @retval         none
  */
/**
  * @brief          擦除flash
  * @param[in]      address: flash 地址
  * @param[in]      len: 页数量
  * @retval         none
  */

void flash_erase_address(uint32_t address, uint16_t num_sectors)
{
    FLASH_EraseInitTypeDef erase_cfg = {0};
    uint32_t first_sector = get_h7_flash_sector(address);
    uint32_t sector_error;

    if (first_sector == 0xFFFFFFFF) return;

    erase_cfg.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_cfg.Banks = FLASH_BANK_1;
    erase_cfg.Sector = first_sector;
    erase_cfg.NbSectors = num_sectors;
    erase_cfg.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    __disable_irq();
    HAL_FLASHEx_Erase(&erase_cfg, &sector_error);
    __enable_irq();
    HAL_FLASH_Lock();
}


/**
  * @brief          get the sector number of flash
  * @param[in]      address: flash address
  * @retval         sector number
  */
/**
  * @brief          获取H7系列Flash的扇区号
  * @param[in]      address: Flash地址（必须位于用户Flash区域）
  * @retval         扇区号（FLASH_SECTOR_0~FLASH_SECTOR_7）
  * @retval         0xFFFFFFFF表示地址非法
  */
uint32_t get_h7_flash_sector(uint32_t address)
{
    // 用户Flash地址范围检查（0x08000000 - 0x080FFFFF）
    if ((address < FLASH_BASE) || (address >= (FLASH_BASE + FLASH_SIZE))) {
        return 0xFFFFFFFF;
    }
    
    // H723每个扇区128KB，直接计算偏移
    uint32_t sector_num = (address - FLASH_BASE) / 0x20000; // 128KB = 0x20000
    
    // 转换为HAL库定义的扇区号
    switch (sector_num) {
        case 0: return FLASH_SECTOR_0;
        case 1: return FLASH_SECTOR_1;
        case 2: return FLASH_SECTOR_2;
        case 3: return FLASH_SECTOR_3;
        case 4: return FLASH_SECTOR_4;
        case 5: return FLASH_SECTOR_5;
        case 6: return FLASH_SECTOR_6;
        case 7: return FLASH_SECTOR_7;
        default: return 0xFFFFFFFF;
    }
}


/**
  * @brief          get the next page flash address
  * @param[in]      address: flash address
  * @retval         next page flash address
  */
/**
  * @brief          获取下一个Flash扇区的起始地址（适配STM32H723）
  * @param[in]      address: 当前Flash地址
  * @retval         下一个扇区起始地址，若已是最后扇区返回FLASH_END_ADDR
  */
uint32_t get_next_flash_address(uint32_t address)
{
    /*------------------- 用户Flash区域定义 -------------------*/
    // STM32H723用户Flash扇区地址表（共8个扇区，每个128KB）
    const uint32_t SECTOR_ADDR_TABLE[8] = 
			{
        0x08000000, // Sector 0
        0x08020000, // Sector 1
        0x08040000, // Sector 2
        0x08060000, // Sector 3
        0x08080000, // Sector 4
        0x080A0000, // Sector 5
        0x080C0000, // Sector 6
        0x080E0000  // Sector 7
    };


    /*------------------- 参数检查 -------------------*/
    if (address < SECTOR_ADDR_TABLE[0] || address >= FLASH_END_ADDR) {
        return FLASH_END_ADDR; // 非法地址返回结束标记
    }

    /*------------------- 计算当前扇区索引 -------------------*/
    uint32_t sector_index = (address - SECTOR_ADDR_TABLE[0]) / 0x20000; // 128KB=0x20000

    /*------------------- 返回下一个扇区地址 -------------------*/
    if (sector_index < 7) {
        return SECTOR_ADDR_TABLE[sector_index + 1];
    } else {
        return FLASH_END_ADDR; // 已是最后扇区
    }
}



/**
  * @brief          read data for flash
  * @param[in]      address: flash address
  * @param[out]     buf: data point
  * @param[in]      len: data num
  * @retval         none
  */
/**
  * @brief          从flash读数据
  * @param[in]      start_address: flash 地址
  * @param[out]     buf: 数据指针
  * @param[in]      len: 数据长度
  * @retval         none
  */
void flash_read(uint32_t address, uint32_t *buf, uint32_t len)
{
    memcpy(buf, (void*)address, len *4);
}
#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                     //flash read function, flash 读取函数

/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[4] = {0};
    uint8_t i = 0;
    uint16_t offset = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        // 读取校准数据
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        offset += cali_sensor[i].flash_len *4;

        // 读取头（直接按字节解析）
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t*)flash_read_buf, 1); // 读取4字节
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];
        offset +=4;

        // 状态检查
        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook) {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}


/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static uint8_t cali_gyro_hook(uint32_t *cali, uint8_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
          
            return 1;
        }
        else
        {

            return 0;
        }
    }

    return 0;
}


/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         none
  */
void INS_set_cali_gyro(float cali_scale[3], float cali_offset[3])
{
    gyro_cali_offset[0] = cali_offset[0];
    gyro_cali_offset[1] = cali_offset[1];
    gyro_cali_offset[2] = cali_offset[2];
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         none
  */
void INS_cali_gyro(float cali_scale[3], float cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
  * @retval         none
  */
void gyro_offset_calc(float gyro_offset[3], float gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}

uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    HAL_Delay(1);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(1);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
		HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], &res);
				HAL_Delay(1);
        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    HAL_Delay(1);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(1);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    HAL_Delay(1);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        HAL_Delay(1);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], &res);
        HAL_Delay(1);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}
/*****************BMI088 R/W reg********************/
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
/*****************BMI088_Accel R/W reg***************/
static void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data)
{
	BMI088_ACCEL_NS_L();                     
	BMI088_write_single_reg(reg, data);  
	BMI088_ACCEL_NS_H();
}

static void BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data)
{
	BMI088_ACCEL_NS_L();                     
	BMI088_read_write_byte(reg | 0x80);  
  BMI088_read_write_byte(0x55);         
  *data = BMI088_read_write_byte(0x55);
	BMI088_ACCEL_NS_H();
}

static void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{                                              
		BMI088_ACCEL_NS_L();                       
		BMI088_read_write_byte((reg) | 0x80);      
		BMI088_read_muli_reg(reg, data, len);      
		BMI088_ACCEL_NS_H();                       
}

/*****************BMI088_Gyro R/W reg***************/
static void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data)
{
	BMI088_GYRO_NS_L();                     
	BMI088_write_single_reg(reg, data);  
	BMI088_GYRO_NS_H();
}

static void BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data)
{
	BMI088_GYRO_NS_L();                     
	BMI088_read_single_reg(reg, data);
	BMI088_GYRO_NS_H();
}

static void BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{                                              
		BMI088_GYRO_NS_L();                            
		BMI088_read_muli_reg(reg, data, len);      
		BMI088_GYRO_NS_H();                       
}

/**************************************/
/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(float gyro[3], float accel[3], bmi088_real_data_t *bmi088)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
//			gyro[i] +=  gyro_offset[i];
//      accel[i] += accel_offset[i];
		
		}
}
int a = 0;
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    a = HAL_SPI_TransmitReceive(&hspi2, &txdata, &rx_data, 1, 500);
    return rx_data;
}

void GyroAngle_Analysis(void);

#if USE_FREERTOS
static TaskHandle_t INS_task_local_handler;
#endif //#if USE_FREERTOS

//中断回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (SPI_UpdataFlag == IMU_Stop) return;
	
	if (GPIO_Pin == INT1_ACCEL_Pin)
	{
		Accel_flag 			|= 1 << IMU_DR_SHFITS;
		Temperate_flag 	|= 1 << IMU_DR_SHFITS;
		
		if (SPI_UpdataFlag == IMU_IDLE)
		{
			imu_cmd_spi_dma();
		}
	}
	else if (GPIO_Pin == INT1_GYRO_Pin)
	{
		/*
		if (SPI_UpdataFlag == IMU_Gyro_Updata)
		{
			//SPI_UpdataFlag = IMU_Gyro_Updata;
			IMU_Info.Time_LastRxGyro_ms = IMU_Info.Time_RxGyro_ms;
			IMU_Info.Time_RxGyro_ms = HAL_GetTick();
			BMI088_GYRO_NS_L();
			HAL_SPI_TransmitReceive_DMA(&hspi1, gyro_dma_tx_buf, gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
		}
		*/
		Gyro_flag |= 1 << IMU_DR_SHFITS;
		
		if (SPI_UpdataFlag == IMU_IDLE)
		{
			imu_cmd_spi_dma();
		}
	}
	else if (GPIO_Pin == GPIO_PIN_0)//Gyro Updata
	{
//#if USE_FREERTOS
//		if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
//        {
//            static BaseType_t xHigherPriorityTaskWoken;
//            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
//            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        }
//#else
//		
			GyroAngle_Analysis();
		
//#endif //#if USE_FREERTOS
		
//		osSignalSet(INS_TaskHandle, 1);
//		//INS_TaskHandle //GyroAngle_Analysis();
	}
}
#if USE_FREERTOS
void Start_INS_Task(void const *pvParameters)
{
	INS_task_local_handler = INS_TaskHandle;
	//osDelay(1);
	for (;;)
	{
		while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
		{
		}
		GyroAngle_Analysis();
		
	}
}
#endif //#if USE_FREERTOS

int16_t Debug_Yawnow,Debug_Pitnow,Debug_Rolnow;
void GyroAngle_Analysis(void)
{
	uint8_t* rx_buf = NULL;
	int16_t bmi088_raw_temp;
	
		
	if(Gyro_flag & (1 << IMU_UPDATE_SHFITS))
	{
		float* gyro = bmi088_real_data.gyro;
		
		Gyro_flag &= ~(1 << IMU_UPDATE_SHFITS);
		//BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
		rx_buf = gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET;
		
		bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
		gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
		bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
		gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
		bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
		gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
	}

	if(Accel_flag & (1 << IMU_UPDATE_SHFITS))
	{
		float* accel = bmi088_real_data.accel;
		uint32_t sensor_time;
		
		Accel_flag &= ~(1 << IMU_UPDATE_SHFITS);
		//BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
		
		
		rx_buf = accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET;
		bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
		accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
		bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
		accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
		bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
		accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
		sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
		bmi088_real_data.time = IMU_Info.time = sensor_time * 39.0625f;
		
	}

	if(Temperate_flag & (1 << IMU_UPDATE_SHFITS))
	{
		Temperate_flag &= ~(1 << IMU_UPDATE_SHFITS);
		
		rx_buf = accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET;
		
		bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));

		if (bmi088_raw_temp > 1023)
		{
				bmi088_raw_temp -= 2048;
		}
		bmi088_real_data.temp = IMU_Info.Temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
		imu_temp_control(IMU_Info.Temperate);
		
	}
	
	imu_cali_slove(INS_gyro, INS_accel, &bmi088_real_data);
	
	
	accel_fliter_1[0] = accel_fliter_2[0];
	accel_fliter_2[0] = accel_fliter_3[0];

	accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

	accel_fliter_1[1] = accel_fliter_2[1];
	accel_fliter_2[1] = accel_fliter_3[1];

	accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

	accel_fliter_1[2] = accel_fliter_2[2];
	accel_fliter_2[2] = accel_fliter_3[2];

	accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

	const uint32_t timing_time = 1;
	
	AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3);
	
//	for(uint8_t i = 0;i<3;i++)
//  {
//		IMU_Info.Gyro[i] = INS_gyro[i]*57.29578f;
//  }

	IMU_Info.Gyro.Roll = INS_gyro[GYRO_ROLL_OFFSET] * 57.29578f;
	IMU_Info.Gyro.Pitch = INS_gyro[GYRO_PITCH_OFFSET] * 57.29578f;
	IMU_Info.Gyro.Yaw = INS_gyro[GYRO_YAW_OFFSET] * 57.29578f;
//	for(uint8_t i = 0;i<3;i++)
//  {
//		IMU_Info.Accel[i] = INS_accel[i];
//  }
	IMU_Info.Accel.Roll = INS_accel[GYRO_ROLL_OFFSET];
	IMU_Info.Accel.Pitch = INS_accel[GYRO_PITCH_OFFSET];
	IMU_Info.Accel.Yaw = INS_accel[GYRO_YAW_OFFSET];
	get_angle(INS_quat, &IMU_Info.Angle.Yaw, &IMU_Info.Angle.Pitch, &IMU_Info.Angle.Roll);

	IMU_Info.UpDataFlag_IMU = 1;
}

void AHRS_update(float quat[4], uint32_t time_ms, float gyro[3], float accel[3])
{
		float frequency = 1000.0f/time_ms;
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], frequency);
}
void get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f)*57.29578f;
    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2]))*57.29578f;
    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f)*57.29578f;
}

void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az,float frequency) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float sampleFreq = frequency;
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}
void AHRS_init(float quat[4], float accel[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//SPI错误回调函数
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI2)
	{
		if (hspi->ErrorCode == HAL_SPI_ERROR_DMA)
		{
			hspi->ErrorCode = HAL_SPI_ERROR_NONE;
			
			if (READ_FLAG_BIT(Gyro_flag, IMU_SPI_SHFITS))
			{
				RESET_FLAG_BIT(Gyro_flag, IMU_SPI_SHFITS);
				SET_FLAG_BIT(Gyro_flag, IMU_DR_SHFITS);
			}
			else if (READ_FLAG_BIT(Temperate_flag, IMU_SPI_SHFITS))
			{
				RESET_FLAG_BIT(Temperate_flag, IMU_SPI_SHFITS);
				SET_FLAG_BIT(Temperate_flag, IMU_DR_SHFITS);
			}
			else if (READ_FLAG_BIT(Accel_flag, IMU_SPI_SHFITS))
			{
				RESET_FLAG_BIT(Accel_flag, IMU_SPI_SHFITS);
				SET_FLAG_BIT(Accel_flag, IMU_DR_SHFITS);
			}
			SPI_UpdataFlag = IMU_IDLE;
		}
		else if (hspi->ErrorCode == HAL_SPI_ERROR_NONE)
		{
			return;
		}
		else 
		{
			Error_Handler();
		}
	}
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	
	if (hspi->Instance == SPI2)
	{
		HAL_SPI_DMAStop(hspi);
		//uint8_t* rx_buf = NULL;
		//int16_t bmi088_raw_temp;
		
		//accel read over
		//加速度计读取完毕
		if (SPI_UpdataFlag == IMU_ACCEL_Updata && Accel_flag & (1 << IMU_SPI_SHFITS))
		{
			/*
			uint32_t sensor_time;
			BMI088_ACCEL_NS_H();
			SPI_UpdataFlag = IMU_IDLE;
			
			rx_buf = accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET;
			bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
			INS_accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
			bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
			INS_accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
			bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
			INS_accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
			sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
			IMU_Info.time = sensor_time * 39.0625f;
			
			return;
			*/
			BMI088_ACCEL_NS_H();
			SPI_UpdataFlag = IMU_IDLE;
			Accel_flag &= ~(1 << IMU_SPI_SHFITS);
			Accel_flag |=  (1 << IMU_UPDATE_SHFITS);
			
		}
		
		//gyro read over
		//陀螺仪读取完毕
		if (SPI_UpdataFlag == IMU_Gyro_Updata && Gyro_flag & (1 << IMU_SPI_SHFITS))
		{
			/*
			rx_buf = gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET;
			BMI088_GYRO_NS_H();
			SPI_UpdataFlag = IMU_IDLE;
			
			
			bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
			INS_gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
			bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
			INS_gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
			bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
			INS_gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
			
			__HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
			return;
			*/
			BMI088_GYRO_NS_H();
			SPI_UpdataFlag = IMU_IDLE;
			Gyro_flag &= ~(1 << IMU_SPI_SHFITS);
			Gyro_flag |=  (1 << IMU_UPDATE_SHFITS);
			
		}
		
		//temperature read over
    //温度读取完毕
		if (SPI_UpdataFlag == IMU_Temperate_Updata && Temperate_flag & (1 << IMU_SPI_SHFITS))
		{
			/*
			BMI088_ACCEL_NS_H();
			SPI_UpdataFlag = IMU_IDLE;
			
			rx_buf = accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET;
			bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));

			if (bmi088_raw_temp > 1023)
			{
					bmi088_raw_temp -= 2048;
			}
			IMU_Info.Temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
			imu_temp_control(IMU_Info.Temperate);
			*/
			BMI088_ACCEL_NS_H();
			SPI_UpdataFlag = IMU_IDLE;
			
			Temperate_flag &= ~(1 << IMU_SPI_SHFITS);
			Temperate_flag |=  (1 << IMU_UPDATE_SHFITS);
		}
		
		imu_cmd_spi_dma();
		
		if (Gyro_flag & (1 << IMU_UPDATE_SHFITS))
		{
			__HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
		}
	}
}

/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      none
  * @retval        	none
  */

static void imu_cmd_spi_dma(void)
{
	if (SPI_UpdataFlag != IMU_IDLE || SPI_UpdataFlag == IMU_Stop) return;
#if USE_FREERTOS
	UBaseType_t uxSavedInterruptStatus;
   uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
#endif
	
	
	//开启陀螺仪的DMA传输
	if (Gyro_flag & (1 << IMU_DR_SHFITS) && SPI_UpdataFlag == IMU_IDLE)
	{
		SPI_UpdataFlag = IMU_Gyro_Updata;
		Gyro_flag &= ~(1 << IMU_DR_SHFITS);
		Gyro_flag |=  (1 << IMU_SPI_SHFITS);
		
		BMI088_GYRO_NS_L();
		HAL_SPI_TransmitReceive_DMA(&hspi2, gyro_dma_tx_buf, gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
#if USE_FREERTOS
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
#endif
		return;
	}
	
	//开启加速度计的DMA传输
	if (Accel_flag & (1 << IMU_DR_SHFITS) && SPI_UpdataFlag == IMU_IDLE)
	{
		SPI_UpdataFlag = IMU_ACCEL_Updata;
		Accel_flag &= ~(1 << IMU_DR_SHFITS);
		Accel_flag |=  (1 << IMU_SPI_SHFITS);
		
		BMI088_ACCEL_NS_L();
		HAL_SPI_TransmitReceive_DMA(&hspi2, accel_dma_tx_buf, accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
		
#if USE_FREERTOS
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
#endif //#if USE_FREERTOS
		
		return;		
	}
	//开启温度计的DMA传输
	if (Temperate_flag & (1 << IMU_DR_SHFITS) && SPI_UpdataFlag == IMU_IDLE)
	{
		SPI_UpdataFlag = IMU_Temperate_Updata;
		Temperate_flag &= ~(1 << IMU_DR_SHFITS);
		Temperate_flag |=  (1 << IMU_SPI_SHFITS);
		
		BMI088_ACCEL_NS_L();
		HAL_SPI_TransmitReceive_DMA(&hspi2, accel_temp_dma_tx_buf, accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
		
#if USE_FREERTOS
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
#endif //#if USE_FREERTOS
		
		return;		
	}
	
	
	
#if USE_FREERTOS
	taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
#endif //#if USE_FREERTOS
	
}




