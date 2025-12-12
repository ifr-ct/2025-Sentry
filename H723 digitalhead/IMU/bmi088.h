#ifndef __BMI088_H_
#define __BMI088_H_

#include "string.h"
#include "main.h"
#include "spi.h"
#include "BMI088reg.h"
#include "tim.h"
#include "ifr_pid.h"
#include "stm32h7xx_hal.h"
#include "stm32_hal_legacy.h"
//#include "ifr_lib.h"
//#include "tim_count_us.h"

#define USE_FREERTOS 0

#if USE_FREERTOS
void Start_INS_Task(void const *pvParameters);
#endif //#if USE_FREERTOS


extern float INS_quat[4];//四元数
//以下四个IO口配置 CS1_ACCEL(CSB1)和CS1_GYRO(CSB2) 对应的IO口应设置为Output		INT1_ACCEL和INT1_GRYO 对应的IO口应设置为EXTI中断
#if !defined(CS1_ACCEL_Pin)
#define CS1_ACCEL_Pin GPIO_PIN_0
#endif

#if !defined(CS1_ACCEL_GPIO_Port)
#define CS1_ACCEL_GPIO_Port GPIOC
#endif

#if !defined(CS1_GYRO_Pin)
#define CS1_GYRO_Pin GPIO_PIN_3
#endif

#if !defined(CS1_GYRO_GPIO_Port)
#define CS1_GYRO_GPIO_Port GPIOC
#endif

#if !defined(INT1_ACCEL_Pin)
#define INT1_ACCEL_Pin GPIO_PIN_10
#endif

#if !defined(INT1_ACCEL_GPIO_Port)
#define INT1_ACCEL_GPIO_Port GPIOE
#endif

#if !defined(INT1_GRYO_Pin)
#define INT1_GRYO_Pin GPIO_PIN_12
#endif

#if !defined(INT1_GRYO_GPIO_Port)
#define INT1_GRYO_GPIO_Port GPIOE
#endif



#define INT1_GYRO_Pin INT1_GRYO_Pin
#define INT1_GYRO_GPIO_Port INT1_GRYO_GPIO_Port
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn  //PD14

/***********driver*********************/
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150


#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125


#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f
/**************************************/
#define SELF_ID                 0                   //ID 
#define GYRO_CONST_MAX_TEMP     40.0f               //max control temperature of gyro,最大陀螺仪控制温度
#define FIRMWARE_VERSION        12345               //handware version.

#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))     //flash write function,flash 写入函数
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))              //flash erase function,flash擦除函数


/**************************************/
#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2


//#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
//    {1.0f, 0.0f, 0.0f},                     \
//    {0.0f, 1.0f, 0.0},                     \
//    {0.0f, 0.0f, 1.0f}                      \

//若主控板的安装旋转了（即陀螺仪对应轴旋转方向与电机旋转不一致），请修改下方这个矩阵，遵循右手坐标系（食指为x轴）右手法则（大拇指指向轴的正方向，四指弯曲方向为正旋转）
//若旋转多轴则为左乘
#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, -0.9063f, -0.4226f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.4226f, -0.9063f}                      \
/*
		绕X轴旋转角度θ对应矩阵:   
		{1.0f, 0.0f, 0.0f},                     \
    {0.0f, cosθ, -sinθ},                     \
    {0.0f, sinθ, cosθ}                      \
		绕y轴旋转角度θ对应矩阵:   
		{cosθ, 0.0f, sinθ},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {-sinθ, 0.0f, cosθ}                      \
		绕z轴旋转角度θ对应矩阵:   
		{cosθ, -sinθ, 0.0f},                     \
    {sinθ, cosθ, 0.0f},                    \
    {0.0f, 0.0f, 1.0f}                      \
*/

#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

/****************************************/
#define CALI_FUNC_CMD_ON        1                   //need calibrate,设置校准
#define CALI_FUNC_CMD_INIT      0                   //has been calibrated, set value to init.已经校准过，设置校准值
// calc the zero drift function of gyro, 计算陀螺仪零漂
#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//set the zero drift to the INS task, 设置在INS task内的陀螺仪零漂
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))
#define GYRO_CALIBRATE_TIME         20000   //gyro calibrate time,陀螺仪校准时间
#define CALIED_FLAG             0x55                // means it has been calibrated
		
/* Base address of the Flash sectors */
/* STM32H723 User Flash Sector Addresses */
#define ADDR_FLASH_SECTOR_0  ((uint32_t)0x08000000)  // Sector 0, 128 KB
#define ADDR_FLASH_SECTOR_1  ((uint32_t)0x08020000)  // Sector 1, 128 KB
#define ADDR_FLASH_SECTOR_2  ((uint32_t)0x08040000)  // Sector 2, 128 KB
#define ADDR_FLASH_SECTOR_3  ((uint32_t)0x08060000)  // Sector 3, 128 KB
#define ADDR_FLASH_SECTOR_4  ((uint32_t)0x08080000)  // Sector 4, 128 KB
#define ADDR_FLASH_SECTOR_5  ((uint32_t)0x080A0000)  // Sector 5, 128 KB
#define ADDR_FLASH_SECTOR_6  ((uint32_t)0x080C0000)  // Sector 6, 128 KB
#define ADDR_FLASH_SECTOR_7  ((uint32_t)0x080E0000)  // Sector 7, 128 KB

#define FLASH_END_ADDR ((uint32_t)0x08100000)       /* Base address of Sector 7, 128 Kbytes */

#define FLASH_USER_ADDR         ADDR_FLASH_SECTOR_2 //write flash page 9,保存的flash页地址

/******************************************/
#define TEMPERATURE_PID_KP 200.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.5f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 10.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   500.0f 	//温度控制PID的max_out	//若想减慢温度上升可以修改这个
#define TEMPERATURE_PID_MAX_IOUT 	200.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 500 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1


#ifdef __cplusplus
 extern "C" {
#endif
	 
enum
{
    BMI088_NO_ERROR = 0x00,
    BMI088_ACC_PWR_CTRL_ERROR = 0x01,
    BMI088_ACC_PWR_CONF_ERROR = 0x02,
    BMI088_ACC_CONF_ERROR = 0x03,
    BMI088_ACC_SELF_TEST_ERROR = 0x04,
    BMI088_ACC_RANGE_ERROR = 0x05,
    BMI088_INT1_IO_CTRL_ERROR = 0x06,
    BMI088_INT_MAP_DATA_ERROR = 0x07,
    BMI088_GYRO_RANGE_ERROR = 0x08,
    BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
    BMI088_GYRO_LPM1_ERROR = 0x0A,
    BMI088_GYRO_CTRL_ERROR = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

    BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
    BMI088_SELF_TEST_GYRO_ERROR = 0x40,
    BMI088_NO_SENSOR = 0xFF,
	
		BMI088_GYRO_YAW_OFFSET_ERROR = 0x20,
};


typedef enum
{
	IMU_IDLE,
	IMU_ACCEL_Updata,
	IMU_Gyro_Updata,
	IMU_Temperate_Updata,
	IMU_Stop,
}IMU_UpdataFlagTypedef;

extern IMU_UpdataFlagTypedef SPI_UpdataFlag;

typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
} bmi088_real_data_t;
/**************calibrate*******************/
#define CALI_HEADER_UINT32_LEN  1
typedef __packed struct 
{
    uint8_t name[3];      // 设备名称（3字节）
    uint8_t cali_done;    // 校准标志（0x55表示已校准）
    uint8_t flash_len :7; // 校准数据长度（以uint32_t为单位，占7位）
    uint8_t cali_cmd :1;  // 校准命令位（1表示需要校准）
    uint32_t *flash_buf;  // 指向校准数据的指针
    uint8_t (*cali_hook)(uint32_t *point, uint8_t cmd); // 校准回调函数
} cali_sensor_t;//9

typedef struct {
    float offset[3];    // 12字节xyz
    float scale[3];     // 12字节xyz
    uint8_t reserved[8]; // 填充到32字节
} imu_cali_t;           // 总长度=32字节

//cali device name
typedef enum
{
    CALI_HEAD = 0,
    CALI_GIMBAL = 1,
    CALI_GYRO = 2,
    CALI_ACC = 3,
    //CALI_MAG = 4,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;

//header device
typedef __packed struct
{
    uint8_t self_id;            // the "SELF_ID"
    uint16_t firmware_version;  // set to the "FIRMWARE_VERSION"
    //'temperature' and 'latitude' should not be in the head_cali, because don't want to create a new sensor
    //'temperature' and 'latitude'不应该在head_cali,因为不想创建一个新的设备就放这了
    int8_t temperature;         // imu control temperature
    float latitude;              // latitude
} head_cali_t;//8

//gimbal device
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    float yaw_max_angle;
    float yaw_min_angle;
    float pitch_max_angle;
    float pitch_min_angle;
} gimbal_cali_t;//20
//gyro, accel, mag device

//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)

/****************************************/
#define GYRO_YAW_OFFSET  	2
#define GYRO_ROLL_OFFSET	0
#define GYRO_PITCH_OFFSET	1


typedef struct
{
	float Yaw;
	float Pitch;
	float Roll;
}IMU_AngleTypedef;

typedef struct
{
	IMU_AngleTypedef Gyro;	
	IMU_AngleTypedef Accel;
	//float Gyro[3];
	//float Accel[3];
	IMU_AngleTypedef Angle;
	float Temperate;
	
	float time;//加速度计时间差（us）

	uint8_t  UpDataFlag_IMU;
}IMU_InfoTypedef;

extern IMU_InfoTypedef IMU_Info;

uint8_t BMI088_init(void);
uint8_t bmi088_accel_init(void);
uint8_t bmi088_gyro_init(void);
uint8_t BMI088_read_write_byte(uint8_t txdata);
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);

void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);

void INS_set_cali_gyro(float cali_scale[3], float cali_offset[3]);
void INS_cali_gyro(float cali_scale[3], float cali_offset[3], uint16_t *time_count);
void gyro_offset_calc(float gyro_offset[3], float gyro[3], uint16_t *offset_time_count);
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az,float frequency) ;
float invSqrt(float x);
void AHRS_update(float quat[4], uint32_t time_us, float gyro[3], float accel[3]);

	


void AHRS_init(float quat[4], float accel[3]);
void cali_param_init(void);
void flash_erase_address(uint32_t address, uint16_t len);
uint32_t ger_sector(uint32_t address);
uint32_t get_next_flash_address(uint32_t address);
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len);
void Calibrate(void);
void calibrate_task(void const *pvParameters);

static uint8_t YawZeroOffset_Init(IMU_InfoTypedef* BMI088INFO, uint8_t MaxData_abs);
static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);
static void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data);
static void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
static void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data);
static void BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t *data, uint8_t len);
static void imu_cali_slove(float gyro[3], float accel[3], bmi088_real_data_t *bmi088);
static uint8_t cali_gyro_hook(uint32_t *cali, uint8_t cmd);
static void imu_temp_control(float temp);
static void imu_cmd_spi_dma(void);
static void cali_data_write(void);
static void cali_data_read(void);
static uint32_t STMFLASH_GetFlashSector(uint32_t Address);
uint32_t get_h7_flash_sector(uint32_t address);
#ifdef __cplusplus
	}
#endif
	void get_angle(float q[4], float *yaw, float *pitch, float *roll);
#endif //#ifndef __BMI088_H_

