/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       calibrate_task.c/h
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  *             chassis. gimbal calibration is to calc the midpoint, max/min 
  *             relative angle. gyro calibration is to calc the zero drift.
  *             accel and mag calibration have not been implemented yet, because
  *             accel is not necessary to calibrate, mag is not used. chassis 
  *             calibration is to make motor 3508 enter quick reset ID mode.
  *             因为加速度计还没有必要去校准,而磁力计还没有用.底盘校准是使M3508进入快速
  *             设置ID模式.
  *             设置ID模式.
  *             设置ID模式.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-25-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis clabration
  *
  @verbatim
  ==============================================================================
  *             use the remote control begin calibrate,
  *             first: two switchs of remote control are down
  *             second:hold for 2 seconds, two rockers set to V, like \../;  \. means the letf rocker go bottom right.
  *             third:hold for 2 seconds, two rockers set to ./\., begin the gyro calibration
  *                     or set to '\/', begin the gimbal calibration
  *                     or set to /''\, begin the chassis calibration
  *
  *             data in flash, include cali data and name[3] and cali_flag
  *             for example, head_cali has 8 bytes, and it need 12 bytes in flash. if it starts in 0x080A0000
  *             0x080A0000-0x080A0007: head_cali data
  *             0x080A0008: name[0]
  *             0x080A0009: name[1]
  *             0x080A000A: name[2]
  *             0x080A000B: cali_flag, when cali_flag == 0x55, means head_cali has been calibrated.
  *             if add a sensor
  *             1.add cail sensro name in cali_id_e at calibrate_task.h, like
  *             typedef enum
  *             {
  *                 ...
  *                 //add more...
  *                 CALI_XXX,
  *                 CALI_LIST_LENGHT,
  *             } cali_id_e;
  *             2. add the new data struct in calibrate_task.h, must be 4 four-byte mulitple  like
  *
  *             typedef struct
  *             {
  *                 uint16_t xxx;
  *                 uint16_t yyy;
  *                 fp32 zzz;
  *             } xxx_cali_t; //size: 8 bytes, must be 4, 8, 12, 16...
  *             3.in "FLASH_WRITE_BUF_LENGHT", add "sizeof(xxx_cali_t)", and implement new function.
  *             bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), and add the name in "cali_name[CALI_LIST_LENGHT][3]"
  *             and declare variable xxx_cali_t xxx_cail, add the data address in cali_sensor_buf[CALI_LIST_LENGHT]
  *             and add the data lenght in cali_sensor_size, at last, add function in cali_hook_fun[CALI_LIST_LENGHT]
  *             第三步:摇杆打成./\. 开始陀螺仪校准
  *             第三步:摇杆打成./\. 开始陀螺仪校准
  *             第三步:摇杆打成./\. 开始陀螺仪校准
  *             第三步:摇杆打成./\. 开始陀螺仪校准
  *                    第一步:遥控器拨杆打左上 开始云台校准
  *                    第二步:遥控器拨杆打右上 开始陀螺仪校准
  *
  *             保存在flash中,包括校准数据和名字 name[3] 以及 校准标志位 cali_flag
  *             其中head_cali有八个字节,因此需要12字节存flash,存储从0x080A0000开始
  *             0x080A0000-0x080A0007: head_cali数据
  *             0x080A0008: 名字name[0]
  *             0x080A0009: 名字name[1]
  *             0x080A000A: 名字name[2]
  *             0x080A000B: 校准标志位 cali_flag,当校准标志位为0x55,意味着head_cali已经校准了
  *             1.添加设备名在calibrate_task.h的cali_id_e, 像
  *             1.添加设备名在calibrate_task.h的cali_id_e, 像
  *             typedef enum
  *             {
  *                 ...
  *                 //add more...
  *                 CALI_XXX,
  *                 CALI_LIST_LENGHT,
  *             } cali_id_e;
  *             2. 添加数据结构在 calibrate_task.h, 必须4字节倍数，像
  *
  *             typedef struct
  *             {
  *                 uint16_t xxx;
  *                 uint16_t yyy;
  *                 fp32 zzz;
  *             } xxx_cali_t; //长度:8字节 8 bytes, 必须是 4, 8, 12, 16...
  *             3.在 "FLASH_WRITE_BUF_LENGHT",添加"sizeof(xxx_cali_t)", 和实现新函数
  *             bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), 添加新名字在 "cali_name[CALI_LIST_LENGHT][3]"
  *             和申明变量 xxx_cali_t xxx_cail, 添加变量地址在cali_sensor_buf[CALI_LIST_LENGHT]
  *             在cali_sensor_size[CALI_LIST_LENGHT]添加数据长度, 最后在cali_hook_fun[CALI_LIST_LENGHT]添加函数
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */


#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "struct_typedef.h"

/* 旧 DJI 校准蜂鸣宏 — 已删除，改用 buzzer_init_done() 旋律 */


//get stm32 chip temperature, to calc imu control temperature.获取stm32片内温度，计算imu的控制温度
#define cali_get_mcu_temperature()  get_temprate()      



#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                     //flash read function, flash 读取函数
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))     //flash write function,flash 写入函数
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))              //flash erase function,flash擦除函数



#define gyro_cali_disable_control()         do {} while(0)
#define gyro_cali_enable_control()          do {} while(0)

// calc the zero drift function of gyro, 计算陀螺仪的零漂函数
#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//set the zero drift to the INS task, 设置在INS task内的陀螺仪零漂
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))



#define FLASH_USER_ADDR         ADDR_FLASH_SECTOR_9 //write flash page 9,保存的flash页地址

#define GYRO_CONST_MAX_TEMP     45.0f               //max control temperature of gyro,最大陀螺仪控制温度

#define CALI_FUNC_CMD_ON        1                   //need calibrate,设置校准
#define CALI_FUNC_CMD_INIT      0                   //has been calibrated, set value to init.已经校准过，设置校准值

#define CALIBRATE_CONTROL_TIME  1                   //osDelay time,  means 1ms.1ms 系统延时

#define CALI_SENSOR_HEAD_LEGHT  1

#define SELF_ID                 0                   //ID 
#define FIRMWARE_VERSION        12345               //handware version.
#define CALIED_FLAG             0x55                // means it has been calibrated
//you have 20 seconds to calibrate by remote control. 有20s可以用遥控器进行校准
#define CALIBRATE_END_TIME          20000
//when 10 second, buzzer frequency change to high frequency of gimbal calibration.当10s的时候,蜂鸣器切成高频声音
#define RC_CALI_BUZZER_MIDDLE_TIME  10000
//in the beginning, buzzer frequency change to low frequency of imu calibration.当开始校准的时候,蜂鸣器切成低频声音
#define RC_CALI_BUZZER_START_TIME   0


#define RC_CMD_LONG_TIME            2000

#define RCCALI_BUZZER_CYCLE_TIME    400        
#define RC_CALI_BUZZER_PAUSE_TIME   200       
#define RC_CALI_VALUE_HOLE          600     //remote control threshold, the max value of remote control channel is 660. 


#define GYRO_CALIBRATE_TIME         2000    //gyro calibrate time, 2s

//cali device name
typedef enum
{
    CALI_HEAD = 0,
    CALI_GIMBAL = 1,
    CALI_GYRO = 2,
    CALI_ACC = 3,
    CALI_MAG = 4,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;


typedef __packed struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buf;                                //link to device calibration data
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);   //cali function
} cali_sensor_t;

//header device
typedef __packed struct
{
    uint8_t self_id;            // the "SELF_ID"
    uint16_t firmware_version;  // set to the "FIRMWARE_VERSION"
    //'temperature' and 'latitude' should not be in the head_cali, because don't want to create a new sensor
    //'temperature' and 'latitude'不应该在head_cali,因为不想创建一个新的设备就放这了
    int8_t temperature;         // imu control temperature
    fp32 latitude;              // latitude
} head_cali_t;
//gimbal device
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    fp32 yaw_max_angle;
    fp32 yaw_min_angle;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} gimbal_cali_t;
//gyro, accel, mag device
typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;


/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
extern void cali_param_init(void);
/**
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         imu control temperature
  */
/**
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval imu控制温度
  */
extern int8_t get_control_temperature(void);

/**
  * @brief          get latitude, default 22.0f
  * @param[out]     latitude: the point to fp32 
  * @retval         none
  */
/**
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param [out]     latitude: fp32指针
  * @retval         none
  */
extern void get_flash_latitude(float *latitude);

/**
  * @brief          calibrate task, created by main function
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param [in]      pvParameters:  空
  * @retval         none
  */
extern void calibrate_task(void const *pvParameters);

/**
 * @brief  Trigger calibration for a specific sensor.
 * @param  id: CALI_ACC or CALI_MAG
 */
extern void calibrate_trigger(uint8_t id);

/**
 * @brief  同时校准陀螺仪+加速度计(非阻塞触发)。
 *         调用后车必须水平静止约2.2s, 校准在 calibrate_task 中后台进行。
 */
extern void cali_gyro_accel_start(void);

/**
 * @brief  阻塞等待陀螺仪+加速度计校准完成。
 * @param  timeout_ms: 超时时间(ms)
 * @retval true: 校准完成; false: 超时
 */
extern bool_t cali_gyro_accel_wait(uint32_t timeout_ms);

/**
 * @brief  便捷接口: 触发并阻塞等待陀螺仪+加速度计校准完成。
 *         车须水平静止约2.2s。典型调用 cali_gyro_accel(5000)。
 * @param  timeout_ms: 超时时间(ms)
 * @retval true: 校准完成; false: 超时
 */
extern bool_t cali_gyro_accel(uint32_t timeout_ms);

/**
 * @brief  磁力计校准(非阻塞触发)。车须绕Z轴旋转约10s(触发后保持旋转直到降调音效),
 *         校准完成后播放降调五连音特殊音效。
 */
extern void cali_mag_start(void);

/**
 * @brief  阻塞等待磁力计校准完成。
 * @param  timeout_ms: 超时时间(ms)
 * @retval true: 校准完成; false: 超时
 */
extern bool_t cali_mag_wait(uint32_t timeout_ms);

/**
 * @brief  便捷接口: 触发并阻塞等待磁力计校准完成。
 *         车须绕Z轴旋转约10s。典型调用 cali_mag(15000)。
 * @param  timeout_ms: 超时时间(ms)
 * @retval true: 校准完成; false: 超时
 */
extern bool_t cali_mag(uint32_t timeout_ms);


#endif
