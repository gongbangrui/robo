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

#include "calibrate_task.h"
#include "bsp_buzzer.h"
#include "string.h"
#include "cmsis_os.h"

#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_flash.h"

#include "CAN_receive.h"

#include "INS_task.h"


//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)




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

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void);


/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void);


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
  * @retval 0: 校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   //header device cali function

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
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
  * @retval 0: 校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   //gyro device cali function

/**
  * @brief          accel cali function
  * @note           Collects samples while stationary, computes gravity-axis offset.
  *                 Place car on level surface before triggering.
  */
static bool_t cali_accel_hook(uint32_t *cali, bool_t cmd);

/**
  * @brief          mag cali function
  * @note           Collects min/max while car is slowly rotated in all directions.
  *                 User must rotate the car during calibration.
  */
static bool_t cali_mag_hook(uint32_t *cali, bool_t cmd);



#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif



static head_cali_t     head_cali;       //head cali data
static gimbal_cali_t   gimbal_cali;     //gimbal cali data
static imu_cali_t      accel_cali;      //accel cali data
static imu_cali_t      gyro_cali;       //gyro cali data
static imu_cali_t      mag_cali;        //mag cali data


static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; 

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};

//cali data address
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
        (uint32_t *)&mag_cali};


static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, NULL, cali_gyro_hook, cali_accel_hook, cali_mag_hook};

static uint32_t calibrate_systemTick;


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
void calibrate_task(void const *pvParameters)
{
    static uint8_t i = 0;
    


    while (1)
    {

        /* auto_calibrate() 已禁用: 其 boot_once 触发的 GYRO 校准与
         * run_task 的 cali_gyro_accel() 重复, 造成上电多次音效。 */

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
        osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

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
int8_t get_control_temperature(void)
{

    return head_cali.temperature;
}

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
void get_flash_latitude(float *latitude)
{

    if (latitude == NULL)
    {

        return;
    }
    if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
    {
        *latitude = head_cali.latitude;
    }
    else
    {
        *latitude = 22.0f;
    }
}

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
void cali_param_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
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

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {

        //read the data in flash, 
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        
        offset += cali_sensor[i].flash_len * 4;

        //read the name and cali flag,
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
        
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];
        
        offset += CALI_SENSOR_HEAD_LEGHT * 4;

        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
        {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}


/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief 使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;

    /* 只持久化磁力计校准值(CALI_MAG)。
     * 其余(gyro/accel/head/gimbal)段填 0xFF: 重启后不读历史数据,
     * gyro/accel 由上电 1s 自动校准覆盖, 避免历史数据二次叠加。 */
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (i == CALI_MAG)
        {
            //copy the data of device calibration data
            memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
            offset += cali_sensor[i].flash_len * 4;

            //copy the name and "CALI_FLAG" of device
            memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
            offset += CALI_SENSOR_HEAD_LEGHT * 4;
        }
        else
        {
            memset((void *)(flash_write_buf + offset), 0xFF,
                   cali_sensor[i].flash_len * 4 + CALI_SENSOR_HEAD_LEGHT * 4);
            offset += cali_sensor[i].flash_len * 4 + CALI_SENSOR_HEAD_LEGHT * 4;
        }
    }

    //erase the page
    cali_flash_erase(FLASH_USER_ADDR,1);
    //write data
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
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
  * @retval 0: 校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
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
    local_cali_t->temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
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
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
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
  * @retval 0: 校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
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
            gyro_cali_enable_control();
            return 1;
        }
        else
        {
            gyro_cali_disable_control(); //disable the remote control to make robot no move

            return 0;
        }
    }

    return 0;
}

void calibrate_trigger(uint8_t id)
{
    if (id < CALI_LIST_LENGHT && cali_sensor[id].cali_hook != NULL) {
        cali_sensor[id].cali_cmd = 1;
    }
}

/* ---- 同时校准陀螺仪+加速度计 (车须水平静止, 约2.2s) ---- */

void cali_gyro_accel_start(void)
{
    cali_sensor[CALI_GYRO].cali_cmd = 0;
    cali_sensor[CALI_ACC].cali_cmd = 0;
    calibrate_trigger(CALI_GYRO);
    calibrate_trigger(CALI_ACC);
}

bool_t cali_gyro_accel_wait(uint32_t timeout_ms)
{
    uint32_t start = osKernelSysTick();
    while (cali_sensor[CALI_GYRO].cali_cmd || cali_sensor[CALI_ACC].cali_cmd)
    {
        if ((osKernelSysTick() - start) >= timeout_ms)
        {
            return -1;
        }
        osDelay(5);
    }
    buzzer_cali_mag_done();   /* 校准完成: 降调 (run_task/手动触发) */
    return 0;
}

bool_t cali_gyro_accel(uint32_t timeout_ms)
{
    cali_gyro_accel_start();
    return cali_gyro_accel_wait(timeout_ms);
}

/* ---- 磁力计校准 (车须绕Z轴旋转, 约0.5s) ---- */

void cali_mag_start(void)
{
    cali_sensor[CALI_MAG].cali_cmd = 0;
    calibrate_trigger(CALI_MAG);
}

bool_t cali_mag_wait(uint32_t timeout_ms)
{
    uint32_t start = osKernelSysTick();
    while (cali_sensor[CALI_MAG].cali_cmd)
    {
        if ((osKernelSysTick() - start) >= timeout_ms)
        {
            return -1;
        }
        osDelay(5);
    }
    buzzer_cali_mag_done();   /* 磁力计校准完成: 降调 */
    return 0;
}

bool_t cali_mag(uint32_t timeout_ms)
{
    cali_mag_start();
    return cali_mag_wait(timeout_ms);
}

/* ---- Accelerometer calibration ---- */

#define ACCEL_CALI_SAMPLES  200   /* ~200ms at 1ms INS_task rate */

static bool_t cali_accel_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        /* Apply stored offsets if calibrated (x/y→0, z→9.8 basis) */
        INS_set_cali_accel(local_cali_t->scale, local_cali_t->offset);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count = 0;
        static fp32 sum[3] = {0, 0, 0};

        if (count == 0)
        {
            /* 基于原始读数校准: 先清零已有 offset, 避免二次叠加。
             * 否则若历史 offset 错误, 采样到的 az 偏置会被反向放大。 */
            fp32 zero[3] = {0.0f, 0.0f, 0.0f};
            fp32 ones[3] = {1.0f, 1.0f, 1.0f};
            INS_set_cali_accel(ones, zero);
        }

        const fp32 *accel = get_accel_data_point();
        sum[0] += accel[0];
        sum[1] += accel[1];
        sum[2] += accel[2];
        count++;

        if (count >= ACCEL_CALI_SAMPLES)
        {
            /* Expected: ax=0, ay=0, az=+9.8 on level surface */
            local_cali_t->offset[0] = -sum[0] / count;
            local_cali_t->offset[1] = -sum[1] / count;
            local_cali_t->offset[2] = -(sum[2] / count - 9.8f);
            local_cali_t->scale[0] = 1.0f;
            local_cali_t->scale[1] = 1.0f;
            local_cali_t->scale[2] = 1.0f;

            count = 0;
            sum[0] = sum[1] = sum[2] = 0;
            /* 立即生效 */
            INS_set_cali_accel(local_cali_t->scale, local_cali_t->offset);
            return 1;
        }
        else
        {
            buzzer_off();
            return 0;
        }
    }
    return 0;
}

/* ---- Magnetometer calibration ---- */

#define MAG_CALI_SAMPLES  10000  /* ~10s at 1ms calibrate_task rate, user rotates car */

static bool_t cali_mag_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        /* Apply stored hard-iron offsets if calibrated */
        INS_set_cali_mag(local_cali_t->scale, local_cali_t->offset);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count = 0;
        static fp32 min[3] = {9999, 9999, 9999};
        static fp32 max[3] = {-9999, -9999, -9999};

        const fp32 *mag = get_mag_data_point();
        for (int i = 0; i < 3; i++) {
            if (mag[i] < min[i]) min[i] = mag[i];
            if (mag[i] > max[i]) max[i] = mag[i];
        }
        count++;

        if (count >= MAG_CALI_SAMPLES)
        {
            /* Hard-iron offset: center of min/max range */
            local_cali_t->offset[0] = (min[0] + max[0]) / 2.0f;
            local_cali_t->offset[1] = (min[1] + max[1]) / 2.0f;
            local_cali_t->offset[2] = (min[2] + max[2]) / 2.0f;
            /* Soft-iron scale: normalize to average range */
            fp32 avg_range = ((max[0]-min[0]) + (max[1]-min[1]) + (max[2]-min[2])) / 3.0f;
            if (avg_range > 0.1f) {
                local_cali_t->scale[0] = avg_range / (max[0] - min[0]);
                local_cali_t->scale[1] = avg_range / (max[1] - min[1]);
                local_cali_t->scale[2] = avg_range / (max[2] - min[2]);
            } else {
                local_cali_t->scale[0] = local_cali_t->scale[1] = local_cali_t->scale[2] = 1.0f;
            }

            count = 0;
            min[0] = min[1] = min[2] = 9999;
            max[0] = max[1] = max[2] = -9999;
            /* 立即生效 */
            INS_set_cali_mag(local_cali_t->scale, local_cali_t->offset);
            return 1;
        }
        else
        {
            buzzer_off();
            return 0;
        }
    }
    return 0;
}

