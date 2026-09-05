/**
 * ir_sensor.c
 * 红外传感器设备驱动实现。
 */
#include "ir_sensor.h"
#include "bsp_ir_sensor.h"
#include "detect_task.h"

static ir_sensor_t ir_data;

void ir_sensor_poll(void)
{
    ir_data.front = bsp_ir_front_read();
    ir_data.left  = bsp_ir_left_read();
    ir_data.right = bsp_ir_right_read();
    ir_data.extra = 0;   /* PE14 改作 TIM1-CH4 舵机, 无此传感器 */

    /* 通知看门狗：红外传感器数据到达 */
    detect_hook(IR_SENSOR_TOE);
}

const ir_sensor_t *get_ir_sensor_point(void)
{
    return &ir_data;
}
