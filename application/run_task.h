/**
 * run_task.h — 技能赛2025 主控任务 (适配 master 底层)
 */
#ifndef RUN_TASK_H
#define RUN_TASK_H
#include "struct_typedef.h"

extern void run_task(void const * argument);
extern fp32 vx_run_set ;
extern fp32 angle_run_set ;
extern fp32 go_yaw,go_pitch,go_yaw_inia ;
extern int16_t v_r,v_l ,V,v_delt;
extern fp32  biass;
extern fp32 angle_sum;
extern fp32 speed_sum;
extern int16_t set_speed_rpm[4];
extern int last_r ;
extern uint8_t data_storage[27];
extern uint8_t hui[16];
extern uint16_t HUI_data;
#endif
