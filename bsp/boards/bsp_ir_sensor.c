/**
 * bsp_ir_sensor.c
 * 红外传感器 BSP 实现 — 3 路 GPIO 读取。
 */
#include "bsp_ir_sensor.h"
#include "main.h"

uint8_t bsp_ir_front_read(void)
{
    return (HAL_GPIO_ReadPin(IR_FRONT_PORT, IR_FRONT_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

uint8_t bsp_ir_left_read(void)
{
    return (HAL_GPIO_ReadPin(IR_LEFT_PORT, IR_LEFT_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

uint8_t bsp_ir_right_read(void)
{
    return (HAL_GPIO_ReadPin(IR_RIGHT_PORT, IR_RIGHT_PIN) == GPIO_PIN_SET) ? 1 : 0;
}
