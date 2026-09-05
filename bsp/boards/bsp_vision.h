/**
 * bsp_vision.h
 * 视觉模块 BSP 层 — UART 中断接收。
 *
 * 使用 USART3 (DBUS 接口, PC10 TX, PC11 RX)，单字节中断接收 + 环形缓冲区。
 * 模块通过串口发送 ASCII 数字字符 '0'-'9'。
 *
 * 注意: DBUS 接口有外部反向器，会翻转标准 UART 信号极性。
 * 需在外部再加一级反向电路（或用支持反相输出的视觉模块）。
 *
 * === 用户配置区 ===
 * 对照 CubeMX 引脚配置修改以下宏。
 * CubeMX 中: USART3, 异步模式, 波特率匹配视觉模块, 8N1,
 *           关闭硬件流控, 使能 USART3 全局中断。
 */
#ifndef BSP_VISION_H
#define BSP_VISION_H

#include "struct_typedef.h"
#include "usart.h"      /* UART_HandleTypeDef */

/* ================================================================
 * 用户配置区
 * ================================================================ */

/* 视觉模块使用的 UART (CubeMX 生成的句柄名)。
 * 默认 USART3 — DJI C板 DBUS 接口 (PC10 TX, PC11 RX)。
 * 需外部反向器或在 CubeMX 中换用其他 UART。 */
#define VISION_UART          huart3

/* 接收环形缓冲区大小（2 的幂，用于快速取模）。 */
#define VISION_RX_BUF_SIZE   64

/* ================================================================
 * 用户配置区结束
 * ================================================================ */

void bsp_vision_init(void);
uint8_t bsp_vision_getchar(uint8_t *byte);
void bsp_vision_uart_isr(UART_HandleTypeDef *huart);

#endif
