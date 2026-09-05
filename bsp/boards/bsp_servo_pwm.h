#ifndef BSP_SERVO_PWM_H
#define BSP_SERVO_PWM_H

#include "struct_typedef.h"

/**
 * @brief  Set PWM compare value for servo channel (50Hz).
 * @param  pwm: compare value (0~20000)
 * @param  i:   channel index (0=CH4/PE14, 1=CH1/PC6, 2=CH2/PI6, 3=CH3/PI7)
 */
extern void servo_pwm_set(uint16_t pwm, uint8_t i);

#endif
