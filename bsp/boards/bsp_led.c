#include "bsp_led.h"
#include "main.h"

extern TIM_HandleTypeDef htim5;
/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief 显示RGB
  * @param [in]      aRGB: 0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * @retval         none
  */
void aRGB_led_show(uint32_t aRGB)
{
    uint8_t alpha = (aRGB & 0xFF000000) >> 24;
    uint16_t r = ((aRGB & 0x00FF0000) >> 16) * alpha;
    uint16_t g = ((aRGB & 0x0000FF00) >> 8)  * alpha;
    uint16_t b = ((aRGB & 0x000000FF) >> 0)  * alpha;

    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, b);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, g);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, r);
}


