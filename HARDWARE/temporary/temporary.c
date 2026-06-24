/**
 * @file    temporary.c
 * @brief   PWM整流侧输出函数 (备用)
 * @note    单极性PWM输出, 用于整流桥控制
 */

#include "temporary.h"

/**
 * @brief  整流侧PWM输出 (TIM5 CH3/CH4)
 * @param  duty: 占空比 (-1.0 ~ 1.0)
 *         duty > 0: CH4输出, CH3关断 (正向)
 *         duty < 0: CH3输出, CH4关断 (负向)
 * @note   当前未被调用, 为有源整流预留
 */
void PWM_output_zheng_liu_ce(float duty)
{
    if (duty > 0)
    {
        TIM_SetCompare3(TIM5, 0);
        TIM_SetCompare4(TIM5, duty * 4200);
    }
    else
    {
        TIM_SetCompare4(TIM5, 0);
        TIM_SetCompare3(TIM5, -duty * 4200);
    }
}
