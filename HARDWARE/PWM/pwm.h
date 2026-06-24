/**
 * @file    pwm.h
 * @brief   PWM输出模块 - 定时器PWM初始化
 * @note    三个PWM定时器:
 *          TIM5 (GP):  20kHz 三通道 (PA1/PA2/PA3) [当前使用]
 *          TIM8 (ADV): 10kHz 互补+死区 (PC6/PA5/PA6) [备用]
 *          TIM1 (ADV): 10kHz 互补+死区 (PA8/PA7/PA6) [备用]
 */

#ifndef _PWM_H
#define _PWM_H

#include "config.h"

void PWM5_Init(u32 arr, u32 psc);   // TIM5,  通用PWM, 20kHz
void PWM8_Init(u32 arr, u32 psc);   // TIM8,  高级定时器, 互补+死区
void PWM1_Init(u16 per, u16 psc);   // TIM1,  高级定时器, 互补+死区

#endif
