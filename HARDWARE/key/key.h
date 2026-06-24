/**
 * @file    key.h
 * @brief   按键模块 (5键, 外部中断方式)
 * @note    按键映射:
 *          KEY1 - PA0 (EXTI0): 模式切换 (开环/闭环)
 *          KEY2 - PE4 (EXTI4): 电压+
 *          KEY3 - PE2 (EXTI2): 电压-
 *          KEY4 - PE3 (EXTI3): 频率+
 *          KEY5 - PE1 (EXTI1): 预留
 */

#ifndef _KEY_H
#define _KEY_H

#include "config.h"

/* 按键电平定义 (上拉输入, 按下为低电平) */
#define KEY_ON  1
#define KEY_OFF 0

/* 引脚读取宏 */
#define PE4     GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)
#define PE3     GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)
#define PA0     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)

/* 全局按键值 (1~5) */
extern uint8_t Key;

/* 函数接口 */
void Key_Init(void);
uint8_t Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif
