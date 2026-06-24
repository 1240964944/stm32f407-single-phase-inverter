/**
 * @file    dac.h
 * @brief   DAC输出模块 (备用)
 * @note    DAC CH1: PA4,  12位, 用于调试波形输出
 *          DAC CH2: PA5,  12位
 *          当前未使用 (main.c中Dac1_Init已注释)
 */

#ifndef __DAC_H
#define __DAC_H

#include "sys.h"
#include "config.h"

/* DAC1 硬件定义 */
#define DAC_CLK         RCC_APB1Periph_DAC
#define DAC_GPIO_PORT   GPIOA
#define DAC_GPIO_PIN    GPIO_Pin_4
#define DAC_GPIO_CLK    RCC_AHB1Periph_GPIOA
#define DAC_CHANNEL     DAC_Channel_1

void Dac1_Init(void);   // DAC通道1初始化
void Dac2_Init(void);   // DAC通道2初始化

#endif
