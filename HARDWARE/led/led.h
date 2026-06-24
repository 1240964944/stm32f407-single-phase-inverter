/**
 * @file    led.h
 * @brief   LED指示灯模块 (PC3)
 */

#ifndef __LED_H
#define __LED_H

#include "config.h"

#define LED_GPIO_CLK    RCC_AHB1Periph_GPIOC
#define LED_GPIO_PORT   GPIOC
#define LED_GPIO_PIN    GPIO_Pin_3

void LED_Init(void);

#endif
