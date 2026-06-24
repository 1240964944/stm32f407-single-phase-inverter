/**
 * @file    led.c
 * @brief   LED初始化 (PC3, 推挽输出, 默认高电平=灭)
 * @note    在TIM7 ISR中用作调试翻转: PCout(3) = !PCout(3)
 */

#include "led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(LED_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure);
    GPIO_SetBits(LED_GPIO_PORT, LED_GPIO_PIN);  // 初始灭
}
