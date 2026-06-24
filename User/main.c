/**
 * @file    main.c
 * @brief   单相逆变器主程序
 * @note    系统时钟 168MHz (HSE 25MHz -> PLL)
 *          APB1 = 42MHz, APB2 = 84MHz
 *          启动文件 startup_stm32f40_41xxx.s 中已调用 SystemInit() 初始化时钟
 *
 * 中断控制架构:
 *   TIM7 (0.1ms / 10kHz): 核心控制环 - ADC读取、PI调节、SPWM生成
 *   TIM3 (5ms / 200Hz):   OLED显示刷新
 *   EXTI0~4:              按键中断 (模式切换、电压/频率调节)
 */

#include "stm32f4xx.h"
#include "config.h"

int main(void)
{
    /* --- 基础外设初始化 --- */
    LED_Init();                 // 状态指示灯 (PC3)
    Debug_USART_Config();       // 调试串口 USART1 (PA9/PA10, 115200bps)
    OLED_Init();                // OLED显示屏 (SPI: PB12~PB15 + PG2)
    OLED_Clear();
    delay_init(168);            // 系统延时初始化 (基于SysTick, 168MHz)
    Key_Init();                 // 5按键初始化 (PA0, PE1~PE4, 外部中断)

    /* --- 测量模块初始化 (必须在定时器之前, 否则TIM7 ISR会访问未初始化的FSMC) --- */
#if AD7606_ENABLE
    bsp_InitAD7606();               // AD7606 8通道ADC (FSMC接口, ±5V量程)
    AD7606_Reset();                 // 硬件复位 (确保上电稳定)
    AD7606_SetOS(AD_OS_NO);         // 不过采样 (单次转换4μs)
    AD7606_SetInputRange(0);        // 量程: 0=±5V, 1=±10V
#endif

    /* --- PWM输出初始化 --- */
    PWM5_Init(4200-1, 1-1);     // TIM5 CH2/3/4 (PA1/PA2/PA3), 20kHz载波
    // 备用PWM (高级定时器, 支持互补输出+死区):
    // PWM8_Init(8400-1, 1-1);  // TIM8  PC6/PA5/PA6  10kHz
    // PWM1_Init(8400-1, 1-1);  // TIM1  PA8/PA7/PA6  10kHz

    /* --- 控制中断初始化 (AD7606就绪后才启动) --- */
    BasicTIM1_EXTI_Init(8399, 0);   // TIM7: 0.1ms (10kHz) 核心控制中断
    BasicTIM2_EXTI_Init(8399, 99);  // TIM3: 5ms (200Hz)   显示刷新中断

    /* --- 备用外设 (已配置, 暂不使用) --- */
    // Dac1_Init();                 // DAC通道1 (PA4)
    // TM1638_Init();               // TM1638数码管显示

    while(1)
    {
        // 主循环空闲, 所有控制逻辑在中断服务程序中完成
    }
}
