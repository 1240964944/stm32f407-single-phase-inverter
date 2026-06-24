/**
 * @file    time.h
 * @brief   定时器模块 - 中断服务与逆变控制核心
 * @note    两个核心定时中断:
 *          TIM7: 0.1ms (10kHz) - 核心控制环 (ADC采样、PI调节、SPWM生成)
 *          TIM3:   5ms (200Hz) - OLED显示刷新
 *
 *          控制模式:
 *          mode=0 (开环): 使用固定调制比 g_inv_open_loop_mod
 *          mode=1 (闭环): 使用PI控制器调节输出电压
 *
 *          电力参数:
 *          - 载波频率: 20kHz (PWM定时器TIM5)
 *          - 控制频率: 10kHz (TIM7中断)
 *          - 最大调制比: 0.98, 最小调制比: 0.5
 *          - 直流母线电压: 40V (实际值, 非高压母线)
 */

#ifndef _TIME_H
#define _TIME_H

#include "config.h"

/* ========== 定时器硬件定义 ========== */
/* 定时器1: 核心控制中断 */
#define BASICTIME1_TIMER                TIM7
#define BASICTIME1_TIMER_CLK            RCC_APB1Periph_TIM7
#define BASICTIME1_TIMER_Channel        TIM7_IRQn
#define BASICTIME1_TIMER_IRQHandler     TIM7_IRQHandler

/* 定时器2: 显示刷新中断 */
#define BASICTIME2_TIMER                TIM3
#define BASICTIME2_TIMER_CLK            RCC_APB1Periph_TIM3
#define BASICTIME2_TIMER_Channel        TIM3_IRQn
#define BASICTIME2_TIMER_IRQHandler     TIM3_IRQHandler

/* ========== AD7606 电压测量开关 ========== */
/*
 * AD7606 模块使能开关
 *   0 = 关闭:  不使用AD7606, 电压值来自开环调制比反算 (纯开环, 无硬件依赖)
 *   1 = 打开:  启用AD7606 FSMC采集 + 霍尔传感器标定 (需接好AD7606硬件)
 *
 * 首次调试建议先用 0, 确认SPWM和按键正常后再改为 1 启用测量
 */
#define AD7606_ENABLE           1

/* ========== 逆变器参数定义 ========== */
#define INV_INT_FREQ            10000.0f    // 控制中断频率 (Hz)
#define INV_MAX_MODULATION      0.98f       // 最大调制比
#define INV_MIN_MODULATION      0.5f        // 最小调制比

/* ========== 全局变量 ========== */
extern uint8_t mode;                        // 控制模式: 0=开环, 1=闭环

/* ========== 函数接口 ========== */

/* 定时器初始化 */
void BasicTIM1_EXTI_Init(u16 arr, u16 psc);     // TIM7初始化
void BasicTIM2_EXTI_Init(u16 arr, u16 psc);     // TIM3初始化

/* 逆变器控制接口 (开环/闭环通用) */
void Inverter_SetFreq(float freq_hz);           // 设置输出频率 (Hz)
void Inverter_SetVoltage(float v_rms);          // 设置输出电压 (Vrms)
void Inverter_SetMode(uint8_t mode_num);        // 切换控制模式

#endif
