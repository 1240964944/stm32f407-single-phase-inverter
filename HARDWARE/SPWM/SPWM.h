/**
 * @file    SPWM.h
 * @brief   SPWM模块 - 基于TIM1高级定时器的正弦PWM方案 (备用)
 * @note    当前实际使用的SPWM在 time.c 中基于TIM5实现
 *          此模块为基于TIM1的升级方案 (支持互补输出+死区)
 *          - 载波频率: 20kHz
 *          - 正弦表: 400点
 *          - 直流母线电压: 400V
 *          - 频率范围: 1~400Hz
 *          - 电压范围: 0~230Vrms
 */

#ifndef __SPWM_H
#define __SPWM_H

/* --- 系统参数定义 --- */
#define DC_BUS_VOLTAGE  400.0f      // 直流母线电压 (V)
#define CARRIER_FREQ    20000       // 载波频率 (Hz)
#define TABLE_SIZE      400         // 正弦表点数
#define Q_SHIFT         24          // 定点数移位 (相位累加器精度)
#define FREQ_SCALE      (1UL << Q_SHIFT)   // 频率计算缩放因子

/* --- 正弦查找表 (400点, 中心值1800, 对应50%占空比) --- */
extern u16 sinData[TABLE_SIZE];

/* --- 函数接口 --- */
void SPWM_Init(void);                           // SPWM初始化 (默认50Hz/220V)
void SPWM_Update(void);                         // SPWM更新 (需在中断中周期调用)
void SPWM_SetVoltage(u16 voltage_V);            // 设置输出电压 (Vrms, 1V步进)
void SPWM_SetFrequency(u16 freq_Hz);            // 设置输出频率 (Hz, 1Hz步进)

#endif
