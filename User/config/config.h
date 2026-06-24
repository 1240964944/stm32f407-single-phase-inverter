/**
 * @file    config.h
 * @brief   全局配置文件 - 包含所有模块头文件和全局变量声明
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>

/* --- 底层驱动 --- */
#include "sys.h"
#include "led.h"
#include "key.h"
#include "usart.h"
#include "time.h"

/* --- 外设驱动 --- */
#include "pwm.h"
#include "bsp_ad7606.h"
#include "TM1638.h"
#include "oled.h"
#include "dac.h"

/* --- 算法库 --- */
#include "arm_math.h"
#include "global.h"
#include "temporary.h"

/* ========== 全局变量声明 ========== */

/* 按键 */
extern unsigned char key;
extern u16 key_num;

/* 控制参数 */
extern float Uout_ref;          // 目标输出电压 (Vrms)
extern float Iout_ref;          // 目标输出电流 (A)
extern float Uout_ref_F_Hz;     // 目标输出频率 (Hz)
extern float Uout;              // 当前输出电压 (Vrms)
extern float U1, I1;            // 当前电压/电流测量值
extern uint8_t mode;            // 控制模式: 0=开环, 1=闭环

/* 系统状态 */
extern u8 time_num;             // 时基计数器 (0~199, 200=1个周期)
extern float sin_mode1;         // 当前正弦值 (用于SPWM计算)

#endif
