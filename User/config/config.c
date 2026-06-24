/**
 * @file    config.c
 * @brief   全局变量定义与初始化
 */

#include "config.h"

/* --- 按键 --- */
unsigned char key = 0;
u16 key_num = 0;

/* --- 控制模式: 0=开环(默认), 1=闭环 --- */
uint8_t mode = 0;

/* --- 控制目标值 --- */
float Uout_ref = 25.0f;         // 目标输出电压 25Vrms
float Iout_ref = 2.00f;         // 目标输出电流 2A
float Uout_ref_F_Hz = 50.0f;    // 目标输出频率 50Hz

/* --- 实时测量值 --- */
float Uout = 0.0f;
float U1 = 0.0f, I1 = 0.0f;

/* --- 时基计数 --- */
u8 time_num = 0;

/* --- 当前正弦值 --- */
float sin_mode1 = 0.0f;
