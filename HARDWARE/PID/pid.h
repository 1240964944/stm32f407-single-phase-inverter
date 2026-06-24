/**
 * @file    PID.h
 * @brief   标准PID控制器模块
 * @note    提供位置式PID算法, 支持独立参数配置和多实例
 *          PID结构体:
 *            Kp_P: 比例系数 (作用于误差增量)
 *            Kp_I: 积分系数
 *            Kp_D: 微分系数
 *            T:    采样周期 (ms)
 *            Ti:   积分时间常数
 *            Td:   微分时间常数
 *
 *          WARNING: PID()函数内部使用static变量存储累计输出,
 *                   当前实现存在多实例共享状态问题, 建议仅单实例使用
 *                   闭环应用建议使用 global.c 中的 pid_reg_calc()
 */

#ifndef _PID_H
#define _PID_H

#include "stm32f4xx.h"
#include "config.h"

typedef struct
{
    float Sv;           // 设定值
    float Pv;           // 过程值 (反馈)
    float Ek_1;         // 前次误差
    float Ek_2;         // 前两次误差
    float Ek;           // 当前误差
    float T;            // 采样周期
    float Ti;           // 积分时间常数
    float Td;           // 微分时间常数
    float Kp_P;         // 比例系数
    float Kp_I;         // 积分系数
    float Kp_D;         // 微分系数
    float OUT_Single;   // 单次增量
    float Pout;         // 比例项输出
    float Iout;         // 积分项输出
    float Dout;         // 微分项输出
} PID_Struct;

/* PID计算函数 */
float PID(PID_Struct *pid, float NOW, float Target,
          float SIGNLE_ADD_NUM_LIMIT, float SUM_OUTPUT_NUM_LIMIT);

/* PID初始化函数 */
void init_buck_PID_V(void);     // Buck电压环
void init_boost_PID_V(void);    // Boost电压环
void init_buck_PID_I(void);     // 电流环

#endif
