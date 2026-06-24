/**
 * @file    global.h
 * @brief   高级控制算法模块 - QPR/PR控制器、SOGI滤波器、SOGI_PLL锁相环 (闭环预留)
 * @note    当前开环模式下这些算法未被调用, 为闭环升级预留
 *          包含:
 *          Part1: 快速排序工具
 *          Part2: QPR (准比例谐振) 控制器 (双线性变换/Tustin离散化)
 *          Part3: PID控制器 (带积分退饱和)
 *          Part4: SOGI (二阶广义积分器) 滤波器
 *          Part5: SOGI_PLL 锁相环 (SOGI + Park变换 + PI)
 */

#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "sys.h"
#include "temporary.h"
#include "arm_math.h"

#define PI_RAD  3.1415926535898f   // 圆周率

/* ========== Part1: 快速排序 ========== */
int qusort(int s[], u32 start, u32 end);

/* ========== Part2: QPR (准比例谐振) 控制器 ========== */
/* 二阶IIR滤波器结构 (Tustin离散化) */
typedef struct {
    float x[4];   // 输入历史 (x[0]当前, x[1]前一次, ...)
    float y[4];   // 输出历史
} Controller_Structure;

/* QPR控制器函数 (基频50Hz) */
void AAL_Control_Controller(Controller_Structure *p, float x);

/* QPR控制器函数 (3次谐波150Hz) */
void AAL_Control_Controller_no3(Controller_Structure *p, float x);

/* ========== Part3: PID控制器 (带积分退饱和) ========== */
typedef struct {
    volatile float  pid_ref_reg;     // 参考输入
    volatile float  pid_fdb_reg;     // 反馈输入
    volatile float  e_reg;           // 误差
    volatile float  Kp_reg;          // 比例增益
    volatile float  up_reg;          // 比例输出
    volatile float  ui_reg;          // 积分输出
    volatile float  ud_reg;          // 微分输出
    volatile float  uprsat_reg;      // 饱和前输出
    volatile float  pid_out_max;     // 输出上限
    volatile float  pid_out_min;     // 输出下限
    volatile float  pid_out_reg;     // PID输出
    volatile float  saterr_reg;      // 饱和误差
    volatile float  Ki_reg;          // 积分增益
    volatile float  Kc_reg;          // 积分修正增益 (退饱和系数)
    volatile float  Kd_reg;          // 微分增益
    volatile float  up1_reg;         // 前次比例输出
} PIDREG;

void pid_reg_calc(PIDREG *v);

/* ========== Part4: SOGI滤波器 ========== */
typedef struct {
    volatile float  SOGI_Un;         // 输入
    volatile float  SOGI_Yn;         // 输出
    volatile float  SOGI_X1n;        // 状态1 (同相分量)
    volatile float  SOGI_X2n;        // 状态2 (正交分量)
    volatile float  SOGI_X1n_1;      // 状态1 (下一拍)
    volatile float  SOGI_X2n_1;      // 状态2 (下一拍)
    volatile float  SOGI_A[2][2];    // 状态矩阵A
    volatile float  SOGI_B[2];       // 输入矩阵B
    volatile float  SOGI_C[2];       // 输出矩阵C
    volatile float  SOGI_D;          // 直通系数D
} SOGI_Filter;

void SOGI_Filter_Calc(SOGI_Filter *p);

/* ========== Part5: SOGI_PLL 锁相环 ========== */
typedef struct {
    volatile float  Value;           // 输入电压
    volatile float  Alpha;           // α轴分量
    volatile float  Beta;            // β轴分量
    volatile float  D;               // D轴分量 (Park变换)
    volatile float  Q;               // Q轴分量
    volatile float  VectorAngle;     // 矢量角度 (-1~1 对应 -π~π)
    volatile float  SinTheta;        // sin值
    volatile float  CosTheta;        // cos值
} SinglePhaseVector;

void SOGI_PLL(SinglePhaseVector *p, SOGI_Filter *v, PIDREG *k);
void Park(SinglePhaseVector *p);                    // Park变换 (αβ -> DQ)
void PERIOD_DEFINE_2PI(float Angle);               // 角度归一化 (-1,1) -> (-pi,pi)

#endif
