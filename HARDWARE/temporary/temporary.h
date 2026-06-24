/**
 * @file    temporary.h
 * @brief   高级控制算法参数定义 (SOGI_PLL, PR控制器)
 * @note    包含:
 *          - SOGI_PLL 锁相环PI参数
 *          - 电压环PI参数
 *          - PR/QPR谐振控制器参数 (基频50Hz + 3次谐波150Hz)
 *          - SOGI滤波器系数 (50Hz)  [10kHz采样率]
 *          - PWM整流侧输出函数
 */

#ifndef __TEMPORARY_H
#define __TEMPORARY_H

#include "sys.h"
#include "global.h"
#include "arm_math.h"

/* PLL状态标志 */
#define PLL_IS_SET  1
#define PLL_NO_SET  0

/* ========== SOGI_PLL 锁相环PI参数 ========== */
#define SPLL_KP         0.2f
#define SPLL_KI         0.02f
#define SPLL_OUT_MAX    0.05f
#define SPLL_OUT_MIN   -0.05f
#define SOGI_PID_INIT   {0,0,0,SPLL_KP,0,0,0,0,SPLL_OUT_MAX,SPLL_OUT_MIN,0,0,SPLL_KI,SPLL_KI,0,0}

/* ========== 电压环PI参数 (备用) ========== */
#define VOL_KP          0.6f
#define VOL_KI          0.125f
#define VOL_OUT_MAX     3.0f
#define VOL_OUT_MIN     0.0f
#define VOL_PID_INIT    {0,0,0,VOL_KP,0,0,0,0,VOL_OUT_MAX,VOL_OUT_MIN,0,0,VOL_KI,VOL_KI,0,0}

/* ========== QPR 基频50Hz 谐振控制器参数 (Tustin离散化) ========== */
#define PR_Kr    -50.0f                     // 谐振增益
#define PR_Kp    -0.025f                    // 比例增益
#define PR_w_c    2.0f                      // 截止频率 (rad/s)
#define PR_w_0   (2.0f * 50.0f * PI_RAD)    // 谐振频率 = 100*PI (50Hz)
#define PR_Ts     1.0f / (10000.0f)         // 采样周期 = 10kHz
#define PR_Max    1.0f                      // 输出上限
#define PR_Min   -1.0f                      // 输出下限

/* ========== QPR 3次谐波150Hz 谐振控制器参数 ========== */
#define PR_Kr_3   -28.0f
#define PR_Kp_3   -0.003f
#define PR_w_c_3   20.0f
#define PR_w_0_3  (2.0f * 150.0f * PI_RAD)  // 谐振频率 = 300*PI (150Hz)
#define PR_Ts_3    1.0f / (10000.0f)
#define PR_Max_3   1.0f
#define PR_Min_3  -1.0f

/* ========== SOGI滤波器参数 (50Hz, 10kHz采样) ========== */
#define SOGI_FILTRE_50HZ_INIT { \
    0, 0, 0, 0, 0, 0, \
    { \
        { 0.999513786364320f,   0.0307233098775979f}, \
        {-0.0307233098775979f,  0.956071026197396f } \
    }, \
    { 0.000687506080851944f, 0.0434427601669234f }, \
    { 0, 0 }, \
    0 \
}

/* ========== 函数接口 ========== */
void PWM_output_zheng_liu_ce(float duty);  // 整流侧PWM输出

#endif
