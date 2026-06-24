/**
 * @file    PID.c
 * @brief   标准PID控制器实现
 * @note    PID算法公式 (增量式):
 *            Pout = Kp_P * (Ek - Ek_1)
 *            Iout = Kp_I * Ek * T / Ti
 *            Dout = Kp_D * Td * (Ek - 2*Ek_1 + Ek_2) / T
 *            OUT_Single = Pout + Iout + Dout
 *
 *          输出 = 上次输出 + OUT_Single (带限幅)
 *
 *          WARNING: 函数内部使用static变量, 不适合多实例并发使用.
 *                   推荐使用 global.c 中的 pid_reg_calc() 替代.
 */

#include "PID.h"
#include "config.h"

/* PID实例 (当前仅初始化, 未被ISR调用) */
PID_Struct pid_buck_v;
PID_Struct pid_boost_v;
PID_Struct pid_I;

/**
 * @brief  增量式PID计算
 * @param  pid: PID参数结构体
 * @param  NOW: 当前测量值
 * @param  Target: 目标值
 * @param  SIGNLE_ADD_NUM_LIMIT: 单次增量限幅 (如±100)
 * @param  SUM_OUTPUT_NUM_LIMIT: 累计输出限幅 (如0~100)
 * @return PID输出值
 * @note   WARNING: 内部static变量OUT在所有实例间共享!
 *          多实例调用会导致输出互相干扰.
 */
float PID(PID_Struct *pid, float NOW, float Target,
          float SIGNLE_ADD_NUM_LIMIT, float SUM_OUTPUT_NUM_LIMIT)
{
    static float OUT = 150;  // WARNING: 所有PID实例共享此变量!

    if (mode == 0) OUT = 0;  // 开环模式清零

    pid->Pv = NOW;
    pid->Ek = Target - pid->Pv;

    /* 增量式PID计算 */
    pid->Pout = pid->Kp_P * (pid->Ek - pid->Ek_1);
    pid->Iout = pid->Kp_I * pid->Ek * pid->T / pid->Ti;
    pid->Dout = pid->Kp_D * pid->Td
              * (pid->Ek - pid->Ek_1 - pid->Ek_1 + pid->Ek_2) / pid->T;
    pid->OUT_Single = pid->Pout + pid->Iout + pid->Dout;

    /* 单次增量限幅 */
    if (pid->OUT_Single > SIGNLE_ADD_NUM_LIMIT)
        pid->OUT_Single = SIGNLE_ADD_NUM_LIMIT;
    else if (pid->OUT_Single < -SIGNLE_ADD_NUM_LIMIT)
        pid->OUT_Single = -SIGNLE_ADD_NUM_LIMIT;

    /* 累计输出 */
    OUT += pid->OUT_Single;

    /* 累计输出限幅 */
    if (OUT > SUM_OUTPUT_NUM_LIMIT)
        OUT = SUM_OUTPUT_NUM_LIMIT;
    else if (OUT < 0)
        OUT = 0;

    /* 误差历史更新 */
    pid->Ek_2 = pid->Ek_1;
    pid->Ek_1 = pid->Ek;

    return OUT;
}

/**
 * @brief  初始化Buck电压环PID参数
 * @note   当前未被调用, 参数仅供参考
 */
void init_buck_PID_V(void)
{
    pid_buck_v.Kp_P = 0.01f;
    pid_buck_v.Kp_I = 0.08f;
    pid_buck_v.Kp_D = 0.0f;
    pid_buck_v.T = 1;       // 1ms采样
    pid_buck_v.Td = 2.5f;
    pid_buck_v.Ti = 150.0f;
    pid_buck_v.Pv = 1000.0f;
    pid_buck_v.Ek_2 = 0.0f;
    pid_buck_v.Ek_1 = 0.0f;
    pid_buck_v.Ek = 0.0f;
}

/**
 * @brief  初始化Boost电压环PID参数
 */
void init_boost_PID_V(void)
{
    pid_boost_v.Kp_P = 0.2f;
    pid_boost_v.Kp_I = 1.8f;
    pid_boost_v.Kp_D = 0.0f;
    pid_boost_v.T = 1;
    pid_boost_v.Td = 2.5f;
    pid_boost_v.Ti = 150.0f;
    pid_boost_v.Pv = 1000.0f;
    pid_boost_v.Ek_2 = 0.0f;
    pid_boost_v.Ek_1 = 0.0f;
    pid_boost_v.Ek = 0.0f;
}

/**
 * @brief  初始化电流环PID参数
 */
void init_buck_PID_I(void)
{
    pid_I.Kp_P = 0.0f;
    pid_I.Kp_I = 20.0f;
    pid_I.Kp_D = 0.001f;
    pid_I.T = 1;
    pid_I.Td = 2.5f;
    pid_I.Ti = 150.0f;
    pid_I.Pv = 1000.0f;
    pid_I.Ek_2 = 0.0f;
    pid_I.Ek_1 = 0.0f;
    pid_I.Ek = 0.0f;
}
