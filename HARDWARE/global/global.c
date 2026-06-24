/**
 * @file    global.c
 * @brief   高级控制算法实现 - QPR、SOGI、SOGI_PLL (闭环预留)
 * @note    当前开环模式未使用, 为闭环电压/电流控制预留
 */

#include "global.h"

/* ========== Part1: 快速排序 ========== */
/* 注意: s[0]作为基准值暂存位, 实际数据从s[1]开始 */

/**
 * @brief  快速排序 (递归实现)
 * @param  s: 待排序数组 (s[0]为暂存位)
 * @param  start: 起始索引
 * @param  end: 结束索引
 */
int qusort(int s[], u32 start, u32 end)
{
    int i, j;
    i = start;
    j = end;
    s[0] = s[start];    // 保存基准值到s[0]

    while (i < j)
    {
        while (i < j && s[0] < s[j]) j--;
        if (i < j)
        {
            s[i] = s[j];
            i++;
        }
        while (i < j && s[i] <= s[0]) i++;
        if (i < j)
        {
            s[j] = s[i];
            j--;
        }
    }
    s[i] = s[0];  // 基准值归位

    if (start < i) qusort(s, start, j - 1);
    if (i < end)  qusort(s, j + 1, end);
    return 0;
}

/* ========== Part2: QPR (准比例谐振) 控制器 ========== */
/* 基频50Hz QPR - Tustin离散化系数 */
/* 传递函数: G(s) = Kp + Kr*ωc*s / (s^2 + ωc*s + ω0^2) */
float cy1 = (2.0f * PR_Ts * PR_Ts * PR_w_0 * PR_w_0 - 8.0f)
          / (PR_Ts * PR_Ts * PR_w_0 * PR_w_0 + 4.0f * PR_w_c * PR_Ts + 4.0f);
float cy2 = (PR_Ts * PR_Ts * PR_w_0 * PR_w_0 + 4.0f - 4.0f * PR_Ts * PR_w_c)
          / (PR_Ts * PR_Ts * PR_w_0 * PR_w_0 + 4.0f * PR_w_c * PR_Ts + 4.0f);
float cx0 = (4.0f * PR_Kp + PR_Ts * PR_Ts * PR_Kp * PR_w_0 * PR_w_0
          + 4.0f * PR_Ts * PR_Kp * PR_w_c + 4.0f * PR_Ts * PR_Kr * PR_w_c)
          / (PR_Ts * PR_Ts * PR_w_0 * PR_w_0 + 4.0f * PR_w_c * PR_Ts + 4.0f);
float cx1 = (2.0f * PR_Kp * PR_Ts * PR_Ts * PR_w_0 * PR_w_0 - 8.0f * PR_Kp)
          / (PR_Ts * PR_Ts * PR_w_0 * PR_w_0 + 4.0f * PR_w_c * PR_Ts + 4.0f);
float cx2 = (4.0f * PR_Kp + PR_Ts * PR_Ts * PR_Kp * PR_w_0 * PR_w_0
          - 4.0f * PR_Ts * PR_Kp * PR_w_c - 4.0f * PR_Ts * PR_Kr * PR_w_c)
          / (PR_Ts * PR_Ts * PR_w_0 * PR_w_0 + 4.0f * PR_w_c * PR_Ts + 4.0f);

/**
 * @brief  基频QPR控制器 (50Hz) - 差分方程计算
 *         y[n] = -cy1*y[n-1] - cy2*y[n-2] + cx0*x[n] + cx1*x[n-1] + cx2*x[n-2]
 */
void AAL_Control_Controller(Controller_Structure *p, float x)
{
    p->x[0] = x;

    /* Tustin离散化PR差分方程 */
    p->y[0] = -(cy1) * (p->y[1]) - cy2 * (p->y[2])
            + cx0 * (p->x[0]) + cx1 * (p->x[1]) + cx2 * (p->x[2]);

    /* 输出限幅 */
    if (p->y[0] > PR_Max) p->y[0] = PR_Max;
    if (p->y[0] < PR_Min) p->y[0] = PR_Min;

    /* 历史数据移位 */
    p->x[3] = p->x[2];
    p->x[2] = p->x[1];
    p->x[1] = p->x[0];
    p->y[3] = p->y[2];
    p->y[2] = p->y[1];
    p->y[1] = p->y[0];
}

/* 3次谐波150Hz QPR - Tustin离散化系数 */
float cy1_3 = (2.0f * PR_Ts_3 * PR_Ts_3 * PR_w_0_3 * PR_w_0_3 - 8.0f)
            / (PR_Ts_3 * PR_Ts_3 * PR_w_0_3 * PR_w_0_3 + 4.0f * PR_w_c_3 * PR_Ts_3 + 4.0f);
float cy2_3 = (PR_Ts_3 * PR_Ts_3 * PR_w_0_3 * PR_w_0_3 + 4.0f - 4.0f * PR_Ts_3 * PR_w_c_3)
            / (PR_Ts_3 * PR_Ts_3 * PR_w_0_3 * PR_w_0_3 + 4.0f * PR_w_c_3 * PR_Ts_3 + 4.0f);
float cx0_3 = (4.0f * PR_Kp_3 + PR_Ts_3 * PR_Ts_3 * PR_Kp_3 * PR_w_0_3 * PR_w_0_3
            + 4.0f * PR_Ts_3 * PR_Kp_3 * PR_w_c_3 + 4.0f * PR_Ts_3 * PR_Kr_3 * PR_w_c_3)
            / (PR_Ts_3 * PR_Ts_3 * PR_w_0_3 * PR_w_0_3 + 4.0f * PR_w_c_3 * PR_Ts_3 + 4.0f);
float cx1_3 = (2.0f * PR_Kp_3 * PR_Ts_3 * PR_Ts_3 * PR_w_0_3 * PR_w_0_3 - 8.0f * PR_Kp_3)
            / (PR_Ts_3 * PR_Ts_3 * PR_w_0_3 * PR_w_0_3 + 4.0f * PR_w_c_3 * PR_Ts_3 + 4.0f);
float cx2_3 = (4.0f * PR_Kp_3 + PR_Ts_3 * PR_Ts_3 * PR_Kp_3 * PR_w_0_3 * PR_w_0_3
            - 4.0f * PR_Ts_3 * PR_Kp_3 * PR_w_c_3 - 4.0f * PR_Ts_3 * PR_Kr_3 * PR_w_c_3)
            / (PR_Ts_3 * PR_Ts_3 * PR_w_0_3 * PR_w_0_3 + 4.0f * PR_w_c_3 * PR_Ts_3 + 4.0f);

/**
 * @brief  3次谐波QPR控制器 (150Hz) - 差分方程计算
 */
void AAL_Control_Controller_no3(Controller_Structure *p, float x)
{
    p->x[0] = x;

    /* Tustin离散化PR差分方程 */
    p->y[0] = -(cy1_3) * (p->y[1]) - cy2_3 * (p->y[2])
            + cx0_3 * (p->x[0]) + cx1_3 * (p->x[1]) + cx2_3 * (p->x[2]);

    /* 输出限幅 */
    if (p->y[0] > PR_Max) p->y[0] = PR_Max;
    if (p->y[0] < PR_Min) p->y[0] = PR_Min;

    /* 历史数据移位 */
    p->x[3] = p->x[2];
    p->x[2] = p->x[1];
    p->x[1] = p->x[0];
    p->y[3] = p->y[2];
    p->y[2] = p->y[1];
    p->y[1] = p->y[0];
}

/* ========== Part4: SOGI滤波器 ========== */

/**
 * @brief  SOGI (二阶广义积分器) 滤波器状态更新
 * @note   用于提取电网电压的基波分量及其正交分量
 *         离散状态空间方程 (前向欧拉):
 *           X[n+1] = A * X[n] + B * U[n]
 *           Y[n]   = C * X[n] + D * U[n]
 */
void SOGI_Filter_Calc(SOGI_Filter *p)
{
    /* 计算输出 */
    p->SOGI_Yn = p->SOGI_C[0] * p->SOGI_X1n
               + p->SOGI_C[1] * p->SOGI_X2n
               + p->SOGI_D * p->SOGI_Un;

    /* 计算下一拍状态 */
    p->SOGI_X1n_1 = p->SOGI_A[0][0] * p->SOGI_X1n
                  + p->SOGI_A[0][1] * p->SOGI_X2n
                  + p->SOGI_B[0] * p->SOGI_Un;
    p->SOGI_X2n_1 = p->SOGI_A[1][0] * p->SOGI_X1n
                  + p->SOGI_A[1][1] * p->SOGI_X2n
                  + p->SOGI_B[1] * p->SOGI_Un;

    /* 状态更新 */
    p->SOGI_X1n = p->SOGI_X1n_1;
    p->SOGI_X2n = p->SOGI_X2n_1;
}

/* ========== Part3: PID控制器 (带积分退饱和) ========== */

/**
 * @brief  增量式PID计算 (位置式 + 积分退饱和)
 * @note   退饱和机制: 当输出达到限幅值时, 自动调整积分项防止饱和
 */
void pid_reg_calc(PIDREG *v)
{
    /* 误差计算 */
    v->e_reg = v->pid_ref_reg - v->pid_fdb_reg;

    /* 比例项 */
    v->up_reg = v->Kp_reg * v->e_reg;

    /* 饱和前输出 = 比例 + 积分 */
    v->uprsat_reg = v->up_reg + v->ui_reg;

    /* 输出限幅 */
    if (v->uprsat_reg > v->pid_out_max)
        v->pid_out_reg = v->pid_out_max;
    else if (v->uprsat_reg < v->pid_out_min)
        v->pid_out_reg = v->pid_out_min;
    else
        v->pid_out_reg = v->uprsat_reg;

    /* 计算饱和误差 */
    v->saterr_reg = v->pid_out_reg - v->uprsat_reg;

    /* 积分更新 (带退饱和修正) */
    v->ui_reg = v->ui_reg + v->Ki_reg * v->up_reg + v->Kc_reg * v->saterr_reg;
}

/* ========== Part5: SOGI_PLL 锁相环 ========== */

/**
 * @brief  Park变换 (αβ -> DQ)
 *         D = α*cos(θ) + β*sin(θ)
 *         Q = β*cos(θ) - α*sin(θ)
 */
void Park(SinglePhaseVector *p)
{
    p->D = p->Alpha * p->CosTheta + p->Beta * p->SinTheta;
    p->Q = p->Beta * p->CosTheta - p->Alpha * p->SinTheta;
}

/**
 * @brief  角度归一化到 (-1, 1) 范围 (对应 -π ~ π)
 */
void PERIOD_DEFINE_2PI(float Angle)
{
    if (Angle > 1.0f)
        Angle = Angle - 2.0f;
    else if (Angle < -1.0f)
        Angle = Angle + 2.0f;
}

/**
 * @brief  SOGI_PLL 锁相环主函数
 * @note   流程: 输入电压 -> SOGI滤波(αβ分量) -> Park变换(DQ) -> PI调节Q->0 -> 锁相
 *         锁定时: Q=0, D=电压幅值, VectorAngle=电网相位
 */
void SOGI_PLL(SinglePhaseVector *p, SOGI_Filter *v, PIDREG *k)
{
    /* 计算sin/cos */
    p->SinTheta = arm_sin_f32(p->VectorAngle * PI_RAD);
    p->CosTheta = arm_cos_f32(p->VectorAngle * PI_RAD);

    /* 输入电压送入SOGI滤波器 */
    v->SOGI_Un = p->Value;
    SOGI_Filter_Calc(v);

    /* 提取αβ分量 (β超前α 90°) */
    p->Alpha = v->SOGI_X2n;   // α分量 (与输入同相)
    p->Beta  = v->SOGI_X1n;   // β分量 (正交)

    /* Park变换 (αβ -> DQ) */
    Park(p);

    /* PI控制器调节Q轴分量为0 (锁相目标) */
    k->pid_fdb_reg = 0;       // 参考: Q=0
    k->pid_ref_reg = p->Q;    // 反馈: 当前Q值
    pid_reg_calc(k);

    /* 更新相位角 */
    p->VectorAngle = p->VectorAngle + k->pid_out_reg;

    /* 角度归一化 */
    PERIOD_DEFINE_2PI(p->VectorAngle);
}
