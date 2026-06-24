/**
 * @file    time.c
 * @brief   逆变器核心控制 - 定时中断服务程序
 *
 * @note    中断架构:
 *          TIM7 ISR (10kHz, 最高优先级):
 *            1. 读取AD7606 ADC数据 (输出电压Uout)
 *            2. 闭环模式下执行PI电压调节
 *            3. 软件DDS正弦波生成 (arm_sin_f32)
 *            4. SPWM调制输出 (TIM5 CH3/CH4, 单极性调制)
 *            5. 时基计数器更新
 *
 *          TIM3 ISR (200Hz):
 *            1. OLED实时数据显示 (mode, Uout, Uout_ref, Uout_F_Hz)
 *
 *          SPWM调制策略 (单极性):
 *            sin > 0: CH3=0, CH4=sin*4199*modulation (正向)
 *            sin < 0: CH4=0, CH3=|sin|*4199*modulation (负向)
 *
 *          PI控制器 (闭环模式):
 *            err = Uout_ref - Uout
 *            out = kp*err + ki*integral(err)
 *            输出限幅: [pid_mode1_out_min, pid_mode1_out_max]
 *            积分分离+退饱和
 */

#include "time.h"
#include "arm_math.h"

/* ========== 内部静态变量 ========== */
static float s_phase_acc = 0.0f;                // DDS相位累加器
static float s_phase_step = 0.01f * PI;         // 相位步进 (默认50Hz, 与10kHz中断匹配)
#define INV_BUS_VOLTAGE     40.0f                // 直流母线电压 (V) - 实验平台低压侧

/* ========== 霍尔传感器标定 ========== */
/*
 * 电压霍尔传感器标定系数
 *
 * 数据链路:
 *   实际电压 → [霍尔传感器] → 弱电电压 → [AD7606 ±5V量程] → 16位有符号数 → 软件换算
 *
 * 换算公式:
 *   实际电压 = AD7606引脚电压 × HALL_V_SCALE
 *
 * 标定方法:
 *   1. 用万用表测量实际交流电压有效值 V_actual
 *   2. 读串口或OLED上显示的 Uout (即AD7606引脚电压)
 *   3. HALL_V_SCALE = V_actual / Uout
 *
 * 示例 (基于LV25-P, 原边40kΩ, 副边100Ω):
 *   原边400V → 原边电流10mA → 副边电流10mA (1:1) → 副边电压1V
 *   HALL_V_SCALE = 400 / 1 = 400
 *
 * 当前平台 (低压实验, 母线40V, 输出电压0~28Vrms):
 *   请联系你的霍尔传感器规格和分压电阻计算此系数
 *   若不确定, 先用默认值 1.0 (即不缩放), 然后实测校准
 */
#define HALL_V_SCALE        160.0f     // ← 根据实际霍尔传感器修改此值!

/* ========== 电压测量 - 交流RMS ========== */
/*
 * 交流有效值测量 (平方-低通-开方):
 *   1. 瞬时值 = ADC读数 × 5V / 32767 × HALL_V_SCALE
 *   2. 平方后一阶低通滤波 (均值)
 *   3. 开方得真有效值: Vrms = sqrt(mean(v²))
 *
 * 霍尔传感器输出交流波形 (跟随被测电压), AD7606采集瞬时值,
 * 软件算RMS后得到稳定的有效值, 用于显示和PI反馈.
 *
 * α 自适应 = 频率/10000, 保证各种频率下时间常数 ≈ 1个周期
 * 限幅: [0.001, 0.5] 对应 0.1Hz~5000Hz 范围
 */
static float s_uout_sq_filtered = 0.0f;       // v²低通滤波值 (RMS计算用)
static float s_uout_display    = 0.0f;        // 显示用慢速滤波值 (减少跳变)

/* ========== 全局控制变量 ========== */
extern uint8_t mode;                            // 控制模式: 0=开环, 1=闭环
float g_inv_open_loop_mod = 0.6f;               // 开环调制比 (默认0.6)

/* PI控制器变量 */
float err_mode1, err_mode1_all = 0;             // 误差, 误差积分
float out_put_mode_1 = 0.6f;                    // PI输出
float kp_mode1 = 0.0025f, ki_mode1 = 0.0001f;   // PI参数
float pid_mode1_out_max = 0.98f, pid_mode1_out_min = 0.5f;  // 输出限幅

/* 外部变量引用 */
extern float Uout_ref;
extern float Uout;
extern float sin_mode1;
extern u8 time_num;
extern float Uout_ref_F_Hz;

/* SOGI_PLL相关变量 (闭环预留, 当前未使用) */
SinglePhaseVector Vgrid = { 0.0f };
SOGI_Filter SOGI_Single = SOGI_FILTRE_50HZ_INIT;
PIDREG pid_SPLL = SOGI_PID_INIT;
float flo_cos = 1.0f;

/* ========== 定时器初始化函数 ========== */

/**
 * @brief  初始化核心控制定时器 TIM7 (10kHz中断)
 * @param  arr: 自动重装载值 (8399 -> 0.1ms)
 * @param  psc: 预分频值 (0 -> 不分频)
 * @note   时钟源: APB1 = 42MHz, TIM_CLK = 84MHz (APB1 x2)
 *         T = (psc+1)*(arr+1) / 84MHz = 1*8400/84M = 0.1ms
 */
void BasicTIM1_EXTI_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(BASICTIME1_TIMER_CLK, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(BASICTIME1_TIMER, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(BASICTIME1_TIMER, TIM_IT_Update, ENABLE);
    TIM_Cmd(BASICTIME1_TIMER, ENABLE);

    /* 最高优先级 */
    NVIC_InitStructure.NVIC_IRQChannel = BASICTIME1_TIMER_Channel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  初始化显示刷新定时器 TIM3 (5ms中断)
 * @param  arr: 自动重装载值 (8399)
 * @param  psc: 预分频值 (99 -> 100分频)
 * @note   时钟源: APB1 = 42MHz, TIM_CLK = 84MHz
 *         T = (99+1)*(8399+1) / 84MHz = 840000/84M = 10ms...
 *         实际: 100*8400/84M = 10ms (但注释说5ms)
 *         TODO: 验证实际中断周期
 */
void BasicTIM2_EXTI_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(BASICTIME2_TIMER_CLK, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(BASICTIME2_TIMER, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(BASICTIME2_TIMER, TIM_IT_Update, ENABLE);
    TIM_Cmd(BASICTIME2_TIMER, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = BASICTIME2_TIMER_Channel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* ========== 逆变器控制接口 ========== */

/**
 * @brief  设置输出频率
 * @param  freq_hz: 目标频率 (10~2000Hz)
 * @note   通过改变DDS相位步进实现
 */
void Inverter_SetFreq(float freq_hz)
{
    if (freq_hz < 10.0f)  freq_hz = 10.0f;
    if (freq_hz > 2000.0f) freq_hz = 2000.0f;

    s_phase_step = (2.0f * PI * freq_hz) / INV_INT_FREQ;
}

/**
 * @brief  设置输出电压
 * @param  v_rms: 目标电压有效值 (V)
 * @note   开环模式: 直接设置调制比
 *         闭环模式: 设置PI目标值
 */
void Inverter_SetVoltage(float v_rms)
{
    float max_v = INV_MAX_MODULATION * INV_BUS_VOLTAGE / 1.41421356f;
    float min_v = INV_MIN_MODULATION * INV_BUS_VOLTAGE / 1.41421356f;

    if (v_rms > max_v) v_rms = max_v;
    if (v_rms < min_v) v_rms = min_v;

    if (mode == 1)
    {
        Uout_ref = v_rms;   // 闭环模式: 更新PI参考值
    }
    else
    {
        float mod = v_rms * 1.41421356f / INV_BUS_VOLTAGE;
        g_inv_open_loop_mod = mod;  // 开环模式: 直接计算调制比
    }
}

/**
 * @brief  切换控制模式
 * @param  mode_num: 0=开环, 1=闭环
 * @note   切换时同步调制比, 避免输出电压突变
 */
void Inverter_SetMode(uint8_t mode_num)
{
    if (mode == mode_num) return;

    if (mode_num == 0)
    {
        /* 闭环→开环: 用当前PI输出初始化开环调制比 */
        g_inv_open_loop_mod = out_put_mode_1;
    }
    /* 开环→闭环: PI自动从当前状态开始调节 */

    mode = mode_num;
}

/* ========== 中断服务程序 ========== */

/**
 * @brief  TIM7中断服务程序 (10kHz) - 核心控制环
 *
 * 执行流程:
 *  1. 读取AD7606当前ADC值
 *  2. 闭环模式下执行PI电压调节
 *  3. DDS相位累加 -> 正弦波计算
 *  4. 单极性SPWM输出 (TIM5 CH3/CH4)
 *  5. 时基计数器更新
 *  6. 调试翻转 (PC3)
 */
void BASICTIME1_TIMER_IRQHandler(void)
{
    if (TIM_GetITStatus(BASICTIME1_TIMER, TIM_IT_Update) == SET)
    {
        /* ===== 第1步: 读取/估算输出电压 ===== */
#if AD7606_ENABLE
        /*
         * AD7606 数据链路 (已启用硬件采集):
         *   霍尔传感器输出 → AD7606 CH2 (±5V) → 16位有符号数(int16_t) → 电压值
         *
         * sNowAdc[1]: 范围 -32768~+32767, 对应 AD7606 输入引脚 ±5V
         */
        AD7606_StartConvst();           // 拉高CONVST, 启动8通道同步转换
        delay_us(8);                    // 等待转换完成 (无过采样时4μs)
        AD7606_ReadNowAdc();            // 通过FSMC读取8通道数据

        /* 交流RMS (频率自适应α + 显示平滑) */
        {
            float v_inst = g_tAD7606.sNowAdc[0] * 5.0f / 32767.0f * HALL_V_SCALE;
            float v_sq = v_inst * v_inst;

            /* α = 频率/10000, 保证各频率下均为1周期时间常数 */
            float rms_alpha = Uout_ref_F_Hz / 10000.0f;
            if (rms_alpha > 0.5f)  rms_alpha = 0.5f;
            if (rms_alpha < 0.001f) rms_alpha = 0.001f;

            s_uout_sq_filtered = rms_alpha * v_sq
                               + (1.0f - rms_alpha) * s_uout_sq_filtered;
            Uout = sqrtf(s_uout_sq_filtered);

            /* 显示用慢速滤波: α=1/80, 时间常数~8ms, 消除小数点跳变 */
            s_uout_display = (1.0f / 80.0f) * Uout
                           + (79.0f / 80.0f) * s_uout_display;
        }
#else
        /*
         * 无AD7606硬件, 根据当前控制模式反算显示的Uout:
         *   开环: Uout = g_inv_open_loop_mod × Vbus / √2
         *   闭环: Uout = out_put_mode_1 × Vbus / √2  (PI实际输出)
         *
         * 注意: 闭环模式需要 AD7606 反馈才能正常工作,
         *       无AD7606时PI会迅速饱和到上限或下限
         */
        {
            float mod;
            if (mode == 1)
                mod = out_put_mode_1;        // 闭环: PI实际调制输出
            else
                mod = g_inv_open_loop_mod;   // 开环: 用户设定的调制比

            Uout = mod * INV_BUS_VOLTAGE / 1.41421356f;
        }
#endif

        /* ===== 第2步: 闭环PI电压调节 =====
         * Uout 在此处有两个用途:
         *   用途1 (闭环): 作为PI控制器的反馈量, err = Uout_ref - Uout
         *                 (需 AD7606_ENABLE=1 才有实际反馈)
         *   用途2 (显示): TIM3 ISR中OLED显示的就是这个 Uout
         */
        if (mode == 1)
        {
            err_mode1 = Uout_ref - Uout;
            err_mode1_all = err_mode1_all + err_mode1;
            out_put_mode_1 = kp_mode1 * err_mode1 + ki_mode1 * err_mode1_all;

            /* PI输出限幅 + 积分退饱和 */
            if (out_put_mode_1 >= pid_mode1_out_max)
            {
                out_put_mode_1 = pid_mode1_out_max;
                err_mode1_all = pid_mode1_out_max / ki_mode1;
            }
            else if (out_put_mode_1 <= pid_mode1_out_min)
            {
                out_put_mode_1 = pid_mode1_out_min;
                err_mode1_all = pid_mode1_out_min / ki_mode1;
            }
        }

        /* ===== 第3步: DDS正弦波生成 ===== */
        s_phase_acc += s_phase_step;
        if (s_phase_acc >= 2.0f * PI)
        {
            s_phase_acc -= 2.0f * PI;
        }
        sin_mode1 = arm_sin_f32(s_phase_acc);

        /* ===== 第4步: 选择调制比并输出SPWM ===== */
        float modulation;
        if (mode == 1)
        {
            modulation = out_put_mode_1;    // 闭环: PI输出
        }
        else
        {
            modulation = g_inv_open_loop_mod;  // 开环: 固定调制比
        }

        /* 单极性SPWM调制 (TIM5: CH3=PA2, CH4=PA3) */
        if (sin_mode1 > 0.0f)
        {
            /* 正半周: CH4输出SPWM, CH3关闭 */
            TIM_SetCompare3(TIM5, 0);
            TIM_SetCompare4(TIM5, (uint32_t)(sin_mode1 * 4199.0f * modulation));
        }
        else
        {
            /* 负半周: CH3输出SPWM, CH4关闭 */
            TIM_SetCompare4(TIM5, 0);
            TIM_SetCompare3(TIM5, (uint32_t)(-sin_mode1 * 4199.0f * modulation));
        }

        /* ===== 第5步: 时基计数器 (200次=1个显示周期) ===== */
        time_num++;
        if (time_num == 200) time_num = 0;

        /* ===== 第6步: 调试翻转 ===== */
        PCout(3) = !PCout(3);
    }
    TIM_ClearITPendingBit(BASICTIME1_TIMER, TIM_IT_Update);
}

/* OLED显示缓冲区索引 */
u8 usart_show_data_num = 0;

/**
 * @brief  TIM3中断服务程序 (200Hz) - 显示刷新
 */
void BASICTIME2_TIMER_IRQHandler(void)
{
    if (TIM_GetITStatus(BASICTIME2_TIMER, TIM_IT_Update) == SET)
    {
        /* OLED实时数据显示 */
        OLED_ShowString(0, 0, "mode      ", OLED_6X8);
        OLED_ShowString(0, 16, "Uout      ", OLED_6X8);
        OLED_ShowString(0, 32, "Uout_ref  ", OLED_6X8);
        OLED_ShowString(0, 48, "Uout_F_Hz ", OLED_6X8);

        OLED_ShowFloatNum(80, 0, mode, 1, 0, OLED_6X8);
        OLED_ShowFloatNum(80, 16, s_uout_display, 3, 2, OLED_6X8);
        OLED_ShowFloatNum(80, 32, Uout_ref, 3, 2, OLED_6X8);
        OLED_ShowFloatNum(80, 48, Uout_ref_F_Hz, 3, 2, OLED_6X8);
        OLED_Update();
    }
    TIM_ClearITPendingBit(BASICTIME2_TIMER, TIM_IT_Update);
}
