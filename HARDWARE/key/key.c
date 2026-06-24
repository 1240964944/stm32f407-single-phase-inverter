/**
 * @file    key.c
 * @brief   按键模块实现
 * @note    5个按键均配置为外部中断 (下降沿触发)
 *          KEY1(PA0): 切换开环/闭环模式
 *          KEY2(PE4): 目标电压+1V
 *          KEY3(PE2): 目标电压-1V
 *          KEY4(PE3): 目标频率+1Hz
 *          KEY5(PE1): 预留
 */

#include "key.h"

uint8_t Key = 10;   // 当前按键值 (1~5), 初始=10表示无按键

/* 外部变量引用 */
extern float Uout_ref;
extern float Uout_ref_F_Hz;
extern uint8_t mode;
extern void Inverter_SetVoltage(float v_rms);
extern void Inverter_SetFreq(float freq_hz);

/**
 * @brief  简单延时 (软件循环)
 */
void Key_Delay(__IO u32 nCount)
{
    for (; nCount != 0; nCount--);
}

/**
 * @brief  按键初始化 (PA0, PE1~PE4 外部中断)
 */
void Key_Init(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    /* PA0 - KEY1 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PE1~PE4 - KEY2~KEY5 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* 配置 EXTI 线映射 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);  // PA0 -> EXTI0
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);  // PE1 -> EXTI1
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);  // PE2 -> EXTI2
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);  // PE3 -> EXTI3
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource4);  // PE4 -> EXTI4

    EXTI_InitTypeDef EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;  // 下降沿触发

    /* EXTI0 (KEY1 - PA0) */
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    EXTI_Init(&EXTI_InitStruct);
    /* EXTI1 (KEY5 - PE1) */
    EXTI_InitStruct.EXTI_Line = EXTI_Line1;
    EXTI_Init(&EXTI_InitStruct);
    /* EXTI2 (KEY3 - PE2) */
    EXTI_InitStruct.EXTI_Line = EXTI_Line2;
    EXTI_Init(&EXTI_InitStruct);
    /* EXTI3 (KEY4 - PE3) */
    EXTI_InitStruct.EXTI_Line = EXTI_Line3;
    EXTI_Init(&EXTI_InitStruct);
    /* EXTI4 (KEY2 - PE4) */
    EXTI_InitStruct.EXTI_Line = EXTI_Line4;
    EXTI_Init(&EXTI_InitStruct);

    /* NVIC配置 (全部最低优先级) */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&NVIC_InitStruct);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&NVIC_InitStruct);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_Init(&NVIC_InitStruct);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_Init(&NVIC_InitStruct);
    NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_Init(&NVIC_InitStruct);
}

/* ========== EXTI中断服务程序 ========== */

/**
 * @brief  KEY1 (PA0) - 模式切换
 */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        Key = 1;
        mode = !mode;                   // 开环/闭环切换
        Key_Delay(100);                 // 消抖
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

/**
 * @brief  KEY5 (PE1) - 预留
 */
void EXTI1_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        Key = 5;
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

/**
 * @brief  KEY3 (PE2) - 电压-
 */
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        Key = 3;
        Uout_ref--;
        Inverter_SetVoltage(Uout_ref);
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

/**
 * @brief  KEY4 (PE3) - 频率+
 */
void EXTI3_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        Key = 4;
        Uout_ref_F_Hz++;
        Inverter_SetFreq(Uout_ref_F_Hz);
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
}

/**
 * @brief  KEY2 (PE4) - 电压+
 */
void EXTI4_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        Key = 2;
        Uout_ref++;
        Inverter_SetVoltage(Uout_ref);
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

/**
 * @brief  按键扫描 (轮询方式, 带松手检测)
 * @retval 0=按下, 1=未按下
 */
uint8_t Key_Scan(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    if (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0)
    {
        while (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == 0);  // 等待释放
        return 0;
    }
    return 1;
}
