# 单相逆变器开环控制项目

## 项目概述

基于 STM32F407ZGTx 的单相逆变器控制系统，支持开环/闭环双模式。
- **开发板**: 野火 STM32F407 开发板
- **IDE**: Keil MDK 5 (Project/ RVMDK(uv5)/BH-F407.uvprojx)
- **库**: STM32F4xx 标准外设库 v1.8.0 + CMSIS + ARM DSP (CMSIS-DSP)
- **工具链**: ARM Compiler 5 (armcc)

## 系统时钟

- HSE 25MHz → PLL → SYSCLK = 168MHz
- AHB1 (HCLK) = 168MHz
- APB2 (PCLK2) = 84MHz (TIM1/8/9/10/11 时钟 = 168MHz, 因为 APB2 prescaler ≠ 1)
- APB1 (PCLK1) = 42MHz (TIM2/3/4/5/6/7/12/13/14 时钟 = 84MHz, 因为 APB1 prescaler ≠ 1)

## 目录结构

```
单相逆变开环/
├── User/                       # 应用层代码
│   ├── main.c                  # 主程序入口
│   ├── config/config.h         # 全局配置 (所有模块头文件 + 全局变量声明)
│   ├── config/config.c         # 全局变量定义
│   ├── sys/sys.c               # SysTick延时 (来自正点原子)
│   ├── sys/sys.h               # 位带操作宏 + GPIO快捷读写 (PAout/PBin等)
│   ├── stm32f4xx_conf.h        # STM32标准库配置文件
│   ├── stm32f4xx_it.c          # 系统异常处理 (HardFault等)
│   └── stm32f4xx_it.h          # 系统异常声明
├── HARDWARE/                   # 外设驱动层
│   ├── timer/                   # 核心控制定时器 (TIM7控制环 + TIM3显示刷新)
│   ├── SPWM/                    # SPWM模块 (TIM1高级定时器方案, 备用)
│   ├── PWM/                     # PWM初始化 (TIM5/TIM8/TIM1)
│   ├── PID/                     # 标准PID控制器 (辅助模块)
│   ├── global/                  # 高级算法 (QPR/SOGI/SOGI_PLL, 闭环预留)
│   ├── temporary/               # 算法参数定义 (PR/SOGI/PLL系数)
│   ├── AD7606/                  # AD7606 8通道ADC (FSMC接口)
│   ├── OLED/                    # 0.96寸OLED (SPI, 江西协科技库)
│   ├── usart/                   # USART1调试串口
│   ├── key/                     # 5按键 (外部中断)
│   ├── led/                     # LED指示灯 (PC3)
│   ├── DAC/                     # DAC输出 (备用)
│   ├── TM1638/                  # TM1638数码管 (备用)
│   └── ArmMath/                 # ARM CMSIS-DSP库
└── Libraries/                   # STM32标准库
    ├── CMSIS/                   # CMSIS核心 (core_cm4.h等)
    └── STM32F4xx_StdPeriph_Driver/  # 标准外设库
```

## 中断架构与控制流

```
优先级 (高→低):
  0. TIM7 (10kHz / 0.1ms): 核心控制环
     流程: 读取ADC → PI调节(闭环) → DDS正弦生成 → SPWM输出 → LED翻转
  1. TIM3 (200Hz / 5ms): OLED显示刷新
  2. USART1: 调试串口接收中断
  3. EXTI0~4: 按键中断 (模式切换/电压±/频率+)

TIM7 ISR 详细流程:
  1. 读取AD7606 ADC值 (通道2: 输出电压Uout)
  2. 闭环模式 (mode==1): PI电压调节
     err = Uout_ref - Uout
     out = kp*err + ki*Σerr  (限幅0.5~0.98)
  3. DDS软件正弦波生成 (arm_sin_f32)
     相位步进 = 2*PI*freq / 10000
  4. 单极性SPWM输出 (TIM5 CH3/CH4, ARR=4199)
     sin>0: CH4=sin*4199*mod, CH3=0
     sin<0: CH3=|sin|*4199*mod, CH4=0
  5. time_num++ (200次循环)

按键功能:
  KEY1 (PA0): 开环/闭环切换
  KEY2 (PE4): Uout_ref++
  KEY3 (PE2): Uout_ref--
  KEY4 (PE3): Uout_ref_F_Hz++
  KEY5 (PE1): 预留
```

## 引脚分配

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA1 | TIM5_CH2 | PWM输出1 |
| PA2 | TIM5_CH3 | SPWM负半周 |
| PA3 | TIM5_CH4 | SPWM正半周 |
| PA4 | DAC1 | DAC输出 (备用) |
| PA5 | DAC2 / TIM8_CH1N | DAC / 高级定时器互补 |
| PA6 | TIM8_BKIN / TIM1_BKIN | 刹车输入 |
| PA7 | TIM1_CH1N | 高级定时器互补 |
| PA8 | TIM1_CH1 | 高级定时器主输出 |
| PA9 | USART1_TX | 调试串口发送 |
| PA10 | USART1_RX | 调试串口接收 |
| PB12 | OLED_D0 (SCK) | OLED SPI时钟 |
| PB13 | OLED_D1 (MOSI) | OLED SPI数据 |
| PB14 | OLED_RES | OLED复位 |
| PB15 | OLED_DC | OLED数据/命令 |
| PC3 | LED | 调试翻转 |
| PC6 | TIM8_CH1 | 高级定时器主输出 |
| PE1~4 | KEY2~KEY5 | 按键输入 |
| PG2 | OLED_CS | OLED片选 |
| PG7~9 | TM1638 | 数码管 (备用) |
| PH12 | AD7606_CONVST | ADC转换触发 |
| PI6 | AD7606_BUSY | ADC忙信号 |
| PF6 | AD7606_CONVST | ADC转换触发 (实际) |
| PF7 | AD7606_RESET | ADC复位 |
| PD0/1/4/5/8/9/10/14/15 | FSMC_D0~D15 | AD7606数据总线 |
| PE4~15 | FSMC_A/D | AD7606地址/数据 |
| PG12 | FSMC_NE4 | AD7606片选 |

## 关键控制参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 载波频率 | 20kHz | TIM5 PWM |
| 控制频率 | 10kHz | TIM7中断 |
| 显示频率 | 200Hz | TIM3中断 |
| PWM周期 | 4200 | TIM5 ARR值 |
| 直流母线电压 | 40V | 实验平台低压侧 |
| 最大调制比 | 0.98 | 对应最大输出电压 |
| 最小调制比 | 0.5 | 对应最小输出电压 |
| 输出电压范围 | 10~2000Hz | 软件DDS生成 |
| 电压范围 | 14~28Vrms | 基于40V母线计算 |
| PI_Kp | 0.0025 | 电压环比例增益 |
| PI_Ki | 0.0001 | 电压环积分增益 |

## 控制模式

### 开环模式 (mode=0, 默认)
- 调制比固定 (g_inv_open_loop_mod, 默认0.6)
- 输出电压 = mod * Vbus / √2
- 按键KEY2/KEY3可调整目标电压

### 闭环模式 (mode=1)
- PI电压调节器输出调制比
- 目标电压 = Uout_ref (默认25Vrms)
- PI输出限幅: 0.5~0.98

## 编码规范

- 文件头注释: `@file`, `@brief`, `@note` Doxygen格式
- 函数注释: 功能说明 + 参数说明
- 变量命名:
  - 模块前缀: `g_` = 全局, `s_` = 静态, 无前缀 = 局部
  - 类型后缀: `_f` = float
- 缩进: 4空格
- 编码: UTF-8 (无BOM)
- 语言: 中文注释

## 编译说明

1. 用 Keil MDK 5 打开 `Project/RVMDK(uv5)/BH-F407.uvprojx`
2. 确保 ARM Compiler 5 可用 (ARMCC)
3. 编译前需要在 Keil 中手动从工程树中移除已删除的文件:
   - HARDWARE/ADC/adc.c (已删除)
4. 编译输出在 `Output/` 目录

## 已删除/清理的文件

- HARDWARE/ADC/ (空模块, 已删除)
- 各文件中注释掉的旧代码块 (已清理)
- Output/ 目录中的过期编译产物 (已清理)

## 预留扩展模块

以下模块已开发但当前未启用，为未来升级预留:
- SPWM模块: 基于TIM1高级定时器的SPWM方案 (支持互补输出+死区)
- global模块: QPR控制器 / SOGI滤波器 / SOGI_PLL锁相环 (闭环并网控制)
- temporary模块: PR谐振控制器参数 (50Hz基频 + 150Hz谐波补偿)
- DAC模块: 调试波形输出
- TM1638模块: 数码管辅助显示
- PWM8/PWM1: 高级定时器H桥驱动方案
