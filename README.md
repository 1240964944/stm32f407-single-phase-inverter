# 单相逆变器控制系统

基于 **STM32F407ZGTx** 的单相逆变器开环/闭环双模式控制系统。

> Single-Phase Inverter Control System — dual-mode (open-loop / closed-loop) with SPWM output, PI voltage regulation, and real-time OLED display.

## 硬件平台

| 项目 | 规格 |
|------|------|
| 主控 | STM32F407ZGTx (Cortex-M4F, 168MHz) |
| 开发板 | 野火 STM32F407 |
| ADC | AD7606 8通道 16-bit (FSMC 并行接口) |
| 显示 | 0.96寸 OLED (SPI) |
| 直流母线 | 40V DC |
| IDE | Keil MDK 5 + ARM Compiler 5 |

## 功能特性

- ⚡ **SPWM 输出** — 20kHz 载波，单极性调制，TIM5 CH3/CH4
- 🎛️ **双模式控制** — 开环（固定调制比）/ 闭环（PI 电压调节）
- 📐 **DDS 正弦生成** — 基于 ARM CMSIS-DSP (`arm_sin_f32`)，输出 10~2000Hz
- 📊 **实时显示** — OLED 显示频率/电压/模式（200Hz 刷新率）
- 🔄 **10kHz 控制环** — TIM7 中断驱动，0.1ms 周期
- 🧮 **预留扩展** — QPR 控制器 / SOGI 滤波器 / SOGI-PLL 锁相环（并网控制）

## 控制架构

```
TIM7 ISR (10kHz)               TIM3 ISR (200Hz)
┌─────────────────┐            ┌──────────────┐
│ ADC (AD7606)    │            │ OLED 显示刷新 │
│   ↓             │            └──────────────┘
│ PI 调节 (闭环)   │
│   ↓             │            按键 (EXTI)
│ DDS 正弦生成     │            ├─ KEY1: 模式切换
│   ↓             │            ├─ KEY2: 电压+
│ SPWM 输出        │            ├─ KEY3: 电压-
│   ↓             │            └─ KEY4: 频率+
│ LED 翻转         │
└─────────────────┘
```

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 载波频率 | 20kHz | TIM5 PWM |
| 控制频率 | 10kHz | TIM7 中断 |
| 显示刷新 | 200Hz | TIM3 中断 |
| PWM 周期 | 4200 | TIM5 ARR |
| 输出频率 | 10~2000Hz | DDS 软件生成 |
| 输出电压 | 14~28Vrms | 基于 40V 母线 |
| 调制比范围 | 0.5~0.98 | PI 输出限幅 |
| PI_Kp | 0.0025 | 电压环比例 |
| PI_Ki | 0.0001 | 电压环积分 |

## 目录结构

```
├── User/                  # 应用层
│   ├── main.c             # 主程序入口
│   ├── config/            # 全局配置 & 变量定义
│   ├── sys/               # SysTick 延时 & 位带操作
│   ├── stm32f4xx_conf.h   # 标准库配置
│   └── stm32f4xx_it.c/h   # 中断服务程序
├── HARDWARE/              # 外设驱动
│   ├── timer/             # 核心控制定时器 (TIM7+TIM3)
│   ├── PWM/               # PWM 输出 (TIM5/TIM8)
│   ├── AD7606/            # ADC 驱动 (FSMC)
│   ├── OLED/              # OLED 显示驱动 (SPI)
│   ├── key/               # 按键 (外部中断)
│   ├── led/               # LED 指示
│   ├── usart/             # 调试串口
│   ├── PID/               # PI 控制器
│   ├── SPWM/              # 高级定时器 SPWM (备用)
│   ├── global/            # QPR/SOGI/SOGI-PLL (预留)
│   ├── temporary/         # 算法参数定义 (预留)
│   ├── DAC/               # DAC 输出 (备用)
│   ├── TM1638/            # 数码管显示 (备用)
│   └── ArmMath/           # CMSIS-DSP 库
├── Libraries/             # STM32 标准库
│   ├── CMSIS/             # CMSIS 核心
│   └── STM32F4xx_StdPeriph_Driver/  # 标准外设库 v1.8.0
└── Project/               # Keil MDK 工程文件
    └── RVMDK(uv5)/
        ├── BH-F407.uvprojx  # 工程文件
        └── BH-F407.uvoptx   # 工程选项
```

## 编译与烧录

1. 用 **Keil MDK 5** 打开 `Project/RVMDK(uv5)/BH-F407.uvprojx`
2. 确保 **ARM Compiler 5** (armcc) 可用
3. 编译前从工程树中移除：`HARDWARE/ADC/adc.c`（已删除）
4. 编译输出在 `Output/` 目录
5. 通过 J-Link / ST-Link 烧录

## 引脚分配

<details>
<summary>点击展开完整引脚表</summary>

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA1 | TIM5_CH2 | PWM 输出 |
| PA2 | TIM5_CH3 | SPWM 负半周 |
| PA3 | TIM5_CH4 | SPWM 正半周 |
| PA6 | TIM1_BKIN | 刹车输入 |
| PA7 | TIM1_CH1N | 高级定时器互补 |
| PA8 | TIM1_CH1 | 高级定时器主输出 |
| PA9 | USART1_TX | 调试串口 |
| PA10 | USART1_RX | 调试串口 |
| PB12 | SPI2_SCK | OLED 时钟 |
| PB13 | SPI2_MOSI | OLED 数据 |
| PB14 | GPIO | OLED 复位 |
| PB15 | GPIO | OLED 数据/命令 |
| PC3 | GPIO | LED 指示 |
| PC6 | TIM8_CH1 | 高级定时器 |
| PE1~4 | GPIO | 按键输入 |
| PG2 | GPIO | OLED 片选 |
| PF6 | GPIO | AD7606 CONVST |
| PF7 | GPIO | AD7606 RESET |
| PH12 | GPIO | AD7606 CONVST (备用) |
| PI6 | GPIO | AD7606 BUSY |
| FSMC_D0~D15 | FSMC | AD7606 数据总线 |
| FSMC_NE4 | FSMC | AD7606 片选 |

</details>

## License

MIT License — 仅供学习和研究使用。

---

*Power Electronics · STM32 · Embedded Control*
