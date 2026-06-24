#ifndef _TM1638_H_
#define _TM1638_H_
/**
  ***************************************************************************************
  * TM1638模块功能实现源文件
  * 硬件连接：PA5--STB，PA6--CLK，PA7--DIO
  * 创建人：郭凯瑞
  * 最后修改时间：2021-2-26
  ***************************************************************************************
**/
#include "config.h"

typedef enum {N = 0x00, Y = 0x80} PointState; //是否带小数点 Y：带，N：不带

typedef enum {OFF = 0x00, ON = 0x01} LightState; //灯开关状态 On：开，Off：关

//TM1638模块相关引脚定义
#define STB GPIO_Pin_9
#define CLK GPIO_Pin_8
#define DIO GPIO_Pin_7
#define GPIO_TM1638       GPIOG
#define GPIO_Pin_TM1638   STB|CLK|DIO
#define RCC_TM1638        RCC_AHB1Periph_GPIOG

//引脚高低电平设置
#define STB_0() GPIO_ResetBits(GPIO_TM1638,STB)
#define STB_1() GPIO_SetBits(GPIO_TM1638,STB)

#define CLK_0() GPIO_ResetBits(GPIO_TM1638,CLK)
#define CLK_1() GPIO_SetBits(GPIO_TM1638,CLK)

#define DIO_0() GPIO_ResetBits(GPIO_TM1638,DIO)
#define DIO_1() GPIO_SetBits(GPIO_TM1638,DIO)

#define DIO_Read() GPIO_ReadInputDataBit(GPIO_TM1638,DIO)

//用户层函数
void TM1638_Init(void); //TM1638初始化函数
void TM1638_Display_Num(u32 data); //显示数字
void TM1638_Display_SEG(unsigned int num,unsigned char seg,PointState p); //选择数码管显示0-F
void TM1638_Display_LED(unsigned int num,LightState light); //指定led亮灭

unsigned char TM1638_ReadKey(void); //TM1638读键扫数据函数
void TM1638_SEG_Off(unsigned char num);//TM1638关闭指定数码管函数
void TM1638_Clear(void); //TM1638全清

//底层函数
void TM1638_Write_Byte(u8 byte); //TM1638单写数据，需要在函数外对STB操作
void TM1638_Write_Data(u8 data); //TM1638一个完整数据写入
void TM1638_Write_Addr_Bond(u8 addr,u8 data); //TM1638指定地址写入数据
unsigned char TM1638_Read(void); //TM1638读数据函数
void TM1638_GPIO_Init(void); //TM1638引脚初始化函数




/*
		    

		unsigned char key;
		key=TM1638_ReadKey();
		if(key==1)
		{
				TM1638_Display_SEG(2,'A',N);
		}
*/
#endif
