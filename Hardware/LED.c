#include "stm32f10x.h"                  // Device header


/**
  * 函    数：手电筒初始化 (PB0推挽输出, 默认关闭)
  * 参    数：无
  * 返 回 值：无
  */
void Flashlight_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*默认关闭 (输出高电平, 低电平点亮)*/
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

/**
  * 函    数：手电筒开启
  * 参    数：无
  * 返 回 值：无
  */
void Flashlight_ON(void)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

/**
  * 函    数：手电筒关闭
  * 参    数：无
  * 返 回 值：无
  */
void Flashlight_OFF(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

/**
  * 函    数：手电筒状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void Flashlight_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_0) == 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_0);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
	}
}
