#include "stm32f10x.h"                  // Device header
#include "Delay.h"

//四个按键，分别用来，上移+增加参数A11，下移+减少参数A12，确认+是否可调参A0，返回+长按暂停C14

// 长按相关变量
//4:上键,2:下键,1:返回
static uint32_t key_press_start[5] = {0};  // 按键按下开始时间
static uint8_t key_long_press_flag[5] = {0};  // 长按标志
static uint32_t last_continuous_time[5] = {0};  // 上次连续触发时间


void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 按键引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//确认
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//上
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//下
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//返回
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0);
		Delay_ms(20);
		KeyNum = 1;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == 0);
		Delay_ms(20);
		KeyNum = 11;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0);
		Delay_ms(20);
		KeyNum = 12;
	}
	if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == 0)
	{
		Delay_ms(20);
		while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14) == 0);
		Delay_ms(20);
		KeyNum = 14;
	}
	
	return KeyNum;
}
