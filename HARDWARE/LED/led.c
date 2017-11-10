#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 							  
////////////////////////////////////////////////////////////////////////////////// 	   

//初始化PA3和PA4为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 //LED0-->PA.3 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.3
 GPIO_SetBits(GPIOA,GPIO_Pin_3);						 //PA.3 输出高

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	    		 //LED1-->PA.4 端口配置, 推挽输出
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
 GPIO_SetBits(GPIOA,GPIO_Pin_4); 						 //PA.4 输出低
}
 
