#include "led.h"

//////////////////////////////////////////////////////////////////////////////////	 							  
////////////////////////////////////////////////////////////////////////////////// 	   

//��ʼ��PA3��PA4Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 //LED0-->PA.3 �˿�����
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA.3
 GPIO_SetBits(GPIOA,GPIO_Pin_3);						 //PA.3 �����

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	    		 //LED1-->PA.4 �˿�����, �������
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
 GPIO_SetBits(GPIOA,GPIO_Pin_4); 						 //PA.4 �����
}
 
