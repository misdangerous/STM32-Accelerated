#ifndef __LED_H
#define __LED_H	 
#include "sys.h"						  
////////////////////////////////////////////////////////////////////////////////// 
#define LED0 PAout(3)// PB5
#define LED1 PAout(4)// PE5	
#define LED_Toggle(x,p) GPIO_WriteBit(x,p,(BitAction)(1-(GPIO_ReadOutputDataBit(x,p))))
void LED_Init(void);//≥ı ºªØ

		 				    
#endif
