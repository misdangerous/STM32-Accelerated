#ifndef __MODBUS_H
#define __MODBUS_H

#include "stm32f10x.h"

#define MODBUS_ADDR 0x300

uint16_t MBCrcCheck(uint8_t *puchMsg, uint16_t usDataLen);
uint8_t ReceCrcCheck(uint8_t *pbuff,uint8_t len);
void usart1_send_char(u8 c);
void modbus_report(u8 fun,u8*data,u8 len);
void Data_Report(short *paacx_max,short *paacy_max,short *paacz_max,short *paacx_average,short *paacy_average,short *paacz_average);

#endif
