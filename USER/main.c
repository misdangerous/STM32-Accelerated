#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "math.h"
#include "modbus.h"
short aacx_buff[1000];
short aacy_buff[1000];
short aacz_buff[1000];
u16 aac_port=0;

short aacx_average=0;
short aacy_average=0;
short aacz_average=0;

short aacx_max=0;
short aacy_max=0;
short aacz_max=0;

short aacx_min=0;
short aacy_min=0;
short aacz_min=0;


//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}	
/*******************************************
当程序召唤之后，调用函数求出平均值，去除最大和最小值
同时将数据数量清零
*******************************************/
void mpu6050_average(u16 *num,short *paacx,short *paacy,short *paacz,short *paacx_average,short *paacy_average,short *paacz_average)
{
	u16 i;
	int sumx=0,sumy=0,sumz=0;;
	for(i=0;i < *num;i++)
	{
		sumx += paacx[i];
		sumy += paacy[i];
		sumz += paacz[i];
	}
	sumx -= aacx_max;
	sumy -= aacy_max;
	sumz -= aacz_max;
	
	sumx -= aacx_min;
	sumy -= aacy_min;
	sumz -= aacz_min;
	
	*num -= 2;
	*paacx_average = sumx / *num;
	*paacy_average = sumy / *num;
	*paacz_average = sumz / *num;
	
	*num = 0;
}
void mpu6050_max_min(short *paacx,short *paacy,short *paacz,short *paacx_max,short *paacy_max,short *paacz_max,\
																	short *paacx_min,short *paacy_min,short *paacz_min)
{
	int temp;
	int max_temp;
	int min_temp;
	temp = abs((int)*paacx);
	max_temp = abs((int)*paacx_max);
	min_temp = abs((int)*paacx_min);
	if(temp > max_temp)
	{
		*paacx_max = *paacx;
	}
	else if(temp < min_temp)
	{
		*paacx_min = *paacx;
	}
	
	temp = abs((int)*paacy);
	max_temp = abs((int)*paacy_max);
	min_temp = abs((int)*paacy_min);
	if(temp > max_temp)
	{
		*paacy_max = *paacy;
	}
	else if(temp < min_temp)
	{
		*paacy_min = *paacy;
	}
	
	temp = abs((int)*paacz);
	max_temp = abs((int)*paacz_max);
	min_temp = abs((int)*paacz_min);
	if(temp > max_temp)
	{
		*paacz_max = *paacz;
	}
	else if(temp < min_temp)
	{
		*paacz_min = *paacz;
	}
}

u8 res_mpu=1;

#define ACCEL_FSR 0
#define ACCEL_NUM 2
 int main(void)
 {	 
//	u8 report=1;			//默认开启上报
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度	
	volatile double aacTemp=0;
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(4800);	 	//串口初始化为9600
	delay_init();	//延时初始化 
	LED_Init();		  			//初始化与LED连接的硬件接口
	MPU_Init();					//初始化MPU6050
	while(mpu_dmp_init())
	{
		
	}
	GPIO_ResetBits(GPIOA,GPIO_Pin_4); 						 //PA.4 输出低
 	while(1)
	{
		if((USART_RX_STA&0x8000) >> 15)
		{
			if((USART_RX_STA&0X3FFF) > 3)
			{
				if(ReceCrcCheck(USART_RX_BUF,(uint8_t)USART_RX_STA&0X3FFF))
				{
					if(((USART_RX_BUF[0]<<8) | USART_RX_BUF[1]) == MODBUS_ADDR)
					{
						if(USART_RX_BUF[2] == 3)
						{
							UART_TX();
							mpu6050_average(&aac_port,aacx_buff,aacy_buff,aacz_buff,&aacx_average,&aacy_average,&aacz_average);
							delay_ms(50);
							aacx_max = (((ACCEL_NUM * aacx_max) * 10000) / 32768);
							aacy_max = (((ACCEL_NUM * aacy_max) * 10000) / 32768);
							aacz_max = (((ACCEL_NUM * aacz_max) * 10000) / 32768);
							aacx_average = (((ACCEL_NUM * aacx_average) * 10000) / 32768);
							aacy_average = (((ACCEL_NUM * aacy_average) * 10000) / 32768);
							aacz_average = (((ACCEL_NUM * aacz_average) * 10000) / 32768);
							Data_Report(&aacx_max,&aacy_max,&aacz_max,&aacx_average,&aacy_average,&aacz_average);
							LED_Toggle(GPIOA,GPIO_Pin_3);
							UART_RX();
						}
					}
				}
			}
			USART_RX_STA = 0;
		}
		temp=MPU_Get_Temperature();	//得到温度值
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		aacx_buff[aac_port] = aacx;
		aacy_buff[aac_port] = aacy;
		aacz_buff[aac_port] = aacz;
		if(aac_port == 0)
		{
			aacx_max = aacx;
			aacy_max = aacy;
			aacz_max = aacz;
			aacx_min = aacx;
			aacy_min = aacy;
			aacz_min = aacz;
		}
		mpu6050_max_min(&aacx,&aacy,&aacz,&aacx_max,&aacy_max,&aacz_max,&aacx_min,&aacy_min,&aacz_min);
		aac_port++;
		if(aac_port > 1000)
		{
			LED_Toggle(GPIOA,GPIO_Pin_4);
//			mpu6050_average(&aac_port,aacx_buff,aacy_buff,aacz_buff,&aacx_average,&aacy_average,&aacz_average);
			aac_port = 0;
		}
//		if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
		
	}	
}

