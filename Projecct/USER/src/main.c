/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/
#include "init.h"

int main()
{
	System_Init();
    while(1)
    {
			Dip_Switch();
//			if(send==1)
//				{
////					//printf("L=%d ,R=%d \n",speed[Left][Avg],speed[Right][Avg]);
////					//printf("%5.3f,",(DOSM(L_ADC,R_ADC)*DOSM(L_ADC,R_ADC))*10000.0);
////					//printf("%5.3f,",fabs(DOSM(adc_value[2],adc_value[3])*100.0));
//							if(velocity[SUM]!=0)
//							printf("%d,",velocity[SUM]);
////						wave[2]=speed[Left] [Avg]/3.25;//1.3/4=0.325 10次 3.25
////						wave[3]=speed[Right][Avg]/3.25;
////						send=0;
//				}
				if(stop==0)//一按停止stop=1，数据不会再记录
				{
					//wave[0]=DOSM(L_ADC,R_ADC)*100.0;
					//wave[1]+=wave[0];
//					wave[4]=Duty[Left] [Go]-Duty[Left] [Re];
//					wave[5]=Duty[Right][Go]-Duty[Right][Re];
//					wave[6]=velocity[P];
//					wave[7]=velocity[D];
//					D_x+=wave[0]*wave[0];
					if(send==1)
					{
//            printf("%d,",abs(DOSM(adc_value[2],adc_value[3])*100.0));
						send=0;
					}
					time++;
				}
			//蓝牙串口,发送字符串
			//uart_putstr(uart2,"seekfree");
				Send_Ware(wave,sizeof(wave));
				systick_delay_ms(10);	
				Oled();
			//ems_coe  duty=15%,1094			20%,3562 3188			25% 6562 3488，30%,	8193 7332						
    }
}

void Oled()
{
//	disable_irq(KBI0_IRQn);
	switch(page)
	{
		case 1:
		{//(32,6,adc_once(ADC0_SE2,ADC_12bit)*(5000.0/4096.0)*4.0);
			OLED_P6x8Str(0 ,0,"L :");		OLED_Print_Num1(18,0,wave[0]);
			OLED_P6x8Str(70,0,"R :");  	OLED_Print_Num1(88,0,wave[1]);
			OLED_P6x8Str(0 ,1,"TL:");	  OLED_Print_Num1(18,1,wave[4]);
		  OLED_P6x8Str(70,1,"TR:");  OLED_Print_Num1(88,1,wave[5]);
			OLED_P6x8Str(0 ,2,"F :");  OLED_Print_Num1(18,2,wave[2]);
			OLED_P6x8Str(70,2,"B :");  OLED_Print_Num1(88,2,wave[3]);
			OLED_P6x8Str(0 ,3,"Duty:");  OLED_Print_Num1(30,3,ready_duty);
			OLED_P6x8Str(0 ,4,"BAT:      mV");  OLED_Print_Num1(24,4,BAT_V);
			OLED_P6x8Str(0 ,5,"PL:");  OLED_Print_Num1(18,5,EMS[P_LAND]);
			OLED_P6x8Str(0 ,6,"TL:");  OLED_Print_Num1(18,6,EMS[PT_LAND]);
			OLED_P6x8Str(0 ,7,"DL:");  OLED_Print_Num1(18,7,EMS[D_LAND]);
			OLED_P6x8Str(70,5,"RE:"); OLED_Print_Num1(88,5,reduce);
			OLED_P6x8Str(70,6,"UP:"); OLED_Print_Num1(88,6,LIMIT_INTEGRAL_UPP);
			OLED_P6x8Str(70,7,"LO:"); OLED_Print_Num1(88,7,LIMIT_INTEGRAL_LOW);
		}break;
//		case 2:
//		{
//			OLED_P6x8Str(0,0,"POW:      ");					OLED_Print_Num1(24,0,adc_once(ADC0_SE2,ADC_12bit)*(5000/4096));
//		}break;
	}
//	enable_irq(KBI0_IRQn);
}

void Dip_Switch()
{
	if(stop==1)
	{
		gpio_set(C7,1);//灯
		gpio_set(C6,1);
		gpio_set(I3,1);
		gpio_set(I2,1);
	}
	switch(gpio_get(SW1)*100+gpio_get(SW2)*10+gpio_get(SW3))
	{
		case 0:
		{sw[0]=0;	sw[1]=0;	 sw[2]=0;}	break;
		case 1:
		{sw[0]=0;	sw[1]=0;	 sw[2]=1;}	break;
		case 10:
		{sw[0]=0;	sw[1]=1;	 sw[2]=0;}	break;
		case 11:
		{sw[0]=0;	sw[1]=1;	 sw[2]=1;}	break;
		case 100:
		{sw[0]=1;	sw[1]=0;	 sw[2]=0;}	break;
		case 101:
		{sw[0]=1;	sw[1]=0;	 sw[2]=1;}	break;
		case 110:
		{sw[0]=1;	sw[1]=1;	 sw[2]=0;}	break;
		case 111:
		{sw[0]=1;	sw[1]=1;	 sw[2]=1;}	break;
	}
}
