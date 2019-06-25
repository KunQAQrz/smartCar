/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		中断文件
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.23
 * @Target core		S9KEA128AMLK
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-11-6
 ********************************************************************************************************************/

#include "isr.h"
#include "init.h"

static 	  u8  dir;
static    u8  flag;
static 	  u8  order;
					u8  stop=1;    //在velocity的影响下的停车标志
static float  lastbuff[2];//一阶滤波上次数据
			 float  ftm[2];//编码器滤波
static float  ftm_last[2];
static  	u8  minus[2];
static 	  u8  command[6];
static 	  u8  sequence[3];
			 int16  duty[2];
			 int16	ready_duty=250;
static int16  duty_buff[2];
static int16  same[2];
static 		u8	decelerate;	
static 		u8	inside;//转弯，减速
				 u16	reduce=100;
static    u8  land[5];//Now,Last,Inside,Dir,Exit
static		u8	round;
static		u8	last_pipe_state=1;
static		u8	sound;

#define LPF_code 0.92
#define DOSM_ML DOSM(F_ADC,L_ADC)
#define DOSM_MR DOSM(F_ADC,R_ADC)
#define DOSM_LR DOSM(L_ADC,R_ADC)
#define DOSM_FB DOSM(F_ADC,B_ADC)
#define DOSM_T_LR DOSM(TL_ADC,TR_ADC)
#define DOSM_FB_LR DOSM(F_ADC+B_ADC,L_ADC+R_ADC)

#define ABS_DOSM_ML ABS_DOSM(F_ADC,L_ADC)
#define ABS_DOSM_MR ABS_DOSM(F_ADC,R_ADC)
#define ABS_DOSM_LR ABS_DOSM(L_ADC,R_ADC)
#define ABS_DOSM_FB ABS_DOSM(F_ADC,B_ADC)
#define ABS_DOSM_FB_LR ABS_DOSM(F_ADC+B_ADC,L_ADC+R_ADC)
#define ABS_DOSM_TL_TR ABS_DOSM(TL_ADC,TR_ADC)

void PIT_CH0_IRQHandler(void)
{	
		static u8 i;
		static u8 freq=0;
		int16 t;
		static float spD;
		int16 velocity_D_now;	
//*************************************电感滤波*********************************************
		adc_value[0]=WeightValueFilter(adc_buff[0],adc_once(ADC0_SE15,ADC_12bit));
		adc_value[1]=WeightValueFilter(adc_buff[1],adc_once(ADC0_SE13,ADC_12bit));
  	adc_value[2]=WeightValueFilter(adc_buff[2],adc_once(ADC0_SE14,ADC_12bit));
    adc_value[3]=WeightValueFilter(adc_buff[3],adc_once(ADC0_SE12,ADC_12bit));
		adc_value[4]=WeightValueFilter(adc_buff[4],adc_once(ADC0_SE7 ,ADC_12bit));
  	adc_value[5]=WeightValueFilter(adc_buff[5],adc_once(ADC0_SE6 ,ADC_12bit));	
//***************************************** 编码器滤波 *******************************************		
			if( gpio_get(FTM0_DIR_PIN))
				 ftm[Left] =LowPass_FirstFilter(LPF_code, ftm_count_get(ftm1)*5,lastbuff[Left]);
			else
				 ftm[Left] =LowPass_FirstFilter(LPF_code,-ftm_count_get(ftm1)*5,lastbuff[Left]);
			lastbuff[Left] = ftm[Left]; 
			
			if(!gpio_get(FTM1_DIR_PIN))
				 ftm[Right]=LowPass_FirstFilter(LPF_code, ftm_count_get(ftm0)*5,lastbuff[Right]);// /0.325
			else
				 ftm[Right]=LowPass_FirstFilter(LPF_code,-ftm_count_get(ftm0)*5,lastbuff[Right]);
			lastbuff[Right]= ftm[Right];

//			if(freq==10)	
//				{ftm_last[Left]=ftm[Left];	ftm_last[Right]=ftm[Right];}
//************************************************************************************************
	if(last_pipe_state==1&&gpio_get(PIPE)==0)		{round++;sound=1;}
		 last_pipe_state=gpio_get(PIPE);	
	if(sound==1)	
	{
		static u16 sound_time;	gpio_set(E3,1);	sound_time++;
		if(sound_time==300)			{sound=0;			 sound_time=0;}
	}
	if((freq==3)&&(/*(B_ADC<LINE_OUT)||*/(round>1)))//出线停车
		{  
			duty[Left]=duty[Right]=0;
			duty_buff[Left] =duty[Left];
			duty_buff[Right]=duty[Right];
			if(stop==0&&(B_ADC<LINE_OUT))
			{printf("\n Line out!\n");gpio_set(E3,0);}
			else if(stop==0&&round>1)
					printf("\n Pipe on\n");
//			else if(round>1)	gpio_set(E3,1);
			stop=1; 
		}
//	if(freq==10)	
//		{
//			spD=((ftm[Left]-ftm_last[Left])-(ftm[Right]-ftm_last[Right]))*3.1;
////			wave[4]=abs(spD);
//			ftm_last[Left]=ftm[Left];				ftm_last[Right]=ftm[Right];
//		}					
//****************************** PID转向环 ********************************			
	if(((L_ADC>1000/*(Base_Voltage_LR*1.304)*/&&R_ADC<900/*(Base_Voltage_LR*0.761)*/)||(R_ADC>1000/*(Base_Voltage_LR*1.304)*/&&L_ADC<900/*(Base_Voltage_LR*0.761)*/))&&(B_ADC<1200/*(Base_Voltage_FB*0.969)*/)&&(ABS_DOSM_LR>0.06))
		land[Dir]=(L_ADC>R_ADC)?Left:Right;//提前判断入弯方向
	
	if((F_ADC>2000/*(Base_Voltage_FB*1.212)*/)&&(B_ADC>2000/*(Base_Voltage_FB*1.212)*/))//环岛区直道
      land[Now]=1;
	if((land[Now]==land[Last]==1)&&(land[Inside]==0)&&(F_ADC>2300/*(Base_Voltage_FB*1.515)*/)&&(B_ADC>2300/*(Base_Voltage_FB*1.515)*/)&&(DOSM_FB<0.1)&&(DOSM_FB>-0.1))//入弯
			land[Inside]=1;
	if((land[Last]==0)&&(land[Now]==1)&&(land[Inside]==1))//出弯
		{ land[Inside]=0;	 land[Exit]=1; }
	if((F_ADC<=1800/*(Base_Voltage_FB*1.09)*/)&&(B_ADC<=1800/*(Base_Voltage_FB*1.09)*/))
		{ 
			land[Now]=0;//环岛区弯道
			if((ABS_DOSM_LR<=0.03)&&(ABS_DOSM_FB<=0.03)&&(ABS_DOSM_FB_LR>0.3)&&(ABS_DOSM_TL_TR<0.3))//非环岛区
				{ land[Inside]=0; land[Exit]=0; }
		}
		if(land[Exit]==0)//记录上次land状态
			 land[Last]=land[Now];
/******************************************************************************************/		
		if(stop==0)//stop=0,转向PD失能
		{
			run_time+=0.002;
			t=(ftm[Left]>ftm[Right])?ftm[Left]:ftm[Right];
		
			if			(fabs((ftm[Left]-ftm[Right])*3.1)>100/*&&ABS_DOSM_FB>0.05*/)		/*100*/				inside=1;
			else if	(fabs((ftm[Left]-ftm[Right])*3.1)<60/*&&ABS_DOSM_FB<=0.05*/)		/*60*/				inside=0;
//			
//			wave[0]=abs((ftm[Left]-ftm[Right])*3.1);
//			wave[1]=t;
			
			if(fabs((ftm[Left]-ftm[Right])*3.1)>120&&t>60&&ABS_DOSM_FB>0.03/*&&land[Now]==0&&land[Inside]==0*/)/*100 60*/
			{
				duty[Left] =(float)duty_buff[Left] *(1.0-(reduce/100.0)*t*t/(60*60.0));
				duty[Right]=(float)duty_buff[Right]*(1.0-(reduce/100.0)*t*t/(60*60.0));
				decelerate=1;
//					duty[Left] =100;
//					duty[Right]=100;
			}
			else	{duty[Left]=duty_buff[Left];		duty[Right]=duty_buff[Right];decelerate=0;}		
//			else		 inside=(fabs((ftm[Left]-ftm[Right])*3.1)-100)/(120-100);
//			if(inside==1||inside==2)																	inside=0;
//			if(inside==4)																							inside=3;
//			wave[0]=abs(t);
//			wave[1]=(ftm[Left]+ftm[Right])/2.0;
			
			switch(land[Now]*10+land[Inside])
			{/*s*/
				case 00:/*非环岛*/
				case 10:/*岛	区*/
				{	
					
						 velocity_D_now=EMS[D]*10*DOSM(L_ADC,R_ADC);
					gpio_set(E3,0);
						if(DOSM(L_ADC,R_ADC)>0)//判断转弯方向
							velocity[P]= ADJUST_P(EMS[P],EMS[P_TURN], DOSM(L_ADC+inside*TL_ADC,R_ADC+inside*TR_ADC))*sp_coe/1000.0*abs(t);
						else
							velocity[P]=-ADJUST_P(EMS[P],EMS[P_TURN],-DOSM(L_ADC+inside*TL_ADC,R_ADC+inside*TR_ADC))*sp_coe/1000.0*abs(t);
				}break;
				case 01:/*环岛圈内*/
				{
					
						 velocity_D_now=EMS[D]*10*DOSM(L_ADC,R_ADC);
					gpio_set(E3,0);
					
					if(DOSM(L_ADC,R_ADC)>0)//判断转弯方向
						 velocity[P]= ADJUST_P(EMS[P],EMS[P_TURN], DOSM(L_ADC+TL_ADC,R_ADC+TR_ADC))*sp_coe/1000.0*abs(t);
					else
						 velocity[P]=-ADJUST_P(EMS[P],EMS[P_TURN],-DOSM(L_ADC+TL_ADC,R_ADC+TR_ADC))*sp_coe/1000.0*abs(t);

				}break;
				case 11:/*入圈*/
				{
					gpio_set(E3,1);
					
					if(land[Dir]==Left)
					{	
						if(DOSM(TL_ADC,TR_ADC)>0)	//判断转弯方向
							 velocity[P]= ADJUST_P(EMS[P_LAND],EMS[PT_LAND], DOSM(TL_ADC,TR_ADC))*sp_coe/1000.0*abs(t);//2178
						else
							 velocity[P]=-ADJUST_P(EMS[P_LAND],EMS[PT_LAND],-DOSM(TL_ADC,TR_ADC))*sp_coe/1000.0*abs(t);
								velocity_D_now=EMS[D_LAND]*10*DOSM(TL_ADC,TR_ADC);
					}
					else
					{
						if(DOSM(TL_ADC,TR_ADC)>0)	//判断转弯方向	    						
							 velocity[P]= ADJUST_P(EMS[P_LAND],EMS[PT_LAND], DOSM(TL_ADC,TR_ADC))*sp_coe/1000.0*abs(t);			
						else
							 velocity[P]=-ADJUST_P(EMS[P_LAND],EMS[PT_LAND],-DOSM(TL_ADC,TR_ADC))*sp_coe/1000.0*abs(t);
							velocity_D_now=EMS[D_LAND]*10*DOSM(TL_ADC,TR_ADC);
					}
				}break;
			}/*s*/
			
				 velocity[D]=velocity_D_now-velocity[D_LAST];
			velocity[SUM]=velocity[P]+velocity[D];		
		}
		else
			{velocity[SUM]=0; 		velocity_D_now=EMS[D]*10*DOSM(L_ADC,R_ADC);}
					velocity[D_LAST]=velocity_D_now;

//			if(fabs((ftm[Left]-ftm[Right])*3.1)>120&&t>60&&ABS_DOSM_FB>0.05)
//			{
//				if(velocity[P]>0)		
//				{
//					duty[Left] =(float)duty_buff[Left] *(1.0-(reduce/100.0)*t*t/(60*60.0));
//					duty[Right]=(float)duty_buff[Right]*(1.0+(reduce/100.0)*t*t/(60*60.0));
//				}
//				else
//				{
//					duty[Left] =(float)duty_buff[Left] *(1.0+(reduce/100.0)*t*t/(60*60.0));
//					duty[Right]=(float)duty_buff[Right]*(1.0-(reduce/100.0)*t*t/(60*60.0));
//				}
//			}
//				else	{duty[Left]=duty_buff[Left];		duty[Right]=duty_buff[Right];decelerate=0;}		

			
//****************************** PID速度环 *********************************
	if		 ((duty[Left] -velocity[SUM])> MOTOR_LIMIT)
			velocity[SUM]=duty[Left]-MOTOR_LIMIT;
	else if((duty[Left] -velocity[SUM])<-MOTOR_LIMIT)
			velocity[SUM]=duty[Left]+MOTOR_LIMIT;
	
	PID_Position(&left ,duty[Left]-velocity[SUM],ftm[Left] *3.1,Left );
	
	if		 ((duty[Right]+velocity[SUM])> MOTOR_LIMIT)
			velocity[SUM]= MOTOR_LIMIT-duty[Right];
	else if((duty[Right]+velocity[SUM])<-MOTOR_LIMIT)
			velocity[SUM]=-MOTOR_LIMIT-duty[Right];
	
	PID_Position(&right,duty[Right]+velocity[SUM],ftm[Right]*3.1,Right);
//**************************************************************************
		if(left .Out[SUM]>=0)
		{ftm_pwm_duty(ftm2,ftm_ch2,/*MECH_ERROR+*/(int16)left.Out[SUM]);		ftm_pwm_duty(ftm2,ftm_ch3,0);}
		else
		{ftm_pwm_duty(ftm2,ftm_ch2,0);		 ftm_pwm_duty(ftm2,ftm_ch3,/*MECH_ERROR*/-(int16)left.Out[SUM]);}
		
		if(right.Out[SUM]>=0)
		{ftm_pwm_duty(ftm2,ftm_ch4,(int16)right.Out[SUM]);		 ftm_pwm_duty(ftm2,ftm_ch5,0);}
		else
		{ftm_pwm_duty(ftm2,ftm_ch4,0);		 ftm_pwm_duty(ftm2,ftm_ch5,-(int16)right.Out[SUM]);}
		
if(freq==7)
{
//		speed[Left][i] = ftm_count_get(ftm1)*5/**12000/512*/;		speed[Right][i] = ftm_count_get(ftm0)*5/**12000/512*/;
//		
//		//if(ftm_count_get(ftm0)!=0)	
//		//printf("%d,",ftm_count_get(ftm0));//右轮5ms的脉冲数
//																										 //根据方向信号判断正负，假设方向信号是高电平时为反转
//    if(!gpio_get(FTM0_DIR_PIN))    speed[Left] [i]= -speed[Left] [i]; //速度取负
//    if( gpio_get(FTM1_DIR_PIN))    speed[Right][i]= -speed[Right][i]; //速度取负
//************************************************************************************************************

//************************************************************************************************************
wave[0]=L_ADC;
wave[1]=R_ADC;
wave[2]=F_ADC;
wave[3]=B_ADC;
wave[4]=TL_ADC;
wave[5]=TR_ADC;
//************************************************************************************************************		
//	if(i!=Avg)
//		{
//			speed[Left] [Avg]+=speed[Left] [i];
//			speed[Right][Avg]+=speed[Right][i];
//			i++;
//		}
//		else
//		{
//			if((speed[Left][Avg]||speed[Right][Avg])!=0)
//					send=1;
//			i=0;
//		}
}
		freq++;
		if(freq==11)	 freq=0;
		ftm_count_clean(ftm0);
		ftm_count_clean(ftm1);
		
    PIT_FLAG_CLR	 (pit0);
}

//void PIT_CH1_IRQHandler(void)
//{
// 	 //PID_Regulator(&left,Duty[Left],ftm_count_get(ftm1)*20/1.349828,Left);
//	 if(left.Out[SUM]>0)
//	  ftm_pwm_duty(ftm2,ftm_ch2,abs(left.Out[SUM]));
//	 else
//		ftm_pwm_duty(ftm2,ftm_ch3,abs(left.Out[SUM]));
//	 
////	 ftm_count_clean(ftm0);
////	 ftm_count_clean(ftm1);
//	 
//   PIT_FLAG_CLR		(pit1);
//}

//void IRQ_IRQHandler(void)
//{
//    CLEAR_IRQ_FLAG;
//		lap[Right]++;
//}

void KBI0_IRQHandler(void)
{
		systick_delay_ms(20);
	
	  if((!gpio_get(A3)))  //A0即KEY1
		{
			switch(sw[2]*10+sw[1])
			{
				case 00:EMS[P_LAND]+=100;
				case 01:reduce+=50;
				case 10:EMS[P_LAND]-=100;
				case 11:reduce-=50;
			}
		}
		if((!gpio_get(A2)))  //A1即KEY2
		{
			switch(sw[2]*10+sw[1])
			{
				case 00:EMS[PT_LAND]+=100;
				case 01:LIMIT_INTEGRAL_UPP+=20;
				case 10:EMS[PT_LAND]-=100;
				case 11:LIMIT_INTEGRAL_UPP-=20;
			}
		}
		if((!gpio_get(A1)))  //A2即KEY3
		{
			switch(sw[2]*10+sw[1])
			{
				case 00:EMS[D_LAND]+=100;
				case 01:LIMIT_INTEGRAL_LOW+=20;
				case 10:EMS[D_LAND]-=100;
				case 11:LIMIT_INTEGRAL_LOW-=20;
			}	
		}
		if((!gpio_get(A0)))  //A3即KEY4
		{
			if(sw[0]==0)
			{
				static u8 T=1;
				T++;	
				if(T==4)	T=1;
				switch(T)
				{
					case 1:{for(u8 i=0;i<6;i++)	EMS[i]=EMS_super[i];ready_duty=250;	}break;
					case 2:{for(u8 i=0;i<6;i++)	EMS[i]=EMS_common[i];ready_duty=230; }break;
					case 3:{for(u8 i=0;i<6;i++)	EMS[i]=EMS_stable[i];ready_duty=200;	}break;
//					case 4:{for(u8 i=0;i<6;i++)	EMS[i]=EMS20[i];ready_duty=200;	}break;
				}	
			}
			if(sw[0]==1)
			{
				systick_delay_ms(delay);	//延时，等待发车
				stop=0;
				duty[Left]=duty[Right]=ready_duty;
				duty_buff[Left]=duty[Left];
				duty_buff[Right]=duty[Right];
			}
		}	
		
		while(!gpio_get(A0)||!gpio_get(A1)||!gpio_get(A2)||!gpio_get(A3));		
    CLEAN_KBI0_FLAG;
}

void UART2_IRQHandler(void)
{
		uart_getchar(uart2,&order);
		switch (order)
		{
			case '?':
			{
				printf("battery:%.3f V\n",adc_once(ADC0_SE2,ADC_12bit)*4*5.0/4096.0);
				printf("order: ");
				printf("Stop:s  Go:g  Duty:d\n");
				printf("speed: ");
				printf("P:+p=%d  I:+i=%d D:+b=%d\n",K_P,K_I,K_D);
				printf("swerve: ");
				printf("P:+P=%d PT:+T=%d D:+D=%d\n",EMS[P],EMS[P_TURN],EMS[D]);
				printf("island: ");
				printf("P:+L=%d PT:+A=%d D:+N=%d\n",EMS[P_LAND],EMS[PT_LAND],EMS[D_LAND]);
				printf("orther: ");
				printf("reduce:+R=%d ",reduce);
				printf("sp_coe:+S=%d",sp_coe);
//				printf("LPF:+Z=%d\n",LPF);
				
			}break;
//***********************************************
			case 's':
			{
				duty[Left]=duty[Right]=0;
				duty_buff[Left] =duty[Left];
				duty_buff[Right]=duty[Right];
				printf("STOP\n");					  stop=1;
//				printf("\n P(x):%f",wave[1]/time);
				printf("\n D(x):%f",D_x/time);
				printf("\n time=%f s\n",run_time);
				for(u8 j=0;j<5;j++)//清除数据
						wave[j]=0;
				D_x=0;run_time=0;time=0;
				gpio_set(E3,0);
			}break;
//***********************************************
			case 'g':
			{
				duty[Left] =DUTY_L;
				duty[Right]=DUTY_R;
				duty_buff[Left] =duty[Left];
				duty_buff[Right]=duty[Right];
				printf("GO\n");							stop=0;
			}break;
//***********************************************
			case 'd':
			{
				flag=1;
				printf("left duty is:");
			}break;
//***********************************************
			case '+':
			{
				flag=2;
				printf("parameter types:");		
			}break;
//**************************************************************************************************************
			case '%':
			{
				if(flag==1)
				{
				if(sequence[dir]==2)			same[dir]=(command[1]+command[0]*10)*10;
				else
					{
						printf("error!\n");
						command[0]=command[1]=0;
						sequence[Left]=sequence[Right]=0;		flag=0;
						break;
					}	
				command[0]=command[1]=0;			
				if(dir==Left)
					{
					  dir=Right;
					  printf("right duty is:");
					}
				else
					{
						if(minus[Left] ==1) duty[Left] =-same[Left];
						else							 	duty[Left]=  same[Left];
	
						if(minus[Right]==1) duty[Right]=-same[Right];
						else							 	duty[Right]= same[Right];
						
						duty_buff[Left] =duty[Left];
						duty_buff[Right]=duty[Right];
						
						printf("dutyl:%d,dutyr:%d\n",duty[Left],duty[Right]);
						sequence[Left]=sequence[Right]=0;		flag=0;		dir=Left;		minus[Left]=minus[Right]=0;  stop=0;
					}
					
				}
//**************************************************************************************************************
				if(flag==2)
				{	
					u8 j;u16 k;
					if(command[5]>=P&&command[5]<=D_LAND&&sequence[2]<=5)
					{
						EMS[command[5]]=0;
						for(j=sequence[2],k=1;j>0;j--,k*=10)
								EMS[command[5]]+=command[j-1]*k;
						printf("P =%d,PT =%d,D =%d\n",EMS[P],EMS[P_TURN],EMS[D]);
						printf("PL=%d,PtL=%d,DL=%d\n",EMS[P_LAND],EMS[PT_LAND],EMS[D_LAND]);
					}
					else if((command[5]=='p'||command[5]=='i'||command[5]=='b'||command[5]=='R'||command[5]=='S'/*||command[5]=='Z'*/)&&sequence[2]<=5)
					{
						u16 t=0;
						for(j=sequence[2],k=1;j>0;j--,k*=10)
								t+=command[j-1]*k;
						switch(command[5])
						{
							case 'p':{K_P	  =(u16)t;	printf("K_P=%d,K_I=%d,K_D=%d",K_P,K_I,K_D);}	break;
							case 'i':{K_I	  =(u16)t;	printf("K_P=%d,K_I=%d,K_D=%d",K_P,K_I,K_D);}	break;
							case 'b':{K_D	  =(u16)t;	printf("K_P=%d,K_I=%d,K_D=%d",K_P,K_I,K_D);}	break;
							case 'R':{reduce=(u16)t;	printf("reduce=%d",reduce);}									break;
							case 'S':{sp_coe=(u16)t;	printf("sp_coe=%d",sp_coe);}									break;
//							case 'Z':{LPF		=(u8)	t;	printf("LPF=%d",LPF);}												break;
						}
					}
					else
						printf("error!\n");
				for(u8 j=0;j<6;j++)
						command[j]=0;
				sequence[2]=0;		 		 flag=0;
				}
			}
		}
//**************************************************************************************************************
			switch (flag)
			{
				case 1:
				{
					if(order>='0'&&order<='9'&&sequence[dir]<2)
					{			
						command[sequence[dir]]=order-'0';
						sequence[dir]++;
					}
					if(order=='-')
						minus[dir]=1;	
				}break;
//**************************************************************************************************************				
				case 2:
				{
					switch(order)
					{
						case 'P':	command[5]=P;				break;
						case 'T':	command[5]=P_TURN;	break;
						case 'D':	command[5]=D;	      break;
						case 'L':	command[5]=P_LAND;	break;
						case 'A':	command[5]=PT_LAND;	break;
						case 'N':	command[5]=D_LAND;	break;
						case 'p':	command[5]='p';			break;
						case 'i':	command[5]='i';			break;
						case 'b':	command[5]='b';			break;
						case 'R':	command[5]='R';			break;
						case 'S':	command[5]='S';			break;
//						case 'Z':	command[5]='Z';			break;
					}
					if(order>='0'&&order<='9'&&sequence[2]<5)
					{
						command[sequence[2]]=order-'0';
						sequence[2]++;
					}
				}break;
			}
}
/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了周期定时器 通道0得中断
void PIT_CH0_IRQHandler(void)
{
    ;
}
记得进入中断后清除标志位

FTMRE_IRQHandler      
PMC_IRQHandler        
IRQ_IRQHandler        
I2C0_IRQHandler       
I2C1_IRQHandler       
SPI0_IRQHandler       
SPI1_IRQHandler       
UART0_IRQHandler 
UART1_IRQHandler 
UART2_IRQHandler 
ADC0_IRQHandler       
ACMP0_IRQHandler      
FTM0_IRQHandler       
FTM1_IRQHandler       
FTM2_IRQHandler       
RTC_IRQHandler        
ACMP1_IRQHandler      
PIT_CH0_IRQHandler    
PIT_CH1_IRQHandler    
KBI0_IRQHandler       
KBI1_IRQHandler       
Reserved26_IRQHandler 
ICS_IRQHandler        
WDG_IRQHandler        
PWT_IRQHandler        
MSCAN_Rx_IRQHandler   
MSCAN_Tx_IRQHandler   
*/



