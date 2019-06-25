#include "init.h"
static u8 sector=	FLASH_SECTOR_NUM-1;
u32 baud_feedback;
int16	 speed[2][Avg+1];		int16	  lap[2];
u16 adc_buff[6][Avg+1];		
u8 	 	 			  	  page=1;
u8 									send;
u8 							   sw[3];	
float 					 wave[8];
float							   D_x;
u16							 	  time;
float						run_time;
u16 						 K_P=2000;//825;//145;
u16 						 K_I=1500;		 //=1800;
u16 						 K_D=500;
u16        SHIFT_UPP=400;
u16        SHIFT_LOW=200;
int16	LIMIT_INTEGRAL_UPP= 230;
int16	LIMIT_INTEGRAL_LOW=-230;
u16 		 	  adc_value[6];
int16 	 velocity[4];//P,D_last,D,SUM
//				 { P ,P_T, D ,P_L,PT_L,D_L }
u16 EMS[6]={50,500,500,1500,500,500};//25
//u16 EMS[6]={100,200,500,800,200,100};//20
u16 sp_coe=35;//速度因子
PID_InitTypeDef   left;
PID_InitTypeDef  right;

const u8 	coe[Avg+1]={1,2,3,4,5,6,7,8,9,10,55};//加权数组

const u16	EMS20[6]={100,200,500,800,200,100};//20
const u16	EMS_common[6]={50,500,500,1500,500,500};//23 
const u16	EMS_stable[6]={50,500,500,1200,1000,200};//20
const u16	EMS_super[6]={50,800,500,1200,1200,200};//25

//reduce:50：第一版(250)；reduce:100：第二版(200)；reduce:100：第三版(200)速度20
//reduce:100：第一版(200)，速度25；rudece:0：第二版(200)；速度23
void System_Init()
{
	get_clk();             	 									//获取时钟频率 必须执行
	DisableInterrupts;												//关闭总的中断开关
	
	Bluetooth_Interrupt_Init();								//蓝牙初始化

	AD_sensor_Init();													//电感、测试电源电压初始化,电压转数字信号											
	
	Motor_Init();															//电机、PID和编码器初始化
	
	Pit_Interrupt_Init();											//pit定时器中断初始化
	
	//IRQ_Interrupt_Init();										//外部中断初始化
	
	OLED_Init();															//OLED屏幕初始化
	OLED_Fill(0x00);													//清屏
	
  //FLASH_Integration_Init();								//闪存初始化
	
	Others_Integration_Init();							  //杂项初始化
	
	KBI_Interrupt_Init();											//按键中断初始化
	
	EnableInterrupts;													//打开总的中断开关
}
//************************************************************************

void Motor_Init()
{
	//有3个FTM,每个有8个通道
	ftm_pwm_init(ftm2,ftm_ch2,10000,0);		ftm_pwm_init(ftm2,ftm_ch3,10000,0);		//后轮电机   左
	ftm_pwm_init(ftm2,ftm_ch4,10000,0);		ftm_pwm_init(ftm2,ftm_ch5,10000,0);		//后轮电机   右
	//PID_Init();
	ftm_count_init(ftm1);									gpio_init(FTM0_DIR_PIN,GPI,0);        //后轮编码器 左
	ftm_count_init(ftm0);									gpio_init(FTM1_DIR_PIN,GPI,0);				//后轮编码器 右
	
	port_pull(FTM0_DIR_PIN);							port_pull(FTM0_COUNT_PIN);						//E0  拉高，更稳定     左
	port_pull(FTM1_DIR_PIN);							port_pull(FTM1_COUNT_PIN);						//E7									 右
}

void AD_sensor_Init()
{
			adc_init(ADC0_SE2);		//电源电压测试
			adc_init(ADC0_SE15);	//电感通道1
	    adc_init(ADC0_SE13);  //电感通道2
	    adc_init(ADC0_SE14);	//电感通道3
	    adc_init(ADC0_SE12);	//电感通道4
	    adc_init(ADC0_SE7);	  //电感通道5
	    adc_init(ADC0_SE6); 	//电感通道6
}

void Others_Integration_Init()
{
		  gpio_init(E3	,GPO,0);//蜂鸣器
	    gpio_init(I2	,GPO,1);//灯
			gpio_init(I3	,GPO,1);
			gpio_init(C6	,GPO,1);
			gpio_init(C7	,GPO,1);
			gpio_init(PIPE,GPI,1);//干簧管
}

void KBI_Interrupt_Init()
{
	kbi_init(KEY1,IRQ_FALLING_LOW);		     //通道选择为Kbi1，低电平触发
	kbi_init(KEY2,IRQ_FALLING_LOW);				 //Key2
	kbi_init(KEY3,IRQ_FALLING_LOW);				 //Key3
	kbi_init(KEY4,IRQ_FALLING_LOW);				 //Key4

	set_irq_priority(KBI0_IRQn,1);
	enable_irq(KBI0_IRQn);		  							 //打开KBI1_0RQn的中断开关
	
	gpio_init(SW1,GPI,1);port_pull(SW1);			 //拨码开关初始化
	gpio_init(SW2,GPI,1);port_pull(SW2);
	gpio_init(SW3,GPI,1);port_pull(SW3);
}

void IRQ_Interrupt_Init()
{
	irq_init();                          			 //初始化irq，如果需要切换irq的引脚可以在KEA128_port_cfg.h文件修改
	port_pull(IRQ_PIN);                        //设置上拉
	set_irq_priority(IRQ_IRQn,3);							 //设置优先级,根据自己的需求设置 可设置范围为 0 - 3

	enable_irq(IRQ_IRQn);											 //打开irq的中断开关
}

//void FLASH_Integration_Init()
//{
//	FLASH_Init();
//  FLASH_EraseSector(sector);
//}

void Pit_Interrupt_Init()
{
	pit_init_ms(pit0,1);	            //初始化pit0 周期设置为1ms			
  set_irq_priority(PIT_CH0_IRQn,1);	//设置pit0优先级
	enable_irq(PIT_CH0_IRQn);					//开启pit0中断
	//pit0中断函数在isr.c文件 
	
	pit_init_ms(pit1,1);	            //初始化pit1 周期设置为1ms			
  set_irq_priority(PIT_CH1_IRQn,3);	//设置pit1优先级
	//enable_irq(PIT_CH1_IRQn);					//开启pit1中断
}

void Bluetooth_Interrupt_Init()
{	//蓝牙名称：MT		密码：6868
	//由于KEA128没有波特率微调寄存器，因此在设置得波特率较高得情况下误差比较大，因此返回实际波特率用来校验
  baud_feedback = uart_init(uart2,9600);		//蓝牙初始化,初始化串口1为1位起始位、8位数据位、1位停止位、波特率9600
	uart_rx_irq_en(uart2);										//开启UART2接收中断
	
	uart_init(uart1,9600);
	//NRF_Dev_Init();													//NRF无线模块初始化
}

void PID_Position(PID_InitTypeDef *PID,int16 Set,float Present,u8 dir)
{		
		static u8 T[2];		
    PID->Error[0] = (float)Set-Present;//当前误差等于设定占空比减去当前值
	
//		if(fabs(PID->Error[0])<=DEAD)									PID->Error[0]=0;//当前误差设置死区，防止偏差小时抖动
//		
//		if(PID->Error[0]==PID->Error[1]==0)//特殊情况，只有D项，使其运行加快
//		{
//			PID->Out_D[Now]=LPF*PID->Out_D[Last];
//			PID->Out[SUM]=PID->Out_D[Now]-PID->Out_D[Last];
//		}
//		else
//		{
		PID->Out[P]=K_P/1000.0*PID->Error[0];//PID的P项
		
		if((fabs(PID->Error[0])>SHIFT_UPP)||(PID->Error[0])==0)				  PID->Out[I]+=0;//当偏差较大时，大于积分上限时，采用PD控制
		else if(fabs(PID->Error[0])<=SHIFT_LOW)												 	PID->Out[I]+=K_I/1000.0*PID->Error[0]; //较小时，采用I项
		else PID->Out[I]+=K_I/1000.0*PID->Error[0]*(SHIFT_UPP-fabs(PID->Error[0]))/(SHIFT_UPP-SHIFT_LOW);//变积分

		if		 (PID->Out[I]>LIMIT_INTEGRAL_UPP)		PID->Out[I]=LIMIT_INTEGRAL_UPP;
		else if(PID->Out[I]<LIMIT_INTEGRAL_LOW)		PID->Out[I]=LIMIT_INTEGRAL_LOW;//PID限幅

		if(T[dir]==10)
		{
			float Delta_E=PID->Error[0]-PID->Error[1];			//误差的误差
			PID->Out[D]=(1-LPF)*K_D/1000.0*Delta_E+LPF*PID->Out_D[Last];
			PID->Error[1]=PID->Error[0];
			PID->Out_D[Last]=PID->Out[D];//误差更新
		}			
																																				
		PID->Out[SUM]=PID->Out[P]+PID->Out[I]+PID->Out[D];
//		}	
			
		if		 (PID->Out[SUM]>LIMIT_OUT_UPP)			PID->Out[SUM]=LIMIT_OUT_UPP;
		else if(PID->Out[SUM]<LIMIT_OUT_LOW)			PID->Out[SUM]=LIMIT_OUT_LOW;//PID限幅

		T[dir]++; 		if(T[dir]==11)	   T[dir]=0;
}

u16 WeightValueFilter(u16 *ad_buff,u16 ad_value) //加权递推平均滤波法
{
    u32 sum = 0;
    ad_buff[Avg] = ad_value;
    for(u8 i=0;i<Avg;i++)
    {
        ad_buff[i] = ad_buff[i+1];//所有数据左移，低位仍掉
        sum += ad_buff[i] * coe[i];
    }
    return (u16)(sum/coe[Avg]);  
}
