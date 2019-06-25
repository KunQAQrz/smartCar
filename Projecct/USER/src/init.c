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
u16 sp_coe=35;//�ٶ�����
PID_InitTypeDef   left;
PID_InitTypeDef  right;

const u8 	coe[Avg+1]={1,2,3,4,5,6,7,8,9,10,55};//��Ȩ����

const u16	EMS20[6]={100,200,500,800,200,100};//20
const u16	EMS_common[6]={50,500,500,1500,500,500};//23 
const u16	EMS_stable[6]={50,500,500,1200,1000,200};//20
const u16	EMS_super[6]={50,800,500,1200,1200,200};//25

//reduce:50����һ��(250)��reduce:100���ڶ���(200)��reduce:100��������(200)�ٶ�20
//reduce:100����һ��(200)���ٶ�25��rudece:0���ڶ���(200)���ٶ�23
void System_Init()
{
	get_clk();             	 									//��ȡʱ��Ƶ�� ����ִ��
	DisableInterrupts;												//�ر��ܵ��жϿ���
	
	Bluetooth_Interrupt_Init();								//������ʼ��

	AD_sensor_Init();													//��С����Ե�Դ��ѹ��ʼ��,��ѹת�����ź�											
	
	Motor_Init();															//�����PID�ͱ�������ʼ��
	
	Pit_Interrupt_Init();											//pit��ʱ���жϳ�ʼ��
	
	//IRQ_Interrupt_Init();										//�ⲿ�жϳ�ʼ��
	
	OLED_Init();															//OLED��Ļ��ʼ��
	OLED_Fill(0x00);													//����
	
  //FLASH_Integration_Init();								//�����ʼ��
	
	Others_Integration_Init();							  //�����ʼ��
	
	KBI_Interrupt_Init();											//�����жϳ�ʼ��
	
	EnableInterrupts;													//���ܵ��жϿ���
}
//************************************************************************

void Motor_Init()
{
	//��3��FTM,ÿ����8��ͨ��
	ftm_pwm_init(ftm2,ftm_ch2,10000,0);		ftm_pwm_init(ftm2,ftm_ch3,10000,0);		//���ֵ��   ��
	ftm_pwm_init(ftm2,ftm_ch4,10000,0);		ftm_pwm_init(ftm2,ftm_ch5,10000,0);		//���ֵ��   ��
	//PID_Init();
	ftm_count_init(ftm1);									gpio_init(FTM0_DIR_PIN,GPI,0);        //���ֱ����� ��
	ftm_count_init(ftm0);									gpio_init(FTM1_DIR_PIN,GPI,0);				//���ֱ����� ��
	
	port_pull(FTM0_DIR_PIN);							port_pull(FTM0_COUNT_PIN);						//E0  ���ߣ����ȶ�     ��
	port_pull(FTM1_DIR_PIN);							port_pull(FTM1_COUNT_PIN);						//E7									 ��
}

void AD_sensor_Init()
{
			adc_init(ADC0_SE2);		//��Դ��ѹ����
			adc_init(ADC0_SE15);	//���ͨ��1
	    adc_init(ADC0_SE13);  //���ͨ��2
	    adc_init(ADC0_SE14);	//���ͨ��3
	    adc_init(ADC0_SE12);	//���ͨ��4
	    adc_init(ADC0_SE7);	  //���ͨ��5
	    adc_init(ADC0_SE6); 	//���ͨ��6
}

void Others_Integration_Init()
{
		  gpio_init(E3	,GPO,0);//������
	    gpio_init(I2	,GPO,1);//��
			gpio_init(I3	,GPO,1);
			gpio_init(C6	,GPO,1);
			gpio_init(C7	,GPO,1);
			gpio_init(PIPE,GPI,1);//�ɻɹ�
}

void KBI_Interrupt_Init()
{
	kbi_init(KEY1,IRQ_FALLING_LOW);		     //ͨ��ѡ��ΪKbi1���͵�ƽ����
	kbi_init(KEY2,IRQ_FALLING_LOW);				 //Key2
	kbi_init(KEY3,IRQ_FALLING_LOW);				 //Key3
	kbi_init(KEY4,IRQ_FALLING_LOW);				 //Key4

	set_irq_priority(KBI0_IRQn,1);
	enable_irq(KBI0_IRQn);		  							 //��KBI1_0RQn���жϿ���
	
	gpio_init(SW1,GPI,1);port_pull(SW1);			 //���뿪�س�ʼ��
	gpio_init(SW2,GPI,1);port_pull(SW2);
	gpio_init(SW3,GPI,1);port_pull(SW3);
}

void IRQ_Interrupt_Init()
{
	irq_init();                          			 //��ʼ��irq�������Ҫ�л�irq�����ſ�����KEA128_port_cfg.h�ļ��޸�
	port_pull(IRQ_PIN);                        //��������
	set_irq_priority(IRQ_IRQn,3);							 //�������ȼ�,�����Լ����������� �����÷�ΧΪ 0 - 3

	enable_irq(IRQ_IRQn);											 //��irq���жϿ���
}

//void FLASH_Integration_Init()
//{
//	FLASH_Init();
//  FLASH_EraseSector(sector);
//}

void Pit_Interrupt_Init()
{
	pit_init_ms(pit0,1);	            //��ʼ��pit0 ��������Ϊ1ms			
  set_irq_priority(PIT_CH0_IRQn,1);	//����pit0���ȼ�
	enable_irq(PIT_CH0_IRQn);					//����pit0�ж�
	//pit0�жϺ�����isr.c�ļ� 
	
	pit_init_ms(pit1,1);	            //��ʼ��pit1 ��������Ϊ1ms			
  set_irq_priority(PIT_CH1_IRQn,3);	//����pit1���ȼ�
	//enable_irq(PIT_CH1_IRQn);					//����pit1�ж�
}

void Bluetooth_Interrupt_Init()
{	//�������ƣ�MT		���룺6868
	//����KEA128û�в�����΢���Ĵ�������������õò����ʽϸߵ���������Ƚϴ���˷���ʵ�ʲ���������У��
  baud_feedback = uart_init(uart2,9600);		//������ʼ��,��ʼ������1Ϊ1λ��ʼλ��8λ����λ��1λֹͣλ��������9600
	uart_rx_irq_en(uart2);										//����UART2�����ж�
	
	uart_init(uart1,9600);
	//NRF_Dev_Init();													//NRF����ģ���ʼ��
}

void PID_Position(PID_InitTypeDef *PID,int16 Set,float Present,u8 dir)
{		
		static u8 T[2];		
    PID->Error[0] = (float)Set-Present;//��ǰ�������趨ռ�ձȼ�ȥ��ǰֵ
	
//		if(fabs(PID->Error[0])<=DEAD)									PID->Error[0]=0;//��ǰ���������������ֹƫ��Сʱ����
//		
//		if(PID->Error[0]==PID->Error[1]==0)//���������ֻ��D�ʹ�����мӿ�
//		{
//			PID->Out_D[Now]=LPF*PID->Out_D[Last];
//			PID->Out[SUM]=PID->Out_D[Now]-PID->Out_D[Last];
//		}
//		else
//		{
		PID->Out[P]=K_P/1000.0*PID->Error[0];//PID��P��
		
		if((fabs(PID->Error[0])>SHIFT_UPP)||(PID->Error[0])==0)				  PID->Out[I]+=0;//��ƫ��ϴ�ʱ�����ڻ�������ʱ������PD����
		else if(fabs(PID->Error[0])<=SHIFT_LOW)												 	PID->Out[I]+=K_I/1000.0*PID->Error[0]; //��Сʱ������I��
		else PID->Out[I]+=K_I/1000.0*PID->Error[0]*(SHIFT_UPP-fabs(PID->Error[0]))/(SHIFT_UPP-SHIFT_LOW);//�����

		if		 (PID->Out[I]>LIMIT_INTEGRAL_UPP)		PID->Out[I]=LIMIT_INTEGRAL_UPP;
		else if(PID->Out[I]<LIMIT_INTEGRAL_LOW)		PID->Out[I]=LIMIT_INTEGRAL_LOW;//PID�޷�

		if(T[dir]==10)
		{
			float Delta_E=PID->Error[0]-PID->Error[1];			//�������
			PID->Out[D]=(1-LPF)*K_D/1000.0*Delta_E+LPF*PID->Out_D[Last];
			PID->Error[1]=PID->Error[0];
			PID->Out_D[Last]=PID->Out[D];//������
		}			
																																				
		PID->Out[SUM]=PID->Out[P]+PID->Out[I]+PID->Out[D];
//		}	
			
		if		 (PID->Out[SUM]>LIMIT_OUT_UPP)			PID->Out[SUM]=LIMIT_OUT_UPP;
		else if(PID->Out[SUM]<LIMIT_OUT_LOW)			PID->Out[SUM]=LIMIT_OUT_LOW;//PID�޷�

		T[dir]++; 		if(T[dir]==11)	   T[dir]=0;
}

u16 WeightValueFilter(u16 *ad_buff,u16 ad_value) //��Ȩ����ƽ���˲���
{
    u32 sum = 0;
    ad_buff[Avg] = ad_value;
    for(u8 i=0;i<Avg;i++)
    {
        ad_buff[i] = ad_buff[i+1];//�����������ƣ���λ�Ե�
        sum += ad_buff[i] * coe[i];
    }
    return (u16)(sum/coe[Avg]);  
}
