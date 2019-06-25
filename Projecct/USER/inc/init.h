#ifndef _INIT_H
#define _INIT_H

#include "headfile.h"

typedef unsigned char u8;
typedef unsigned short int u16;
typedef unsigned long int u32;

#define delay 1700
#define Avg 10
#define SUM 3

#define P_TURN 1
#define P_LAND 3
#define PT_LAND 4
#define D_LAND 5

#define D_LAST 1
#define MECH_ERROR 5 //机械误差

#define DUTY_L 250
#define DUTY_R 250

#define FTM0_DIR_PIN       H5
#define FTM1_DIR_PIN       E1
#define PIPE							 E2

#define KEY1				  KBI0_P0
#define KEY2				  KBI0_P1
#define KEY3				  KBI0_P2
#define KEY4				  KBI0_P3

#define SW1 				       D2
#define SW2 				       D3
#define SW3 				       D4

//********************** PID *************************
//#define K_P								 0
extern u16 K_P;
//#define K_I								 0
extern u16 K_I;
//#define K_D								 0
extern u16 K_D;
extern u16 SHIFT_UPP;
extern u16 SHIFT_LOW;

#define DEAD							 0		//死区控制参数
#define LPF								 0.08	//一阶低通滤波器参数

//#define SHIFT_UPP	 				 100		//变速
//#define SHIFT_LOW	 				 0

//#define LIMIT_INTEGRAL_UPP  230
//#define LIMIT_INTEGRAL_LOW -230//130    23:250
extern int16	LIMIT_INTEGRAL_UPP;
extern int16	LIMIT_INTEGRAL_LOW;
extern   u16	reduce;
extern int16	ready_duty;

#define LIMIT_OUT_UPP			  750	//输出上限(抗饱和)//700
#define LIMIT_OUT_LOW			 -750	//输出下限

#define MOTOR_LIMIT		 			700//650
typedef struct 
{
	float Out[4];
	float	Out_D[2];
	float Error[4];
	
//	float K[3];
//	float Dout[2];
//  float Lpf_D;
//  float Limit_Integral[2];
//  float iSpeedCoe;						//变速因子
//  float Limit_Output[2];
}PID_InitTypeDef;
//**********************************************************

//********************** ADC *******************************
extern u16 			 adc_value[6];
extern u16 adc_buff[6][Avg+1];
extern int16 			velocity[4];			//速度变化量
extern int16  duty[2];

//#define EMS_COE 100						//电磁速度转换因子
extern u16 EMS[6];
extern u16 sp_coe;
#define LINE_OUT 15
#define CURVE 0.15//0.115	//转弯判断（差比和）
#define	BEND 0.08   //弯道中
#define ISLAND 0.05

//#define DEAD_D 100
//#define LPF_D 0.05
#define L_ADC  adc_value[0]
#define R_ADC  adc_value[5]
#define F_ADC  adc_value[2]
#define B_ADC  adc_value[3]
#define TL_ADC adc_value[1]
#define TR_ADC adc_value[4]
#define BAT_V  adc_once(ADC0_SE2,ADC_12bit)*4*5000.0/4096.0

#define DOSM(L,R) (((L)-(R))*1.0/((L)+(R))*1.0)	//差比和

#define ABS_DOSM(L,R) ((abs((L)-(R)))*1.0/((L)+(R))*1.0)

#define ADJUST_P(P0,PT,E) ((PT)*(E)*(E)+(P0)*(E))

#define LowPass_FirstFilter(lpf,sample,lastbuff) ((lpf)*(sample)+(1-(lpf))*(lastbuff))

#define Base_Voltage_LR 920
#define Base_Voltage_FB 1650
#define Base_Voltage_TLR 680

//**********************************************************
extern const u16	EMS20[6];
extern const u16	EMS_common[6];
extern const u16	EMS_stable[6];
extern const u16	EMS_super[6];


enum Direction    {Left,Right};
enum Direction2 	  	 {Go,Re};
enum P_I_D      	  	 {P,I,D};
enum UL          		 {Upp,Low};
enum D_out      		{Now,Last};
enum SW_PD        	{SI,RM,PM};
enum LAND  {Inside=2,Dir,Exit};

extern u8 	 		         page;
extern u8 				 			 send;
extern u8 							sw[3];
extern int16 					 lap[2];
extern u8  							 stop;
extern float 				 	wave[8];
extern float							D_x;
extern u16						 	 time;
extern float				 run_time;
extern u32 	 		baud_feedback;
extern int16 	speed[2][Avg+1];
extern PID_InitTypeDef 	 left;
extern PID_InitTypeDef  right;



void System_Init	 (void);
void Motor_Init		 (void);
void PID_Init			 (void);
void AD_sensor_Init(void);
void Others_Integration_Init (void);
void IRQ_Interrupt_Init			 (void);
void KBI_Interrupt_Init			 (void);
void Pit_Interrupt_Init			 (void);
void Bluetooth_Interrupt_Init(void);
void FLASH_Integration_Init	 (void);

void Oled(void);
void Dip_Switch(void);
u16  WeightValueFilter(u16 *ad_buff,u16 ad_value);
float PID_Regulator(PID_InitTypeDef *PID_Struct,u16 Set[], float Present,u8 dir);
void PID_Position(PID_InitTypeDef *PID,int16 Set, float Present,u8 dir);




//int16 Differential(int16 fun[],int16 t[]);//微分
//int16 Integral		(int16 fun[],int16 t[]);//积分


#endif
