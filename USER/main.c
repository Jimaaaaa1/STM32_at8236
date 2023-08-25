#include "stm32f10x.h"

#include "delay.h"
#include "gpio.h"
#include "moto.h"
#include "pwm.h"
#include "adc.h"
#include "usart.h"
#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

 /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/



 
int TargetVelocity=800;  
float Velcity_Kp=5,  Velcity_Ki=0.1,  Velcity_Kd=0.1; //相关速度PID参数
float current_value;          //当前值（可认为外部的反馈值）
float CAL_value;              //计算需要输出的值
float error;                  //误差值
float last_error;             //上一次误差值
float pre_error;              //上上一次误差值（增量式pid中使用）
extern PID *pid1,*pid2;

int main(void)
 {	
	 int encoder_A,encoder_B;
	 int Velocity_PWM1,Velocity_PWM2;
	 u16 adcx;
	 float vcc;
   SystemInit(); //配置系统时钟为72M   
   delay_init();    //延时函数初始化
   uart_init(115200);		//串口初始化
   adc_Init();				//ADC1的初始化   
	
   PWM_Int(7199,0);      //初始化pwm输出 72000 000 /7199+1=10000可能有误？ 可能正确：7200*1/36000000 = 1/5000 s
   Encoder_Init_Tim2();
   Encoder_Init_Tim4(); 
	 pid1 = (struct pid_para *)malloc(28);
	 memset(pid1,0,28);
	 pid2 = (struct pid_para *)malloc(28);
	 memset(pid2,0,28);

  while(1)
	{
			adcx=Get_adc_Average(ADC_Channel_2,10);  //获取adc的值
			vcc=(float)adcx*(3.3*11/4096);     		 //求当前电压
			encoder_A=-Read_Encoder(2);
			encoder_B=Read_Encoder(4);               //读取编码器数值
		  //Velocity_PWM1=Velocity_A(TargetVelocity,encoder_A);
		  Velocity_PWM1=Velocity_FeedbackControl(TargetVelocity,encoder_A,pid1);
		  Velocity_PWM2=Velocity_FeedbackControl(TargetVelocity,encoder_B,pid2);
		  //Velocity_PWM2=Velocity_B(TargetVelocity,encoder_B);
		  //motoA(Velocity_PWM1); 
      motoB(Velocity_PWM2);
	  	 
			printf("当前电压=%6.2f V  Encoder_A = %d  Encoder_B=%d  TargetVelocity=%d  Velocity_PWM2=%d\r\n",vcc,encoder_A,encoder_B,TargetVelocity,Velocity_PWM2);				//打印当前电压，保留小数点后两位	
	}
 }

