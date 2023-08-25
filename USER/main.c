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
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/



 
int TargetVelocity=800;  
float Velcity_Kp=5,  Velcity_Ki=0.1,  Velcity_Kd=0.1; //����ٶ�PID����
float current_value;          //��ǰֵ������Ϊ�ⲿ�ķ���ֵ��
float CAL_value;              //������Ҫ�����ֵ
float error;                  //���ֵ
float last_error;             //��һ�����ֵ
float pre_error;              //����һ�����ֵ������ʽpid��ʹ�ã�
extern PID *pid1,*pid2;

int main(void)
 {	
	 int encoder_A,encoder_B;
	 int Velocity_PWM1,Velocity_PWM2;
	 u16 adcx;
	 float vcc;
   SystemInit(); //����ϵͳʱ��Ϊ72M   
   delay_init();    //��ʱ������ʼ��
   uart_init(115200);		//���ڳ�ʼ��
   adc_Init();				//ADC1�ĳ�ʼ��   
	
   PWM_Int(7199,0);      //��ʼ��pwm��� 72000 000 /7199+1=10000�������� ������ȷ��7200*1/36000000 = 1/5000 s
   Encoder_Init_Tim2();
   Encoder_Init_Tim4(); 
	 pid1 = (struct pid_para *)malloc(28);
	 memset(pid1,0,28);
	 pid2 = (struct pid_para *)malloc(28);
	 memset(pid2,0,28);

  while(1)
	{
			adcx=Get_adc_Average(ADC_Channel_2,10);  //��ȡadc��ֵ
			vcc=(float)adcx*(3.3*11/4096);     		 //��ǰ��ѹ
			encoder_A=-Read_Encoder(2);
			encoder_B=Read_Encoder(4);               //��ȡ��������ֵ
		  //Velocity_PWM1=Velocity_A(TargetVelocity,encoder_A);
		  Velocity_PWM1=Velocity_FeedbackControl(TargetVelocity,encoder_A,pid1);
		  Velocity_PWM2=Velocity_FeedbackControl(TargetVelocity,encoder_B,pid2);
		  //Velocity_PWM2=Velocity_B(TargetVelocity,encoder_B);
		  //motoA(Velocity_PWM1); 
      motoB(Velocity_PWM2);
	  	 
			printf("��ǰ��ѹ=%6.2f V  Encoder_A = %d  Encoder_B=%d  TargetVelocity=%d  Velocity_PWM2=%d\r\n",vcc,encoder_A,encoder_B,TargetVelocity,Velocity_PWM2);				//��ӡ��ǰ��ѹ������С�������λ	
	}
 }

