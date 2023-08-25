#include "moto.h"
#include "PWM.h"

#include <stdio.h>
#include <stdlib.h>

/**************************************************************************
�������ܣ����������ת
��ڲ�����mode   mode=0ʱΪ��ת  mode=1ʱ��ת
����  ֵ����
**************************************************************************/
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //����ٶ�PID����
extern float current_value,CAL_value,error,last_error,pre_error;              

extern float Position_KP,Position_KI,Position_KD,Velocity_KP,Velocity_KI;      //PIDϵ��
 
void motoA(int mode)
{
		PWMA_IN1=0;PWMA_IN2=mode;
}
void motoB(int mode)
{
		PWMB_IN1=0;PWMB_IN2=mode;
}

/***************************************************************************
�������ܣ�����ıջ�����
��ڲ���������ı�����ֵ
����ֵ  �������PWM
***************************************************************************/
 
PID *pid1,*pid2;

//PID���ƣ���ʹ�ýṹ��ָ��
//float Velocity_FeedbackControl(float Targetvalue, float Currentvalue, PID *pid){
//			//printf("%f,%f,%f,%f,%f",current_value,CAL_value,error,last_error,pre_error);
//      error =  Targetvalue - Currentvalue;
//      CAL_value +=(Velcity_Kp * (error - last_error) + Velcity_Ki * error + Velcity_Kd * (error - 2* last_error + pre_error));
//      pre_error = last_error;
//      last_error = error;
//      current_value = CAL_value;                    //����û���ⲿ����  ��Ĭ�ϵ�ֵ�Ѿ��������ֵ 
//      return current_value;
//}

//PID���ƣ�ʹ�ýṹ��ָ��
float Velocity_FeedbackControl(float Targetvalue, float Currentvalue, PID *pid){
			//printf("%f,%f,%f,%f,%f,%f,%f",pid->CAL_value,pid->current_value,pid->error,pid->last_error,pid->pre_error,pid->sum_error,pid->target_value);
      pid->error =  Targetvalue - Currentvalue;
      pid->CAL_value +=(Velcity_Kp * (pid->error - pid->last_error) + Velcity_Ki * pid->error + Velcity_Kd * (pid->error - 2* pid->last_error + pid->pre_error));
      pid->pre_error = pid->last_error;
      pid->last_error = pid->error;
      pid->current_value = pid->CAL_value;                    //����û���ⲿ����  ��Ĭ�ϵ�ֵ�Ѿ��������ֵ 
      return pid->current_value;
}



//Դ����PI����
//int Velocity_A(int TargetVelocity, int CurrentVelocity)
//{
//		int Bias;  //������ر���
//		static int ControlVelocity, Last_bias; //��̬�������������ý�������ֵ��Ȼ����
//		
//		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
//		
//		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //����ʽPI������
//                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
//	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
//		Last_bias=Bias;	
//		return ControlVelocity; //�����ٶȿ���ֵ 
//	
//}
//int Velocity_B(int TargetVelocity, int CurrentVelocity)
//{  
//    int Bias;  //������ر���
//		static int ControlVelocity, Last_bias; //��̬�������������ý�������ֵ��Ȼ����
//		
//		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
//		
//		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //����ʽPI������
//                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
//	                                                             //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
//		Last_bias=Bias;	
//		return ControlVelocity; //�����ٶȿ���ֵ
//}


