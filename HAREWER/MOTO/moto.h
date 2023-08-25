#ifndef __MOTO_H
#define	__MOTO_H

#include "stm32f10x.h"

typedef struct pid_para{
   float target_value;           //�趨��Ŀ��ֵ����Ҫ�ﵽ������ֵ
   float current_value;          //��ǰֵ������Ϊ�ⲿ�ķ���ֵ��
   float CAL_value;              //������Ҫ�����ֵ
   float sum_error;              //�ۼƵ�ƫ��ֵ
   float error;                  //���ֵ
   float last_error;             //��һ�����ֵ
   float pre_error;              //����һ�����ֵ������ʽpid��ʹ�ã�
}PID;

void motoA(int mode);
void motoB(int mode);
int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
float Velocity_FeedbackControl(float Targetvalue, float Currentvalue, PID *pid);

#endif
