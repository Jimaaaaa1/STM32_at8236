#ifndef __MOTO_H
#define	__MOTO_H

#include "stm32f10x.h"

typedef struct pid_para{
   float target_value;           //设定的目标值及需要达到的最终值
   float current_value;          //当前值（可认为外部的反馈值）
   float CAL_value;              //计算需要输出的值
   float sum_error;              //累计的偏差值
   float error;                  //误差值
   float last_error;             //上一次误差值
   float pre_error;              //上上一次误差值（增量式pid中使用）
}PID;

void motoA(int mode);
void motoB(int mode);
int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);
float Velocity_FeedbackControl(float Targetvalue, float Currentvalue, PID *pid);

#endif
