#include "moto.h"
#include "PWM.h"

#include <stdio.h>
#include <stdlib.h>

/**************************************************************************
函数功能：电机的正反转
入口参数：mode   mode=0时为正转  mode=1时反转
返回  值：无
**************************************************************************/
extern float Velcity_Kp,  Velcity_Ki,  Velcity_Kd; //相关速度PID参数
extern float current_value,CAL_value,error,last_error,pre_error;              

extern float Position_KP,Position_KI,Position_KD,Velocity_KP,Velocity_KI;      //PID系数
 
void motoA(int mode)
{
		PWMA_IN1=0;PWMA_IN2=mode;
}
void motoB(int mode)
{
		PWMB_IN1=0;PWMB_IN2=mode;
}

/***************************************************************************
函数功能：电机的闭环控制
入口参数：电机的编码器值
返回值  ：电机的PWM
***************************************************************************/
 
PID *pid1,*pid2;

//PID控制，不使用结构体指针
//float Velocity_FeedbackControl(float Targetvalue, float Currentvalue, PID *pid){
//			//printf("%f,%f,%f,%f,%f",current_value,CAL_value,error,last_error,pre_error);
//      error =  Targetvalue - Currentvalue;
//      CAL_value +=(Velcity_Kp * (error - last_error) + Velcity_Ki * error + Velcity_Kd * (error - 2* last_error + pre_error));
//      pre_error = last_error;
//      last_error = error;
//      current_value = CAL_value;                    //由于没有外部反馈  就默认当值已经到达计算值 
//      return current_value;
//}

//PID控制，使用结构体指针
float Velocity_FeedbackControl(float Targetvalue, float Currentvalue, PID *pid){
			//printf("%f,%f,%f,%f,%f,%f,%f",pid->CAL_value,pid->current_value,pid->error,pid->last_error,pid->pre_error,pid->sum_error,pid->target_value);
      pid->error =  Targetvalue - Currentvalue;
      pid->CAL_value +=(Velcity_Kp * (pid->error - pid->last_error) + Velcity_Ki * pid->error + Velcity_Kd * (pid->error - 2* pid->last_error + pid->pre_error));
      pid->pre_error = pid->last_error;
      pid->last_error = pid->error;
      pid->current_value = pid->CAL_value;                    //由于没有外部反馈  就默认当值已经到达计算值 
      return pid->current_value;
}



//源代码PI控制
//int Velocity_A(int TargetVelocity, int CurrentVelocity)
//{
//		int Bias;  //定义相关变量
//		static int ControlVelocity, Last_bias; //静态变量，函数调用结束后其值依然存在
//		
//		Bias=TargetVelocity-CurrentVelocity; //求速度偏差
//		
//		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //增量式PI控制器
//                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
//	                                                                 //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
//		Last_bias=Bias;	
//		return ControlVelocity; //返回速度控制值 
//	
//}
//int Velocity_B(int TargetVelocity, int CurrentVelocity)
//{  
//    int Bias;  //定义相关变量
//		static int ControlVelocity, Last_bias; //静态变量，函数调用结束后其值依然存在
//		
//		Bias=TargetVelocity-CurrentVelocity; //求速度偏差
//		
//		ControlVelocity+=Velcity_Kp*(Bias-Last_bias)+Velcity_Ki*Bias;  //增量式PI控制器
//                                                                   //Velcity_Kp*(Bias-Last_bias) 作用为限制加速度
//	                                                             //Velcity_Ki*Bias             速度控制值由Bias不断积分得到 偏差越大加速度越大
//		Last_bias=Bias;	
//		return ControlVelocity; //返回速度控制值
//}


