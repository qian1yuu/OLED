#include "PID.h"
#include "encoder.h"
#include "tim.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
#include "ai.h"
//传感器数据变量
int Encoder_Left,Encoder_Right;
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short	aacx,aacy,aacz;

//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
float Med_Angle=-3.8;//平衡时角度值偏移量（机械中值）//(6.8+4.4)/2

uint8_t stop;

//参数
// float Vertical_Kp=310,Vertical_Kd=0.9;      //直立环 数量级（Kp：0~1000、Kd：0~10）
// float Velocity_Kp=0.10,Velocity_Ki=0.004;      //速度环 数量级（Kp：0~1）
// float Turn_Kp=4.7,Turn_Kd=0.2;               //转向环

float Vertical_Kp,Vertical_Kd;      //直立环 数量级（Kp：0~1000、Kd：0~10）
float Velocity_Kp,Velocity_Ki;      //速度环 数量级（Kp：0~1）
float Turn_Kp,Turn_Kd;               //转向环




////直立环PD控制器
//输入：期望角度、真实角度、角速度
///
/// @param Med 期望角度
/// @param Angle 真实角度
/// @param gyro_Y 角速度
/// @return
int Vertical(float Med,float Angle,float gyro_Y) {
    int temp;
    temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
    return temp;
}

//速度环PI控制器
//输入：期望速度、左编码器、右编码器
///
/// @param Target       期望速度
/// @param encoder_L    左编码器
/// @param encoder_R    右编码器
/// @return
int Velocity(int Target,int encoder_L,int encoder_R)
{
    static int Err_LowOut_last,Encoder_S;
    static float a=0.7;
    int Err,Err_LowOut,temp;
    Velocity_Ki=Velocity_Kp/200;
    //1计算偏差值
    Err=(encoder_L+encoder_R)-Target;
    //2、低通滤波/////////////////////////////////////////////
    Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
    Err_LowOut_last=Err_LowOut;
    //3、积分
    Encoder_S+=Err_LowOut;
    //4、积分限幅(-20000~20000)
    Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
    if(stop==1)Encoder_S=0,stop=0;
    //5、速度环计算
    temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
    return temp;
}

//转向环PD控制器
//输入：角速度、角度值
/// @param gyro_Z  Z轴速度值
/// @param Target_turn 误差值
/// @return
int Turn(float gyro_Z,int Target_turn)
{
    int temp;
    temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
    return temp;
}



////////////每隔10ms调用一次
void Control(void) {
    int PWM_out;
    //1、读取编码器和陀螺仪的数据
    Encoder_Left=Read_Speed(&htim2);
    Encoder_Right=-Read_Speed(&htim4);
    mpu_dmp_get_data(&pitch,&roll,&yaw);
      MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
     MPU_Get_Accelerometer(&aacx,&aacy,&aacz);


    //2、将数据传入PID控制器，计算输出结果，即左右电机转速值
    Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
    Vertical_out=Vertical(Velocity_out+Med_Angle,roll,gyrox);
    Turn_out=Turn(gyroz,Target_turn);
    PWM_out=Vertical_out;       //前进的速度
    MOTO1=PWM_out-Turn_out;
    MOTO2=PWM_out+Turn_out;
    Limit(&MOTO1,&MOTO2);
    Load(MOTO1,MOTO2);


    SendPID_DataToPC(
       Target_Speed,
       roll,                  // 发送当前角度
       MOTO1,                 // 电机PWM
       0, Vertical_Kp, Vertical_Kd, Velocity_Kp
   );

}





















