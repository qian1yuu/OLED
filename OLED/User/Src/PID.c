#include "PID.h"
#include "encoder.h"
#include "tim.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
#include "serial_plot.h"
#include "ai.h"
#include "bluetooth.h"



//传感器数据变量
int Encoder_Left,Encoder_Right;
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short	aacx,aacy,aacz;




// ========== 添加遥控相关的外部变量和宏定义 ==========
extern uint8_t Fore, Back, Left, Right;
extern float distance;

#define SPEED_Y  25   // 前后最大速度
#define SPEED_Z  250   // 转向最大角度,转向环的目标值
// ====================================================



//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
float Med_Angle=0.23;//平衡时角度值偏移量（机械中值）//(6.8+4.4)/2
uint8_t stop;

// //参数
//参数
// float Vertical_Kp=320,Vertical_Kd=0.5784;      //直立环 数量级（Kp：0~1000、Kd：0~10）
// float Velocity_Kp=0.4015,Velocity_Ki=0.0300;      //速度环 数量级（Kp：0~1）


// float Vertical_Kp=340, Vertical_Kd=1.10;      // 折中阻尼
// float Velocity_Kp=0.35, Velocity_Ki=0.005;    // 恢复力度，固定积分值
// float Turn_Kp=3.7,Turn_Kd=0.004;               //转向环
float Vertical_Kp=340, Vertical_Kd=1.15;      // 折中阻尼
float Velocity_Kp=0.315, Velocity_Ki=0.005;    // 恢复力度，固定积分值
float Turn_Kp=3.7,Turn_Kd=0.004;               //转向环


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
// int Velocity(int Target,int encoder_L,int encoder_R)
// {
//     static int Err_LowOut_last,Encoder_S;
//     static float a=0.7;
//     int Err,Err_LowOut,temp;
//    // Velocity_Ki=Velocity_Kp/200;
//     //1计算偏差值
//     Err=(encoder_L+encoder_R)-Target;
//     //2、低通滤波/////////////////////////////////////////////
//     Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
//     Err_LowOut_last=Err_LowOut;
//     //3、积分
//     Encoder_S+=Err_LowOut;
//     //4、积分限幅(-20000~20000)
//     Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
//     if(stop==1)Encoder_S=0,stop=0;
//     //5、速度环计算
//     temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
//     return temp;
// }


int Velocity(int Target,int encoder_L,int encoder_R)
{
    static int Err_LowOut_last, Encoder_S;
    static float a = 0.4;   // 建议把滤波系数也改小，提升响应（你目前还是0.7）
    int Err, Err_LowOut, temp;
    Err = (encoder_L+encoder_R) - Target;
    // 低通滤波
    Err_LowOut = (1-a)*Err + a*Err_LowOut_last;
    Err_LowOut_last = Err_LowOut;

    // 积分分离：只有偏差较大时才积分，防止静止时累计
    if (abs(Err_LowOut) > 3) {   // 死区阈值，可调
        Encoder_S += Err_LowOut;
    }

    // 积分限幅大幅减小
    Encoder_S = Encoder_S > 20000 ? 20000 : (Encoder_S < -20000 ? -20000 : Encoder_S);
    // 如果 stop 触发清零，保持
    if(stop == 1) { Encoder_S = 0; stop = 0; }

    temp = Velocity_Kp * Err_LowOut + Velocity_Ki * Encoder_S;
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



    // ========== 遥控处理（前后 + 左右 + 超声波防撞）==========
    // 1. 前后控制


    if ((Fore == 0) && (Back == 0)) {
        Target_Speed = 0;          // 无指令，停车
    }
    if (Fore == 1) {
        // 前进时检测距离：如果距离小于30cm（且有效），强制刹停
        if (distance > 0 && distance < 30) {
            Fore == 0;
                           // 急停
        } else {
            Target_Speed++;        // 正常前进
        }
    }
    if (Back == 1) {
        Target_Speed--;            // 后退（不防撞）
    }

    // 速度限幅
    if (Target_Speed > SPEED_Y)  Target_Speed = SPEED_Y;
    if (Target_Speed < -SPEED_Y) Target_Speed = -SPEED_Y;

    // 2. 左右转向
    if ((Left == 0) && (Right == 0)) {
        Target_turn = 0;
    }
    if (Left == 1) {
        Target_turn -= 5;        // 左转（符号根据你的转向环方向调整）
    }
    if (Right == 1) {
        Target_turn += 5;        // 右转
    }

    // 转向限幅
    if (Target_turn > SPEED_Z)  Target_turn = SPEED_Z;
    if (Target_turn < -SPEED_Z) Target_turn = -SPEED_Z;

    // ========== 遥控处理结束 ==========

    //2、将数据传入PID控制器，计算输出结果，即左右电机转速值
    Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
    Vertical_out=Vertical(Velocity_out+Med_Angle,roll, gyrox);
    Turn_out=Turn(gyroz,Target_turn);
    PWM_out=Vertical_out;       //前进的速度
    MOTO1=PWM_out-Turn_out;
    MOTO2=PWM_out+Turn_out;
    Limit(&MOTO1,&MOTO2);
    Load(MOTO1,MOTO2);

            //AI PID
     // float error = roll - (Velocity_out + Med_Angle);
     // float p = Vertical_Kp * error;
     // float d = Vertical_Kd * gyrox;
     //
     // SendPID_DataToPC(
     //     Velocity_out + Med_Angle,  // 目标角度
     //     roll,                     // 实际角度
     //     (float)HAL_GetTick()/1000.0f,  // 时间索引
     //     error,                    // 误差
     //     p,                        // P 项
     //     0,                        // I 项（直立环无积分）
     //     d                         // D 项
     // );



    //SerialPlot_SendWave(roll, Vertical_out, MOTO1);//波形图
}





















