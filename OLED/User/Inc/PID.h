#ifndef OLED_PID_H
#define OLED_PID_H
#include "stm32f1xx_hal.h"
extern int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
int Velocity(int Target,int encoder_L,int encoder_R);
int Vertical(float Med,float Angle,float gyro_Y);
int Turn(float gyro_Z,int Target_turn);
void Control(void);

#endif //OLED_PID_H