#ifndef OLED_MOTOR_H
#define OLED_MOTOR_H



#include "stm32f1xx_hal.h"
int abs(int p);
void Load(int moto1,int moto2);
void Limit(int *motoA,int *motoB);
#endif //OLED_MOTOR_H