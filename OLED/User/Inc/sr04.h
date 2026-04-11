#ifndef OLED_SR04_H
#define OLED_SR04_H
#include "stm32f1xx_hal.h"


void GET_Distance(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif //OLED_SR04_H