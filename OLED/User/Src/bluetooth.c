#include "bluetooth.h"
#include "usart.h"
#include"stdio.h"
uint8_t Fore = 0, Back = 0, Left = 0, Right = 0;
 uint8_t rx_buf[2];
/**
  * @brief 串口接收完成标准回调函数（HAL 标准写法）
  * @param huart 句柄
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 判断是否是 USART3 触发的中断，防止其他串口误入
    if (huart->Instance == USART3)
    {

        // // ========== 【直接打印】验证中断是否进入 ==========
        // uint8_t print_buf[30];
        // int len = sprintf((char*)print_buf, "RX:0x%02X\r\n", rx_buf[0]);
        // HAL_UART_Transmit(&huart3, print_buf, len, 100);   // 通过 USART3 原路打印出去
        // // =================================================

        // 1. 解析接收到的数据（调用你的业务逻辑）
        uint8_t data = rx_buf[0]; // rx_buf 需要在外部定义好
        if(data == 0x00)  { Fore=0; Back=0; Left=0; Right=0; } // 刹
        else if(data == 0x01) { Fore=1; Back=0; Left=0; Right=0; } // 前
        else if(data == 0x05) { Fore=0; Back=1; Left=0; Right=0; } // 后
        else if(data == 0x03) { Fore=0; Back=0; Left=0; Right=1; } // 右
        else if(data == 0x07) { Fore=0; Back=0; Left=1; Right=0; } // 左
        else                  { Fore=0; Back=0; Left=0; Right=0; } // 刹

        // 2. 重新开启下一次中断接收（标准流程）
        HAL_UART_Receive_IT(huart, rx_buf, 1);
    }
}