#ifndef SERIAL_PLOT_H
#define SERIAL_PLOT_H

#include "stm32f1xx_hal.h"   // 根据实际 MCU 修改
#include "usart.h"

#define PLOT_NAME_LENGTH    50      // 变量名最大长度
#define PLOT_RX_BUFF_SIZE   256     // 接收缓冲区大小
#define PLOT_TX_BUFF_SIZE   256     // 发送缓冲区大小
#define PLOT_HEADER         0xAB    // 数据包帧头

#define PLOT_UART      huart3


// 回调函数类型
typedef void (*Plot_AssignCallback_t)(uint16_t index, float value);
typedef void (*Plot_SendCallback_t)(void);

// 初始化（必须最先调用）
// 参数: use_dma - 1: 使用 DMA 发送和接收; 0: 使用中断逐字节接收 + 阻塞发送
void SerialPlot_Init(uint8_t use_dma);

// 选择当前使用的变量列表及回调（支持运行时切换）
void SerialPlot_SelectConfig(char (*var_list)[PLOT_NAME_LENGTH], uint16_t var_num,
                             Plot_AssignCallback_t assign_cb, Plot_SendCallback_t send_cb);

// 添加一个浮点数到发送缓冲区（在 SendCallback 中调用）
void SerialPlot_AddVar(float value);

// 发送数据包（在用户任务或主循环中周期性调用）
void SerialPlot_SendPacket(void);

// 以下函数由串口中断调用（如果使用中断逐字节接收）
void SerialPlot_UART_IRQHandler(void);
void SerialPlot_SendWave(float angle, int vertical_out1, int moto_pwm1);
#endif