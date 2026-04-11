#include "main.h"
#include <stdio.h>
#include "usart.h"
#include "stdint.h"
#include "string.h"
#include "ai.h"

// 你指定的串口
#define AI_uart   &huart3

#define RX_BUFFER_SIZE 256
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;

// 平衡车全局参数（直接对接你的代码）
extern float Vertical_Kp, Vertical_Kd;      // 直立环
extern float Velocity_Kp, Velocity_Ki;      // 速度环
extern float Turn_Kp, Turn_Kd;              // 转向环

// 发送数据给 Python
void SendPID_DataToPC(
    float setpoint, float input, float index,
    float error, float p, float i, float d
) {
    uint32_t timestamp = HAL_GetTick();
    char buffer[128];
    sprintf(buffer, "%lu,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
            timestamp, setpoint, input, index, error, p, i, d);

    // 发送到 huart3
    HAL_UART_Transmit(AI_uart, (uint8_t*)buffer, strlen(buffer), 100);
}

// 解析指令 PID,Kp,Kd,KI
void ProcessAICommand(char* command) {
    float kp, ki, kd;

    if (strncmp(command, "PID,", 4) == 0) {
        sscanf(command, "PID,%f,%f,%f", &kp, &ki, &kd);

        // 直接更新你的平衡车参数
        Vertical_Kp = kp;
        Vertical_Kd = kd;
        Velocity_Kp = ki;
        // Velocity_Ki 自动计算
        Velocity_Ki = Velocity_Kp / 200.0f;

        printf("AI 调参成功：P=%.2f D=%.2f I=%.2f\r\n", kp, ki, kd);
    }
}

// 初始化接收
void AI_UART_Init(void) {
    rx_index = 0;
    HAL_UART_Receive_IT(AI_uart, &rx_buffer[rx_index], 1);
}

// 串口中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == AI_uart) {
        uint8_t ch = rx_buffer[rx_index];

        // 收到换行，指令结束
        if (ch == '\n' || ch == '\r') {
            rx_buffer[rx_index] = '\0';
            ProcessAICommand((char*)rx_buffer);
            rx_index = 0;
        }
        else {
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_index++;
            }
        }

        // 继续接收
        HAL_UART_Receive_IT(AI_uart, &rx_buffer[rx_index], 1);
    }
}