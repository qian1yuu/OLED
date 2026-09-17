#include "serial_plot.h"
#include <string.h>
#include <stdlib.h>
#include "stdio.h"
// 引用外部全局串口句柄（需要在 main.c 或其他地方定义）

// 发送波形数据给 SerialPlot
void SerialPlot_SendWave(float angle, int vertical_out1, int moto_pwm1)
{
    char buf[128];
    // 格式符严格对应：%f对应float，%d对应int
    sprintf(buf, "%f,%d,%d\n", angle, vertical_out1, moto_pwm1);
    HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), 50);
}

// ==================== 静态变量 ====================
static uint8_t s_use_dma = 0;

// 发送缓冲区
static uint8_t s_tx_buf[PLOT_TX_BUFF_SIZE];
static uint16_t s_tx_offset = 0;

// 接收行缓冲区（逐字节方式）
static uint8_t s_rx_line[PLOT_RX_BUFF_SIZE];
static uint16_t s_rx_line_len = 0;

// 用户配置
static char (*s_var_list)[PLOT_NAME_LENGTH] = 0;
static uint16_t s_var_num = 0;
static Plot_AssignCallback_t s_assign_cb = 0;
static Plot_SendCallback_t s_send_cb = 0;

// 临时存储（用于切换配置）
static char (*s_temp_var_list)[PLOT_NAME_LENGTH] = 0;
static uint16_t s_temp_var_num = 0;
static Plot_AssignCallback_t s_temp_assign_cb = 0;
static Plot_SendCallback_t s_temp_send_cb = 0;

// DMA 接收相关
static uint8_t s_dma_rx_buf[PLOT_RX_BUFF_SIZE];
static uint8_t s_dma_rx_temp[PLOT_RX_BUFF_SIZE];
static uint16_t s_dma_rx_len = 0;

// ==================== 辅助函数 ====================
static int is_variable(char *str, uint16_t num, char (*list)[PLOT_NAME_LENGTH]) {
    for (int i = 0; i < num; i++) {
        if (strcmp(str, list[i]) == 0)
            return i;
    }
    return -1;
}

/* 解析一行字符串，格式如 "p_angle=1.23" */
static void parse_line(uint8_t *line, uint16_t len) {
    char var_name[PLOT_NAME_LENGTH];
    char num_str[50];
    float value;
    int i = 0, j = 0, k;

    // 提取变量名（直到 '='）
    for (k = 0; k < len && k < PLOT_NAME_LENGTH - 1; k++) {
        char c = line[k];
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c == '_') || (c >= '0' && c <= '9') || (c == ' ')) {
            var_name[i++] = c;
        } else if (c == '=') {
            break;
        }
    }
    var_name[i] = '\0';
    // 去除末尾空格
    while (i > 0 && var_name[i-1] == ' ') var_name[--i] = '\0';

    // 提取数值部分
    for (; k < len && k < 50; k++) {
        char c = line[k];
        if ((c >= '0' && c <= '9') || c == '.' || c == '-') {
            num_str[j++] = c;
        }
    }
    num_str[j] = '\0';
    value = (float)atof(num_str);

    int idx = is_variable(var_name, s_var_num, s_var_list);
    if (idx >= 0 && s_assign_cb) {
        s_assign_cb((uint16_t)idx, value);
    }
}

// ==================== 逐字节接收处理（中断方式） ====================
static void push_byte(uint8_t data) {
    if (data == '\n' || data == '\r') {
        if (s_rx_line_len > 0) {
            s_rx_line[s_rx_line_len] = '\0';
            parse_line(s_rx_line, s_rx_line_len);
            s_rx_line_len = 0;
        }
    } else if (s_rx_line_len < PLOT_RX_BUFF_SIZE - 1) {
        s_rx_line[s_rx_line_len++] = data;
    } else {
        // 缓冲区溢出，丢弃
        s_rx_line_len = 0;
    }
}

// ==================== DMA 接收处理（空闲中断） ====================
static void process_dma_rx(void) {
    // 假设使用空闲中断，获取已接收字节数
    // 这里需要根据 HAL 库实现：__HAL_UART_GET_FLAG(...) 等
    // 以下为示例逻辑
    if (s_dma_rx_len > 0) {
        // 将接收到的数据复制到临时缓冲区并逐字节解析（遇到换行）
        memcpy(s_dma_rx_temp, s_dma_rx_buf, s_dma_rx_len);
        // 重新启动 DMA 接收
        HAL_UART_Receive_DMA(&PLOT_UART, s_dma_rx_buf, PLOT_RX_BUFF_SIZE);
        // 逐字节处理
        for (uint16_t i = 0; i < s_dma_rx_len; i++) {
            push_byte(s_dma_rx_temp[i]);
        }
        s_dma_rx_len = 0;
    }
}

// ==================== 公共 API ====================
void SerialPlot_Init(uint8_t use_dma) {
    s_use_dma = use_dma;
    s_tx_offset = 0;

    if (use_dma) {
        // 启动 DMA 接收（循环模式或单次，建议配合空闲中断）
        HAL_UART_Receive_DMA(&PLOT_UART, s_dma_rx_buf, PLOT_RX_BUFF_SIZE);
        __HAL_UART_ENABLE_IT(&PLOT_UART, UART_IT_IDLE);  // 使能空闲中断
    } else {
        // 启动中断逐字节接收
        HAL_UART_Receive_IT(&PLOT_UART, s_dma_rx_buf, 1);  // 每次接收1字节
    }
}

void SerialPlot_SelectConfig(char (*var_list)[PLOT_NAME_LENGTH], uint16_t var_num,
                             Plot_AssignCallback_t assign_cb, Plot_SendCallback_t send_cb) {
    s_temp_var_list = var_list;
    s_temp_var_num = var_num;
    s_temp_assign_cb = assign_cb;
    s_temp_send_cb = send_cb;

    s_var_list = s_temp_var_list;
    s_var_num = s_temp_var_num;
    s_assign_cb = s_temp_assign_cb;
    s_send_cb = s_temp_send_cb;
}

void SerialPlot_AddVar(float value) {
    if (s_tx_offset + 4 <= PLOT_TX_BUFF_SIZE) {
        memcpy(s_tx_buf + s_tx_offset, &value, 4);
        s_tx_offset += 4;
    }
}

void SerialPlot_SendPacket(void) {
    if (s_send_cb == 0)   // 只需要检查回调，串口句柄固定存在
        return;
    // 重置偏移，填入帧头
    s_tx_offset = 0;
  // s_tx_buf[0] = PLOT_HEADER;

    // 调用用户打包函数（内部会调用 SerialPlot_AddVar）
    s_send_cb();

    // 发送数据（DMA 或阻塞）
    if (s_use_dma) {
        HAL_UART_Transmit_DMA(&PLOT_UART, s_tx_buf, s_tx_offset);
    } else {
        HAL_UART_Transmit(&PLOT_UART, s_tx_buf, s_tx_offset, 100);  // 超时100ms
    }
}


// ==================== 中断处理入口（需要放在串口中断中调用） ====================
void SerialPlot_UART_IRQHandler(void) {
    // 直接使用全局 PLOT_UART
    if (s_use_dma) {
        // DMA 模式：处理空闲中断
        if (__HAL_UART_GET_FLAG(&PLOT_UART, UART_FLAG_IDLE) != RESET) {
            __HAL_UART_CLEAR_IDLEFLAG(&PLOT_UART);
            HAL_UART_DMAStop(&PLOT_UART);
            // 计算已接收字节数
            s_dma_rx_len = PLOT_RX_BUFF_SIZE - __HAL_DMA_GET_COUNTER(PLOT_UART.hdmarx);
            process_dma_rx();
            HAL_UART_Receive_DMA(&PLOT_UART, s_dma_rx_buf, PLOT_RX_BUFF_SIZE);
        }
    } else {
        // 逐字节中断模式：处理接收完成中断
        if (__HAL_UART_GET_FLAG(&PLOT_UART, UART_FLAG_RXNE) != RESET) {
            uint8_t data = (uint8_t)(PLOT_UART.Instance->DR & 0xFF);//RDRH7//DRf4
            push_byte(data);
            // 继续接收下一个字节
            HAL_UART_Receive_IT(&PLOT_UART, s_dma_rx_buf, 1);
        }
    }
}
