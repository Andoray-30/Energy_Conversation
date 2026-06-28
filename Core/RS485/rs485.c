/**
 * @file    rs485.c
 * @brief   RS485 串口收发底层驱动
 *          负责 USART1 物理收发、DIR 方向控制、IDLE+IT 接收
 */

#include "rs485.h"
#include "usart.h"
#include <string.h>

/*============================= 私有变量 =============================*/

/* 单字节接收暂存，HAL_IT 模式每次接收1字节存入此处 */
static uint8_t uart1_rx_byte = 0;

/* 接收帧缓冲区 */
static uint8_t uart1_rx_buf[UART1_RX_BUF_SIZE];

/* 当前帧已接收字节数 */
volatile uint16_t uart1_rx_len = 0;

/* 帧完成标志：IDLE 中断或缓冲区溢出时置1 */
static volatile uint8_t uart1_rx_frame_complete = 0;

/* 溢出标志：接收字节超过缓冲区大小时置1 */
static volatile uint8_t uart1_rx_overflow = 0;

/*============================= 私有函数 =============================*/

/**
 * 清空接收缓冲区及所有状态标志
 * 在启动新一轮接收前调用
 */
static void UART1_ClearRxBuffer(void)
{
    uart1_rx_len = 0;
    uart1_rx_frame_complete = 0;
    uart1_rx_overflow = 0;
}

/* DIR 引脚拉高，RS485 收发器进入发送模式 */
static void RS485_SetTxMode(void)
{
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);
}

/* DIR 引脚拉低，RS485 收发器进入接收模式 */
static void RS485_SetRxMode(void)
{
    HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_RESET);
}

/*============================= 公共函数 =============================*/

/**
 * 初始化 RS485 模块
 * DIR 默认拉低（接收模式），清空缓冲区
 * 不使能 IDLE 中断——IDLE 由 RS485_Send 尾部或 RS485_StartReceive 启用
 */
void RS485_Init(void)
{
    RS485_SetRxMode();

    UART1_ClearRxBuffer();
}

/**
 * 重新启动接收链路
 * 仅用于初始化阶段或异常恢复后重新挂起接收
 * 正常事务流程中由 RS485_Send 尾部自动启动，无需再调用此函数
 */
HAL_StatusTypeDef RS485_StartReceive(void)
{
    HAL_UART_AbortReceive(&huart1);

    RS485_SetRxMode();

    UART1_ClearRxBuffer();

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_FLUSH_DRREGISTER(&huart1);

    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    return HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

/**
 * 阻塞发送一帧数据，发送完成后自动启动接收链路
 *
 * 时序：DIR=TX → 阻塞发送 → 等TC → DIR=RX → 关中断 → 清回环 → 启接收 → 开中断
 *
 * Send 期间若 RS485 收发器存在回环（DIR 只控 DE 的拓扑），
 * 回环数据会堆入软件 buf 和产生 ORE，尾部全部清掉。
 * 若收发器同时控 DE+RE（DIR=TX 时接收器禁用），清操作为防御性空操作无害。
 *
 * @param data  待发送数据
 * @param len   数据长度
 * @retval      HAL_OK 成功
 */
HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;

    if (data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    RS485_SetTxMode();

    ret = HAL_UART_Transmit(&huart1, data, len, RS485_TX_TIMEOUT_MS);

    /* 等待移位寄存器发送完成，确保最后一字节已离开 TX 引脚 */
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
        ;

    /* 切回接收模式——从站响应数据从此刻起可进入 RX 引脚 */
    RS485_SetRxMode();

    /* 原子窗口：清回环残余 + 重启接收，防止从站首字节插入中途 */
    __disable_irq();

    /* 清 ORE：回环 overrun (拓扑A) 或使能切换毛刺 (拓扑B) */
    __HAL_UART_CLEAR_OREFLAG(&huart1);

    /* 清 RXNE：读走 DR 中可能的回环尾字节或毛刺字节 */
    __HAL_UART_FLUSH_DRREGISTER(&huart1);

    /* 清软件缓冲区：回环垃圾 (拓扑A必须) / 干净buf (拓扑B无害) */
    UART1_ClearRxBuffer();

    /* 使能 IDLE 中断，用于检测从站响应帧结束 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    /* 启动单字节中断接收 */
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);

    __enable_irq();

    return ret;
}

/**
 * 查询帧是否接收完成
 * @retval  1 帧完成（正常结束或溢出截断），0 仍在接收
 */
uint8_t RS485_IsFrameComplete(void)
{
    return uart1_rx_frame_complete;
}

uint8_t RS485_IsRxOverflow(void)
{
    return uart1_rx_overflow;
}

/**
 * 取出已接收的帧数据
 * 如果帧未完成则返回0，不拷贝任何数据
 * @param pBuf    目标缓冲区
 * @param BufSize 目标缓冲区大小
 * @retval        实际拷贝字节数
 */
uint16_t RS485_GetReceivedData(uint8_t *pBuf, uint16_t BufSize)
{
    uint16_t copy_len;

    if (pBuf == NULL || BufSize == 0)
    {
        return 0;
    }

    /* 帧未完成，不提供数据 */
    if (!uart1_rx_frame_complete)
    {
        return 0;
    }

    copy_len = uart1_rx_len;

    if (copy_len > BufSize)
    {
        copy_len = BufSize;
    }

    memcpy(pBuf, uart1_rx_buf, copy_len);

    return copy_len;
}

/**
 * IDLE 空闲中断处理
 * 当总线空闲一个字符时间后触发，表示一帧 Modbus 数据已完整接收
 * 禁止 IDLE 中断和后续字节接收，置帧完成标志，等待上层取走数据
 * @param huart  串口句柄
 */
void RS485_UARTIdleHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 一帧结束，关闭 IDLE 中断，直到下次 RS485_Send 再启用 */
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);

        uart1_rx_frame_complete = 1;
    }
}

/**
 * HAL 串口接收完成回调（单字节 IT 模式）
 * 每收到1字节，存入缓冲区，并重新启动下一字节的接收
 * 如果帧已完成（IDLE 已触发）或缓冲区溢出，则丢弃后续字节
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 帧已完成（IDLE已触发），丢弃后续字节 */
        if (uart1_rx_frame_complete)
        {
            return;
        }

        if (uart1_rx_len < UART1_RX_BUF_SIZE)
        {
            /* 存入当前字节，长度递增 */
            uart1_rx_buf[uart1_rx_len++] = uart1_rx_byte;

            /* 重新启动单字节接收，等待下一字节 */
            HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
        }
        else
        {
            /* 缓冲区已满，标记溢出，视为帧完成 */
            uart1_rx_overflow = 1;
            uart1_rx_frame_complete = 1;

            __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
        }
    }
}
