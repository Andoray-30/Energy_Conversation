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
 * DIR 默认拉低（接收模式），清空缓冲区，启动单字节中断接收
 * 不使能 IDLE 中断——IDLE 仅在 RS485_StartReceive 中按事务周期启用
 */
void RS485_Init(void)
{
    RS485_SetRxMode();

    UART1_ClearRxBuffer();

    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

/**
 * 启动一次完整的 IDLE+IT 接收
 * 调用时机：每次发送查询帧后，开始等待从站响应前
 * 流程：确保接收模式 -> 中止上次接收 -> 清空缓冲 -> 使能 IDLE -> 启动 IT
 */
void RS485_StartReceive(void)
{
    RS485_SetRxMode();

    /* 中止可能正在进行的接收，释放 HAL 状态机 */
    HAL_UART_AbortReceive_IT(&huart1);

    UART1_ClearRxBuffer();

    /* 清除可能残留的 IDLE 标志，再使能 IDLE 中断 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    /* 启动单字节中断接收 */
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}

/**
 * 阻塞发送一帧数据
 * 发送前拉高 DIR 进入发送模式，发送后等待移位寄存器清空再拉低 DIR
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

    RS485_SetRxMode();

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
        /* 一帧结束，关闭 IDLE 中断，直到下次 RS485_StartReceive 再启用 */
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
