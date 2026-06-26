/**
 * @file    rs485.h
 * @brief   RS485 串口收发底层驱动
 *          负责 USART1 物理收发、DIR 方向控制、IDLE+IT 接收
 */

#ifndef RS485_H
#define RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*============================= 类型定义 =============================*/

/*============================= 宏定义 ==============================*/

/* RS485 发送阻塞超时，单位 ms */
#define RS485_TX_TIMEOUT_MS     100

/* 串口1接收缓冲区大小 */
#define UART1_RX_BUF_SIZE       64

/*=========================== 函数声明 =============================*/

/**
 * 初始化 RS485 驱动
 * 设置 DIR 引脚为接收模式，清空接收缓冲区
 * 不使能 IDLE 中断，不启动接收——需要在每次事务开始时调用 RS485_StartReceive
 */
void RS485_Init(void);

/**
 * 发送数据
 * 流程：拉高 DIR -> 阻塞发送 -> 等待 TC -> 拉低 DIR
 * @param data  待发送数据指针
 * @param len   数据长度
 * @retval      HAL_OK 成功，其他值见 HAL_StatusTypeDef
 */
HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t len);

/**
 * 启动一次 IDLE+IT 接收
 * 清空接收缓冲区，使能 IDLE 中断，启动单字节中断接收
 * 每次发送查询帧后调用此函数开始等待从站响应
 * @retval  HAL_OK 成功启动，其他值见 HAL_StatusTypeDef
 */
HAL_StatusTypeDef RS485_StartReceive(void);

/**
 * 查询当前帧是否接收完成
 * @retval  1 帧已完整接收（含溢出截断情况），0 帧仍在接收中
 */
uint8_t RS485_IsFrameComplete(void);

/**
 * 查询接收缓冲区是否溢出
 * @retval  1 溢出（帧数据被截断），0 未溢出
 */
uint8_t RS485_IsRxOverflow(void);

/**
 * 取出已接收的完整帧数据
 * 仅在 RS485_IsFrameComplete() 返回 1 时调用才有意义
 * 取走数据后需调用 RS485_StartReceive() 开启下一轮接收
 * @param pBuf   目标缓冲区
 * @param BufSize 目标缓冲区大小
 * @retval       实际拷贝的字节数
 */
uint16_t RS485_GetReceivedData(uint8_t *pBuf, uint16_t BufSize);

/**
 * IDLE 空闲中断处理
 * 在 USART1_IRQHandler 中检测到 IDLE 标志后调用
 * 置位帧完成标志，禁止后续接收，等待上层取走数据
 * @param huart  串口句柄
 */
void RS485_UARTIdleHandler(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* RS485_H */
