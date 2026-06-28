/**
 * @file    modbus_rtu.h
 * @brief   Modbus RTU 协议层
 *          负责帧组装/解析、CRC16 校验、功能码处理
 *
 *          对外提供事务级接口：一个函数完成 建帧→发送→等待响应→校验→解析
 *          CRC/CheckCRC/CheckResponse 为内部实现，不对外暴露
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "rs485.h"

/*============================= 类型定义 =============================*/

/** Modbus 事务状态码 */
typedef enum
{
    MODBUS_OK = 0,              // 事务成功

    MODBUS_ERR_NULL,            // 空指针参数
    MODBUS_ERR_LEN,             // 帧长度非法
    MODBUS_ERR_CRC,             // 响应帧 CRC 校验失败
    MODBUS_ERR_ADDR,            // 从站地址不匹配
    MODBUS_ERR_FUNC,            // 功能码不匹配
    MODBUS_ERR_EXCEPTION,      // 从站返回异常响应（异常码通过 Modbus_GetExceptionCode 读取）
    MODBUS_ERR_TIMEOUT,        // 等待响应超时
    MODBUS_ERR_OVERFLOW         // 接收缓冲区溢出（帧被截断）

} ModbusStatus_t;

/*============================= 宏定义 ==============================*/

/*------------------------ 帧与缓冲 ------------------------*/

/** Modbus RTU 最小合法帧长度：从站地址(1) + 功能码(1) + CRC(2) */
#define MODBUS_RTU_MIN_FRAME_SIZE  4

/** 内部帧缓冲区大小（收发共用） */
#define MODBUS_BUF_SIZE            64

/** 从站响应超时，单位 ms */
#define MODBUS_RESP_TIMEOUT_MS     250

/*------------------------ 功能码 ------------------------*/

/** 读线圈 / 输出继电器（1~2000 个） */
#define MODBUS_FC_READ_COILS              0x01

/** 读离散输入 / 输入继电器（1~2000 个） */
#define MODBUS_FC_READ_DISCRETE_INPUTS    0x02

/** 读保持寄存器 / 输出寄存器（1~125 个） */
#define MODBUS_FC_READ_HOLDING_REGISTERS  0x03

/** 读输入寄存器（1~125 个） */
#define MODBUS_FC_READ_INPUT_REGISTERS    0x04

/** 写单个线圈 / 输出继电器 */
#define MODBUS_FC_WRITE_SINGLE_COIL       0x05

/** 写单个保持寄存器 / 输出寄存器 */
#define MODBUS_FC_WRITE_SINGLE_REGISTER   0x06

/** 写多个线圈 / 输出继电器（1~1968 个） */
#define MODBUS_FC_WRITE_MULTIPLE_COILS    0x0F

/** 写多个保持寄存器 / 输出寄存器（1~123 个） */
#define MODBUS_FC_WRITE_MULTIPLE_REGS     0x10

/*------------------------ 异常与线圈值 ------------------------*/

/** 异常响应功能码掩码：响应功能码 & 0x80 = 1 表示异常 */
#define MODBUS_EXCEPTION_MASK      0x80

/** 写线圈 ON 值（0xFF00） */
#define MODBUS_COIL_ON_VALUE       0xFF00

/** 写线圈 OFF 值（0x0000） */
#define MODBUS_COIL_OFF_VALUE      0x0000

/*=========================== 函数声明 =============================*/

/*------------------------ 初始化 ------------------------*/

/**
 * 初始化 Modbus RTU 协议层（内部调用 RS485_Init）
 */
void Modbus_RTU_Init(void);

/*------------- 读保持寄存器 (FC 0x03) -------------*/

/**
 * 读保持寄存器
 * @param slave   从站地址
 * @param start   起始寄存器地址
 * @param count   读取数量（1~125）
 * @param out     输出数组，调用者保证空间 >= count
 * @retval        MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t Modbus_ReadHoldingRegs(uint8_t slave, uint16_t start,
                                       uint16_t count, uint16_t *out);

/*------------- 读输入寄存器 (FC 0x04) -------------*/

/**
 * 读输入寄存器
 * @param slave   从站地址
 * @param start   起始寄存器地址
 * @param count   读取数量（1~125）
 * @param out     输出数组，调用者保证空间 >= count
 * @retval        MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t Modbus_ReadInputRegs(uint8_t slave, uint16_t start,
                                     uint16_t count, uint16_t *out);

/*------------- 写单个保持寄存器 (FC 0x06) -------------*/

/**
 * 写单个保持寄存器
 * @param slave   从站地址
 * @param addr    寄存器地址
 * @param value   写入值
 * @retval        MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t Modbus_WriteSingleReg(uint8_t slave, uint16_t addr,
                                      uint16_t value);

/*------------- 写多个保持寄存器 (FC 0x10) -------------*/

/**
 * 写多个保持寄存器
 * @param slave   从站地址
 * @param start   起始寄存器地址
 * @param count   写入数量（1~123）
 * @param values  写入值数组
 * @retval        MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t Modbus_WriteMultiRegs(uint8_t slave, uint16_t start,
                                      uint16_t count, const uint16_t *values);

/*------------- 读线圈 (FC 0x01) -------------*/

/**
 * 读线圈状态
 * @param slave   从站地址
 * @param start   起始线圈地址
 * @param count   读取数量（1~2000）
 * @param out     输出位数组，每位 0 或 1，调用者保证空间 >= count
 * @retval        MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t Modbus_ReadCoils(uint8_t slave, uint16_t start,
                                 uint16_t count, uint8_t *out);

/*------------- 读离散输入 (FC 0x02) -------------*/

/**
 * 读离散输入状态
 * @param slave   从站地址
 * @param start   起始地址
 * @param count   读取数量（1~2000）
 * @param out     输出位数组，每位 0 或 1，调用者保证空间 >= count
 * @retval        MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t Modbus_ReadDiscreteInputs(uint8_t slave, uint16_t start,
                                          uint16_t count, uint8_t *out);

/*------------- 写单个线圈 (FC 0x05) -------------*/

/**
 * 写单个线圈
 * @param slave   从站地址
 * @param addr    线圈地址
 * @param on_off  MODBUS_COIL_ON_VALUE 或 MODBUS_COIL_OFF_VALUE
 * @retval        MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t Modbus_WriteSingleCoil(uint8_t slave, uint16_t addr,
                                       uint16_t on_off);

/*------------- 异常码查询 -------------*/

/**
 * 获取最近一次异常响应的异常码
 * 仅在 ModbusStatus_t 返回 MODBUS_ERR_EXCEPTION 时有意义
 * @retval  异常码（1=非法功能码, 2=非法数据地址, 3=非法数据值, 4=从站故障）
 */
uint8_t Modbus_GetExceptionCode(void);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */
