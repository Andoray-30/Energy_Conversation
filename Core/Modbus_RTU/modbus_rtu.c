/**
 * @file    modbus_rtu.c
 * @brief   Modbus RTU 协议层
 *          负责帧组装/解析、CRC16 校验、功能码处理
 *
 *          对外提供事务级接口：一个函数完成 建帧→发送→等待响应→校验→解析
 */

#include "modbus_rtu.h"
#include <string.h>

/*============================= 私有变量 =============================*/

/** 发送帧缓冲区 */
static uint8_t modbus_tx_buf[MODBUS_BUF_SIZE];

/** 接收帧缓冲区 */
static uint8_t modbus_rx_buf[MODBUS_BUF_SIZE];

/** 最近一次异常响应的异常码 */
static uint8_t modbus_exception_code;

/*============================= 私有函数 =============================*/

/**
 * @brief  计算 Modbus RTU CRC16
 * @param  buf  参与 CRC 计算的数据首地址
 * @param  len  参与 CRC 计算的数据长度，不包含最后两个 CRC 字节
 * @retval CRC16 计算结果
 */
static uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    if (buf == 0)
    {
        return 0;
    }

    for (uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief  给帧追加 CRC（低字节在前，高字节在后）
 * @param  frame         待发送帧缓冲区
 * @param  payload_len   不包含 CRC 的数据长度
 * @note   frame 缓冲区必须至少有 payload_len + 2 字节空间
 */
static void Modbus_AppendCRC(uint8_t *frame, uint16_t payload_len)
{
    uint16_t crc;

    if (frame == 0)
    {
        return;
    }

    crc = Modbus_CRC16(frame, payload_len);

    frame[payload_len]     = (uint8_t)(crc & 0xFF);        // CRC 低字节
    frame[payload_len + 1] = (uint8_t)((crc >> 8) & 0xFF); // CRC 高字节
}

/**
 * @brief  校验接收帧 CRC 是否正确
 * @param  frame      接收到的完整 Modbus RTU 帧
 * @param  frame_len  完整帧长度，包含 CRC
 * @retval 1 CRC 正确，0 CRC 错误或参数非法
 */
static uint8_t Modbus_CheckCRC(const uint8_t *frame, uint16_t frame_len)
{
    uint16_t crc_calc;
    uint16_t crc_recv;

    if (frame == 0 || frame_len < MODBUS_RTU_MIN_FRAME_SIZE)
    {
        return 0;
    }

    crc_calc = Modbus_CRC16(frame, frame_len - 2);

    /* 接收帧 CRC 存储顺序：低字节在前，高字节在后 */
    crc_recv = (uint16_t)frame[frame_len - 2] |
               ((uint16_t)frame[frame_len - 1] << 8);

    return (crc_calc == crc_recv) ? 1 : 0;
}

/**
 * @brief  等待从站响应并取回数据
 * @param  expected_addr  期望从站地址
 * @param  expected_func  期望功能码
 * @param  rx_len         输出：接收帧长度
 * @retval ModbusStatus_t
 */
static ModbusStatus_t Modbus_WaitResponse(uint8_t expected_addr,
                                            uint8_t expected_func,
                                            uint16_t *rx_len)
{
    uint32_t tick_start;
    uint16_t len;

    tick_start = HAL_GetTick();

    /* 轮询等待帧完成或超时 */
    while (1)
    {
        if (RS485_IsFrameComplete())
        {
            break;
        }

        if ((HAL_GetTick() - tick_start) >= MODBUS_RESP_TIMEOUT_MS)
        {
            return MODBUS_ERR_TIMEOUT;
        }
    }

    /* 检查是否溢出 */
    if (RS485_IsRxOverflow())
    {
        return MODBUS_ERR_OVERFLOW;
    }

    /* 取出接收数据 */
    len = RS485_GetReceivedData(modbus_rx_buf, MODBUS_BUF_SIZE);
    if (rx_len != 0)
    {
        *rx_len = len;
    }

    /* CRC 校验 */
    if (!Modbus_CheckCRC(modbus_rx_buf, len))
    {
        return MODBUS_ERR_CRC;
    }

    /* 从站地址校验 */
    if (modbus_rx_buf[0] != expected_addr)
    {
        return MODBUS_ERR_ADDR;
    }

    /* 异常响应检测：功能码最高位置 1 */
    if (modbus_rx_buf[1] == (expected_func | MODBUS_EXCEPTION_MASK))
    {
        modbus_exception_code = modbus_rx_buf[2];
        return MODBUS_ERR_EXCEPTION;
    }

    /* 功能码校验 */
    if (modbus_rx_buf[1] != expected_func)
    {
        return MODBUS_ERR_FUNC;
    }

    return MODBUS_OK;
}

/*============================= 公共函数 =============================*/

void Modbus_RTU_Init(void)
{
    RS485_Init();
    modbus_exception_code = 0;
}

/*------------- 读保持寄存器 (FC 0x03) -------------*/

ModbusStatus_t Modbus_ReadHoldingRegs(uint8_t slave, uint16_t start,
                                        uint16_t count, uint16_t *out)
{
    ModbusStatus_t status;
    uint16_t rx_len;
    uint16_t byte_count;
    uint16_t i;
    HAL_StatusTypeDef hal_ret;

    if (out == 0 || count == 0 || count > 125)
    {
        return MODBUS_ERR_LEN;
    }

    /* 建帧：从站地址 + 功能码 + 起始地址 + 数量 */
    modbus_tx_buf[0] = slave;
    modbus_tx_buf[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    modbus_tx_buf[2] = (uint8_t)(start >> 8);
    modbus_tx_buf[3] = (uint8_t)(start & 0xFF);
    modbus_tx_buf[4] = (uint8_t)(count >> 8);
    modbus_tx_buf[5] = (uint8_t)(count & 0xFF);
    Modbus_AppendCRC(modbus_tx_buf, 6);

    /* 发送请求（Send 尾部自动启动接收链路） */
    hal_ret = RS485_Send(modbus_tx_buf, 8);
    if (hal_ret != HAL_OK)
    {
        return MODBUS_ERR_LEN;
    }

    /* 等待从站响应 */
    status = Modbus_WaitResponse(slave, MODBUS_FC_READ_HOLDING_REGISTERS, &rx_len);
    if (status != MODBUS_OK)
    {
        return status;
    }

    /* 解析响应：[从站][功能码][字节数N][数据N字节] */
    byte_count = modbus_rx_buf[2];

    /* 校验数据长度：字节数 + 帧头(3) + CRC(2) = 总帧长 */
    if (rx_len < (uint16_t)(3 + byte_count + 2))
    {
        return MODBUS_ERR_LEN;
    }

    if (byte_count != count * 2)
    {
        return MODBUS_ERR_LEN;
    }

    /* 提取寄存器值（大端序转主机序） */
    for (i = 0; i < count; i++)
    {
        out[i] = ((uint16_t)modbus_rx_buf[3 + i * 2] << 8) |
                  (uint16_t)modbus_rx_buf[3 + i * 2 + 1];
    }

    return MODBUS_OK;
}

/*------------- 读输入寄存器 (FC 0x04) -------------*/

ModbusStatus_t Modbus_ReadInputRegs(uint8_t slave, uint16_t start,
                                     uint16_t count, uint16_t *out)
{
    ModbusStatus_t status;
    uint16_t rx_len;
    uint16_t byte_count;
    uint16_t i;
    HAL_StatusTypeDef hal_ret;

    if (out == 0 || count == 0 || count > 125)
    {
        return MODBUS_ERR_LEN;
    }

    /* 建帧 */
    modbus_tx_buf[0] = slave;
    modbus_tx_buf[1] = MODBUS_FC_READ_INPUT_REGISTERS;
    modbus_tx_buf[2] = (uint8_t)(start >> 8);
    modbus_tx_buf[3] = (uint8_t)(start & 0xFF);
    modbus_tx_buf[4] = (uint8_t)(count >> 8);
    modbus_tx_buf[5] = (uint8_t)(count & 0xFF);
    Modbus_AppendCRC(modbus_tx_buf, 6);

    hal_ret = RS485_Send(modbus_tx_buf, 8);
    if (hal_ret != HAL_OK)
    {
        return MODBUS_ERR_LEN;
    }

    status = Modbus_WaitResponse(slave, MODBUS_FC_READ_INPUT_REGISTERS, &rx_len);
    if (status != MODBUS_OK)
    {
        return status;
    }

    byte_count = modbus_rx_buf[2];

    if (rx_len < (uint16_t)(3 + byte_count + 2))
    {
        return MODBUS_ERR_LEN;
    }

    if (byte_count != count * 2)
    {
        return MODBUS_ERR_LEN;
    }

    for (i = 0; i < count; i++)
    {
        out[i] = ((uint16_t)modbus_rx_buf[3 + i * 2] << 8) |
                  (uint16_t)modbus_rx_buf[3 + i * 2 + 1];
    }

    return MODBUS_OK;
}

/*------------- 写单个保持寄存器 (FC 0x06) -------------*/

ModbusStatus_t Modbus_WriteSingleReg(uint8_t slave, uint16_t addr,
                                       uint16_t value)
{
    ModbusStatus_t status;
    HAL_StatusTypeDef hal_ret;

    /* 建帧：从站地址 + 功能码 + 寄存器地址 + 写入值 */
    modbus_tx_buf[0] = slave;
    modbus_tx_buf[1] = MODBUS_FC_WRITE_SINGLE_REGISTER;
    modbus_tx_buf[2] = (uint8_t)(addr >> 8);
    modbus_tx_buf[3] = (uint8_t)(addr & 0xFF);
    modbus_tx_buf[4] = (uint8_t)(value >> 8);
    modbus_tx_buf[5] = (uint8_t)(value & 0xFF);
    Modbus_AppendCRC(modbus_tx_buf, 6);

    hal_ret = RS485_Send(modbus_tx_buf, 8);
    if (hal_ret != HAL_OK)
    {
        return MODBUS_ERR_LEN;
    }

    status = Modbus_WaitResponse(slave, MODBUS_FC_WRITE_SINGLE_REGISTER, 0);

    /* 正常响应为请求帧回显，无需额外解析 */
    return status;
}

/*------------- 写多个保持寄存器 (FC 0x10) -------------*/

ModbusStatus_t Modbus_WriteMultiRegs(uint8_t slave, uint16_t start,
                                       uint16_t count, const uint16_t *values)
{
    ModbusStatus_t status;
    uint16_t byte_count;
    uint16_t i;
    uint16_t payload_len;
    HAL_StatusTypeDef hal_ret;

    if (values == 0 || count == 0 || count > 123)
    {
        return MODBUS_ERR_LEN;
    }

    byte_count = count * 2;

    /* 检查帧缓冲区是否足够：从站(1) + 功能码(1) + 起始地址(2) + 数量(2) + 字节数(1) + 数据 + CRC(2) */
    payload_len = 6 + 1 + byte_count;
    if ((payload_len + 2) > MODBUS_BUF_SIZE)
    {
        return MODBUS_ERR_LEN;
    }

    /* 建帧 */
    modbus_tx_buf[0] = slave;
    modbus_tx_buf[1] = MODBUS_FC_WRITE_MULTIPLE_REGS;
    modbus_tx_buf[2] = (uint8_t)(start >> 8);
    modbus_tx_buf[3] = (uint8_t)(start & 0xFF);
    modbus_tx_buf[4] = (uint8_t)(count >> 8);
    modbus_tx_buf[5] = (uint8_t)(count & 0xFF);
    modbus_tx_buf[6] = (uint8_t)byte_count;

    /* 寄存器值（大端序） */
    for (i = 0; i < count; i++)
    {
        modbus_tx_buf[7 + i * 2]     = (uint8_t)(values[i] >> 8);
        modbus_tx_buf[7 + i * 2 + 1] = (uint8_t)(values[i] & 0xFF);
    }

    Modbus_AppendCRC(modbus_tx_buf, payload_len);

    hal_ret = RS485_Send(modbus_tx_buf, payload_len + 2);
    if (hal_ret != HAL_OK)
    {
        return MODBUS_ERR_LEN;
    }

    status = Modbus_WaitResponse(slave, MODBUS_FC_WRITE_MULTIPLE_REGS, 0);

    return status;
}

/*------------- 读线圈 (FC 0x01) -------------*/

ModbusStatus_t Modbus_ReadCoils(uint8_t slave, uint16_t start,
                                  uint16_t count, uint8_t *out)
{
    ModbusStatus_t status;
    uint16_t rx_len;
    uint16_t data_bytes;
    uint16_t byte_idx;
    uint16_t i;
    uint8_t bit_pos;
    HAL_StatusTypeDef hal_ret;

    if (out == 0 || count == 0 || count > 2000)
    {
        return MODBUS_ERR_LEN;
    }

    /* 建帧 */
    modbus_tx_buf[0] = slave;
    modbus_tx_buf[1] = MODBUS_FC_READ_COILS;
    modbus_tx_buf[2] = (uint8_t)(start >> 8);
    modbus_tx_buf[3] = (uint8_t)(start & 0xFF);
    modbus_tx_buf[4] = (uint8_t)(count >> 8);
    modbus_tx_buf[5] = (uint8_t)(count & 0xFF);
    Modbus_AppendCRC(modbus_tx_buf, 6);

    hal_ret = RS485_Send(modbus_tx_buf, 8);
    if (hal_ret != HAL_OK)
    {
        return MODBUS_ERR_LEN;
    }

    status = Modbus_WaitResponse(slave, MODBUS_FC_READ_COILS, &rx_len);
    if (status != MODBUS_OK)
    {
        return status;
    }

    /* 解析响应：[从站][功能码][字节数N][数据N字节] */
    data_bytes = modbus_rx_buf[2];

    if (rx_len < (uint16_t)(3 + data_bytes + 2))
    {
        return MODBUS_ERR_LEN;
    }

    /* 将位域展开为逐位输出 */
    for (i = 0; i < count; i++)
    {
        byte_idx = i / 8;        /* 哪个字节 */
        bit_pos  = i % 8;        /* 字节内哪一位 */

        out[i] = (modbus_rx_buf[3 + byte_idx] >> bit_pos) & 0x01;
    }

    return MODBUS_OK;
}

/*------------- 读离散输入 (FC 0x02) -------------*/

ModbusStatus_t Modbus_ReadDiscreteInputs(uint8_t slave, uint16_t start,
                                           uint16_t count, uint8_t *out)
{
    ModbusStatus_t status;
    uint16_t rx_len;
    uint16_t data_bytes;
    uint16_t byte_idx;
    uint16_t i;
    uint8_t bit_pos;
    HAL_StatusTypeDef hal_ret;

    if (out == 0 || count == 0 || count > 2000)
    {
        return MODBUS_ERR_LEN;
    }

    /* 建帧 */
    modbus_tx_buf[0] = slave;
    modbus_tx_buf[1] = MODBUS_FC_READ_DISCRETE_INPUTS;
    modbus_tx_buf[2] = (uint8_t)(start >> 8);
    modbus_tx_buf[3] = (uint8_t)(start & 0xFF);
    modbus_tx_buf[4] = (uint8_t)(count >> 8);
    modbus_tx_buf[5] = (uint8_t)(count & 0xFF);
    Modbus_AppendCRC(modbus_tx_buf, 6);

    hal_ret = RS485_Send(modbus_tx_buf, 8);
    if (hal_ret != HAL_OK)
    {
        return MODBUS_ERR_LEN;
    }

    status = Modbus_WaitResponse(slave, MODBUS_FC_READ_DISCRETE_INPUTS, &rx_len);
    if (status != MODBUS_OK)
    {
        return status;
    }

    data_bytes = modbus_rx_buf[2];

    if (rx_len < (uint16_t)(3 + data_bytes + 2))
    {
        return MODBUS_ERR_LEN;
    }

    for (i = 0; i < count; i++)
    {
        byte_idx = i / 8;
        bit_pos  = i % 8;

        out[i] = (modbus_rx_buf[3 + byte_idx] >> bit_pos) & 0x01;
    }

    return MODBUS_OK;
}

/*------------- 写单个线圈 (FC 0x05) -------------*/

ModbusStatus_t Modbus_WriteSingleCoil(uint8_t slave, uint16_t addr,
                                        uint16_t on_off)
{
    ModbusStatus_t status;
    HAL_StatusTypeDef hal_ret;

    /* 建帧：从站地址 + 功能码 + 线圈地址 + ON/OFF 值 */
    modbus_tx_buf[0] = slave;
    modbus_tx_buf[1] = MODBUS_FC_WRITE_SINGLE_COIL;
    modbus_tx_buf[2] = (uint8_t)(addr >> 8);
    modbus_tx_buf[3] = (uint8_t)(addr & 0xFF);
    modbus_tx_buf[4] = (uint8_t)(on_off >> 8);
    modbus_tx_buf[5] = (uint8_t)(on_off & 0xFF);
    Modbus_AppendCRC(modbus_tx_buf, 6);

    hal_ret = RS485_Send(modbus_tx_buf, 8);
    if (hal_ret != HAL_OK)
    {
        return MODBUS_ERR_LEN;
    }

    status = Modbus_WaitResponse(slave, MODBUS_FC_WRITE_SINGLE_COIL, 0);

    /* 正常响应为请求帧回显 */
    return status;
}

/*------------- 异常码查询 -------------*/

uint8_t Modbus_GetExceptionCode(void)
{
    return modbus_exception_code;
}
