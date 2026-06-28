/**
 * @file    jbt_driver.c
 * @brief   JBT-BT 通讯型电机驱动器命令层
 *          封装 JBT-BT 的业务命令，调用 Modbus 协议层事务接口完成寄存器/线圈读写
 */

#include "jbt_driver.h"

/*============================= 公共函数 =============================*/

void JBT_Init(void)
{
    Modbus_RTU_Init();
}

/**
 * @brief  读取缺液输入状态 (QY - 离散输入区 FC 0x02)
 */
ModbusStatus_t JBT_ReadQY(uint8_t *state)
{
    if (state == 0)
    {
        return MODBUS_ERR_NULL;
    }

    return Modbus_ReadDiscreteInputs(JBT_SLAVE_ADDR, JBT_QY_ADDR, 1, state);
}

/**
 * @brief  读取启停状态 (QT - 线圈区 FC 0x01)
 */
ModbusStatus_t JBT_ReadQT(uint8_t *state)
{
    if (state == 0)
    {
        return MODBUS_ERR_NULL;
    }

    return Modbus_ReadCoils(JBT_SLAVE_ADDR, JBT_QT_ADDR, 1, state);
}

/**
 * @brief  写入启停命令 (QT - 线圈区 FC 0x05)
 */
ModbusStatus_t JBT_WriteQT(uint16_t on_off)
{
    return Modbus_WriteSingleCoil(JBT_SLAVE_ADDR, JBT_QT_ADDR, on_off);
}

/**
 * @brief  读取正反转状态 (ZF - 线圈区 FC 0x01)
 */
ModbusStatus_t JBT_ReadZF(uint8_t *state)
{
    if (state == 0)
    {
        return MODBUS_ERR_NULL;
    }

    return Modbus_ReadCoils(JBT_SLAVE_ADDR, JBT_ZF_ADDR, 1, state);
}

/**
 * @brief  写入正反转命令 (ZF - 线圈区 FC 0x05)
 */
ModbusStatus_t JBT_WriteZF(uint16_t on_off)
{
    return Modbus_WriteSingleCoil(JBT_SLAVE_ADDR, JBT_ZF_ADDR, on_off);
}

/**
 * @brief  读取转速 (SPEED - 保持寄存器区 FC 0x03)
 */
ModbusStatus_t JBT_ReadSpeed(uint16_t *speed)
{
    if (speed == 0)
    {
        return MODBUS_ERR_NULL;
    }

    return Modbus_ReadHoldingRegs(JBT_SLAVE_ADDR, JBT_SPEED_ADDR, 1, speed);
}

/**
 * @brief  写入转速 (SPEED - 保持寄存器区 FC 0x06)
 */
ModbusStatus_t JBT_WriteSpeed(uint16_t speed)
{
    return Modbus_WriteSingleReg(JBT_SLAVE_ADDR, JBT_SPEED_ADDR, speed);
}
