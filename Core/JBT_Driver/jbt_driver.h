/**
 * @file    jbt_driver.h
 * @brief   JBT-BT 通讯型电机驱动器命令层
 *          封装 JBT-BT 的业务命令，调用 Modbus 协议层事务接口完成寄存器/线圈读写
 *
 *          使用示例：
 *            JBT_Init();
 *            uint16_t speed;
 *            if (JBT_ReadSpeed(&speed) == MODBUS_OK) { ... }
 *            JBT_WriteQT(MODBUS_COIL_ON_VALUE);   // 启动
 *            JBT_WriteSpeed(3000);                 // 设转速
 */

#ifndef JBT_DRIVER_H
#define JBT_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "config.h"

/*============================= 函数声明 =============================*/

/**
 * 初始化 JBT-BT 驱动器（内部调用 Modbus_RTU_Init）
 */
void JBT_Init(void);

/**
 * 读取缺液输入状态 (QY - 离散输入区 FC 0x02)
 * @param state  输出：0=无缺液，1=缺液
 * @retval       MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t JBT_ReadQY(uint8_t *state);

/**
 * 读取启停状态 (QT - 线圈区 FC 0x01)
 * @param state  输出：0=停止，1=运行
 * @retval       MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t JBT_ReadQT(uint8_t *state);

/**
 * 写入启停命令 (QT - 线圈区 FC 0x05)
 * @param on_off  MODBUS_COIL_ON_VALUE=启动，MODBUS_COIL_OFF_VALUE=停止
 * @retval        MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t JBT_WriteQT(uint16_t on_off);

/**
 * 读取正反转状态 (ZF - 线圈区 FC 0x01)
 * @param state  输出：0=正转，1=反转
 * @retval       MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t JBT_ReadZF(uint8_t *state);

/**
 * 写入正反转命令 (ZF - 线圈区 FC 0x05)
 * @param on_off  MODBUS_COIL_ON_VALUE=反转，MODBUS_COIL_OFF_VALUE=正转
 * @retval        MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t JBT_WriteZF(uint16_t on_off);

/**
 * 读取转速 (SPEED - 保持寄存器区 FC 0x03)
 * @param speed  输出：转速值
 * @retval       MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t JBT_ReadSpeed(uint16_t *speed);

/**
 * 写入转速 (SPEED - 保持寄存器区 FC 0x06)
 * @param speed  目标转速值
 * @retval       MODBUS_OK 成功，其他见 ModbusStatus_t
 */
ModbusStatus_t JBT_WriteSpeed(uint16_t speed);

#ifdef __cplusplus
}
#endif

#endif /* JBT_DRIVER_H */
