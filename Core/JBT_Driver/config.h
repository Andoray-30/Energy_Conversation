/**
 * @file    config.h
 * @brief   JBT-BT 驱动器寄存器/线圈地址与 Modbus 功能码映射
 *
 *          JBT-BT 通讯型电机驱动器 Modbus 地址表：
 *
 *          名称     数据区                读功能码   写功能码   地址
 *          QY       离散输入区            0x02       -          0x0000
 *          QT       线圈/输出继电器区      0x01       0x05       0x0000
 *          ZF       线圈/输出继电器区      0x01       0x05       0x0001
 *          SPEED    保持寄存器/输出寄存器  0x03       0x06       0x0000
 */

#ifndef JBT_CONFIG_H
#define JBT_CONFIG_H

#include "modbus_rtu.h"

/* JBT-BT 默认从站地址 */
#define JBT_SLAVE_ADDR              0x01

/* 缺液输入 (QY) - 离散输入区，只读 */
#define JBT_QY_ADDR                 0x0000
#define JBT_QY_FC_READ              MODBUS_FC_READ_DISCRETE_INPUTS

/* 启停 (QT) - 线圈区，可读可写 */
#define JBT_QT_ADDR                 0x0000
#define JBT_QT_FC_READ              MODBUS_FC_READ_COILS
#define JBT_QT_FC_WRITE             MODBUS_FC_WRITE_SINGLE_COIL

/* 正反转 (ZF) - 线圈区，可读可写 */
#define JBT_ZF_ADDR                 0x0001
#define JBT_ZF_FC_READ              MODBUS_FC_READ_COILS
#define JBT_ZF_FC_WRITE             MODBUS_FC_WRITE_SINGLE_COIL

/* 转速 (SPEED) - 保持寄存器区，可读可写 */
#define JBT_SPEED_ADDR              0x0000
#define JBT_SPEED_FC_READ           MODBUS_FC_READ_HOLDING_REGISTERS
#define JBT_SPEED_FC_WRITE          MODBUS_FC_WRITE_SINGLE_REGISTER

#endif /* JBT_CONFIG_H */
