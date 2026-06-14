//
// Created by Administrator on 2026/6/14.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI088_REG_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI088_REG_H

/* ===================== 通用 SPI 协议 ===================== */
#define BMI088_SPI_READ  0x80  // SPI 读标志位（最高位置 1）
#define BMI088_SPI_WRITE 0x00  // SPI 写标志位

/* ===================== 加速度计寄存器 ===================== */
#define ACC_CHIP_ID_REG       0x00  // 芯片 ID，默认值 0x1E
#define ACC_CHIP_ID_VALUE     0x1E

#define ACC_ERR_REG           0x02
#define ACC_STATUS_REG        0x03

#define ACC_X_LSB_REG         0x12  // 加速度数据起始地址
#define ACC_X_MSB_REG         0x13
#define ACC_Y_LSB_REG         0x14
#define ACC_Y_MSB_REG         0x15
#define ACC_Z_LSB_REG         0x16
#define ACC_Z_MSB_REG         0x17

#define ACC_SENSORTIME_0_REG  0x18
#define ACC_SENSORTIME_1_REG  0x19
#define ACC_SENSORTIME_2_REG  0x1A

#define ACC_INT_STAT_1_REG    0x1D  // 中断状态

#define TEMP_MSB_REG          0x22  // 温度数据
#define TEMP_LSB_REG          0x23

#define ACC_CONF_REG          0x40  // 加速度计配置
#define ACC_RANGE_REG         0x41  // 加速度计量程

#define ACC_INT1_IO_CTRL_REG  0x53  // INT1 引脚配置
#define ACC_INT2_IO_CTRL_REG  0x54  // INT2 引脚配置
#define ACC_INT1_INT2_MAP_DATA_REG 0x58  // 数据中断映射

#define ACC_SELF_TEST_REG     0x6D
#define ACC_PWR_CONF_REG      0x7C  // 电源模式配置
#define ACC_PWR_CTRL_REG      0x7D  // 电源控制（开关）
#define ACC_SOFTRESET_REG     0x7E  // 软复位
#define ACC_SOFTRESET_VALUE   0xB6

/* 加速度计配置值 */
#define ACC_CONF_ODR_1600HZ   0xAC  // BWP=normal, ODR=1600Hz
#define ACC_CONF_ODR_800HZ    0xAB  // BWP=normal, ODR=800Hz
#define ACC_CONF_ODR_400HZ    0xAA  // BWP=normal, ODR=400Hz

#define ACC_RANGE_3G          0x00
#define ACC_RANGE_6G          0x01
#define ACC_RANGE_12G         0x02
#define ACC_RANGE_24G         0x03

#define ACC_PWR_CONF_ACTIVE   0x00  // 正常模式
#define ACC_PWR_CONF_SUSPEND  0x03  // 挂起模式
#define ACC_PWR_CTRL_ON       0x04  // 开启加速度计

/* 加速度计 INT 配置 */
#define ACC_INT1_IO_PP_AL     0x08  // INT1: 推挽, 高电平有效
#define ACC_INT1_DRDY_MAP     0x04  // 将 DRDY 映射到 INT1

/* ===================== 陀螺仪寄存器 ===================== */
#define GYRO_CHIP_ID_REG      0x00  // 芯片 ID，默认值 0x0F
#define GYRO_CHIP_ID_VALUE    0x0F

#define GYRO_X_LSB_REG        0x02  // 角速度数据起始地址
#define GYRO_X_MSB_REG        0x03
#define GYRO_Y_LSB_REG        0x04
#define GYRO_Y_MSB_REG        0x05
#define GYRO_Z_LSB_REG        0x06
#define GYRO_Z_MSB_REG        0x07

#define GYRO_INT_STAT_1_REG   0x0A  // 中断状态

#define GYRO_RANGE_REG        0x0F  // 陀螺仪量程
#define GYRO_BANDWIDTH_REG    0x10  // 陀螺仪带宽/ODR

#define GYRO_LPM1_REG         0x11  // 电源模式
#define GYRO_SOFTRESET_REG    0x14  // 软复位
#define GYRO_SOFTRESET_VALUE  0xB6

#define GYRO_INT_CTRL_REG     0x15  // 中断使能
#define GYRO_INT3_INT4_IO_CONF_REG 0x16  // INT3/INT4 引脚配置
#define GYRO_INT3_INT4_IO_MAP_REG  0x18  // 中断映射

#define GYRO_SELF_TEST_REG    0x3C

/* 陀螺仪配置值 */
#define GYRO_RANGE_2000DPS    0x00
#define GYRO_RANGE_1000DPS    0x01
#define GYRO_RANGE_500DPS     0x02
#define GYRO_RANGE_250DPS     0x03
#define GYRO_RANGE_125DPS     0x04

#define GYRO_BW_532HZ_2000HZ_ODR   0x00  // 2000Hz ODR, 532Hz BW
#define GYRO_BW_230HZ_2000HZ_ODR   0x01  // 2000Hz ODR, 230Hz BW
#define GYRO_BW_116HZ_1000HZ_ODR   0x02  // 1000Hz ODR, 116Hz BW
#define GYRO_BW_47HZ_400HZ_ODR     0x03  // 400Hz  ODR, 47Hz  BW
#define GYRO_BW_32HZ_100HZ_ODR     0x07  // 100Hz  ODR, 32Hz  BW

#define GYRO_LPM1_NORMAL      0x00
#define GYRO_LPM1_SUSPEND     0x80
#define GYRO_LPM1_DEEP_SUSPEND 0x20

/* 陀螺仪 INT 配置 */
#define GYRO_INT_CTRL_DRDY_EN       0x80  // 使能 DRDY 中断
#define GYRO_INT3_IO_PP_AL          0x00  // INT3: 推挽, 高电平有效
#define GYRO_INT3_DRDY_MAP          0x01  // 将 DRDY 映射到 INT3


#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI088_REG_H