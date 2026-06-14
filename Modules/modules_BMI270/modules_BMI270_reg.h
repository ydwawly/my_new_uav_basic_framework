//
// Created by Administrator on 2026/6/14.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_REG_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_REG_H

/* ===================== SPI 协议 ===================== */
#define BMI270_SPI_READ      0x80
#define BMI270_SPI_WRITE     0x00

/*
 * BMI270 SPI 读取特殊说明：
 * 和 BMI088 加速度计一样，BMI270 SPI 读操作返回的第一个字节是 dummy
 * 即：发 [addr|0x80, dummy, ...] 收 [dummy_addr, dummy_data, real_data...]
 * 所以读 N 字节数据需要发送 N+2 字节（1 addr + 1 dummy + N data）
 */
#define BMI270_SPI_DUMMY_BYTE  1  // 读操作额外 dummy 字节数

/* ===================== 通用寄存器 ===================== */
#define BMI270_CHIP_ID_REG          0x00  // 芯片 ID = 0x24
#define BMI270_CHIP_ID_VALUE        0x24

#define BMI270_ERR_REG              0x02
#define BMI270_STATUS_REG           0x03

/* ===================== 数据寄存器 ===================== */
/*
 * BMI270 数据寄存器布局（连续地址，一次 burst read 即可）：
 *
 * 0x0C  ACC_X_LSB
 * 0x0D  ACC_X_MSB
 * 0x0E  ACC_Y_LSB
 * 0x0F  ACC_Y_MSB
 * 0x10  ACC_Z_LSB
 * 0x11  ACC_Z_MSB
 * 0x12  GYR_X_LSB
 * 0x13  GYR_X_MSB
 * 0x14  GYR_Y_LSB
 * 0x15  GYR_Y_MSB
 * 0x16  GYR_Z_LSB
 * 0x17  GYR_Z_MSB
 *
 * 共 12 字节 = 6 轴数据
 */
#define BMI270_ACC_X_LSB_REG        0x0C  // 数据起始地址
#define BMI270_DATA_LENGTH          12    // 6轴 * 2字节 = 12字节

/* 传感器时间（24-bit）*/
#define BMI270_SENSORTIME_0_REG     0x18
#define BMI270_SENSORTIME_1_REG     0x19
#define BMI270_SENSORTIME_2_REG     0x1A

/* 温度 */
#define BMI270_TEMP_MSB_REG         0x22
#define BMI270_TEMP_LSB_REG         0x23

/* 中断状态 */
#define BMI270_INT_STATUS_0_REG     0x1C
#define BMI270_INT_STATUS_1_REG     0x1D

/* ===================== 配置寄存器 ===================== */
#define BMI270_ACC_CONF_REG         0x40
#define BMI270_ACC_RANGE_REG        0x41
#define BMI270_GYR_CONF_REG         0x42
#define BMI270_GYR_RANGE_REG        0x43

/* 中断配置 */
#define BMI270_INT1_IO_CTRL_REG     0x53
#define BMI270_INT2_IO_CTRL_REG     0x54
#define BMI270_INT_MAP_DATA_REG     0x58

/* 电源 / 初始化 */
#define BMI270_INIT_CTRL_REG        0x59  // 初始化控制
#define BMI270_INIT_DATA_REG        0x5E  // 配置文件写入地址
#define BMI270_INIT_ADDR_0_REG      0x5B  // 配置文件地址低字节
#define BMI270_INIT_ADDR_1_REG      0x5C  // 配置文件地址高字节
#define BMI270_INTERNAL_STATUS_REG  0x21  // 内部状态（校验初始化结果）

#define BMI270_PWR_CONF_REG         0x7C
#define BMI270_PWR_CTRL_REG         0x7D
#define BMI270_CMD_REG              0x7E  // 命令寄存器（软复位等）

/* ===================== 配置值定义 ===================== */

/* 软复位 */
#define BMI270_CMD_SOFTRESET        0xB6

/* ACC_CONF (0x40) */
#define BMI270_ACC_ODR_800HZ        0x0B  // acc_odr = 800Hz
#define BMI270_ACC_ODR_1600HZ       0x0C
#define BMI270_ACC_BWP_NORMAL       0x20  // acc_bwp = normal (OSR2)
#define BMI270_ACC_BWP_OSR4         0x00
#define BMI270_ACC_FILTER_HP        0x80  // acc_filter_perf = high performance

/* ACC_RANGE (0x41) */
#define BMI270_ACC_RANGE_2G         0x00
#define BMI270_ACC_RANGE_4G         0x01
#define BMI270_ACC_RANGE_8G         0x02
#define BMI270_ACC_RANGE_16G        0x03

/* GYR_CONF (0x42) */
#define BMI270_GYR_ODR_800HZ        0x0B
#define BMI270_GYR_ODR_1600HZ       0x0C
#define BMI270_GYR_ODR_2000HZ       0x0D
#define BMI270_GYR_BWP_NORMAL       0x20  // gyr_bwp = normal (OSR2)
#define BMI270_GYR_BWP_OSR4         0x00
#define BMI270_GYR_FILTER_HP        0x80  // gyr_filter_perf = high performance
#define BMI270_GYR_NOISE_HP         0x40  // gyr_noise_perf = high performance

/* GYR_RANGE (0x43) */
#define BMI270_GYR_RANGE_2000DPS    0x00
#define BMI270_GYR_RANGE_1000DPS    0x01
#define BMI270_GYR_RANGE_500DPS     0x02
#define BMI270_GYR_RANGE_250DPS     0x03
#define BMI270_GYR_RANGE_125DPS     0x04

/* PWR_CONF (0x7C) */
#define BMI270_PWR_CONF_ADV_PWR_SAVE_OFF  0x00  // 关闭高级省电模式
#define BMI270_PWR_CONF_ADV_PWR_SAVE_ON   0x01

/* PWR_CTRL (0x7D) */
#define BMI270_PWR_CTRL_ACC_EN      0x04  // bit2: 加速度计使能
#define BMI270_PWR_CTRL_GYR_EN      0x02  // bit1: 陀螺仪使能
#define BMI270_PWR_CTRL_TEMP_EN     0x01  // bit0: 温度使能
#define BMI270_PWR_CTRL_ALL_EN      0x07  // 全部使能

/* INIT_CTRL (0x59) */
#define BMI270_INIT_CTRL_LOAD_FILE  0x00  // 准备上传配置文件

/* INTERNAL_STATUS (0x21) */
#define BMI270_INTERNAL_STATUS_MSG_MASK  0x0F
#define BMI270_INTERNAL_STATUS_INIT_OK   0x01  // 初始化成功

/* INT1_IO_CTRL (0x53) */
#define BMI270_INT1_OD_PP           0x00  // 推挽输出
#define BMI270_INT1_LVL_HIGH        0x02  // 高电平有效
#define BMI270_INT1_OUTPUT_EN       0x08  // 输出使能

/* INT_MAP_DATA (0x58) */
#define BMI270_INT_MAP_DRDY_INT1    0x04  // DRDY 映射到 INT1

#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_BMI270_REG_H