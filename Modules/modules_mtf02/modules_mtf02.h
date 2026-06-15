//
// Created by Administrator on 2026/6/15.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_MTF02_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_MTF02_H

#include <stdint.h>

#include "bsp_utils_seqlock.h"

#define MTF02_FRAME_SIZE    27U     /* 固定的 Micolink 0x51 消息总帧长 */
#define MTF02_SEQLOCK_RETRY 3

#pragma pack(1)
typedef struct {
    uint32_t time_ms;       // 系统时间 ms
    uint32_t distance;      // 距离(mm) 最小值为10，0表示数据不可用
    uint8_t  strength;      // 信号强度
    uint8_t  precision;     // 精度
    uint8_t  tof_status;    // 状态：1表示测距数据可用
    uint8_t  reserved1;     // 预留
    int16_t  flow_vel_x;    // 光流速度x轴 cm/s @ 1m
    int16_t  flow_vel_y;    // 光流速度y轴 cm/s @ 1m
    uint8_t  flow_quality;  // 光流质量
    uint8_t  flow_status;   // 光流状态：1表示光流数据可用
    uint16_t reserved2;     // 预留
} MTF02_Payload_t;
#pragma pack()

/* 供发布订阅系统使用的数据类型 */
typedef struct {
    uint32_t timestamp_us;  // 接收到数据的时间戳 (us)
    uint8_t  is_valid;      // 综合有效性标志位
    SeqLock_t     data_lock;

    MTF02_Payload_t payload;

} MTF02_Data_t;

void MTF02_Init(void);
void MTF02_Task_Handler(void);


#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_MTF02_H