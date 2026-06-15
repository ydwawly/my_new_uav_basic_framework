//
// Created by Administrator on 2026/6/15.
//

#include "modules_mtf02.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "bsp_utils_seqlock.h"
#include <string.h>

#include "bsp_RTT.h"
#include "bsp_timestamp.h"
#include "bsp_uart.h"


/* ========================== 私有变量 ========================== */

static USARTInstance *mtf02_instance;
static MTF02_Data_t   mtf02_data;

/* 序列锁与共享内存区 */
static uint8_t shared_mtf02_frame[MTF02_FRAME_SIZE];
static uint32_t shared_timestamp_cyc = 0;

/* ========================== 私有函数声明 ========================== */

static void MTF02_Rx_Callback(uint8_t *buf, uint16_t len);
static void MTF02_Decode(const uint8_t *buf, uint32_t timestamp_cyc);

/* ========================== 函数实现 ========================== */

void MTF02_Init(void)
{
    USART_Init_Config_s config;

    // 初始化序列锁
    SeqLock_Init(&mtf02_data.data_lock);

    // 配置串口接收，长度严格固定为 27 字节
    config.usart_handle = &huart2; // 请修改为你实际使用的串口句柄
    config.recv_buff_size = MTF02_FRAME_SIZE;
    config.module_callback = MTF02_Rx_Callback;
    config.rx_mode = USART_RX_MODE_NORMAL; // 正常模式，触发 IDLE 或 TC 都可以

    mtf02_instance = USARTRegister(&config);
    (void)mtf02_instance;

    RTTINFO("[MTF02] MTF02 Optical Flow Init Success !");
}

/**
 * @brief 串口接收回调 (ISR上下文)
 */
static void MTF02_Rx_Callback(uint8_t *buf, uint16_t len)
{
    BaseType_t xWoken = pdFALSE;

    // 严格过滤：只有刚好拿到 27 字节才处理，防错位
    if (len != MTF02_FRAME_SIZE || buf == NULL) {
        return;
    }

    /* 序列锁：写操作 */
    SeqLock_WriteBegin(&mtf02_data.data_lock);
    {
        memcpy(shared_mtf02_frame, buf, MTF02_FRAME_SIZE);
    }
    SeqLock_WriteEnd(&mtf02_data.data_lock);

}

/**
 * @brief 在 SensorHub 任务中被调用进行数据提取
 */
void MTF02_Task_Handler(void)
{
    static uint32_t last_handled_seq = 0;
    uint8_t local_frame[MTF02_FRAME_SIZE];
    uint32_t local_timestamp_cyc = 0;
    uint32_t seq = 0;
    uint8_t read_success = 0;

    /* 序列锁：读操作 */
    for (uint8_t retry = 0; retry < MTF02_SEQLOCK_RETRY; retry++)
    {
        seq = SeqLock_ReadBegin(&mtf02_data.data_lock);

        memcpy(local_frame, shared_mtf02_frame, MTF02_FRAME_SIZE);
        local_timestamp_cyc = shared_timestamp_cyc;

        if (!SeqLock_ReadRetry(&mtf02_data.data_lock, seq))
        {
            read_success = 1;
            break; // 读取期间未被打断，数据一致
        }
    }

    // 如果多次重试均失败（极少发生），直接退出等下一次调度
    if (!read_success) {
        return;
    }

    // 防重处理与有效性检查（利用 seq 作为数据版本号校验）
    if (seq == 0U || seq == last_handled_seq || local_timestamp_cyc == 0U) {
        return;
    }

    last_handled_seq = seq;
}

/**
 * @brief 数据解包与校验
 */
static void MTF02_Decode(const uint8_t *buf, uint32_t timestamp_cyc)
{
    uint8_t checksum = 0;

    if (buf == NULL || timestamp_cyc == 0U) {
        return;
    }

    // 1. 检查帧头和消息ID：帧头 0xEF, 设备ID 0x0F, 系统ID 0x00, 消息ID 0x51
    if (buf[0] != 0xEF || buf[1] != 0x0F || buf[2] != 0x00 || buf[3] != 0x51) {
        return;
    }

    // 2. 校验和计算 (前面 26 个字节之和)
    for (uint8_t i = 0; i < (MTF02_FRAME_SIZE - 1); i++) {
        checksum += buf[i];
    }

    // 如果校验不通过则直接丢弃
    if (checksum != buf[MTF02_FRAME_SIZE - 1]) {
        return;
    }

    // 3. 数据提取：直接把第 6 字节开始的数据拷贝进结构体
    // buf[6] 是 Payload 起始位置
    memcpy(&mtf02_data.payload, &buf[6], sizeof(MTF02_Payload_t));

    // 4. 评估数据可用性
    if (mtf02_data.payload.tof_status == 1 && mtf02_data.payload.flow_status == 1) {
        mtf02_data.is_valid = 1U;
    } else {
        mtf02_data.is_valid = 0U;
    }

    // 5. 更新系统时间戳与发布机制
    mtf02_data.timestamp_us = Bsp_Timestamp_us_Get();

}
