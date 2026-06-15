/**
 * @file    modules_message_center.h
 * @brief   基于发布-订阅（Pub-Sub）模型的无锁消息中心
 * @note    适用于高频数据分发场景（如飞控系统的 IMU 数据广播、控制指令下发等）。
 * 采用静态内存池 + 顺序锁（Seqlock）机制，保证中断/高优先级任务
 * 与低优先级任务之间的数据安全，且无锁操作不会引发死锁或优先级反转。
 */

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_MESSAGE_CENTER_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_MESSAGE_CENTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* FreeRTOS 核心头文件，用于任务句柄和任务通知机制 */
#include "FreeRTOS.h"
#include "task.h"

/* 引入底层硬件/汇编级别的序列锁封装 */
#include "bsp_utils_seqlock.h"

/* ================================================================
 * 系统容量与内存配置参数
 * ================================================================ */

#ifndef MAX_TOPICS
#define MAX_TOPICS              32U     /* 系统最多支持注册的主题（Topic）数量 */
#endif

#ifndef MAX_SUBSCRIBERS
#define MAX_SUBSCRIBERS         32U     /* 全局系统中最多允许存在的订阅者实例总数 */
#endif

#ifndef MAX_SUBS_PER_TOPIC
#define MAX_SUBS_PER_TOPIC      8U      /* 单个主题最多允许被多少个订阅者同时订阅 */
#endif

#ifndef MAX_TOPIC_NAME_LEN
#define MAX_TOPIC_NAME_LEN      31U     /* 主题名称的最大长度（不含结尾的 '\0'） */
#endif

#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE           256U    /* 单个主题单次传输数据的最大字节数 */
#endif

#ifndef Message_PAYLOAD_POOL_SIZE
#define Message_PAYLOAD_POOL_SIZE 2048U /* 全局数据缓存池总大小（字节） */
#endif

/* 读取数据冲突时的最大重试次数。
 * 在多任务抢占环境中，限制重试次数可防止极端情况下死循环。 */
#ifndef Message_center_MAX_READ_RETRY
#define Message_center_MAX_READ_RETRY 3U
#endif

/* ================================================================
 * 结构体前向声明
 * ================================================================ */
typedef struct Publisher Publisher_t;
typedef struct Subscriber Subscriber_t;

/* ================================================================
 * 发布者（Publisher）结构体定义
 *
 * @note 每个 Topic 在系统中只对应一个唯一的 Publisher 实例。
 * Publisher 拥有该 Topic 唯一的一块物理数据缓存（data_ptr）。
 * 所有的 Subscriber 在读取数据时，都是从这块唯一的缓存中拷贝数据。
 * ================================================================ */
struct Publisher {
    char          topic_name[MAX_TOPIC_NAME_LEN + 1U];  /* 主题名称，系统查找的唯一键值 */
    uint16_t      data_len;                              /* 约定的数据包长度（字节） */
    uint16_t      sub_count;                             /* 当前已订阅该主题的订阅者数量 */
    uint8_t      *data_ptr;                              /* 指向静态分配的共享数据缓存区 */
    SeqLock_t     seqlock;                               /* 底层序列锁，保护并发读写的一致性 */
    Subscriber_t *subs[MAX_SUBS_PER_TOPIC];              /* 订阅者指针数组，记录关注者 */
};

/* ================================================================
 * 订阅者（Subscriber）结构体定义
 *
 * @note 订阅者本身不占用独立的数据缓存，只相当于一个“游标”，
 * 记录了自己当前读取到的数据版本号以及挂起状态。
 * ================================================================ */
struct Subscriber {
    Publisher_t  *pub;              /* 指向关联的目标发布者 */
    uint32_t      last_read_seq;    /* 记录上次成功读取到的序列锁版本号 */
    TaskHandle_t  waiting_task;     /* 阻塞等待机制：记录调用阻塞读的任务句柄 */
    volatile uint8_t notify_flag;   /* 唤醒标志：1表示该订阅者正在挂起等待新数据 */
};

/* ================================================================
 * 核心 API 函数声明
 * ================================================================ */

/**
 * @brief 初始化整个消息中心（清空所有静态内存池）
 */
void Message_Center_Init(void);

/**
 * @brief 冻结消息中心
 * @note  调用后禁止再注册新主题或订阅者，防止运行期出现内存碎片或分配冲突。
 */
void Message_Center_Freeze(void);

/**
 * @brief 注册发布者（创建新主题）
 */
Publisher_t *PubRegister(char *name, uint16_t data_len);

/**
 * @brief 注册订阅者（挂载到已有主题）
 */
Subscriber_t *SubRegister(char *name, uint16_t data_len);

/**
 * @brief 向主题发布（推送）最新数据 (支持在中断中调用)
 */
uint8_t PubPushMessage(Publisher_t *pub, void *data);

/**
 * @brief 订阅者获取（拉取）最新数据
 */
uint8_t SubGetMessage(Subscriber_t *sub, void *out, TickType_t xWait);

#ifdef __cplusplus
}
#endif

#endif // MY_NEW_UAV_BAICE_FRAMEWORK_MODULES_MESSAGE_CENTER_H