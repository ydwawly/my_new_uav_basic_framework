//
// Created by Administrator on 2026/6/15.
//

#include "modules_Message_center.h"
#include <string.h>
#include "bsp_RTT.h"
#include "cmsis_compiler.h"

/* ================================================================
 * 全局静态内存池（Static Memory Pools）
 *
 * 为确保嵌入式系统的绝对稳定性，本框架严禁使用 malloc/free，
 * 所有 Publisher、Subscriber 实例以及真实传输的 Payload，
 * 全部在编译期划分好最大边界，并在运行时采用“游标单向偏移”进行分配。
 * ================================================================ */
static Publisher_t  Publisher_Pool[MAX_TOPICS];         /* 发布者控制块池 */
static Subscriber_t Subscriber_Pool[MAX_SUBSCRIBERS];   /* 订阅者控制块池 (修复: 使用 MAX_SUBSCRIBERS) */
static uint8_t      Message_Payload_Pool[Message_PAYLOAD_POOL_SIZE]; /* 物理数据缓存大内存池 */

static uint8_t Publisher_Used[MAX_TOPICS];              /* 标记 Publisher 槽位是否被占用（1=占用） */
static uint8_t Subscriber_Used[MAX_SUBSCRIBERS];        /* 标记 Subscriber 槽位是否被占用 */

static uint32_t Message_Payload_Offset = 0;             /* 数据缓存池分配游标 */
static uint8_t  Message_Center_Frozen = 0;              /* 冻结标志 (1=禁止新注册) */

/* ================================================================
 * 内部私有工具函数
 * ================================================================ */

/**
 * @brief 安全的字符串长度计算，带有硬件防护上限
 */
static uint32_t StrnLenLocal(const char *s, uint32_t max_len)
{
    uint32_t n = 0U;
    while ((n < max_len) && (s[n] != '\0'))
    {
        n++;
    }
    return n;
}

/**
 * @brief 致命错误拦截：检查 Topic 命名的合法性
 */
static void CheckName(const char *name)
{
    if (name == NULL)
    {
        RTTERROR("[MsgCenter] 名字为空");
        while (1) {}
    }

    if (StrnLenLocal(name, MAX_TOPIC_NAME_LEN + 1U) > MAX_TOPIC_NAME_LEN)
    {
        RTTERROR("[MsgCenter] 名字太长");
        while (1) {}
    }
}

/**
 * @brief 计算内存 4 字节向上对齐（Word Alignment）
 */
static uint32_t AlignUp4(uint32_t v)
{
    return (v + 3U) & ~3U;
}

/**
 * @brief 在对象池中根据名称寻找已存在的发布者
 */
static Publisher_t *FindPublisher(const char *name)
{
    uint32_t i;
    for (i = 0; i < MAX_TOPICS; i++)
    {
        if (Publisher_Used[i] && (strcmp(Publisher_Pool[i].topic_name, name) == 0))
        {
            return &Publisher_Pool[i];
        }
    }
    return NULL;
}

/**
 * @brief 从控制块池中申请一个空的 Publisher 槽位
 */
static Publisher_t *AllocPublisher(void)
{
    uint32_t i;
    for (i = 0; i < MAX_TOPICS; i++)
    {
        if (!Publisher_Used[i])
        {
            Publisher_Used[i] = 1;
            memset(&Publisher_Pool[i], 0, sizeof(Publisher_t)); /* 确保新块干净 */
            return &Publisher_Pool[i];
        }
    }
    return NULL;
}

/**
 * @brief 释放指定的 Publisher 槽位（用于 AllocPayload 内存不够时的事务回滚）
 */
static void FreePublisher(Publisher_t *pub)
{
    uint32_t idx;
    if (pub == NULL) return;

    idx = (uint32_t)(pub - Publisher_Pool);
    if (idx >= MAX_TOPICS) return;

    memset(&Publisher_Pool[idx], 0, sizeof(Publisher_t));
    Publisher_Used[idx] = 0;
}

/**
 * @brief 从控制块池中申请一个空的 Subscriber 槽位
 */
static Subscriber_t *AllocSubscriber(void)
{
    uint32_t i;
    for (i = 0U; i < MAX_SUBSCRIBERS; i++)
    {
        if (!Subscriber_Used[i])
        {
            Subscriber_Used[i] = 1U;
            memset(&Subscriber_Pool[i], 0, sizeof(Subscriber_t));
            return &Subscriber_Pool[i];
        }
    }
    return NULL;
}

/**
 * @brief 从全局 Payload 池中切分一块 4 字节对齐的内存
 */
static uint8_t *AllocPayload(uint32_t bytes)
{
    uint32_t offset = AlignUp4(Message_Payload_Offset);
    uint8_t *block;

    if ((offset + bytes) > Message_PAYLOAD_POOL_SIZE)
    {
        return NULL;
    }

    block = &Message_Payload_Pool[offset];
    Message_Payload_Offset = offset + bytes;
    memset(block, 0, bytes); /* 初始化为0，防止出现脏数据 */
    return block;
}

/* ================================================================
 * 核心底层读取函数：TryRead (无锁核心逻辑)
 * ================================================================ */
static uint8_t TryRead(Subscriber_t *sub, void *out)
{
    uint32_t retry;
    Publisher_t *pub;

    if ((sub == NULL) || (out == NULL)) return 0;

    pub = sub->pub;
    if ((pub == NULL) || (pub->data_ptr == NULL) ||(pub->data_len == 0)) return 0;

    for (retry = 0U; retry < Message_center_MAX_READ_RETRY; retry++)
    {
        uint32_t seq;

        /* 第一步：调用底层 API 进行非阻塞探查当前版本号 */
        seq = SeqLock_TryReadBegin(&pub->seqlock);

        /* 如果是奇数，表示底层正在写入，非阻塞直接进入下一次重试 */
        if(seq & 1U)
        {
            continue;
        }

        /* 检查数据是否有更新：0表示从未发布，或者与上次读取一致表示无新数据 */
        if ((seq == 0) || (seq == sub->last_read_seq))
        {
            return 0;
        }

        /* 第二步：执行内存搬运 */
        memcpy(out, pub->data_ptr, pub->data_len);

        /* 第三步：通过底层重试检查，验证读取期间是否被高优先级任务打断 */
        if (SeqLock_ReadRetry(&pub->seqlock, seq))
        {
            continue;
        }

        /* 成功读取，更新版本号 */
        sub->last_read_seq = seq;
        return 1;
    }
    /* 重试次数超限，强行放弃本帧数据，保障调度实时性 */
    return 0U;
}

/* ================================================================
 * 对外的用户 API 实现
 * ================================================================ */

void Message_Center_Init(void)
{
    /* 修复: memset的长度必须是整个池子的大小，而不是单个结构体大小 */
    memset(Publisher_Pool, 0, sizeof(Publisher_Pool));
    memset(Subscriber_Pool, 0, sizeof(Subscriber_Pool));
    memset(Publisher_Used, 0, sizeof(Publisher_Used));
    memset(Subscriber_Used, 0, sizeof(Subscriber_Used));
    memset(Message_Payload_Pool, 0, sizeof(Message_Payload_Pool));

    Message_Payload_Offset = 0;
    Message_Center_Frozen = 0;
}

void Message_Center_Freeze(void)
{
    /* 进入临界区，锁定框架，拒绝任何新注册 */
    taskENTER_CRITICAL();
    Message_Center_Frozen = 1;
    taskEXIT_CRITICAL();
}

Publisher_t *PubRegister(char *name, uint16_t data_len)
{
    Publisher_t *pub;
    uint8_t *payload;
    uint32_t payload_backup;
    uint32_t name_len;

    CheckName(name);

    if ((data_len == 0U) || (data_len > DATA_BUF_SIZE))
    {
        RTTERROR("[MsgCenter] 数据长度无效: %u", data_len);
        return NULL;
    }

    taskENTER_CRITICAL();

    pub = FindPublisher(name);
    if (pub != NULL)
    {
        if (pub->data_len != data_len)
        {
            taskEXIT_CRITICAL();
            RTTERROR("[MsgCenter] 数据长度不匹配: %s", name);
            return NULL;
        }
        taskEXIT_CRITICAL();
        return pub;
    }

    /* 如果框架已经冻结，禁止再申请新内存 */
    if (Message_Center_Frozen)
    {
        taskEXIT_CRITICAL();
        RTTERROR("[MsgCenter] 冻结后不允许注册: %s", name);
        return NULL;
    }

    payload_backup = Message_Payload_Offset;

    pub = AllocPublisher();
    if (pub == NULL)
    {
        taskEXIT_CRITICAL();
        RTTERROR("[MsgCenter] Publisher 池已满");
        return NULL;
    }

    payload = AllocPayload(AlignUp4(data_len));
    if (payload == NULL)
    {
        Message_Payload_Offset = payload_backup;
        FreePublisher(pub);
        taskEXIT_CRITICAL();
        RTTERROR("[MsgCenter] 数据缓存池已满");
        return NULL;
    }

    name_len = StrnLenLocal(name, MAX_TOPIC_NAME_LEN);
    memcpy(pub->topic_name, name, name_len);
    pub->topic_name[name_len] = '\0';

    pub->data_len  = data_len;
    pub->sub_count = 0U;
    pub->data_ptr  = payload;

    SeqLock_Init(&pub->seqlock);

    taskEXIT_CRITICAL();
    return pub;
}

Subscriber_t *SubRegister(char *name, uint16_t data_len)
{
    Publisher_t *pub;
    Subscriber_t *sub;

    pub = PubRegister(name, data_len);
    if (pub == NULL)
    {
        return NULL;
    }

    taskENTER_CRITICAL();

    if (Message_Center_Frozen)
    {
        taskEXIT_CRITICAL();
        RTTERROR("[MsgCenter] 冻结后不允许订阅: %s", name);
        return NULL;
    }

    if (pub->sub_count >= MAX_SUBS_PER_TOPIC)
    {
        taskEXIT_CRITICAL();
        RTTERROR("[MsgCenter] 订阅者数量已达上限: %s", name);
        return NULL;
    }

    sub = AllocSubscriber();
    if (sub == NULL)
    {
        taskEXIT_CRITICAL();
        RTTERROR("[MsgCenter] Subscriber 池已满");
        return NULL;
    }

    sub->pub           = pub;
    sub->last_read_seq = 0U;
    sub->waiting_task  = NULL;
    sub->notify_flag   = 0U;

    pub->subs[pub->sub_count++] = sub;

    taskEXIT_CRITICAL();
    return sub;
}

uint8_t PubPushMessage(Publisher_t *pub, void *data)
{
    uint32_t i;
    uint16_t sub_count;

    if ((pub == NULL) || (data == NULL))
    {
        return 0;
    }

    /* === 阶段 1：安全的写入共享内存 === */
    SeqLock_WriteBegin(&pub->seqlock);
    memcpy(pub->data_ptr, data, pub->data_len);
    SeqLock_WriteEnd(&pub->seqlock);

    /* === 阶段 2：唤醒挂起休眠的对应任务 === */
    sub_count = pub->sub_count;
    for (i = 0; i < sub_count; i++)
    {
        Subscriber_t *sub = pub->subs[i];
        TaskHandle_t task;

        if (sub == NULL || !sub->notify_flag)
        {
            continue;
        }

        task = sub->waiting_task;
        if (task == NULL)
        {
            continue;
        }

        if (xPortIsInsideInterrupt())
        {
            BaseType_t woken = pdFALSE;
            vTaskNotifyGiveFromISR(task, &woken);
            portYIELD_FROM_ISR(woken);
        } else
        {
            xTaskNotifyGive(task);
        }
    }
    return (sub_count > 0U) ? 1U : 0U;
}

uint8_t SubGetMessage(Subscriber_t *sub, void *out, TickType_t xWait)
{
    TickType_t start;
    TickType_t remaining;

    if ((sub == NULL) || (out == NULL))
    {
        return 0;
    }

    /* 非阻塞或在中断环境，直接抓取 */
    if ((xWait == 0) || xPortIsInsideInterrupt())
    {
        return TryRead(sub, out);
    }

    /* 阻塞环境：进休眠前先快速抓取一次 */
    if (TryRead(sub, out))
    {
        return 1;
    }

    start = xTaskGetTickCount();
    remaining = xWait;

    while (remaining > 0)
    {
        /* 设置标志让发布者知道我在等，必须使用 DMB 确保指令不乱序 */
        sub->waiting_task = xTaskGetCurrentTaskHandle();
        __DMB();
        sub->notify_flag = 1U;
        __DMB();

        /* 防止设标志的缝隙里正好来了数据，做一次兜底读取 */
        if (TryRead(sub, out))
        {
            sub->notify_flag = 0U;
            sub->waiting_task = NULL;
            return 1;
        }

        /* 交出 CPU，挂起等待 */
        (void)ulTaskNotifyTake(pdTRUE, remaining);

        /* 醒来后立刻复位标志 */
        sub->notify_flag = 0U;
        sub->waiting_task = NULL;
        __DMB();

        /* 再次尝试读取 */
        if (TryRead(sub, out))
        {
            return 1;
        }

        /* 检查是否超时 */
        if ((xTaskGetTickCount() - start) >= xWait)
        {
            break;
        }

        remaining = xWait - (xTaskGetTickCount() - start);
    }
    return TryRead(sub, out);
}

