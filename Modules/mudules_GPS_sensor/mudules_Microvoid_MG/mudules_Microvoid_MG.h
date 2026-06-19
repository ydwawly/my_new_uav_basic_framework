//
// Created by Administrator on 2026/6/18.
//

#ifndef MY_NEW_UAV_BAICE_FRAMEWORK_MUDULES_MICROVOID_MG_H
#define MY_NEW_UAV_BAICE_FRAMEWORK_MUDULES_MICROVOID_MG_H

#include <stdint.h>
#include "bsp_uart.h"
#include "bsp_utils_seqlock.h"
#include "modules_Message_center.h"
#include "usart.h"

/* ========================== 私有配置 ========================== */

#define GPS_UART_HANDLE          huart2   /* 按实际硬件修改 */
#define GPS_SEQLOCK_MAX_RETRY    3U
#define GPS_RX_BUF_SIZE          200U
/* ========================== 配置 ========================== */

#define GPS_TOPIC_NAME  "gps_data"
/* ========================== UBX 协议定义 ========================== */

#define UBX_SYNC1               0xB5U
#define UBX_SYNC2               0x62U

#define UBX_NAV_CLASS           0x01U
#define UBX_CFG_CLASS           0x06U

#define UBX_NAV_PVT             0x07U
#define UBX_CFG_PRT             0x00U
#define UBX_CFG_MSG             0x01U
#define UBX_CFG_RATE            0x08U
#define UBX_CFG_NAV5            0x24U

#define UBX_PVT_PAYLOAD_LEN     92U
#define UBX_MAX_PAYLOAD_LEN     96U

/* ========================== 对外发布数据结构 ========================== */

/**
 * @brief GPS 解析后的导航数据
 *
 * @note  fix_type: 0=无定位 2=2D 3=3D 4=GNSS+DR 5=仅时间
 *        速度单位 m/s, 高度单位 m, 精度单位 m, 航向单位 deg
 */
typedef struct
{
    /* 定位 */
    double   latitude;       /* deg */
    double   longitude;      /* deg */
    float    altitude_msl;   /* m 海拔 */
    float    altitude_ellip; /* m 椭球高 */

    /* 速度 */
    float    velN;           /* m/s 北向 */
    float    velE;           /* m/s 东向 */
    float    velD;           /* m/s 地向(向下为正) */
    float    ground_speed;   /* m/s 地面速度 */
    float    heading;        /* deg 运动航向 */

    /* 精度估计 */
    float    hAcc;           /* m 水平精度 */
    float    vAcc;           /* m 垂直精度 */
    float    sAcc;           /* m/s 速度精度 */
    float    headAcc;        /* deg 航向精度 */
    float    pDOP;           /* 位置精度因子 */

    /* 定位状态 */
    uint8_t  fix_type;       /* GNSS 定位类型 */
    uint8_t  num_sv;         /* 参与定位卫星数 */
    uint8_t  fix_flags;      /* 定位标志位 */

    /* UTC 时间 */
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  time_valid;     /* 时间有效性标志 */

    /* 本地采集时间戳 */
    uint64_t GPS_Timestamp;
} GPS_Data_t;

/* ========================== UBX 解析器 ========================== */

typedef enum
{
    UBX_STATE_SYNC1 = 0,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN1,
    UBX_STATE_LEN2,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CK_A,
    UBX_STATE_CK_B,
} UBX_ParseState_e;

typedef struct
{
    UBX_ParseState_e state;
    uint8_t  msg_class;
    uint8_t  msg_id;
    uint16_t length;
    uint16_t count;
    uint8_t  ck_a;
    uint8_t  ck_b;
    uint8_t  payload[UBX_MAX_PAYLOAD_LEN];
} UBX_Parser_t;

/* ========================== NAV-PVT 负载结构体 ========================== */

/*
 * UBX-NAV-PVT payload = 92 字节
 *
 * 偏移  类型     名称
 * ────────────────────────────
 * 0     U4       iTOW
 * 4     U2       year
 * 6     U1       month
 * 7     U1       day
 * 8     U1       hour
 * 9     U1       min
 * 10    U1       sec
 * 11    X1       valid
 * 12    U4       tAcc
 * 16    I4       nano
 * 20    U1       fixType
 * 21    X1       flags
 * 22    X1       flags2
 * 23    U1       numSV
 * 24    I4       lon          (1e-7 deg)
 * 28    I4       lat          (1e-7 deg)
 * 32    I4       height       (mm, 椭球高)
 * 36    I4       hMSL         (mm, 海拔)
 * 40    U4       hAcc         (mm)
 * 44    U4       vAcc         (mm)
 * 48    I4       velN         (mm/s)
 * 52    I4       velE         (mm/s)
 * 56    I4       velD         (mm/s)
 * 60    I4       gSpeed       (mm/s)
 * 64    I4       headMot      (1e-5 deg)
 * 68    U4       sAcc         (mm/s)
 * 72    U4       headAcc      (1e-5 deg)
 * 76    U2       pDOP         (0.01)
 * 78    X2       flags3
 * 80    U1[4]    reserved0
 * 84    I4       headVeh      (1e-5 deg)
 * 88    I2       magDec       (1e-2 deg)
 * 90    U2       magAcc       (1e-2 deg)
 */
#pragma pack(push, 1)
typedef struct
{
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t  velN;
    int32_t  velE;
    int32_t  velD;
    int32_t  gSpeed;
    int32_t  headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint16_t flags3;
    uint8_t  reserved0[4];
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;
} UBX_NAV_PVT_Payload_t; /* = 92 bytes */
#pragma pack(pop)

/* ========================== 驱动实例 ========================== */

typedef struct
{
    SeqLock_t data_lock;

    uint8_t  raw_rx_buf[GPS_RX_BUF_SIZE];
    uint16_t raw_len;
    uint64_t capture_timestamp;
    uint32_t rx_frame_count;
    uint32_t last_proc_frame_count;

    /* UBX 状态机(仅在任务里使用, 不需要 SeqLock) */
    UBX_Parser_t parser;

    volatile uint32_t rx_err_count;
    volatile uint32_t parse_err_count;
    volatile uint32_t pvt_count;

    Publisher_t   *publisher;
    USARTInstance *usart_instance;
} GPS_Instance_t;
/* ========================== 对外接口 ========================== */

/**
 * @brief 初始化 GPS 驱动
 *
 * @return 1 成功  0 失败
 *
 * @note 内部自动完成:
 *       1. 注册消息中心发布者
 *       2. 注册 UART, DMA Normal 模式
 *       3. 发送 UBX 配置命令(端口/速率/导航模式/PVT 使能)
 *       4. 初始化 UBX 解析器
 *
 * @warning CubeMX 串口配置: 波特率 115200, 8N1
 */
uint8_t GPS_Init(void);

/**
 * @brief GPS 任务处理函数
 *
 * @return 1 成功解析并发布了一帧新 PVT 数据
 * @return 0 无新数据或解析未完成
 *
 * @note 在任务中周期调用, 建议调用周期 <= 5ms
 *       内部使用 UBX 状态机逐字节解析,
 *       即使一帧跨越多次 IDLE 触发也能正确处理
 */
uint8_t GPS_Task_Handler(void);
#endif //MY_NEW_UAV_BAICE_FRAMEWORK_MUDULES_MICROVOID_MG_H