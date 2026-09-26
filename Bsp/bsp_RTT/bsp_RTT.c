#include "bsp_RTT.h"

/**
 * @file bsp_RTT.c
 * @brief SEGGER RTT 初始化、格式化输出及无浮点 printf 场景的数值转换。
 */

/** @brief 初始化 RTT 控制块和上下行缓冲区。 */
void Bsp_RTT_Init()
{
    SEGGER_RTT_Init();
}

/** @brief 向配置的 RTT 终端写入格式化文本，返回 SEGGER RTT 的写入结果。 */
int Print_RTT(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    /* RTT 支持多个终端，本工程统一写入 BUFFER_INDEX 指定的单一终端。 */
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args);
    va_end(args);
    return n;
}

/**
 * @brief 将浮点数按三位小数写入字符串，避免依赖 printf 的浮点格式化支持。
 * @return 成功时返回字符数；参数非法或缓冲区不足时返回 -1。
 */
int Float2Str(char *str, size_t len, float va)
{
    if ((str == NULL) || (len == 0u))
    {
        return -1;
    }

    int scaled = (va >= 0.0f) ? (int)(va * 1000.0f + 0.5f) : (int)(va * 1000.0f - 0.5f);
    int head   = scaled / 1000;
    int point  = abs(scaled % 1000);

    int written = snprintf(str, len, "%d.%03d", head, point);
    if ((written < 0) || ((size_t)written >= len))
    {
        if (len > 0u)
        {
            str[len - 1u] = '\0';
        }
        return -1;
    }

    return written;
}
