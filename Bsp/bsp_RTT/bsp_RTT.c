//
// Created by Administrator on 2026/6/9.
//

#include "bsp_RTT.h"

void Bsp_RTT_Init()
{
    SEGGER_RTT_Init();
}

int Print_RTT(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
}

int Float2Str(char *str, size_t len, float va)
{
    if ((str == NULL) || (len == 0u))
    {
        return -1;
    }

    int scaled = (va >= 0.0f) ? (int)(va * 1000.0f + 0.5f) : (int)(va * 1000.0f - 0.5f);
    int head = scaled / 1000;
    int point = abs(scaled % 1000);

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

