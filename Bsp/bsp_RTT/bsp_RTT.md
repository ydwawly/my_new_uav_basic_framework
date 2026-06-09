# bsp_RTT 使用说明

## 1. 作用

`bsp_RTT` 是这个工程对 SEGGER RTT 的一层轻量封装，用来做以下几件事：

- 在调试时输出日志，不占用串口。
- 通过 J-Link + Ozone / J-Link RTT Viewer 实时查看打印内容。
- 给 `printf()` 提供输出通道，统一转发到 RTT Channel 0。

这个工程当前已经把 RTT、SystemView、FreeRTOS 跟踪链路接好了，RTT 可以直接用。

## 2. 工程里当前是怎么接入的

### 2.1 初始化路径

`Bsp_Init()` 里已经调用了：

```c
Bsp_Timestamp_Init();
Bsp_RTT_Init();
```

也就是说，如果系统启动流程已经执行了 `Bsp_Init()`，RTT 就已经初始化了，不需要你在业务代码里再单独调用一次 `Bsp_RTT_Init()`。

### 2.2 `printf()` 已重定向到 RTT

工程里的 `_write()` 已经改成：

```c
return (int)SEGGER_RTT_Write(0u, ptr, (unsigned)len);
```

因此下面两种写法都可以看到输出：

```c
printf("hello\r\n");
RTTINFO("hello");
```

默认使用的是 `RTT Channel 0`。

### 2.3 SystemView 也会占用 RTT

启动阶段已经执行：

```c
SEGGER_SYSVIEW_Conf();
SEGGER_SYSVIEW_Start();
```

所以当前 RTT 不只是普通日志输出通道，也同时服务于 SystemView 的事件采集。

## 3. 头文件与接口

使用前包含：

```c
#include "bsp_RTT.h"
```

当前可用接口如下。

### 3.1 初始化

```c
void Bsp_RTT_Init(void);
```

作用：初始化 SEGGER RTT。

说明：通常不需要手动调用，因为 `Bsp_Init()` 已经做了这件事。

### 3.2 普通格式化输出

```c
int Print_RTT(const char *fmt, ...);
```

作用：调用 `SEGGER_RTT_vprintf()` 输出格式化字符串。

说明：

- 适合做简单日志输出。
- 不建议依赖浮点格式化。

示例：

```c
Print_RTT("cnt=%d, state=%d\r\n", cnt, state);
```

### 3.3 浮点转字符串

```c
int Float2Str(char *str, size_t len, float va);
```

作用：把 `float` 转成字符串，再交给 RTT 输出。

特点：

- 保留 3 位小数。
- 已增加缓冲区长度保护。
- 成功时返回写入字符数，失败返回 `-1`。

示例：

```c
char buf[16];
if (Float2Str(buf, sizeof(buf), imu_yaw) > 0)
{
    RTTINFO("yaw=%s", buf);
}
```

## 4. 推荐使用的日志宏

### 4.1 无级别输出

```c
RTT("mode=%d", mode);
```

效果：

- 输出到 RTT。
- 自动补 `\r\n`。
- 不带日志级别前缀。

### 4.2 信息日志

```c
RTTINFO("arm=%d", arm_state);
```

效果：

- 前缀为 `I:`
- 绿色显示

### 4.3 警告日志

```c
RTTWARNING("battery low: %d", battery_mv);
```

效果：

- 前缀为 `W:`
- 黄色显示

### 4.4 错误日志

```c
RTTERROR("imu init failed: %d", ret);
```

效果：

- 前缀为 `E:`
- 红色显示

### 4.5 清屏

```c
RTT_CLEAR();
```

作用：向 RTT 终端发送清屏控制码。

## 5. 典型用法

### 5.1 最常用写法

```c
void App_Task(void)
{
    RTTINFO("task start");
    RTTWARNING("loop=%d", loop_cnt);
}
```

### 5.2 打印浮点

```c
char pitch_str[16];
if (Float2Str(pitch_str, sizeof(pitch_str), pitch_deg) > 0)
{
    RTTINFO("pitch=%s", pitch_str);
}
```

### 5.3 直接用 `printf`

```c
printf("roll=%d\r\n", roll_raw);
```

适合快速验证，但如果你希望统一颜色、级别和格式，优先使用 `RTTINFO / RTTWARNING / RTTERROR`。

## 6. 如何在电脑端查看输出

### 6.1 用 Ozone

1. 用 J-Link 连接飞控。
2. 在 Ozone 中加载工程生成的 `.elf`。
3. 连接目标后打开 RTT 窗口。
4. 查看 Channel 0 输出。

适合边调试边看变量和 RTT 日志。

### 6.2 用 J-Link RTT Viewer

1. 连接 J-Link。
2. 选择目标芯片 `STM32H743`。
3. 启动 RTT Viewer。
4. 观察 Terminal 0。

适合单独看日志，不需要完整图形调试。

### 6.3 用 SystemView

如果你要看任务切换、ISR、运行时序，就用 SystemView，不要把它当成纯日志窗口。SystemView 主要看事件轨迹，RTT 主要看文本输出。

## 7. 使用注意事项

### 7.1 浮点不要直接赌格式化

虽然某些 `printf` 配置下可能能打印浮点，但在嵌入式工程里这样通常代价更高，也更不稳定。这个工程建议统一写成：

```c
char buf[16];
Float2Str(buf, sizeof(buf), value);
RTTINFO("value=%s", buf);
```

### 7.2 不要在高频中断里疯狂打印

RTT 比串口快很多，但它不是“零成本”。如果在高频中断、控制环或 DMA 回调里持续打印，仍然会影响实时性。

建议：

- 高频路径只打印关键错误。
- 周期任务内限频打印。
- 调参日志尽量集中在低优先级任务。

### 7.3 Release 版本可关闭日志

当前封装支持：

```c
#define DISABLE_RTT_SYSTEM 1
```

或者在编译选项里定义该宏，这样 `RTTINFO / RTTWARNING / RTTERROR` 会被编译为空。

说明：`printf()` 是否关闭，还取决于你是否继续调用它，以及 `_write()` 是否保留。

### 7.4 当前默认缓冲区是 0 号通道

封装里定义的是：

```c
#define BUFFER_INDEX 0
```

如果后续你需要：

- 一个通道打印控制日志
- 一个通道打印传感器原始数据

那就可以继续扩展多通道 RTT，而不是所有内容都混在同一个窗口里。

## 8. 推荐实践

- 常规状态变化用 `RTTINFO()`。
- 异常但未致命的问题用 `RTTWARNING()`。
- 初始化失败、数据非法、保护触发用 `RTTERROR()`。
- 调试临时变量时可以直接 `RTT()` 或 `printf()`。
- 浮点一律先 `Float2Str()` 再打印。

## 9. 一段完整示例

```c
#include "bsp_RTT.h"

void App_DebugOutput(float roll, float pitch, int armed)
{
    char roll_str[16];
    char pitch_str[16];

    RTTINFO("debug start");

    if (Float2Str(roll_str, sizeof(roll_str), roll) > 0 &&
        Float2Str(pitch_str, sizeof(pitch_str), pitch) > 0)
    {
        RTTINFO("armed=%d roll=%s pitch=%s", armed, roll_str, pitch_str);
    }
    else
    {
        RTTERROR("float format failed");
    }
}
```

## 10. 结论

对这个工程来说，`bsp_RTT` 现在已经处于可直接使用状态：

- 初始化链路已接通。
- `printf()` 已转发到 RTT。
- Ozone / RTT Viewer 可以直接看输出。
- SystemView 跟踪也已启用。

如果后面你要继续扩展，我建议优先做这两件事：

1. 增加多通道 RTT 规划。
2. 给不同模块统一日志前缀，例如 `[IMU]`、`[CTRL]`、`[MOTOR]`。
