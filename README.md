# Sensors

## UART 接收缓冲区

基于 `protocol::UartRxSync` 的传感器驱动使用调用方持有的 DMA 接收缓冲区。
构造函数的第二个参数统一为 `Buffer&`，不再支持省略缓冲区的旧构造形式。
`Buffer` 是驱动继承的公开类型别名，对应固定容量的 `memory::DMABuffer<N>`，不分配堆内存。

| 驱动 | 接收容量（字节） | 构造参数 |
| --- | ---: | --- |
| `sensors::gyro::HWT101CT` | 11 | `huart, buffer` |
| `sensor::gyro::JY901S` | 11 | `huart, buffer, pose_in_body` 或 `huart, buffer, pose_in_body, config` |
| `sensors::laser::DT35Board` | 24 | `huart, buffer` |
| `sensors::laser::STP23L` | 195 | `huart, buffer` |
| `sensors::ops::ActionOPS` | 28 | `huart, buffer, config` |

例如，为 DT35 板保留独立的静态接收存储：

```cpp
#include "DT35.hpp"

extern UART_HandleTypeDef huart1;

static sensors::laser::DT35Board::Buffer dt35_rx_buffer;
static sensors::laser::DT35Board dt35_board(&huart1, dt35_rx_buffer);
```

- 缓冲区必须在构造驱动前存在，并保持有效直到 DMA 接收及相关回调不再访问它。
  对于永久运行的驱动，优先使用静态存储；不要在初始化函数中创建临时局部缓冲区。
- 每个同时接收的驱动实例使用独立缓冲区，不得共享仍在使用的接收存储。
- 默认 32 字节对齐不保证 DMA 可达或不可缓存；实例所在内存区域、缓存策略和同步由项目负责。
  STM32F407 应使用 DMA 可访问的 SRAM，不能将接收缓冲区放入 CCM。
- UART RX DMA 必须配置为循环模式；完成回调注册后再调用 `startReceive()`，并检查返回值。
