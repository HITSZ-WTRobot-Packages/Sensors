# HWT101CT

包名：`SensorGyro::HWT101CT`。依赖 `protocol::UartRxSync`、`utils`。[仓库总览](../../README.md)

## 接入条件

[HWT101CT.hpp](HWT101CT.hpp) 提供 `sensors::gyro::HWT101CT`。构造形式为 `HWT101CT(UART_HandleTypeDef* huart, Buffer& buffer)`，`Buffer` 对应 `memory::DMABuffer<11>`，接收容量为 11 字节。调用方必须先创建并独占持有缓冲区，直至 DMA 和回调不再访问它。

宿主以 CubeMX 配置 UART、循环 RX DMA、IRQ 及 UART Register Callbacks。按[UART 接收共性](../../README.md#uart-接收共性)注册 RX 完成和错误回调，然后调用 `startReceive()` 并检查返回值。缓冲区的 DMA 可达性、缓存一致性和生命周期由宿主保证，不能只依赖 32 字节对齐。

## 运行边界

驱动提供航向角、角速度及传感器输出速率/校准操作，不负责宿主外设初始化。调用 `resetYaw()` 会立即清除本地航向跟踪基准，但不会立即修改 `getYaw()` 的返回值；该值要等后续有效航向帧（`0x53`）到达后才更新。校准需要传感器静止且会阻塞；相关等待和调用条件以 [HWT101CT.hpp](HWT101CT.hpp) 的现有 Doxygen 为准，不应从中断上下文发起阻塞校准。

本包沿用 `protocol::UartRxSync` 的 HAL1 与经典循环 DMA 配置要求，未声明对全部 STM32 系列逐料号验证；选择宿主 HAL 版本时需核对依赖包能力。
