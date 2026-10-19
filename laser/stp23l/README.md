# STP23L

包名：`SensorLaser::STP23L`。依赖 `protocol::UartRxSync`。[仓库总览](../../README.md)

## 接入条件

[STP23L.hpp](STP23L.hpp) 提供 `sensors::laser::STP23L`。构造形式为 `STP23L(UART_HandleTypeDef* huart, Buffer& buffer)`，`Buffer` 对应 `memory::DMABuffer<195>`，接收容量为 195 字节。缓冲区由调用方独占持有，且必须覆盖接收及回调的全部使用期。

宿主以 CubeMX 配置 UART、循环 RX DMA、IRQ 及 UART Register Callbacks，按[UART 接收共性](../../README.md#uart-接收共性)注册 RX 完成和错误回调后再调用 `startReceive()`，并检查返回值。Buffer 必须位于 DMA 可达且满足缓存一致性要求的内存，32 字节对齐不能代替这些条件。

## 输出与支持边界

驱动消费固定格式的测距数据，提供距离反馈，单位为 mm；不是通用点云接口。帧解析和反馈更新规则以 [STP23L.hpp](STP23L.hpp)、[STP23L.cpp](STP23L.cpp) 及已有源码说明为准，不另述逐字段解析算法。

本包沿用 `protocol::UartRxSync` 的 HAL1 与经典循环 DMA 接口要求；没有逐料号兼容承诺，也不负责重新初始化宿主外设。
