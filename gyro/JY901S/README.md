# JY901S

包名：`SensorGyro::JY901S`。依赖 `FreeRTOS`、`Math::Geometry`、`protocol::UartRxSync`、`utils`。[仓库总览](../../README.md)

## 接入条件

[JY901S.hpp](JY901S.hpp) 提供 `sensor::gyro::JY901S`，注意现有命名空间是单数 `sensor`。构造参数为 `huart, buffer, pose_in_body`，或追加 `config`；`pose_in_body` 的类型为 `const math::Posef&`，配置类型为 `JY901S::Config`。`Buffer` 对应 `memory::DMABuffer<11>`，接收容量为 11 字节，必须由调用方持有，不能省略。

宿主以 CubeMX 配置 UART、循环 RX DMA、IRQ 及 UART Register Callbacks，并按[UART 接收共性](../../README.md#uart-接收共性)完成回调注册，再调用 `startReceive()` 并检查返回值。每实例使用独立的 DMA 可达、满足缓存一致性要求的 Buffer，且其生命周期覆盖全部接收和回调。

## 运行边界

安装位姿由宿主按实际装配提供，不能套用固定板卡或车体的参数。构造不代替 `init()` 对传感器输出内容、速率和轴模式的配置；这些是传感器运行命令，不是重新初始化宿主 UART。初始化和校准使用 CMSIS-RTOS 延时，须在适当的任务上下文调用，不在 IRQ 中执行。

姿态状态的加速度采用 m/s²、角速度采用 rad/s，显示用欧拉角采用度；坐标变换、状态更新及具体接口以 [JY901S.hpp](JY901S.hpp) 和现有源码说明为准，不另建平行的解算文档。

本包需要相应 RTOS 环境及 `protocol::UartRxSync` 支持的 HAL1 UART/DMA 接口，不因仓库共享系列列表而承诺全系列兼容。
