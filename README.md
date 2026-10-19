# Sensors

本仓库作为宿主 STM32 项目的 Git SubModule，提供传感器及灯带驱动；不提供独立固件构建入口。包名和依赖以各包的 `cpkg.toml` 为准，由宿主通过 cpkg 集成。

## 包索引

| 包 | 目录与使用说明 | 接入资源 |
| --- | --- | --- |
| `SensorGyro::HWT101CT` | [gyro/HWT101CT](gyro/HWT101CT/README.md) | UART RX DMA |
| `SensorGyro::JY901S` | [gyro/JY901S](gyro/JY901S/README.md) | UART RX DMA、安装位姿；依赖 FreeRTOS |
| `SensorLaser::DT35` | [laser/dt35](laser/dt35/README.md) | DT35 转接板 UART RX DMA |
| `SensorLaser::STP23L` | [laser/stp23l](laser/stp23l/README.md) | UART RX DMA |
| `SensorOPS::ActionOPS` | [ops/ActionOPS](ops/ActionOPS/README.md) | UART RX DMA、外部航向角；依赖 FreeRTOS |
| `SensorPressure::XGZP6847D` | [pressure/XGZP6847D](pressure/XGZP6847D/README.md) | I2C DMA 总线与设备 manager |
| `SensorLed::WS2812B` | [led/ws2812](led/ws2812/README.md) | 专用 TIM PWM、CC 循环 DMA |

包之间的 HAL、DMA 和运行环境要求不同。共享宿主可能采用 F4、G4、H7、H5，不代表每个包均直接兼容这些系列；尤其 WS2812B 的 F4、H5 等限制见其包文档。代码级细节以对应头文件及现有 Doxygen 为准。

## 通用资源契约

- 宿主使用 CubeMX 配置时钟、GPIO、外设、DMA 请求及 IRQ，完成 HAL1 初始化后再启动驱动；运行期操作和包级校验的边界见各包 README。不得把新维护规则理解为当前代码已逐项校验所有硬件配置。
- 驱动借用的句柄、Buffer 和其他外部数据必须覆盖所有异步访问的生命周期；宿主负责 IRQ 分发、回调注册或转发及资源退出顺序。不能以栈上临时资源支撑长期 DMA。
- DMA Buffer 的对齐不保证 DMA 可达或缓存一致；存储区域、缓存策略及必要同步由宿主负责。STM32F407 的 CCM、经典 H7 默认 DTCM 不能供对应 DMA1/2 使用。不要直接套用某个消费工程的链接段或内存布局。
- 硬件配置优先使用 CubeMX 和 HAL1；HAL1 不足或所需校验无法完成时，维护者须先通过 Ask 确认，不自行新增寄存器绕过或静默忽略方案。

## UART 接收共性

HWT101CT、JY901S、DT35Board、STP23L、ActionOPS 基于 `protocol::UartRxSync`，使用调用方持有的 DMA 接收缓冲区：

- 构造函数第二个参数为 `Buffer&`，不能省略。公开继承的 `Buffer` 类型对应固定容量的 `memory::DMABuffer<N>`，缓冲区本身不分配堆内存；容量和构造形式见对应包 README。
- 缓冲区必须在构造驱动前存在，并保持有效直到 DMA 接收及相关回调不再访问它。永久运行的驱动优先使用静态存储；不要在初始化函数中创建临时局部缓冲区。每个同时接收的实例必须使用独立缓冲区。
- 默认 32 字节对齐仍须满足上述 DMA 可达性和缓存契约，不能只对齐后就假定安全。
- CubeMX 必须启用 HAL UART、DMA 和 UART Register Callbacks（`USE_HAL_UART_REGISTER_CALLBACKS`）；RX DMA 必须为循环模式。宿主同时服务 UART 与 DMA IRQ。
- 按 UART 句柄将 RX 完成回调转发到实例的 `receiveCallback()`，将错误回调转发到 `errorHandler()`，并保留同句柄其他使用方所需的错误处理。注册完成后再调用 `startReceive()`，检查其布尔返回值。回调可能继续访问驱动和 Buffer，不能在接收仍活动时销毁或移除转发。

上述回调注册和接收行为以依赖包 `protocol::UartRxSync` 的接口及现有 Doxygen 为准；此处不另行定义其同步或错误恢复算法。
