# ActionOPS 平面定位

包名：`SensorOPS::ActionOPS`。依赖 `protocol::UartRxSync`、`FreeRTOS`。[仓库总览](../../README.md)

## 接入条件

[ActionOPS.hpp](ActionOPS.hpp) 提供 `sensors::ops::ActionOPS`。构造形式为 `ActionOPS(UART_HandleTypeDef* huart, Buffer& buffer, const Config& cfg)`；`Buffer` 对应 `memory::DMABuffer<28>`，接收容量为 28 字节。调用方必须先创建独立 Buffer，保持有效直到接收和相关回调全部停止访问。

宿主以 CubeMX 配置 UART、循环 RX DMA、IRQ 及 UART Register Callbacks，按[UART 接收共性](../../README.md#uart-接收共性)注册 RX 完成和错误回调，再调用 `startReceive()` 并检查返回值。DMA 可达性和缓存一致性由宿主负责，不仅是满足对齐。

## 安装与运行边界

- 配置中的安装偏移 `x_offset`、`y_offset` 使用 mm，车体前方/左侧为正；`yaw_offset` 和借用的车体航向角 `yaw_car` 使用度，逆时针为正。由宿主提供实际安装数据，不固定车体尺寸或实例。
- `yaw_car` 指向外部航向角存储，不能指向临时值；须在驱动解算和重定位使用期间保持有效，并由宿主管理更新与访问上下文。
- 车体位置反馈使用 m，航向反馈使用度。启动时应建立所需世界坐标系，并检查重定位操作返回值；构造本身不等于完成世界坐标系初始化。
- 驱动使用 CMSIS-RTOS 互斥量和阻塞 UART 发送，宿主须提供相应 RTOS 环境并按任务生命周期安排初始化与命令操作。这些运行期命令不是宿主 UART 静态配置。

坐标系定义、重定位语义及具体接口以 [ActionOPS.hpp](ActionOPS.hpp) 的现有 Doxygen 为准，不复制其变换推导。UART/DMA 支持范围取决于 `protocol::UartRxSync` 与宿主 HAL1，不承诺所有共享目标系列均直接适用。
