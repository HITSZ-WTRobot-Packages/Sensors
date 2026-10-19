# DT35 转接板

包名：`SensorLaser::DT35`。依赖 `protocol::UartRxSync`。[仓库总览](../../README.md)

## 接入条件

[DT35.hpp](DT35.hpp) 提供 `sensors::laser::DT35Board` 和单通道 `sensors::laser::DT35`。UART 接收对象是转接板，不是单个模拟量传感器；板端协议及校正原理来源见 [WTR_DT35_HW](https://github.com/SSC202/WTR_DT35_HW)。

`DT35Board` 构造形式为 `DT35Board(UART_HandleTypeDef* huart, Buffer& buffer)`，`Buffer` 对应 `memory::DMABuffer<24>`，接收容量为 24 字节。例如，为板实例保留独立的静态接收存储，UART 句柄由宿主提供，不指定外设编号：

```cpp
#include "DT35.hpp"

extern UART_HandleTypeDef sensor_uart;

static sensors::laser::DT35Board::Buffer rx_buffer;
static sensors::laser::DT35Board board(&sensor_uart, rx_buffer);
```

此示例只展示构造和存储期；宿主必须先完成 HAL 初始化，再使用接收功能。宿主使用 CubeMX 配置循环 RX DMA、IRQ 和 UART Register Callbacks，按[UART 接收共性](../../README.md#uart-接收共性)转发回调后调用 `startReceive()` 并检查返回值。Buffer 必须 DMA 可达、满足缓存一致性并保持独占，直到全部接收和回调停止访问。

## 通道与校准边界

板支持索引 0～3 的通道绑定；单通道 `DT35` 用 `DT35::Config` 构造，提供近点/远点标定数据及校正系数，再由板实例注册。板借用通道对象指针，通道对象必须在注册和接收使用期间有效；注册失败不能当作绑定成功。校准点需满足近点距离和原始值均小于远点，对距离使用一致的标定单位，不假定固定量程或板级接线。

本包接收接口继承 `protocol::UartRxSync`，系列能力取决于宿主 HAL1、UART/DMA 与依赖包；无逐料号兼容承诺。具体接口与实现细节见 [DT35.hpp](DT35.hpp) 及现有源码说明。
