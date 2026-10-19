# XGZP6847D 压力传感器

包名：`SensorPressure::XGZP6847D`。依赖 `services::I2CUpdateManager`、`stm32cubemx`。[仓库总览](../../README.md)

## 集成与量程

[XGZP6847DDevice.hpp](XGZP6847DDevice.hpp) 提供全局命名空间中的 `XGZP6847DDevice`，由设备 manager 调度并通过 `I2CBusDMA` 访问；不是 UART 接收包，不使用本仓库 UART 包的 `Buffer&` 构造形式。宿主负责 CubeMX 的 I2C、DMA、GPIO、时钟与 IRQ 配置，并按依赖包契约管理总线、manager 和设备对象的生命周期。

构造参数为 `pressure_range_kpa` 和可选的 7 位 I2C 地址（默认 `0x6D`）。当前实现采用旧版协议的 K 表换算；`pressure_range_kpa` 必须取 `max(abs(Pmin), abs(Pmax))`，不是总量程跨度。例如 -100～100 kPa 填 100，-100～300 kPa 填 300。不能仅因器件名相同就假定新版协议或任意量程兼容。

## 采样边界

设备按触发转换、等待、读取的流程更新样本，调用方不能把一次 I2C 读等同于一次新转换完成。样本包含 Pa 压力、摄氏度温度、成功更新时间及有效位；消费缓存时应检查有效性和时间，不将初始值当作已测量值。

协议来源、量程换算、有效性和具体接口以 [XGZP6847DDevice.hpp](XGZP6847DDevice.hpp) 的现有 Doxygen 为准，包括其引用的[官方产品页](https://cfsensor.com/product/xgzp6847d/)和 [V3.0 datasheet](https://cfsensor.com/wp-content/uploads/2026/04/XGZP6847D-Pressure-Sensor-V3.0.pdf)。本包的适用范围还受宿主 HAL1 与 `services::I2CUpdateManager` 的 I2C DMA 能力约束，不是全 STM32 系列兼容声明。
