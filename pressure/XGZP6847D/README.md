# XGZP6847D 压力传感器

包名：`SensorPressure::XGZP6847D`。依赖 `services::I2CUpdateManager`、`stm32cubemx`。[仓库总览](../../README.md)

## 集成与量程

[XGZP6847DDevice.hpp](XGZP6847DDevice.hpp) 提供全局命名空间中的 `XGZP6847DDevice`，由设备 manager 调度并通过 `I2CBusDMA` 访问；不是 UART 接收包，不使用本仓库 UART 包的 `Buffer&` 构造形式。

构造参数为 `pressure_range_kpa` 和可选的 7 位 I2C 地址（默认 `0x6D`）。当前实现采用旧版协议的 K 表换算；`pressure_range_kpa` 必须取 `max(abs(Pmin), abs(Pmax))`，不是总量程跨度。例如 -100～100 kPa 填 100，-100～300 kPa 填 300。不能仅因器件名相同就假定新版协议或任意量程兼容。

## 实际检查与宿主前提

- 本设备只按 I2C 读写返回值判断探活、触发与读取是否成功，并解析读回数据；不识别器件版本、不核对实际量程，也不校验宿主 CubeMX 硬件配置。
- `services::I2CUpdateManager` 检查注册时机、设备数量上限、非零周期与线程创建结果，按设备访问结果维护初始化和采样状态；它同样不校验 I2C、DMA、GPIO、时钟或 IRQ 配置。通信成功不等于配置、器件版本或量程已经过验证。
- 宿主须在启动调度前通过 CubeMX 完成 I2C、DMA、GPIO、时钟及 IRQ 配置和 HAL 初始化，并满足底层 I2C DMA 驱动的运行前提。总线由 manager 线程独占，不得由其他线程并发访问。
- manager、总线及设备对象必须覆盖后台访问和相关 DMA 回调的整个使用期。`stop()` 只是退出请求，不是线程 join，不能把其返回当作可以立即销毁这些对象的保证。

## 采样边界

设备按触发转换、等待、读取的流程更新样本，调用方不能把一次 I2C 读等同于一次新转换完成。样本包含 Pa 压力、摄氏度温度、成功更新时间及有效位；消费缓存时应检查有效性和时间，不将初始值当作已测量值。

## 协议资料与版本边界

旧版量程换算、有效性和具体接口以 [XGZP6847DDevice.hpp](XGZP6847DDevice.hpp) 的现有 Doxygen 为准。其中保留的 V3.0 历史外链已失效，本包暂无可用的旧版规范官方下载链接。厂商[官方产品页](https://cfsensor.com/product/xgzp6847d/)当前提供可访问的 [V3.2 datasheet](https://cfsensor.com/wp-content/uploads/2026/09/XGZP6847D-Pressure-Sensor-V3.2.pdf)；这是现行资料入口，不是本包旧版 K 表协议的等价依据，也不能据此声明驱动兼容新版器件。使用前须向厂商确认器件版次及对应的旧版规范。

本包的适用范围还受宿主 HAL1 与 `services::I2CUpdateManager` 的 I2C DMA 能力约束，不是全 STM32 系列兼容声明。
