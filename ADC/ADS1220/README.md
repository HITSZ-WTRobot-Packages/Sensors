# ADS1220 使用指南

本文关注“怎样接入业务代码”。配置取值、硬件限制和接口细节请查阅 [ADS1220.hpp](ADS1220.hpp)，这里不重复列出。

## 接入工程

1. 将 `ADS1220.cpp` 加入 C++ 构建目标，让编译器可以找到本目录及 STM32 HAL 头文件。本工程会自动收集 `UserCode` 下的源文件，无需额外添加。
2. 先完成板级初始化，再调用示例的 `Init()`；SPI 句柄和 HAL 毫秒时基此时必须可用。接线与 SPI 配置按头文件中的硬件约束检查。
3. 业务任务周期性调用 `Poll()`，只在返回 `Ok` 时使用本次结果。

下面两个案例均可独立放入一个 `.cpp` 文件，假定 `UserCode` 已加入包含路径，且 `spi.h` 提供 `hspi1`。**同一颗 ADS1220 选择其中一个案例，不要同时用两个对象管理它。** 对象使用静态存储，不需要 `new`。

## 案例一：连续读取差分电压

适用于桥式传感器等小信号输入。这个案例按现有工程使用 **AIN0-AIN1、128 倍增益、外部 REFP0/REFN0 的 5 V 参考**；外部参考必须真实接入，不能只在代码中填写电压。

```cpp
#include "ADS1220/ADS1220.hpp"
#include "spi.h"

namespace differential_demo
{
ADS1220::Config MakeConfig()
{
    ADS1220::Config config{};
    config.input = ADS1220::InputMux::Ain0Ain1;
    config.gain = ADS1220::Gain::X128;
    config.data_rate = ADS1220::DataRate::Normal20;
    config.filter = ADS1220::FirFilter::Reject50And60;
    config.voltage_reference = ADS1220::VoltageReference::ExternalRef0;
    config.reference_voltage = 5.0f;
    return config;
}

static ADS1220 adc(&hspi1, MakeConfig());

ADS1220::Status Init()
{
    return adc.Init();
}

ADS1220::Status Poll(float& voltage_v)
{
    const auto status = adc.Update();
    if (status == ADS1220::Status::Ok)
    {
        voltage_v = adc.GetVoltage();
    }
    return status;
}
} // namespace differential_demo
```

业务调用顺序：启动阶段检查 `differential_demo::Init()` 是否成功；之后在任务或主循环中重复调用 `differential_demo::Poll(voltage_v)`。

这里输出的是**传感器输入端电压，单位 V**，业务换算不要再次除以 128。连续采样不需要反复调用 `Start()`，否则会不断重启当前转换。

## 案例二：按需触发一次单端测量

适用于偶尔测量一路电压的场景。这个案例使用 **AIN0 相对 AVSS、内部参考、1 倍增益**；测量信号及共模必须满足器件限制，不能直接接入任意幅值的电源。

```cpp
#include "ADS1220/ADS1220.hpp"
#include "spi.h"

namespace single_shot_demo
{
ADS1220::Config MakeConfig()
{
    ADS1220::Config config{};
    config.input = ADS1220::InputMux::Ain0Avss;
    config.gain = ADS1220::Gain::X1;
    config.bypass_pga = true;
    config.voltage_reference = ADS1220::VoltageReference::Internal2048;
    config.data_rate = ADS1220::DataRate::Normal20;
    config.conversion_mode = ADS1220::ConversionMode::SingleShot;
    return config;
}

static ADS1220 adc(&hspi1, MakeConfig());

ADS1220::Status Init()
{
    return adc.Init();
}

ADS1220::Status Request()
{
    return adc.Start();
}

ADS1220::Status Poll(float& voltage_v)
{
    const auto status = adc.Update();
    if (status == ADS1220::Status::Ok)
    {
        voltage_v = adc.GetVoltage();
    }
    return status;
}
} // namespace single_shot_demo
```

先检查 `single_shot_demo::Init()` 是否成功。以后每次需要测量时，只调用一次 `Request()`，检查启动成功后再周期性 `Poll()`；收到 `Ok` 即完成本次请求。

`Init()` 本身也会启动首个转换：可以直接轮询这次结果，也可以用 `Request()` 重新开始一次。读取完成后，只有下一次请求才会产生新的单次测量；不要在每次轮询前都调用 `Request()`。

## 放进业务任务时

- `Ok`：处理本次 `voltage_v`，例如换算压力、力矩或电压比例。
- `NotReady`：本轮没有新结果，继续做其他工作；示例不会覆盖传入的 `voltage_v`。
- 其他状态：进入工程已有的错误处理流程，不要把旧值当作新测量发布。

轮询可以比采样快，但不要用无限忙循环占满任务时间；FreeRTOS 中可在任务循环中适当 `osDelay()`。`Update()` 不等待 ADC 转换完成，**不代表整个库是异步的**：SPI 收发仍同步执行，初始化及必要的重新校准也会耗时，应放在允许阻塞的上下文中。

若业务需要“上电无载归零”，应在传感器稳定后平均多次成功读取的输入电压，作为业务零点保存；不要在首次成功 `Poll()` 前直接取缓存做零点，也不要把正常负载当成零点。
