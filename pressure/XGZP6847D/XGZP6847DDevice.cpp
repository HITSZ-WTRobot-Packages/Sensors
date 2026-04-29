/**
 * @file    XGZP6847DDevice.cpp
 * @brief   XGZP6847D 压力传感器实现
 */
#include "XGZP6847DDevice.hpp"

#include "FreeRTOS.h"
#include "../../../BasicComponents/bsp/i2c_driver/I2CBusDMA.hpp"
#include "task.h"

XGZP6847DDevice::XGZP6847DDevice(const float pressure_range_kpa, const uint8_t address_7bit) :
    address_(address_7bit)
{
    k_ = calcK(pressure_range_kpa);
}

bool XGZP6847DDevice::init(I2CBusDMA& bus, const uint32_t timeout_ms)
{
    uint8_t status = 0;
    // 读命令寄存器作为最小探活操作，不依赖设备处于某个特定测量状态。
    return bus.memRead(address_, RegCmd, &status, 1, timeout_ms);
}

bool XGZP6847DDevice::onTrigger(I2CBusDMA& bus, const uint32_t timeout_ms)
{
    const uint8_t cmd = CmdCombined;
    // 触发一次压力和温度的联合转换。
    return bus.memWrite(address_, RegCmd, &cmd, 1, timeout_ms);
}

bool XGZP6847DDevice::onRead(I2CBusDMA& bus, const uint32_t now_ms, const uint32_t timeout_ms)
{
    uint8_t raw[5]{};
    // 连续读取 3 字节压力和 2 字节温度原始值。
    if (!bus.memRead(address_, RegPressure, raw, 5, timeout_ms))
        return false;

    Sample next{};
    next.timestamp_ms  = now_ms;
    next.valid         = true;
    next.pressure_pa   = static_cast<float>(signExtend24(raw)) / static_cast<float>(k_);
    const int16_t temp = static_cast<int16_t>((static_cast<uint16_t>(raw[3]) << 8U) | raw[4]);
    next.temperature_c = static_cast<float>(temp) / 256.0f;

    // 对外暴露缓存快照，因此这里用临界区保证样本整体替换是原子的。
    taskENTER_CRITICAL();
    sample_ = next;
    taskEXIT_CRITICAL();
    return true;
}

void XGZP6847DDevice::onDataInvalidated()
{
    // 父类只维护抽象的 data_valid_ 语义；这里同步清理对外样本上的 valid 位，
    // 这样业务层只看 snapshot() 也不会误把旧样本当成新数据。
    taskENTER_CRITICAL();
    sample_.valid = false;
    taskEXIT_CRITICAL();
}

float XGZP6847DDevice::getPressure() const
{
    return snapshot().pressure_pa;
}

XGZP6847DDevice::Sample XGZP6847DDevice::snapshot() const
{
    // 采样任务会整体替换 sample_，这里也用同一临界区做整包拷贝，避免读到撕裂数据。
    taskENTER_CRITICAL();
    const Sample copy = sample_;
    taskEXIT_CRITICAL();
    return copy;
}

int32_t XGZP6847DDevice::calcK(const float pressure_range_kpa)
{
    // 旧版 datasheet 的 K 表按 max(abs(Pmin), abs(Pmax)) 选取。
    // 例如 -100~100 kPa 传 100，-100~300 kPa 传 300。
    if (pressure_range_kpa >= 1000.0f)
        return 4;
    if (pressure_range_kpa > 500.0f && pressure_range_kpa <= 1000.0f)
        return 8;
    if (pressure_range_kpa > 260.0f)
        return 16;
    if (pressure_range_kpa > 130.0f)
        return 32;
    if (pressure_range_kpa > 65.0f)
        return 64;
    if (pressure_range_kpa > 32.0f)
        return 128;
    if (pressure_range_kpa > 16.0f)
        return 256;
    if (pressure_range_kpa > 8.0f)
        return 512;
    if (pressure_range_kpa > 4.0f)
        return 1024;
    if (pressure_range_kpa > 2.0f)
        return 2048;
    if (pressure_range_kpa >= 1.0f)
        return 4096;
    return 8192;
}

int32_t XGZP6847DDevice::signExtend24(const uint8_t data[3])
{
    // 芯片输出的是 24 位补码，需要手动扩展符号位。
    int32_t value = (static_cast<int32_t>(data[0]) << 16) | (static_cast<int32_t>(data[1]) << 8) |
                    static_cast<int32_t>(data[2]);
    if ((value & 0x00800000) != 0)
        value -= 0x01000000;
    return value;
}
