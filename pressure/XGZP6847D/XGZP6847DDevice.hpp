/**
 * @file    XGZP6847DDevice.hpp
 * @brief   由 manager 调度、通过 I2CBusDMA 访问的 XGZP6847D 压力传感器
 */
#pragma once

#include <array>
#include <atomic>

#include "I2CBusDMA.hpp"

#include "I2CDevice.hpp"

/**
XGZP6847D 数字压力传感器驱动。

旧版协议/量程换算参考资料：
- 官方产品页: https://cfsensor.com/product/xgzp6847d/
- V3.0 datasheet: https://cfsensor.com/wp-content/uploads/2026/04/XGZP6847D-Pressure-Sensor-V3.0.pdf
- V2.8 检索页:
https://www.scribd.com/document/891923775/XGZP6847D-Pressure-Sensor-V2-2025-05-04-21-28-42

当前实现使用的是旧版协议里的 K 表换算：
pressure_pa = signExtend24(raw_pressure) / K

这里的 pressure_range_kpa 不是总量程跨度，而是 max(abs(Pmin), abs(Pmax))。
例如：
- 量程 `-100~100 kPa -> pressure_range_kpa = 100 -> K = 64`
- 量程 `-100~300 kPa -> pressure_range_kpa = 300 -> K = 16`

该设备需要先写命令触发一次联合转换，再等待芯片完成内部测量，最后读取
压力和温度原始值并更新缓存。因此它重写了 onTrigger()、conversionMs()
和 onRead() 三个状态机钩子。
**/
class XGZP6847DDevice final : public I2CDevice
{
public:
    /**
     * @brief 保存最近一次成功更新后的传感器样本
     */
    struct Sample
    {
        float    pressure_pa{ 0.0f };   ///< 最近一次采样得到的压力值，单位 Pa
        float    temperature_c{ 0.0f }; ///< 最近一次采样得到的温度值，单位摄氏度
        uint32_t timestamp_ms{ 0 };     ///< 最近一次成功更新时间戳
        bool     valid{ false };        ///< 当前样本是否有效
    };

    /**
     * @brief 构造压力传感器设备对象
     * @param pressure_range_kpa 旧版 K 表使用的等效量程，单位 kPa；
     *        取 max(abs(Pmin), abs(Pmax))，不是 Pmax - Pmin
     * @param address_7bit 设备的 7 位 I2C 地址
     */
    explicit XGZP6847DDevice(float pressure_range_kpa, uint8_t address_7bit = DefaultAddress);

    /**
     * @brief 获取设备名称
     * @return 固定返回设备名字符串
     */
    const char* name() const override
    {
        return "XGZP6847D";
    }
    /**
     * @brief 获取设备的 7 位 I2C 地址
     * @return 当前设备地址
     */
    uint8_t address7bit() const override
    {
        return address_;
    }

    /**
     * @brief 通过最小寄存器读操作确认设备在线
     * @param bus 当前设备所在的 I2C 总线
     * @param timeout_ms 单次事务超时时间，单位毫秒
     * @return 初始化探活是否成功
     */
    bool init(I2CBusDMA& bus, uint32_t timeout_ms) override;

    /**
     * @brief 返回当前缓存的样本快照
     * @return 当前缓存样本的拷贝
     */
    Sample snapshot() const;

    /**
     * @brief 获取当前缓存的压力值
     * @return 当前缓存压力，单位 Pa
     */
    float getPressure() const;

protected:
    /**
     * @brief 发送一次联合转换命令
     * @param bus 当前设备所在的 I2C 总线
     * @param timeout_ms 单次事务超时时间，单位毫秒
     * @return 触发采样是否成功
     */
    bool onTrigger(I2CBusDMA& bus, uint32_t timeout_ms) override;

    /**
     * @brief 获取芯片一次联合转换的最大等待时间
     * @return 转换时间上限，单位毫秒
     */
    uint32_t conversionMs() const override
    {
        return ConversionMs;
    }

    /**
     * @brief 读取原始数据并更新缓存
     * @param bus 当前设备所在的 I2C 总线
     * @param now_ms 当前时间戳，单位毫秒
     * @param timeout_ms 单次事务超时时间，单位毫秒
     * @return 读取并解析是否成功
     */
    bool onRead(I2CBusDMA& bus, uint32_t now_ms, uint32_t timeout_ms) override;

    /**
     * @brief 在父类判定数据失效时同步清理样本有效位
     */
    void onDataInvalidated() override;

private:
    static constexpr uint8_t  DefaultAddress = 0x6D; ///< 设备默认 7 位 I2C 地址
    static constexpr uint8_t  RegPressure    = 0x06; ///< 压力原始值起始寄存器地址
    static constexpr uint8_t  RegCmd         = 0x30; ///< 命令寄存器地址
    static constexpr uint8_t  CmdCombined    = 0x0A; ///< 压力和温度联合转换命令
    static constexpr uint32_t ConversionMs   = 30U;  ///< datasheet 给出的转换时间上限

    /**
     * @brief 根据量程计算压力换算系数
     * @param pressure_range_kpa 旧版 K 表使用的等效量程，单位 kPa；
     *        取 max(abs(Pmin), abs(Pmax))，不是 Pmax - Pmin
     * @return 对应的压力换算系数
     */
    static int32_t calcK(float pressure_range_kpa);

    /**
     * @brief 将 24 位补码原始压力扩展为 32 位有符号整数
     * @param data 指向 3 字节原始压力数据
     * @return 符号扩展后的 32 位压力原始值
     */
    static int32_t signExtend24(const uint8_t data[3]);

    uint8_t              address_{ DefaultAddress };   ///< 当前设备实际使用的 I2C 地址
    int32_t              k_{ 4096 };                   ///< 当前量程对应的压力换算系数
    std::array<Sample, 2> sample_buffers_{};           ///< 双缓冲样本缓存
    std::atomic<uint8_t> active_sample_index_{ 0U };   ///< 当前对外发布的样本缓冲索引
};
