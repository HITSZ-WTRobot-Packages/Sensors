/**
 * @file    STP23L.hpp
 * @author  syhanjin
 * @date    2026-02-02
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#pragma once
#include "UartRxSync.hpp"
#include <cstddef>

namespace sensors::laser
{

class STP23L final : public protocol::UartRxSync<4, 195>
{
public:
    explicit STP23L(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

protected:
    const std::array<uint8_t, 4>& header() const override
    {
        return HEADER;
    }

    bool decode(const uint8_t data[191]) override;

private:
    static constexpr std::array<uint8_t, 4> HEADER = { 0xAA, 0xAA, 0xAA, 0xAA };

    constexpr static size_t PointSize = 15, PointNum = 12;

    struct LidarPoint
    {
        uint16_t distance;   ///< 距离 (mm)
        uint16_t noise;      ///< 环境噪声
        uint32_t peak;       ///< 接收信号强度
        uint8_t  confidence; ///< 置信度
        uint32_t intg;       ///< 积分次数
        uint16_t reftof;     ///< 温度表征值
    };

    uint32_t timestamp_{ 0 };
    float    distance_{ 0 };

public:
};

} // namespace sensors::laser
