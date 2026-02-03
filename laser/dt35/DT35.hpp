/**
 * @file    DT35.hpp
 * @author  syhanjin
 * @date    2026-02-03
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#ifndef DT35_HPP
#define DT35_HPP
#include "UartRxSync.hpp"

#include <cstdint>

namespace sensors::laser
{
class DT35
{
public:
    struct Config
    {
        struct
        {
            uint32_t raw_data;
            float    distance;
        } near, far; ///< 近点和远点的数据

        /**
         * 矫正系数
         * 相关原理参见 https://github.com/SSC202/WTR_DT35_HW
         */
        float k;
    };
    explicit DT35(const Config& cfg);

    [[nodiscard]] const float& getDistance() const
    {
        return distance_;
    }

    void setConfig(const Config& cfg);

private:
    void updateRawdata(const uint32_t& rawdata);
    void calcDistance();
    friend class DT35Board;

private:
    // distance = k_ * rawdata + b;
    float    k_, b_;
    uint32_t rawdata_{ 0 };
    float    distance_{ 0 };

    void applySetConfig(const Config& cfg);
};

class DT35Board final : public protocol::UartRxSync<2, 24>
{
public:
    explicit DT35Board(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

    bool  registerChannel(size_t i, DT35* dt35);
    DT35* unregisterChannel(size_t i);

protected:
    const std::array<uint8_t, 2>& header() const override
    {
        return HEADER;
    }

    bool decode(const uint8_t data[22]) override;

private:
    static constexpr std::array<uint8_t, 2> HEADER = { 0xAA, 0xBB };

    std::array<DT35*, 4> channel_{ nullptr };
};

} // namespace sensors::laser

#endif // DT35_HPP
