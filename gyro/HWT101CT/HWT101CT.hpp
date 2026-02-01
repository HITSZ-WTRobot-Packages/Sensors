/**
 * @file    HWT101CT.hpp
 * @author  syhanjin
 * @date    2026-02-01
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#ifndef HWT101CT_HPP
#define HWT101CT_HPP
#include "UartRxSync.hpp"

namespace sensors::gyro
{

class HWT101CT final : public protocol::UartRxSync<1, 11>
{
public:
    /**
     * 模块输出速率枚举
     */
    enum class RRate : uint8_t
    {
        RRate_0_2HZ  = 0x01, ///< 0.2Hz
        RRate_0_5HZ  = 0x02, ///< 0.5Hz
        RRate_1HZ    = 0x03, ///< 1Hz
        RRate_2HZ    = 0x04, ///< 2Hz
        RRate_5HZ    = 0x05, ///< 5Hz
        RRate_10HZ   = 0x06, ///< 10Hz
        RRate_20HZ   = 0x07, ///< 20Hz
        RRate_50HZ   = 0x08, ///< 50Hz
        RRate_100HZ  = 0x09, ///< 100Hz
        RRate_200HZ  = 0x0B, ///< 200Hz
        RRate_500HZ  = 0x0C, ///< 500Hz
        RRate_1000Hz = 0x0D, ///< 1000Hz
    };

    explicit HWT101CT(UART_HandleTypeDef* huart) : UartRxSync(huart) {}

    /**
     * 重置 yaw
     *
     * @attention 本函数并不会立即设置 yaw 为 0，而是写入传感器寄存器，下一次接收到数据时才会重置。
     *            同时本函数带有大约 2ms 的阻塞
     */
    void resetYaw();

    /**
     * 校准陀螺仪零偏
     *
     * @attention 校准过程中，请确保传感器处于静止状态。故该函数为阻塞函数
     * @param duration_ms 零偏校准持续时间
     */
    void calibrate(uint32_t duration_ms);

    void setOutputRate(RRate rate);

    [[nodiscard]] const float& getYaw() const
    {
        return yaw_;
    }
    [[nodiscard]] const float& getWz() const
    {
        return wz_;
    }

protected:
    [[nodiscard]] const std::array<uint8_t, 1>& header() const override
    {
        static std::array<uint8_t, 1> header_{ 0x55 };
        return header_;
    };

    bool decode(const uint8_t data[10]) override;

private:
    float yaw_{ 0.0f };
    float wz_{ 0.0f };

    float   feedback_yaw_{ 0.0f };
    int32_t round_{ 0 };

    void write_reg(uint8_t addr, uint8_t data_l, uint8_t data_h);
};

} // namespace sensors::gyro

#endif // HWT101CT_HPP
