/**
 * @file    HWT101CT.cpp
 * @author  syhanjin
 * @date    2026-02-01
 */
#include "HWT101CT.hpp"
#include "utils.h"

#ifdef USE_RTOS
#    include "cmsis_os2.h"
#endif

namespace sensors::gyro
{

static int16_t read_int16(const uint8_t data[2])
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[1]) << 8 | data[0]);
}

void HWT101CT::resetYaw()
{
    round_        = 0;
    feedback_yaw_ = 0.0f;
    write_reg(0x76, 0x00, 0x00);
}
void HWT101CT::calibrate(const uint32_t duration_ms)
{
    write_reg(0xA6, 0x01, 0x01);
#ifdef USE_RTOS
    osDelay(duration_ms);
#else
    HAL_Delay(duration_ms);
#endif

    write_reg(0xA6, 0x04, 0x00);
}

void HWT101CT::setOutputRate(const RRate rate)
{
    write_reg(0x03, static_cast<uint8_t>(rate), 0x00);
}

bool HWT101CT::decode(const uint8_t data[10])
{
    uint8_t sum = 0x55;
    for (uint8_t i = 0; i < 9; i++)
        sum += data[i];

    if (sum != data[9])
        return false;

    switch (data[0])
    {
    case 0x53:
    {
        const float new_yaw = read_int16(&data[5]) / 32768.0f * 180.0f;
        if (feedback_yaw_ > 90.0f && new_yaw < -90.0f)
            round_++;
        else if (feedback_yaw_ < -90.0f && new_yaw > 90.0f)
            round_--;
        yaw_          = new_yaw + static_cast<float>(round_) * 360.0f;
        feedback_yaw_ = new_yaw;
        break;
    }
    case 0x52:
        wz_ = (read_int16(&data[5])) / 32768.0f * 2000.0f;
        break;
    default:
        return false;
    }
    return true;
}

void HWT101CT::write_reg(const uint8_t addr, const uint8_t data_l, const uint8_t data_h)
{
    static const uint8_t unlock_cmd[5] = { 0xFF, 0xAA, 0x69, 0x88, 0xB5 };
    static const uint8_t save_cmd[5]   = { 0xFF, 0xAA, 0x00, 0x00, 0x00 };
    const uint8_t        cmd[5]        = { 0xFF, 0xAA, addr, data_l, data_h };
    HAL_UART_Transmit(huart(), unlock_cmd, 5, 20);
    delay_us(250);
    HAL_UART_Transmit(huart(), cmd, 5, 20);
    delay_us(250);
    HAL_UART_Transmit(huart(), save_cmd, 5, 20);
}
} // namespace sensors::gyro