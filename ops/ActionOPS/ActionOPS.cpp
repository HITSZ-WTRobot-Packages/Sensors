/**
 * @file    ActionOPS.cpp
 * @author  syhanjin
 * @date    2026-02-01
 */
#include "ActionOPS.hpp"

#include <cmath>

namespace sensors::ops
{
ActionOPS::ActionOPS(UART_HandleTypeDef* huart, const Config& cfg) :
    UartRxSync(huart), lock_(osMutexNew(nullptr)),
    setup_{ .x = cfg.x_offset, .y = cfg.y_offset, .yaw = cfg.yaw_offset }, gyro_yaw_(cfg.yaw_car)
{
}

bool ActionOPS::resetWorldCoord()
{
    if (huart() == nullptr || gyro_yaw_ == nullptr)
        return false;

    zeroClearing();
    gyro_offset_ = *gyro_yaw_;

    p_base_.x = setup_.x;
    p_base_.y = setup_.y;

    p_offset_.x = setup_.x;
    p_offset_.y = setup_.y;

    theta_offset_ = 0;
    R_base_.cos   = cosf(DEG2RAD(setup_.yaw));
    R_base_.sin   = sinf(DEG2RAD(setup_.yaw));

    return true;
}

bool ActionOPS::resetWorldCoordByPose(const Posture& posture)
{
    if (huart() == nullptr || gyro_yaw_ == nullptr)
        return false;

    const auto& [x, y, yaw] = posture;

    zeroClearing();
    gyro_offset_ = *gyro_yaw_;

    const float yaw_rad = DEG2RAD(yaw);
    const R     Rn      = {
                 .cos = cosf(yaw_rad),
                 .sin = sinf(yaw_rad),
    };

    p_base_.x = Rn.cos * setup_.x - Rn.sin * setup_.y;
    p_base_.y = Rn.sin * setup_.x + Rn.cos * setup_.y;

    p_offset_.x = p_base_.x + x;
    p_offset_.y = p_base_.y + y;

    theta_offset_            = yaw;
    const float base_yaw_rad = DEG2RAD(yaw + setup_.yaw);
    R_base_.cos              = cosf(base_yaw_rad);
    R_base_.sin              = sinf(base_yaw_rad);

    return true;
}

bool ActionOPS::decode(const uint8_t data[26])
{
    // check tail
    if (data[24] != 0x0A || data[25] != 0x0D)
        return false;

    float values[6];

    for (size_t i = 0; i < 6; i++)
        memcpy(&values[i], &data[4 * i], sizeof(float));

    // 解析24字节数据为6个float
    feedback_.zangle = values[0];
    feedback_.xangle = values[1];
    feedback_.yangle = values[2];
    feedback_.pos_x  = values[3] * 1e-3f;
    feedback_.pos_y  = values[4] * 1e-3f;
    feedback_.w_z    = values[5];

    transform();

    return true;
}

void ActionOPS::sendData(const uint8_t* data, const uint16_t len) const
{
    // 阻塞发送
    if (__get_IPSR() != 0)
        HAL_UART_Transmit(huart(), data, len, 100);
    else
    {
        osMutexAcquire(lock_, osWaitForever);
        HAL_UART_Transmit(huart(), data, len, 100);
        osMutexRelease(lock_);
    }
}

void ActionOPS::calibration() const
{
    constexpr uint8_t calib_cmd[4] = { 'A', 'C', 'T', 'R' };
    sendData(calib_cmd, sizeof(calib_cmd));
}

void ActionOPS::zeroClearing() const
{
    constexpr uint8_t zero_cmd[4] = { 'A', 'C', 'T', '0' }; // 清零命令
    sendData(zero_cmd, sizeof(zero_cmd));
}

void ActionOPS::updateYaw(const float angle) const
{
    uint8_t cmd[8] = "ACTJ";
    memcpy(cmd + 4, &angle, 4);
    sendData(cmd, sizeof(cmd));
}

void ActionOPS::updateX(const float posx) const
{
    uint8_t     cmd[8] = "ACTX";
    const float value  = posx * 1000.0f;
    memcpy(cmd + 4, &value, 4);
    sendData(cmd, sizeof(cmd));
}

void ActionOPS::updateY(const float posy) const
{
    uint8_t     cmd[8] = "ACTY";
    const float value  = posy * 1000.0f;
    memcpy(cmd + 4, &value, 4);
    sendData(cmd, sizeof(cmd));
}
void ActionOPS::updateXY(float posx, float posy) const
{
    uint8_t     cmd[12] = "ACTD";
    const float value1  = posx * 1000.0f;
    const float value2  = posy * 1000.0f;
    memcpy(cmd + 4, &value1, 4);
    memcpy(cmd + 8, &value2, 4);
    sendData(cmd, sizeof(cmd));
}

void ActionOPS::transform()
{
    // 认为 OPS_World 相对于 World 的位姿即为 OPS 相对于 Body 的位姿
    // OPS 在 OPS_World 中的位置
    const float xf    = feedback_.pos_x;
    const float yf    = feedback_.pos_y;
    const float yaw_f = *gyro_yaw_ - gyro_offset_;

    // 计算OPS相对于世界坐标系的实际偏航角（车体角+安装角偏移）
    const float yaw_f_rad = DEG2RAD(yaw_f);

    const R Rf = { .cos = cosf(yaw_f_rad), .sin = sinf(yaw_f_rad) };

    // 解算车体中心的世界坐标（Cx, Cy）
    body_x_ = p_offset_.x                                // p_offset
              + R_base_.cos * xf - R_base_.sin * yf      //  + R_base * p_ow
              - Rf.cos * p_base_.x + Rf.sin * p_base_.y; //  - R_ow * p_base

    body_y_ = p_offset_.y                                // p_offset
              + R_base_.sin * xf + R_base_.cos * yf      //+ R_base * p_ow
              - Rf.sin * p_base_.x - Rf.cos * p_base_.y; //  - R_ow * p_base

    body_yaw_ = yaw_f + theta_offset_;
}

} // namespace sensors::ops