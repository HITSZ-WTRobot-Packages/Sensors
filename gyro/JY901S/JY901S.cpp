/**
 * @file    JY901S.cpp
 * @author  yli584364
 * @date    2026-03-09
 */
#include "JY901S.hpp"

#include "cmsis_os2.h"
#include "utils.h"

namespace sensor::gyro
{

static float get_feedback_dt(const JY901S::RRate rr)
{
    switch (rr)
    {
    case JY901S::RRate::R0_2Hz:
        return 5.0f;
    case JY901S::RRate::R0_5Hz:
        return 2.0f;
    case JY901S::RRate::R1Hz:
        return 1.0f;
    case JY901S::RRate::R2Hz:
        return 0.5f;
    case JY901S::RRate::R5Hz:
        return 0.2f;
    case JY901S::RRate::R10Hz:
        return 0.1f;
    case JY901S::RRate::R20Hz:
        return 0.05f;
    case JY901S::RRate::R50Hz:
        return 0.02f;
    case JY901S::RRate::R100Hz:
        return 0.01f;
    case JY901S::RRate::R200Hz:
        return 0.005f;
    case JY901S::RRate::ROnce:
    case JY901S::RRate::RNone:
    default:
        return std::numeric_limits<float>::infinity();
    }
}

static int16_t read_int16(const uint8_t data[2])
{
    return static_cast<int16_t>(data[1] << 8 | data[0]);
}

static constexpr float mapIntoRange(const int16_t input, const float max)
{
    return static_cast<float>(input) * max / 32768;
}

JY901S::JY901S(UART_HandleTypeDef* huart, const math::Posef& pose_in_body, const Config& config) :
    UartRxSync(huart), cfg_(config), pose_in_body_(pose_in_body),
    feedback_dt_(get_feedback_dt(cfg_.rrate))
{
}
JY901S::JY901S(UART_HandleTypeDef* huart, const math::Posef& pose_in_body) :
    UartRxSync(huart), cfg_({}), pose_in_body_(pose_in_body),
    feedback_dt_(get_feedback_dt(cfg_.rrate))
{
}

void JY901S::init()
{
    unlock();
    osDelay(100); // 为啥需要这么久，是录了吗
    sendCMD(Reg::RRate, static_cast<uint16_t>(cfg_.rrate));
    delay_us(250);
    sendCMD(Reg::RSW, cfg_.output);
    delay_us(250);
    unlock();
    osDelay(100);
    save();
}

void JY901S::calibrateAcc() const
{
    unlock();
    osDelay(200);
    sendCMD(Reg::CalSw, 0x0001);
    osDelay(4000);
    sendCMD(Reg::CalSw, 0x0000);
    osDelay(100);
    save();
}

bool JY901S::decode(const uint8_t data[10])
{
    // 校验和
    uint8_t sum = HEADER[0];
    for (size_t i = 0; i < 9; i++)
        sum += data[i];
    if (sum != data[9])
        return false;
    switch (data[0])
    {
    case 0x50:
        time_ms_ = read_int16(&data[7])  // ms
                   + 1000 * data[6]      // s
                   + 60000 * data[5]     // min
                   + 3600000 * data[4]   // hour
                   + 86400000 * data[3]; // day
        // 后面的数据溢出了，不用了
        feedbackTransform();
        break;
    case 0x51:
        acc_raw_ = { mapIntoRange(read_int16(&data[1]), MaxAccel),
                     mapIntoRange(read_int16(&data[3]), MaxAccel),
                     mapIntoRange(read_int16(&data[5]), MaxAccel) };

        temperature_ = static_cast<float>(read_int16(&data[7])) / 100.0f;

        feedback_ok_.acc = true;
        break;
    case 0x52:
        gyro_s_ = {
            mapIntoRange(read_int16(&data[1]), MaxGyro),
            mapIntoRange(read_int16(&data[3]), MaxGyro),
            mapIntoRange(read_int16(&data[5]), MaxGyro),
        };

        feedback_ok_.gyro = true;
        break;
    case 0x53:
        angles_w_ = {
            mapIntoRange(read_int16(&data[1]), MaxAngle),
            mapIntoRange(read_int16(&data[3]), MaxAngle),
            mapIntoRange(read_int16(&data[5]), MaxAngle),
        };
        if (!(cfg_.output & Output::Quat))
            quat_w_ = math::Quatf(angles_w_);

        feedback_ok_.quat = true;
        break;
    case 0x59:
        quat_w_ = {
            mapIntoRange(read_int16(&data[1]), MaxQuat),
            mapIntoRange(read_int16(&data[3]), MaxQuat),
            mapIntoRange(read_int16(&data[5]), MaxQuat),
            mapIntoRange(read_int16(&data[7]), MaxQuat),
        };
        if (!(cfg_.output & Output::Angle))
            angles_w_ = math::EulerDegf(quat_w_);

        feedback_ok_.quat = true;
    default:;
    }
    if (feedback_ok_.acc && feedback_ok_.gyro && feedback_ok_.quat)
    {
        feedback_ok_ = {};
        feedbackTransform();
        if (trig_ != nullptr)
            trig_(state_body_);
    }
    return true;
}

inline void JY901S::_send(const std::array<uint8_t, 5>& cmd) const
{
    HAL_UART_Transmit(huart(), cmd.data(), 5, 10);
}

void JY901S::unlock() const
{
    constexpr std::array<uint8_t, 5> UnlockCMD = { 0xFF, 0xAA, Reg::Key, 0x88, 0xB5 };
    _send(UnlockCMD);
}

void JY901S::save() const
{
    constexpr std::array<uint8_t, 5> SaveCMD = { 0xFF, 0xAA, Reg::Save, 0x00, 0x00 };
    _send(SaveCMD);
}

void JY901S::sendCMD(const uint8_t cmd, const uint16_t data) const
{
    const std::array<uint8_t, 5> _data = {
        0xFF, 0xAA, cmd, static_cast<uint8_t>(data), static_cast<uint8_t>(data >> 8)
    };
    _send(_data);
}

void JY901S::feedbackTransform()
{
    BodyState body;

    const auto& R = pose_in_body_.rot;

    // orientation
    body.quat_w   = quat_w_ * R.inverse();
    body.angles_w = math::EulerDegf(body.quat_w);

    // angular velocity
    body.gyro_b = R.rotateVector(gyro_s_);
    body.gyro_w = body.quat_w.rotateVector(body.gyro_b);

    // linear acceleration of imu in world (remove gravity)
    acc_w_ = quat_w_.rotateVector(acc_raw_) - vec_g;

    // imu position relative to body origin, expressed in world
    const auto r_bi_w = body.quat_w.rotateVector(pose_in_body_.pos);

    // angular acceleration in world
    const auto alpha_w = (body.gyro_b - state_body_.gyro_b) / feedback_dt_;

    body.acc_w = acc_w_ - alpha_w.cross(r_bi_w) - body.gyro_w.cross(body.gyro_w.cross(r_bi_w));

    body.tick = HAL_GetTick();

    state_body_ = body;
}

} // namespace sensor::gyro