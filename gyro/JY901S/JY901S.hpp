/**
 * @file    JY901S.hpp
 * @author  yli584364
 * @date    2026-03-09
 */
#pragma once
#include "Euler.hpp"
#include "Pose.hpp"
#include "UartRxSync.hpp"
#include "Vec.hpp"
#include "Quaternion.hpp"

namespace sensor::gyro
{

class JY901S : public protocol::UartRxSync<1, 11>
{
public:
    struct State
    {
        math::Vec3f     acc_w;
        math::Vec3f     gyro_w;
        math::Vec3f     gyro_b;
        math::Quatf     quat_w;
        math::EulerDegf angles_w;
    };

    /**
     * @document https://wit-motion.yuque.com/wumwnr/ltst03/vl3tpy#S6al6
     */
    using OutputType = uint16_t;
    struct Output
    {
        static constexpr OutputType None     = 0;
        static constexpr OutputType Time     = 1u;
        static constexpr OutputType Acc      = 2u;
        static constexpr OutputType Gyro     = 4u;
        static constexpr OutputType Angle    = 8u;
        static constexpr OutputType Mag      = 16u;
        static constexpr OutputType Port     = 32u;
        static constexpr OutputType Press    = 64u;
        static constexpr OutputType Gps      = 128u;
        static constexpr OutputType Velocity = 256u;
        static constexpr OutputType Quat     = 512u;
        static constexpr OutputType GSA      = 1024u;
    };
    static constexpr OutputType DefaultOutputType = Output::Time | Output::Acc | Output::Gyro |
                                                    Output::Quat;
    enum class RRate : uint8_t
    {
        R0_2Hz = 0x01,
        R0_5Hz = 0x02,
        R1Hz   = 0x03,
        R2Hz   = 0x04,
        R5Hz   = 0x05,
        R10Hz  = 0x06,
        R20Hz  = 0x07,
        R50Hz  = 0x08,
        R100Hz = 0x09,
        R200Hz = 0x0B,
        ROnce  = 0x0C,
        RNone  = 0x0D,
    };
    enum class AccRange : uint8_t
    {
        g2  = 0x00,
        g4  = 0x01,
        g8  = 0x02,
        g16 = 0x03,
    };

    /** 自己在上位机改
     * enum class Baud : uint8_t
     * {
     *     B4800   = 0x01,
     *     B9600   = 0x02,
     *     B19200  = 0x03,
     *     B38400  = 0x04,
     *     B57600  = 0x05,
     *     B115200 = 0x06,
     *     B230400 = 0x07,
     *     B460800 = 0x08,
     *     B500000 = 0x09,
     * };
     */

    struct Config
    {
        OutputType output = DefaultOutputType;
        RRate      rrate  = RRate::R200Hz;
    };

    JY901S(UART_HandleTypeDef* huart, const math::Posef& pose_in_body, const Config& config);
    JY901S(UART_HandleTypeDef* huart, const math::Posef& pose_in_body);

    void init();

    void calibrateAcc() const;

protected:
    static constexpr std::array<uint8_t, 1> HEADER = { 0x55 };

    [[nodiscard]] const std::array<uint8_t, 1>& header() const override { return HEADER; }

    bool decode(const uint8_t data[10]) override;

private:
    /**
     * 解释一下单位：
     * 1. 加速度单位 m/s^2，陀螺仪传输单位为 g
     * 2. 角速度单位 rad/s，方便计算
     * 3. 四元数为单位四元数
     * 4. 欧拉角为角度，仅用于显示，不用于计算。故采用易读取的方式
     */
    static constexpr float g        = 9.8f;
    static constexpr float MaxAccel = 16 * g;
    static constexpr float MaxGyro  = 2000.0f / 180.0f * M_PI;
    static constexpr float MaxAngle = 180.0f;
    static constexpr float MaxQuat  = 1.0f;

    static constexpr math::Vec3f vec_g = { 0, 0, g };

    struct Reg
    {
        static constexpr uint8_t Save         = 0x00; ///< 保存/重启/恢复出厂
        static constexpr uint8_t CalSw        = 0x01; ///< 校准模式
        static constexpr uint8_t RSW          = 0x02; ///< 输出内容
        static constexpr uint8_t RRate        = 0x03; ///< 输出速率
        static constexpr uint8_t Baud         = 0x04; ///< 串口波特率
        static constexpr uint8_t AxOffset     = 0x05; ///< 加速度X零偏
        static constexpr uint8_t AyOffset     = 0x06; ///< 加速度Y零偏
        static constexpr uint8_t AzOffset     = 0x07; ///< 加速度Z零偏
        static constexpr uint8_t GxOffset     = 0x08; ///< 角速度X零偏
        static constexpr uint8_t GyOffset     = 0x09; ///< 角速度Y零偏
        static constexpr uint8_t GzOffset     = 0x0A; ///< 角速度Z零偏
        static constexpr uint8_t HxOffset     = 0x0B; ///< 磁场X零偏
        static constexpr uint8_t HyOffset     = 0x0C; ///< 磁场Y零偏
        static constexpr uint8_t HzOffset     = 0x0D; ///< 磁场Z零偏
        static constexpr uint8_t D0Mode       = 0x0E; ///< D0引脚模式
        static constexpr uint8_t D1Mode       = 0x0F; ///< D1引脚模式
        static constexpr uint8_t D2Mode       = 0x10; ///< D2引脚模式
        static constexpr uint8_t D3Mode       = 0x11; ///< D3引脚模式
        static constexpr uint8_t IicAddr      = 0x1A; ///< IIC设备地址
        static constexpr uint8_t LedOff       = 0x1B; ///< 关闭LED灯
        static constexpr uint8_t MagRangeX    = 0x1C; ///< 磁场X校准范围
        static constexpr uint8_t MagRangeY    = 0x1D; ///< 磁场Y校准范围
        static constexpr uint8_t MagRangeZ    = 0x1E; ///< 磁场Z校准范围
        static constexpr uint8_t Bandwidth    = 0x1F; ///< 带宽
        static constexpr uint8_t GyroRange    = 0x20; ///< 陀螺仪量程
        static constexpr uint8_t AccRange     = 0x21; ///< 加速度量程
        static constexpr uint8_t Sleep        = 0x22; ///< 休眠
        static constexpr uint8_t Orient       = 0x23; ///< 安装方向
        static constexpr uint8_t Axis6        = 0x24; ///< 算法
        static constexpr uint8_t FiltK        = 0x25; ///< 动态滤波
        static constexpr uint8_t GpsBaud      = 0x26; ///< GPS波特率
        static constexpr uint8_t ReadAddr     = 0x27; ///< 读取寄存器
        static constexpr uint8_t AccFilt      = 0x2A; ///< 加速度滤波
        static constexpr uint8_t PowerOnSend  = 0x2D; ///< 指令启动
        static constexpr uint8_t Version      = 0x2E; ///< 版本号
        static constexpr uint8_t YyMm         = 0x30; ///< 年月
        static constexpr uint8_t DdHh         = 0x31; ///< 时日
        static constexpr uint8_t MmSs         = 0x32; ///< 分秒
        static constexpr uint8_t MsL          = 0x33; ///< 毫秒
        static constexpr uint8_t Ax           = 0x34; ///< 加速度X
        static constexpr uint8_t Ay           = 0x35; ///< 加速度Y
        static constexpr uint8_t Az           = 0x36; ///< 加速度Z
        static constexpr uint8_t Gx           = 0x37; ///< 角速度X
        static constexpr uint8_t Gy           = 0x38; ///< 角速度Y
        static constexpr uint8_t Gz           = 0x39; ///< 角速度Z
        static constexpr uint8_t Hx           = 0x3A; ///< 磁场X
        static constexpr uint8_t Hy           = 0x3B; ///< 磁场Y
        static constexpr uint8_t Hz           = 0x3C; ///< 磁场Z
        static constexpr uint8_t Roll         = 0x3D; ///< 横滚角
        static constexpr uint8_t Pitch        = 0x3E; ///< 俯仰
        static constexpr uint8_t Yaw          = 0x3F; ///< 航向角
        static constexpr uint8_t Temp         = 0x40; ///< 温度
        static constexpr uint8_t D0Status     = 0x41; ///< D0引脚状态
        static constexpr uint8_t D1Status     = 0x42; ///< D1引脚状态
        static constexpr uint8_t D2Status     = 0x43; ///< D2引脚状态
        static constexpr uint8_t D3Status     = 0x44; ///< D3引脚状态
        static constexpr uint8_t PressureL    = 0x45; ///< 气压低16位
        static constexpr uint8_t PressureH    = 0x46; ///< 气压高16位
        static constexpr uint8_t HeightL      = 0x47; ///< 高度低16位
        static constexpr uint8_t HeightH      = 0x48; ///< 高度高16位
        static constexpr uint8_t LonL         = 0x49; ///< 经度低16位
        static constexpr uint8_t LonH         = 0x4A; ///< 经度高16位
        static constexpr uint8_t LatL         = 0x4B; ///< 纬度低16位
        static constexpr uint8_t LatH         = 0x4C; ///< 纬度高16位
        static constexpr uint8_t GpsHeight    = 0x4D; ///< GPS海拔
        static constexpr uint8_t GpsYaw       = 0x4E; ///< GPS航向角
        static constexpr uint8_t GpsVL        = 0x4F; ///< GPS地速低16位
        static constexpr uint8_t GpsVH        = 0x50; ///< GPS地速高16位
        static constexpr uint8_t Q0           = 0x51; ///< 四元数0
        static constexpr uint8_t Q1           = 0x52; ///< 四元数1
        static constexpr uint8_t Q2           = 0x53; ///< 四元数2
        static constexpr uint8_t Q3           = 0x54; ///< 四元数3
        static constexpr uint8_t SvNum        = 0x55; ///< 卫星数
        static constexpr uint8_t Pdop         = 0x56; ///< 位置精度
        static constexpr uint8_t Hdop         = 0x57; ///< 水平精度
        static constexpr uint8_t Vdop         = 0x58; ///< 垂直精度
        static constexpr uint8_t DelayT       = 0x59; ///< 报警信号延迟
        static constexpr uint8_t XMin         = 0x5A; ///< X轴角度报警最小值
        static constexpr uint8_t XMax         = 0x5B; ///< X轴角度报警最大值
        static constexpr uint8_t BatVal       = 0x5C; ///< 供电电压
        static constexpr uint8_t AlarmPin     = 0x5D; ///< 报警引脚映射
        static constexpr uint8_t YMin         = 0x5E; ///< Y轴角度报警最小值
        static constexpr uint8_t YMax         = 0x5F; ///< Y轴角度报警最大值
        static constexpr uint8_t GyroCaliThr  = 0x61; ///< 陀螺仪静止阈值
        static constexpr uint8_t AlarmLevel   = 0x62; ///< 角度报警电平
        static constexpr uint8_t GyroCaliTime = 0x63; ///< 陀螺仪自动校准时间
        static constexpr uint8_t TrigTime     = 0x68; ///< 报警连续触发时间
        static constexpr uint8_t Key          = 0x69; ///< 解锁
        static constexpr uint8_t WError       = 0x6A; ///< 陀螺仪变化值
        static constexpr uint8_t TimeZone     = 0x6B; ///< GPS时区
        static constexpr uint8_t WzTime       = 0x6E; ///< 角速度连续静止时间
        static constexpr uint8_t WzStatic     = 0x6F; ///< 角速度积分阈值
        static constexpr uint8_t ModDelay     = 0x74; ///< 485数据应答延时
        static constexpr uint8_t XRefRoll     = 0x79; ///< 横滚角零位参考值
        static constexpr uint8_t YRefRoll     = 0x7A; ///< 俯仰角零位参考值
        static constexpr uint8_t NumberId1    = 0x7F; ///< 设备编号1-2
        static constexpr uint8_t NumberId2    = 0x80; ///< 设备编号3-4
        static constexpr uint8_t NumberId3    = 0x81; ///< 设备编号5-6
        static constexpr uint8_t NumberId4    = 0x82; ///< 设备编号7-8
        static constexpr uint8_t NumberId5    = 0x83; ///< 设备编号9-10
        static constexpr uint8_t NumberId6    = 0x84; ///< 设备编号11-12
    };

    Config cfg_;

    math::Posef pose_in_body_;

    uint32_t time_ms_{}; // 时间
    float    feedback_dt_;

    /**
     * 陀螺仪相对于自身的加速度，角速度和在世界中的姿态
     */
    math::Quatf     quat_w;
    math::Vec3f     acc_raw;
    math::Vec3f     acc_w;
    math::Vec3f     gyro_s;
    math::EulerDegf angles_w;

    struct
    {
        bool quat, acc, gyro;
    } feedback_ok_{};

    /**
     * body 在世界中的加速度，角速度，姿态
     */
    State state_body_;

    float temperature_{};

    void _send(const std::array<uint8_t, 5>& cmd) const;
    void unlock() const;
    void save() const;
    void sendCMD(uint8_t cmd, uint16_t data) const;

    void feedbackTransform();
};

} // namespace sensor::gyro