/**
 * @file    ActionOPS.hpp
 * @authors mogegee syhanjin
 * @date    2025-12-24 / 2026-02-01
 * @brief   ops全方位平面定位系统
 *
 * 简述本驱动解算逻辑：
 *
 * 坐标系定义：
 *    - W   世界坐标系：本驱动认为的世界坐标系，同时也是返回值的参考坐标系
 *    - B   车身坐标系：固定在车体上的坐标系，原点在底盘集合中心，车体正前方为 x，左侧为 y
 *    - OW  码盘世界坐标系：Action 的码盘认为的世界坐标系
 *    - O   码盘坐标系：Action 码盘手册所述的坐标系
 *    - Now 重定位坐标系：重定位时刻的 B
 * 符号定义：
 *    T_{X}_{Y} 为坐标系 X 到坐标系 Y 的齐次变换矩阵.
 *    R 为 T = [R p; 0 1] 中的旋转矩阵，p 为平移向量.
 *
 * 初始化时我们会传入 O 与 B 的相对位置，由于码盘是固定在车体的，所以该位置不变，
 * 定义变换矩阵 T0 = T_O_B 为 O 到 B 的变换矩阵。
 *
 * 现在解释重置坐标系的变换原理（ W = B ）：
 * 向码盘发送重置命令，此时 OW 变为 O，同时 W 重置为 B，于是有变换关系 T_OW_W = T0,
 * 码盘将会返回数据 P 为 O 在 OW 中的位置，由陀螺仪可以得出 O 在 OW 时的 yaw，
 * 即：反馈量为 O 到 OW 的变换 Tf = T_O_OW，目标量为 B 到 W 的变换 T = T_B_W.
 * 所以有 T = T_OW_W * Tf * T_O_B^(-1) = T0 * Tf * T0^(-1).
 * 另外 T = [ R p ;
 *           0 1 ], 我们只需要 p
 * 拆分可得 p = R0 * pf + (I - R0 * Rf * R0^(-1)) * p0;
 * 由于二维情况下 RA * RB = RB * RA，所以可以化简为 p = R0 * pf + (I - Rf) * p0;
 * 得到最终结果：
 *
 *          p = p0 + R0 * pf - Rf * p0;
 *
 * 现在解释以当前位置为反馈进行重定位解算的原理 （ T_B_W = T_Now_W = Tn ）：
 * 本情况与上面的区别在于 T_OW_W != T0, 而是满足 T_OW_W = T_Now_W * T_OW_Now，
 * 由于 T_OW_Now = T0，有 T_OW_W = Tn * T0;
 * 此时带入到上面过程有 T = Tn * T0 * Tf * T0^(-1);
 * 所以得到这种情况下结果就是对上面的 p 再做一次 Tn 变换，
 * 可以得到 p = Rn * R0 * pf + Rn * (I - R0 * Rf * R0^(-1)) * p0 + pn
 *          = (Rn * p0 + pn) + (Rn * R0) * pf - Rn * Rf * p0
 *          = (Rn * p0 + pn) + (Rn * R0) * pf - Rf * Rn * p0
 *          = p_offset + R_base * pf - Rf * p_base;
 * 其中： p_base     = Rn * p0,
 *       R_base     = Rn * R0 = R(theta_n + theta_0), theta_base = theta_n + theta_0.
 *       p_offset   = p_base + pn,
 *
 * 所以最后的更新计算函数可以统一为
 *
 *          p   = p_offset + R_base * pf - Rf * p_base;
 *          yaw = yaw_f + theta_n
 *
 */

#ifndef ACTIONOPS_HPP
#define ACTIONOPS_HPP

#include "UartRxSync.hpp"
#include "cmsis_os2.h"

#include <cstring>

#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180.0f)

namespace sensors::ops
{

class ActionOPS final : public protocol::UartRxSync<2, 28>
{
public:
    struct Config
    {
        float x_offset;   /// X轴偏移（mm，OPS在车体中心前方为正）
        float y_offset;   /// Y轴偏移（mm，OPS在车体中心左侧为正）
        float yaw_offset; /// 初始角度偏移（度，逆时针为正）

        const float* yaw_car; /// 车体中心偏航角（度，逆时针为正，由陀螺仪获得）
    };

    struct Posture
    {
        float x, y, yaw;
    };

    ActionOPS(UART_HandleTypeDef* huart, const Config& cfg);

    /**
     * @brief  世界坐标系重置函数（清零pos_x、pos_y和陀螺仪偏航角）
     */
    bool resetWorldCoord();
    /**
     * @brief  通过传入的位姿重设世界坐标系
     */
    bool resetWorldCoordByPose(const Posture& posture);

    [[nodiscard]] const float& getBodyX() const
    {
        return body_x_;
    }

    [[nodiscard]] const float& getBodyY() const
    {
        return body_y_;
    }

    [[nodiscard]] const float& getBodyYaw() const
    {
        return body_yaw_;
    }

protected:
    [[nodiscard]] const std::array<uint8_t, 2>& header() const override
    {
        static constexpr std::array<uint8_t, 2> HEADER = { 0x0D, 0x0A };
        return HEADER;
    }

    bool decode(const uint8_t data[26]) override;

private:
    /**
     * @brief  OPS串口发送数据
     * @attention 该函数为阻塞发送
     * @param  data: 发送数据指针
     * @param  len: 发送数据长度（字节）
     */
    void sendData(const uint8_t* data, const uint16_t len) const;

    /**
     * @brief  OPS校准命令（发送"ACTR"）
     * @attention 该函数一般情况下不会被使用
     */
    void calibration() const;

    /**
     * @brief  OPS清零命令（发送"ACT0"）
     * @note   清零后角度/坐标重置为0
     */
    void zeroClearing() const;

    /**
     * @brief  OPS更新航向角（发送"ACTJ"+4字节float）
     * @param  angle: 目标航向角（范围-180~180度，float类型）
     */
    void updateYaw(const float angle) const;

    /**
     * @brief  OPS更新X坐标（发送"ACTX"+4字节float）
     * @param  posx: 目标X坐标（单位m，float类型）
     */
    void updateX(const float posx) const;

    /**
     * @brief  OPS更新Y坐标（发送"ACTY"+4字节float）
     * @param  posy: 目标Y坐标（单位m，float类型）
     */
    void updateY(const float posy) const;

    /**
     * @brief  OPS更新XY坐标（发送"ACTD"+8字节float）
     * @param  posx: 目标X坐标（单位m，float类型）
     * @param  posy: 目标Y坐标（单位m，float类型）
     */
    void updateXY(float posx, float posy) const;

    /**
     * @brief  解算车体中心位姿
     */
    void transform();

private:
    struct R
    {
        float cos;
        float sin;
    };

    osMutexId_t lock_; // 发送锁
    struct
    {
        float pos_x;  // X坐标（单位：m）
        float pos_y;  // Y坐标（单位：m）
        float zangle; // Z轴角度（航向角，单位：度）
        float xangle; // X轴角度（单位：度）
        float yangle; // Y轴角度（单位：度）
        float w_z;    // Z轴角速度（单位：dps）
    } feedback_{ 0 };

    struct
    {
        float x;
        float y;
    } p_base_{ 0 }, p_offset_{ 0 };

    R R_base_{ 0 };

    float theta_offset_{ 0 };

    struct
    {
        float x;
        float y;
        float yaw;
    } setup_;

    const float* gyro_yaw_;         // 车体中心偏航角（度，逆时针为正，由陀螺仪获得）
    float        gyro_offset_{ 0 }; // Body yaw 零点

    float body_x_{ 0 };   // 车体相对世界坐标系位姿x（单位：m）
    float body_y_{ 0 };   // 车体相对世界坐标系位姿y（单位：m）
    float body_yaw_{ 0 }; // 车体相对世界坐标系位姿yaw（单位：度）
};

} // namespace sensors::ops

#endif // ACTIONOPS_HPP
