/**
 * @file    STP23L.cpp
 * @author  syhanjin
 * @date    2026-02-02
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "STP23L.hpp"

namespace sensors::laser
{
// 按小端序读取 uint16_t
static uint16_t read_u16(const uint8_t* p)
{
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

// 按小端序读取 uint32_t
static uint32_t read_u32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24));
}

bool STP23L::decode(const uint8_t data[191])
{
    const uint8_t* pData = data;
    // 从数据手册获得这是一个保留位，不鸟它
    UNUSED((*pData++ != 0x00));
    // 指令位
    if (*pData++ != 0x02)
    { // 不是获取距离的数据包，忽略
        return true;
    }
    // offset
    const uint16_t offset = read_u16(pData);
    pData += 2;
    if (offset != 0x0000)
    { // offset 不为 0，错误
        return false;
    }
    const uint16_t data_len = read_u16(pData);
    pData += 2;
    if (data_len != PointNum * PointSize + 4)
    {
        return false;
    }
    uint16_t conf_sum = 0;
    float    distance = 0.0f;
    for (size_t i = 0; i < PointNum; i++)
    {
        const uint16_t point_distance   = read_u16(pData + 0);
        const uint16_t point_noise      = read_u16(pData + 2);
        const uint32_t point_peak       = read_u32(pData + 4);
        const uint8_t  point_confidence = pData[8];
        const uint32_t point_intg       = read_u32(pData + 9);
        const uint16_t point_reftof     = read_u16(pData + 13);
        // 对 distance 基于权重 confidence 做加权平均
        distance += static_cast<float>(point_confidence) * static_cast<float>(point_distance);
        conf_sum += static_cast<uint16_t>(point_confidence);
        // 未使用的参数
        UNUSED(point_noise);
        UNUSED(point_peak);
        UNUSED(point_intg);
        UNUSED(point_reftof);
        pData += PointSize;
    }
    distance /= static_cast<float>(conf_sum);
    const uint32_t timestamp = read_u32(pData);
    pData += 4;
    uint8_t sum = 0;
    for (size_t i = 0; i < 190; i++)
        sum += data[i];
    if (sum != *pData)
    { // 校验和校验不通过
        return false;
    }
    // 保存数据
    distance_  = distance;
    timestamp_ = timestamp;

    return true;
}
} // namespace sensors::laser