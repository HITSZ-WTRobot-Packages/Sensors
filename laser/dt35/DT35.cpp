/**
 * @file    DT35.cpp
 * @author  syhanjin
 * @date    2026-02-03
 */
#include "DT35.hpp"

namespace sensors::laser
{
DT35::DT35(const Config& cfg)
{
    applySetConfig(cfg);
}

void DT35::calcDistance()
{
    distance_ = k_ * static_cast<float>(rawdata_) + b_;
}

void DT35::updateRawdata(const uint32_t& rawdata)
{
    rawdata_ = rawdata;
}

void DT35::setConfig(const Config& cfg)
{
    applySetConfig(cfg);
    calcDistance();
}

void DT35::applySetConfig(const Config& cfg)
{
    if (cfg.near.distance >= cfg.far.distance || cfg.near.raw_data >= cfg.far.raw_data)
        return;

    k_ = cfg.k * (cfg.far.distance - cfg.near.distance) /
         static_cast<float>(cfg.far.raw_data - cfg.near.raw_data);
    b_ = cfg.near.distance - k_ * static_cast<float>(cfg.near.raw_data);
}

bool DT35Board::registerChannel(const size_t i, DT35* dt35)
{
    if (i >= 4)
        return false;
    if (channel_[i] != nullptr)
        return false;
    channel_[i] = dt35;
    return true;
}

DT35* DT35Board::unregisterChannel(const size_t i)
{
    if (i >= 4)
        return nullptr;
    DT35* tmp   = channel_[i];
    channel_[i] = nullptr;
    return tmp;
}

static uint32_t be2u32(const uint8_t* bytes)
{
    return static_cast<uint32_t>(bytes[0]) << 24 | static_cast<uint32_t>(bytes[1]) << 16 |
           static_cast<uint32_t>(bytes[2]) << 8 | static_cast<uint32_t>(bytes[3]);
}

bool DT35Board::decode(const uint8_t data[22])
{
    // check tail
    if (data[20] != 0xCC || data[21] != 0xDD)
        return false;

    // check channel id
    for (size_t i = 0; i < 4; i++)
        if (i + 1 != data[i * 5])
            return false;
    for (size_t i = 0; i < 4; i++)
    {
        if (channel_[i] != nullptr)
        {
            channel_[i]->updateRawdata(be2u32(&data[i * 5 + 1]));
            channel_[i]->calcDistance();
        }
    }
    return true;
}
} // namespace sensors::laser