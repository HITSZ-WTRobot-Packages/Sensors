/**
 * @file    ADS1220.cpp
 * @author  Wleaf.
 * @date    2026-09-18
 */

#include "ADS1220.hpp"

// 完整配置构造方式同样只保存参数，不在构造函数中访问尚未就绪的硬件。

ADS1220::ADS1220(SPI_HandleTypeDef* hspi, const Config& config) : hspi_(hspi), Config_(config) {}

ADS1220::Status ADS1220::Init()
{
    // 初始化顺序固定为：参数校验 -> 芯片复位 -> 写寄存器 -> 读回校验 -> 启动转换。
    // 任一步失败都会立即返回，避免业务层使用部分初始化的对象。

    if (hspi_ == nullptr)
    {
        LastStatus_ = Status::InvalidArgument;
        return LastStatus_;
    }

    LastStatus_ = ValidateConfig(Config_);
    if (LastStatus_ != Status::Ok)
    {
        return LastStatus_;
    }

    // RESET 会恢复 ADS1220 的寄存器默认值，并将 Initialized_ 清零。

    LastStatus_ = Reset();
    if (LastStatus_ != Status::Ok)
    {
        return LastStatus_;
    }

    // ApplyConfig() 只有在四个寄存器全部写入且读回一致后才返回成功。

    LastStatus_ = ApplyConfig();
    if (LastStatus_ == Status::Ok)
    {
        Initialized_ = true;
    }
    return LastStatus_;
}

ADS1220::Status ADS1220::Update()
{
    // RDATA 返回芯片内部缓存的最近一次转换值；这里不轮询 DRDY。
    // SPI 失败时保留上一次有效的 RawValue_、Voltage_ 和 Temperature_。

    if (!Initialized_)
    {
        LastStatus_ = Status::NotInitialized;
        return LastStatus_;
    }

    // 先发送 RDATA 命令，再额外产生 24 个 SCLK 读取 MSB、MID、LSB。

    LastStatus_ = SendCommand(Cmd::RDATA);
    if (LastStatus_ != Status::Ok)
    {
        return LastStatus_;
    }

    uint8_t data[3] = {};
    LastStatus_     = ReadData(data, 3);
    if (LastStatus_ != Status::Ok)
    {
        return LastStatus_;
    }

    // 只有完整收到三个字节后才更新业务层可见的测量缓存。

    DecodeValue(data);
    return LastStatus_;
}

ADS1220::Status ADS1220::Configure(const Config& config)
{
    // 先验证候选配置，防止无效组合覆盖当前仍可使用的 Config_。

    LastStatus_ = ValidateConfig(config);
    if (LastStatus_ != Status::Ok)
    {
        return LastStatus_;
    }

    // 未初始化时仅缓存配置，便于构造后、Init() 前分阶段设置参数。

    if (!Initialized_)
    {
        LastStatus_ = Status::NotInitialized;
        return LastStatus_;
    }

    // 运行期间重新配置会重写全部寄存器；ApplyConfig() 最后重新发送 START。
    Config_     = config;
    LastStatus_ = ApplyConfig();
    // Todo:这里如果重新配置失败软硬件的config会不匹配，应检查配置正常后在修改config

    return LastStatus_;
}

ADS1220::Status ADS1220::Start()
{
    if (!Initialized_)
    {
        LastStatus_ = Status::NotInitialized;
        return LastStatus_;
    }

    LastStatus_ = SendCommand(Cmd::START);
    return LastStatus_;
}

ADS1220::Status ADS1220::PowerDown()
{
    if (!Initialized_)
    {
        LastStatus_ = Status::NotInitialized;
        return LastStatus_;
    }

    LastStatus_ = SendCommand(Cmd::POWERDOWN);
    return LastStatus_;
}

ADS1220::Status ADS1220::Reset()
{
    if (hspi_ == nullptr)
    {
        LastStatus_ = Status::InvalidArgument;
        return LastStatus_;
    }

    // 数据手册要求 RESET 后至少等待 50 us + 32 个系统时钟周期；1 ms 留有充分裕量。

    LastStatus_ = SendCommand(Cmd::RESET);
    if (LastStatus_ == Status::Ok)
    {
        HAL_Delay(1);
        Initialized_ = false;
    }
    return LastStatus_;
}

ADS1220::Status ADS1220::SetGain(Gain gain)
{
    Config cfg = Config_;
    cfg.gain   = gain;
    return Configure(cfg);
}

ADS1220::Status ADS1220::SetVref(float Vref)
{
    if (Vref <= 0.0f)
    {
        LastStatus_ = Status::InvalidArgument;
        return LastStatus_;
    }

    // reference_voltage 只参与数字码到电压的换算，因此无需重写硬件寄存器。
    Config_.reference_voltage = Vref;
    LastStatus_               = Status::Ok;
    return LastStatus_;
}

ADS1220::Status ADS1220::SendData(uint8_t* pData, uint8_t size)
{
    return FromHalStatus(HAL_SPI_Transmit(hspi_, pData, size, 10));
}

ADS1220::Status ADS1220::ReadData(uint8_t* pData, uint8_t size)
{
    return FromHalStatus(HAL_SPI_Receive(hspi_, pData, size, 100));
}

ADS1220::Status ADS1220::SendCommand(Cmd cmd)
{
    uint8_t command = static_cast<uint8_t>(cmd);
    return FromHalStatus(HAL_SPI_Transmit(hspi_, &command, 1, 100));
}

ADS1220::Status ADS1220::WriteRegisters(const uint8_t registers[4])
{
    // WREG 命令低两位 nn = 3，表示从寄存器 0 起连续写入 nn + 1 = 4 个字节。
    // 默认一次写入四个寄存器

    uint8_t message[5] = {
        static_cast<uint8_t>(Cmd::WREG), registers[0], registers[1], registers[2], registers[3],
    };
    return SendData(message, 5);
}

ADS1220::Status ADS1220::ReadRegisters(uint8_t registers[4])
{
    // RREG 命令低两位 nn = 3，表示从寄存器 0 起连续读出全部四个配置字节。
    // 默认一次读取四个配置寄存器

    Status status = SendCommand(Cmd::RREG);
    if (status != Status::Ok)
    {
        return status;
    }
    return ReadData(registers, 4);
}

ADS1220::Status ADS1220::ApplyConfig()
{
    // expected 是 Config_ 的唯一寄存器表示；业务层不会接触这些原始字节。

    uint8_t expected[4] = {};
    BuildRegisters(expected);

    Status status = WriteRegisters(expected);
    if (status != Status::Ok)
    {
        return status;
    }

    uint8_t actual[4] = {};
    status            = ReadRegisters(actual);
    if (status != Status::Ok)
    {
        return status;
    }

    // 逐字节读回校验能够发现 SPI 链路异常或芯片未接受配置。

    for (uint8_t index = 0; index < 4; ++index)
    {
        if (actual[index] != expected[index])
        {
            return Status::ConfigurationMismatch;
        }
    }

    return SendCommand(Cmd::START);
}

ADS1220::Status ADS1220::ValidateConfig(const Config& config) const
{
    // public 枚举仍可能被 static_cast 构造出非法值，因此不能只依赖枚举类型安全。

    const uint8_t input      = static_cast<uint8_t>(config.input);
    const uint8_t gain       = static_cast<uint8_t>(config.gain);
    const uint8_t rate       = static_cast<uint8_t>(config.data_rate);
    const uint8_t mode       = static_cast<uint8_t>((rate >> 3U) & 0x03U);
    const uint8_t dr         = static_cast<uint8_t>((rate >> 5U) & 0x07U);
    const uint8_t conversion = static_cast<uint8_t>(config.conversion_mode);
    const uint8_t reference  = static_cast<uint8_t>(config.voltage_reference);
    const uint8_t filter     = static_cast<uint8_t>(config.filter);
    const uint8_t current    = static_cast<uint8_t>(config.idac_current);
    const uint8_t route1     = static_cast<uint8_t>(config.idac1_route);
    const uint8_t route2     = static_cast<uint8_t>(config.idac2_route);

    // 检验是否符合手册
    if ((input > 0xE0U) || ((input & 0x0FU) != 0U) || (gain > 7U) || ((rate & 0x07U) != 0U) ||
        (mode > 2U) || (dr > 6U) || ((conversion != 0x00U) && (conversion != 0x04U)) ||
        (reference > 0xC0U) || ((reference & 0x3FU) != 0U) || (filter > 0x30U) ||
        ((filter & 0x0FU) != 0U) || (current > 7U) || (route1 > 6U) || (route2 > 6U) ||
        (config.reference_voltage <= 0.0f))
    {
        return Status::InvalidArgument;
    }

    // 数据手册规定：AINx-AVSS 单端输入必须旁路 PGA，且旁路时最大只能使用 4 倍增益。

    const bool single_ended = (input >= 0x80U) && (input <= 0xB0U);
    if ((config.bypass_pga && gain > static_cast<uint8_t>(Gain::X4)) ||
        (single_ended && (!config.bypass_pga || gain > static_cast<uint8_t>(Gain::X4))))
    {
        return Status::InvalidArgument;
    }

    // FIR 工频抑制只在正常模式 20 SPS 或占空比模式 5 SPS 下有效。

    if ((config.filter != FirFilter::None) && (rate != static_cast<uint8_t>(DataRate::Normal20)) &&
        (rate != static_cast<uint8_t>(DataRate::Duty5)))
    {
        return Status::InvalidArgument;
    }

    return Status::Ok;
}

ADS1220::Status ADS1220::FromHalStatus(HAL_StatusTypeDef status) const
{
    switch (status)
    {
    case HAL_OK:
        return Status::Ok;
    case HAL_BUSY:
        return Status::SpiBusy;
    case HAL_TIMEOUT:
        return Status::SpiTimeout;
    case HAL_ERROR:
    default:
        return Status::SpiError;
    }
}

void ADS1220::BuildRegisters(uint8_t registers[4]) const
{
    // 配置寄存器 0：MUX[7:4]、GAIN[3:1]、PGA_BYPASS[0]。

    registers[0] = static_cast<uint8_t>(Config_.input) |
                   static_cast<uint8_t>(static_cast<uint8_t>(Config_.gain) << 1U) |
                   static_cast<uint8_t>(Config_.bypass_pga ? 0x01U : 0x00U);

    // 配置寄存器 1：DR[7:5]、MODE[4:3]、CM[2]、TS[1]、BCS[0]。

    registers[1] = static_cast<uint8_t>(Config_.data_rate) |
                   static_cast<uint8_t>(Config_.conversion_mode) |
                   static_cast<uint8_t>(Config_.temperature_sensor ? 0x02U : 0x00U) |
                   static_cast<uint8_t>(Config_.burn_out_sources ? 0x01U : 0x00U);

    // 配置寄存器 2：VREF[7:6]、FIR[5:4]、PSW[3]、IDAC[2:0]。

    registers[2] = static_cast<uint8_t>(Config_.voltage_reference) |
                   static_cast<uint8_t>(Config_.filter) |
                   static_cast<uint8_t>(Config_.low_side_switch ? 0x08U : 0x00U) |
                   static_cast<uint8_t>(Config_.idac_current);

    // 配置寄存器 3：I1MUX[7:5]、I2MUX[4:2]、DRDYM[1]；保留位始终写 0。

    registers[3] = static_cast<uint8_t>(static_cast<uint8_t>(Config_.idac1_route) << 5U) |
                   static_cast<uint8_t>(static_cast<uint8_t>(Config_.idac2_route) << 2U) |
                   static_cast<uint8_t>(Config_.dout_drdy_enabled ? 0x02U : 0x00U);
}

void ADS1220::DecodeValue(const uint8_t data[3])
{
    // ADS1220 按 MSB 优先输出三个字节，先拼成无符号 24 位值以避免有符号左移问题。

    const uint32_t packed = (static_cast<uint32_t>(data[0]) << 16U) |
                            (static_cast<uint32_t>(data[1]) << 8U) | static_cast<uint32_t>(data[2]);

    // bit23 是 24 位补码符号位；置位时减去 2^24 完成符号扩展。
    RawValue_ = static_cast<int32_t>(packed);
    if ((packed & 0x00800000U) != 0U)
    {
        RawValue_ -= 0x01000000;
    }

    // 温度结果是左对齐的 14 位补码，最低 10 位无效，每个有效 LSB 为 0.03125 °C。

    if (Config_.temperature_sensor)
    {
        int32_t temperature_code = static_cast<int32_t>(packed >> 10U);
        if ((temperature_code & 0x2000) != 0)
        {
            temperature_code -= 0x4000;
        }
        Temperature_ = static_cast<float>(temperature_code) * 0.03125f;
        Voltage_     = 0.0f;
        GainVoltage_ = 0.0f;
        return;
    }

    // 模拟输入模式下：VIN = Code × VREF / (Gain × 2^23)。
    // 选择内部参考时，芯片固定使用 2.048 V，不采用 Config_ 中的外部参考电压值。

    const float reference_voltage = Config_.voltage_reference == VoltageReference::Internal2048
                                            ? 2.048f
                                            : Config_.reference_voltage;

    switch (Config_.input)
    {
    case (InputMux::ReferenceDiv4):
    case (InputMux::SupplyDiv4):
        Voltage_     = static_cast<float>(RawValue_) * 2.048f / (AdcPositiveScale_);
        GainVoltage_ = Voltage_ * 4;
        break;
    default:
        Voltage_     = static_cast<float>(RawValue_) * reference_voltage /
                       (static_cast<float>(GetGainValue()) * AdcPositiveScale_);
        GainVoltage_ = Voltage_ * GetGainValue();
    }
}

uint16_t ADS1220::GetGainValue() const
{
    // Gain 枚举保存 log2(倍率)，左移即可得到实际倍率且不需要查表。
    return static_cast<uint16_t>(1U << static_cast<uint8_t>(Config_.gain));
}
