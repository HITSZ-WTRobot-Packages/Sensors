/**
 * @file    ADS1220.cpp
 * @author  Wleaf.
 * @date    2026-09-18
 */

#include "ADS1220.hpp"

#include <cmath>

// 完整配置构造方式同样只保存参数，不在构造函数中访问尚未就绪的硬件。

ADS1220::ADS1220(SPI_HandleTypeDef* hspi, const Config& config) : hspi_(hspi), Config_(config) {}

ADS1220::Status ADS1220::Calibrate()
{
    if (!Initialized_)
    {
        return LastStatus_ = Status::NotInitialized;
    }
    if (!IsAnalogInput(Config_))
    {
        return LastStatus_ = Status::InvalidArgument;
    }

    const Config original_config = Config_;
    const int32_t previous_offset = CodeOffset_;
    const int32_t previous_raw = RawValue_;
    const float previous_voltage = Voltage_;
    const float previous_gain_voltage = GainVoltage_;
    const float previous_temperature = Temperature_;
    const float previous_monitor_voltage = MonitorVoltage_;
    const bool previous_has_sample = HasSample_;

    // 按数据手册 8.3.12 短接 PGA 两端，保持原增益和参考源。
    // 校准流程自身等待转换完成；正常 Update() 只检查就绪，不阻塞等待。
    Config_.input = InputMux::ShortedMidSupply;
    Config_.conversion_mode = ConversionMode::SingleShot;
    Config_.burn_out_sources = false;
    CodeOffset_ = 0;
    Status status = ApplyConfig();
    constexpr uint16_t sample_count = 32;
    int64_t sum = 0;
    for (uint16_t index = 0; index < sample_count && status == Status::Ok; ++index)
    {
        if (index != 0)
        {
            status = Start();
        }
        if (status == Status::Ok)
        {
            WaitForConversion();
            status = Update();
        }
        if (status == Status::Ok)
        {
            sum += RawValue_;
        }
    }

    // 失败也要恢复并读回验证原配置，不能仅回滚软件变量。
    Config_ = original_config;
    CodeOffset_ = previous_offset;
    const Status restore_status = ApplyConfig();
    if (restore_status != Status::Ok)
    {
        Initialized_ = false;
        Converting_ = false;
        ClearSample();
        return LastStatus_ = restore_status;
    }
    if (status == Status::Ok)
    {
        CodeOffset_ = static_cast<int32_t>(sum / sample_count);
        // 缓存中不能保留按旧校准值换算的结果，下次 Update() 才发布真实通道数据。
        ClearSample();
    }
    else
    {
        RawValue_ = previous_raw;
        Voltage_ = previous_voltage;
        GainVoltage_ = previous_gain_voltage;
        Temperature_ = previous_temperature;
        MonitorVoltage_ = previous_monitor_voltage;
        HasSample_ = previous_has_sample;
    }
    return LastStatus_ = status;
}

ADS1220::Status ADS1220::Init()
{
    // 初始化顺序：参数校验 -> 复位 -> 配置校验 -> 模拟输入偏移校准 -> 恢复采样。
    // 只有包括校准在内的完整流程成功，业务层才可以开始采样。

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
    if (LastStatus_ != Status::Ok)
    {
        return LastStatus_;
    }
    Initialized_ = true;
    // 温度和系统监测模式不使用 PGA 偏移校准。
    if (IsAnalogInput(Config_))
    {
        LastStatus_ = Calibrate();
        if (LastStatus_ != Status::Ok)
        {
            Initialized_ = false;
            Converting_ = false;
            ClearSample();
        }
    }
    return LastStatus_;
}

ADS1220::Status ADS1220::Update()
{
    if (!Initialized_)
    {
        return LastStatus_ = Status::NotInitialized;
    }
    if (!Converting_)
    {
        return LastStatus_ = Status::NotReady;
    }

    // 尚未经过完整保守周期时立即返回；不等待、不访问 SPI、不修改测量缓存。
    // 无符号差值允许 HAL 毫秒 tick 跨越 UINT32_MAX 回绕。
    if (HAL_GetTick() - ConversionStartedAt_ < GetConversionDelayMs())
    {
        return LastStatus_ = Status::NotReady;
    }
    LastStatus_ = SendCommand(Cmd::RDATA);
    if (LastStatus_ != Status::Ok)
    {
        return LastStatus_;
    }

    uint8_t data[3] = {};
    LastStatus_ = ReadData(data, 3);
    if (LastStatus_ != Status::Ok)
    {
        return LastStatus_;
    }

    DecodeValue(data);
    HasSample_ = true;
    Converting_ = Config_.conversion_mode == ConversionMode::Continuous;
    ConversionStartedAt_ = HAL_GetTick();
    return LastStatus_;
}

ADS1220::Status ADS1220::Configure(const Config& config)
{
    LastStatus_ = ValidateConfig(config);
    if (LastStatus_ != Status::Ok)
    {
        return LastStatus_;
    }
    if (!Initialized_)
    {
        Config_ = config;
        CodeOffset_ = 0;
        Converting_ = false;
        ClearSample();
        return LastStatus_ = Status::Ok;
    }

    const Config previous_config = Config_;
    const int32_t previous_offset = CodeOffset_;
    const int32_t previous_raw = RawValue_;
    const float previous_voltage = Voltage_;
    const float previous_gain_voltage = GainVoltage_;
    const float previous_temperature = Temperature_;
    const float previous_monitor_voltage = MonitorVoltage_;
    const bool previous_has_sample = HasSample_;
    const bool previous_converting = Converting_;

    const bool needs_calibration = IsAnalogInput(config) &&
        (!IsAnalogInput(previous_config) || config.gain != previous_config.gain ||
         config.bypass_pga != previous_config.bypass_pga ||
         config.data_rate != previous_config.data_rate ||
         config.voltage_reference != previous_config.voltage_reference ||
         config.reference_voltage != previous_config.reference_voltage);

    // 候选配置仅在本次非并发事务内部使用；失败后必须恢复硬件并验证。
    Config_ = config;
    Status status = ApplyConfig();
    if (status == Status::Ok && needs_calibration)
    {
        status = Calibrate();
    }
    if (status == Status::Ok)
    {
        if (!IsAnalogInput(Config_))
        {
            CodeOffset_ = 0;
        }
        ClearSample();
        return LastStatus_ = Status::Ok;
    }

    Config_ = previous_config;
    CodeOffset_ = previous_offset;
    Status restore_status = ApplyConfig();
    if (restore_status == Status::Ok && !previous_converting)
    {
        restore_status = SendCommand(Cmd::POWERDOWN);
        Converting_ = false;
    }
    if (restore_status != Status::Ok)
    {
        Initialized_ = false;
        Converting_ = false;
        ClearSample();
        return LastStatus_ = restore_status;
    }

    Initialized_ = true;
    RawValue_ = previous_raw;
    Voltage_ = previous_voltage;
    GainVoltage_ = previous_gain_voltage;
    Temperature_ = previous_temperature;
    MonitorVoltage_ = previous_monitor_voltage;
    HasSample_ = previous_has_sample;
    return LastStatus_ = status;
}

ADS1220::Status ADS1220::Start()
{
    if (!Initialized_)
    {
        LastStatus_ = Status::NotInitialized;
        return LastStatus_;
    }

    LastStatus_ = SendCommand(Cmd::START);
    if (LastStatus_ == Status::Ok)
    {
        Converting_ = true;
        ConversionStartedAt_ = HAL_GetTick();
        ClearSample();
    }
    else
    {
        Initialized_ = false;
        Converting_ = false;
        ClearSample();
    }
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
    Converting_ = false;
    ClearSample();
    if (LastStatus_ != Status::Ok)
    {
        Initialized_ = false;
    }
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

    // 即使传输超时也无法确定芯片是否已复位，先使软件状态失效。
    Initialized_ = false;
    Converting_ = false;
    CodeOffset_ = 0;
    ClearSample();
    LastStatus_ = SendCommand(Cmd::RESET);
    if (LastStatus_ == Status::Ok)
    {
        HAL_Delay(1);
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
    Config config = Config_;
    config.reference_voltage = Vref;
    return Configure(config);
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

    status = SendCommand(Cmd::START);
    Converting_ = status == Status::Ok;
    ClearSample();
    if (Converting_)
    {
        ConversionStartedAt_ = HAL_GetTick();
    }
    return status;
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
        (!std::isfinite(config.reference_voltage) || config.reference_voltage <= 0.0f))
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
        MonitorVoltage_ = 0.0f;
        return;
    }

    // 模拟输入模式下：VIN = Code × VREF / (Gain × 2^23)。
    // 选择内部参考时，芯片固定使用 2.048 V，不采用 Config_ 中的外部参考电压值。
    // 系统监测绕过 PGA，不能扣除普通输入路径校准得到的偏移。
    Temperature_ = 0.0f;
    MonitorVoltage_ = 0.0f;
    const float reference_voltage = Config_.voltage_reference == VoltageReference::Internal2048
                                            ? 2.048f
                                            : Config_.reference_voltage;

    switch (Config_.input)
    {
    case (InputMux::ReferenceDiv4):
    case (InputMux::SupplyDiv4):
        Voltage_ = static_cast<float>(RawValue_) * 2.048f / AdcPositiveScale_;
        MonitorVoltage_ = Voltage_ * 4.0f;
        GainVoltage_ = 0.0f;
        break;
    default:
        RawValue_ -= CodeOffset_;
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

uint32_t ADS1220::GetConversionDelayMs() const
{
    // 数据率表：正常、占空比、Turbo；假定内部振荡器或 4.096 MHz 外部时钟。
    // 周期向上取整，再留 10% 时钟裕量和 2 ms 启动时间；不依赖未接出的 DRDY。
    static constexpr uint32_t periods_us[3][7] = {
        {50000, 22223, 11112, 5715, 3031, 1667, 1000},
        {200000, 88889, 44445, 22728, 12122, 6667, 4000},
        {25000, 11112, 5556, 2858, 1516, 834, 500},
    };
    const uint8_t rate = static_cast<uint8_t>(Config_.data_rate);
    const uint8_t mode = (rate >> 3U) & 0x03U;
    const uint8_t dr = rate >> 5U;
    return (periods_us[mode][dr] * 11U + 9999U) / 10000U + 2U;
}

void ADS1220::WaitForConversion() const
{
    // 无符号差值允许 HAL 毫秒 tick 跨越 UINT32_MAX 回绕。
    const uint32_t elapsed = HAL_GetTick() - ConversionStartedAt_;
    const uint32_t required = GetConversionDelayMs();
    if (elapsed < required)
    {
        HAL_Delay(required - elapsed);
    }
}

void ADS1220::ClearSample()
{
    RawValue_ = 0;
    Voltage_ = 0.0f;
    GainVoltage_ = 0.0f;
    Temperature_ = 0.0f;
    MonitorVoltage_ = 0.0f;
    HasSample_ = false;
}

bool ADS1220::IsAnalogInput(const Config& config)
{
    return !config.temperature_sensor && static_cast<uint8_t>(config.input) < 0xC0U;
}
