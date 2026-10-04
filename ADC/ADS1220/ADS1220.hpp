
/**
 * @file    ADS1220.hpp
 * @author  Wleaf.
 * @date    2026-09-18
 * @brief   针对ADS1220 24位ADC的封装
 *
 * Detailed description (optional).
 *
 */

#pragma once

#include "main.h"

/**
 * @brief ADS1220 24 位 ADC 面向对象驱动。
 *
 * 具体使用请对应ti数据手册 https://www.ti.com/cn/lit/ds/symlink/ads1220.pdf
 *
 *
 * 业务层通过 Config、Init()、Configure()、Update() 和各类读取接口使用芯片；
 * SPI 命令、配置寄存器编码、24 位补码解码等底层细节全部封装在 private 区域。
 *
 * 硬件约束：
 * - SPI 必须配置为模式 1（CPOL = 0，CPHA = 1）。
 * - 当前类不控制 CS 引脚，调用期间 CS 必须由硬件或外部代码保持为有效低电平。
 * TODO:增加可选CS引脚
 * - Update() 不等待 DRDY，调用方应在数据就绪后调用，或按已配置的数据率周期调用。
 * - 同一个对象不支持多线程并发调用。
 */
class ADS1220
{
public:
    enum class Status : uint8_t
    {
        Ok,                    ///< 操作成功。
        InvalidArgument,       ///< SPI 句柄、参考电压或配置组合无效。
        NotInitialized,        ///< 在 Init() 成功前调用了依赖初始化的接口。
        SpiError,              ///< HAL 报告普通 SPI 错误。
        SpiBusy,               ///< SPI 外设当前忙。
        SpiTimeout,            ///< SPI 发送或接收超时。
        ConfigurationMismatch, ///< 配置寄存器读回值与写入值不一致。
    };

    /**
     * @brief ADC 正、负输入端选择。
     *
     * AinXAinY 表示差分输入 AINP = X、AINN = Y；AinXAvss 表示单端输入。
     * 单端输入必须旁路 PGA，且增益只能选择 X1、X2 或 X4。
     * ReferenceDiv4、SupplyDiv4 用于系统监测，
     * ReferenceDiv4设置测量外部参考电压四分之一
     * SupplyDiv4设置测量模拟电压四分之一
     * ShortedMidSupply 可用于偏移校准。
     * 具体参考手册手册51页
     */
    enum class InputMux : uint8_t
    {
        Ain0Ain1         = 0x00,
        Ain0Ain2         = 0x10,
        Ain0Ain3         = 0x20,
        Ain1Ain2         = 0x30,
        Ain1Ain3         = 0x40,
        Ain2Ain3         = 0x50,
        Ain1Ain0         = 0x60,
        Ain3Ain2         = 0x70,
        Ain0Avss         = 0x80,
        Ain1Avss         = 0x90,
        Ain2Avss         = 0xA0,
        Ain3Avss         = 0xB0,
        ReferenceDiv4    = 0xC0,
        SupplyDiv4       = 0xD0,
        ShortedMidSupply = 0xE0,
    };

    /**
     * @brief ADC 可编程增益。
     *
     * 满量程差分输入范围为 ±VREF / Gain。X8 至 X128 始终启用 PGA；
     * X1、X2、X4 可以结合 Config::bypass_pga 旁路 PGA。
     */
    enum class Gain : uint8_t
    {
        X1   = 0,
        X2   = 1,
        X4   = 2,
        X8   = 3,
        X16  = 4,
        X32  = 5,
        X64  = 6,
        X128 = 7,
    };

    /**
     * @brief 工作模式与输出数据率的有效组合。
     *
     * Normal 为正常模式（256khz），Duty 为 1:3
     * 占空比低功耗模式，一个周期工作时间和休眠时间之比为1：4，Turbo 为高速模式（512khz）。
     * 数值基于内部振荡器或 4.096 MHz 外部时钟；使用其他外部时钟时会同比缩放。
     * 以下单位都是SPS（每秒采样次数）
     * _代表小数点
     */
    enum class DataRate : uint8_t
    {
        Normal20   = 0x00,
        Normal45   = 0x20,
        Normal90   = 0x40,
        Normal175  = 0x60,
        Normal330  = 0x80,
        Normal600  = 0xA0,
        Normal1000 = 0xC0,
        Duty5      = 0x08,
        Duty11_25  = 0x28,
        Duty22_5   = 0x48,
        Duty44     = 0x68,
        Duty82_5   = 0x88,
        Duty150    = 0xA8,
        Duty250    = 0xC8,
        Turbo40    = 0x10,
        Turbo90    = 0x30,
        Turbo180   = 0x50,
        Turbo350   = 0x70,
        Turbo660   = 0x90,
        Turbo1200  = 0xB0,
        Turbo2000  = 0xD0,
    };

    /**
     * @brief 转换模式。
     *
     * SingleShot 每次 Start() 只完成一次转换并自动进入低功耗状态；
     * Continuous 在 Start() 后连续转换，直到调用 PowerDown()。
     */
    enum class ConversionMode : uint8_t
    {
        SingleShot = 0x00,
        Continuous = 0x04,
    };

    /** @brief 转换使用的参考电压来源。 */
    enum class VoltageReference : uint8_t
    {
        Internal2048 = 0x00, ///< 内部 2.048 V 精密参考。
        ExternalRef0 = 0x40, ///< 专用 REFP0、REFN0 引脚。
        ExternalRef1 = 0x80, ///< 复用AIN0/REFP1、AIN3/REFN1 引脚。
        AnalogSupply = 0xC0, ///< 模拟电源 AVDD - AVSS。
    };

    /**
     * @brief 工频干扰抑制滤波器。
     * @note 非 None 选项仅允许与 Normal20 或 Duty5 配合使用。
     */

    enum class FirFilter : uint8_t
    {
        None          = 0x00,
        Reject50And60 = 0x10,
        Reject50      = 0x20,
        Reject60      = 0x30,
    };

    /**
     * @brief IDAC1 和 IDAC2 共用的激励电流大小。
     * @note 具体输出引脚由 Config::idac1_route 和 Config::idac2_route 决定。
     */

    enum class IdacCurrent : uint8_t
    {
        Off    = 0,
        Ua10   = 1,
        Ua50   = 2,
        Ua100  = 3,
        Ua250  = 4,
        Ua500  = 5,
        Ua1000 = 6,
        Ua1500 = 7,
    };

    /** @brief IDAC 激励电流的输出路由；Disabled 表示该路 IDAC 不连接到引脚。 */
    enum class IdacRoute : uint8_t
    {
        Disabled = 0,
        Ain0     = 1,
        Ain1     = 2,
        Ain2     = 3,
        Ain3     = 4,
        Refp0    = 5,
        Refn0    = 6,
    };

    /**
     * @brief ADS1220 完整工作配置。
     *
     */
    struct Config
    {
        InputMux         input;             ///< ADC 输入通道组合。
        Gain             gain;              ///< 输入增益，参与满量程和电压换算。
        DataRate         data_rate;         ///< 工作模式与输出数据率。
        ConversionMode   conversion_mode;   ///< 单次或连续转换。
        VoltageReference voltage_reference; ///< 硬件参考源。
        float            reference_voltage; ///< 必须为有限正数；外部/电源参考的实际电压，内部参考固定使用 2.048 V。
        FirFilter        filter;            ///< 20 SPS/5 SPS 下的工频抑制方式。
        bool             bypass_pga;        ///< true 时旁路 PGA，仅允许 X1、X2、X4。
        bool             temperature_sensor; ///< true 时测量芯片内部温度，而非模拟输入。
        bool             burn_out_sources;   ///< 启用 10 µA 传感器开路/短路检测电流源。
        bool             low_side_switch;    ///< START 时闭合、POWERDOWN 时断开内部低侧开关。
        IdacCurrent      idac_current;       ///< 两路 IDAC 的公共电流设置。
        IdacRoute        idac1_route;        ///< IDAC1 输出位置。
        IdacRoute        idac2_route;        ///< IDAC2 输出位置。
        bool             dout_drdy_enabled;  ///< 是否让 DOUT/DRDY 同步指示数据就绪。
    };



    /**
     * @brief 使用完整配置构造 ADC。
     * @param hspi STM32 HAL SPI 句柄，生命周期必须长于本对象且不能为 nullptr。
     * @param config 高层功能配置；构造阶段只保存，Init() 时才写入芯片。
     */
    explicit ADS1220(SPI_HandleTypeDef* hspi, const Config& config);

    /** @brief 复位芯片、校验配置、写入并读回寄存器，最后启动转换。 */
    Status Init();

    /**
     * @brief 读取最新转换结果并更新缓存。
     * @note 本函数不会等待 DRDY；读取成功后再调用 GetVoltage() 或 GetTemperature()。
     */
    Status Update();

    /**
     * @brief 应用一套完整配置。
     * @note 初始化前调用仅校验并保存配置，成功返回 Ok，不访问硬件。
     *       初始化后调用会立即写入、校验并重新启动转换，全部成功后才保存新配置。
     *       参数校验失败不改变当前配置或初始化状态；硬件应用失败则保留旧配置并清除初始化状态，
     *       必须重新调用 Init() 成功后才能继续采样。
     */
    Status Configure(const Config& config);

    /** @brief 启动单次转换，或启动/同步连续转换。 */
    Status Start();

    /** @brief 完成当前转换后进入省电模式；寄存器配置保持不变。 */
    Status PowerDown();

    /** @brief 发送硬件复位命令；成功后对象变为未初始化状态。 */
    Status Reset();

    /** @brief 修改增益；配置缓存及失败恢复规则同 Configure()。 */
    Status SetGain(Gain gain);

    /**
     * @brief 修改电压换算使用的参考电压，单位 V。
     * @note 该值是软件换算参数，不会改变 ADS1220 的参考源选择。
     * @return 非有限值或非正数返回 InvalidArgument 并保留原值，否则返回 Ok。
     */
    Status SetVref(float Vref);

    /** @brief 返回最近一次成功采样得到的 24 位有符号原始码。 */
    int32_t GetRawValue() const { return RawValue_; }

    /** @brief 返回最近一次模拟输入采样换算出的差分电压，单位 V。 */
    float GetVoltage() const { return Voltage_; }

    float GetGainVoltage() const { return GainVoltage_; }

    /** @brief 返回最近一次内部温度传感器采样值，单位 °C。 */
    float GetTemperature() const { return Temperature_; }

    /** @brief 返回最近一次操作状态。 */
    Status GetLastStatus() const { return LastStatus_; }

    /** @brief 只读访问当前高层配置；不能通过该引用修改对象。 */
    const Config& GetConfig() const { return Config_; }

    /** @brief Init() 是否已经完整成功。 */
    bool IsInitialized() const { return Initialized_; }

private:
    // 以下内容是寄存器和 SPI 实现细节，不向业务层开放。

    /** @brief ADS1220 SPI 命令字节，仅供驱动内部使用。 */
    enum class Cmd : uint8_t
    {
        RESET     = 0x06,
        START     = 0x08,
        POWERDOWN = 0x02,
        RDATA     = 0x10,
        RREG      = 0x23,
        WREG      = 0x43,
    };

    /** @brief 发送无附加数据的单字节命令。 */
    Status SendCommand(Cmd cmd);

    /** @brief 发送数据。 */
    Status SendData(uint8_t* pData, uint8_t size);

    /** @brief 发送数据。 */
    Status ReadData(uint8_t* pData, uint8_t size);

    /** @brief 从配置寄存器 0 开始连续写入全部 4 个配置寄存器。 */
    Status WriteRegisters(const uint8_t registers[4]);

    /** @brief 从配置寄存器 0 开始连续读取全部 4 个配置寄存器。 */
    Status ReadRegisters(uint8_t registers[4]);

    /** @brief 编码、写入并读回校验候选配置，成功后发送 START。 */
    Status ApplyConfig(const Config& config);

    /** @brief 检查枚举范围及数据手册规定的配置组合限制。 */
    Status ValidateConfig(const Config& config) const;

    /** @brief 将 STM32 HAL SPI 状态转换为公共 Status。 */
    Status FromHalStatus(HAL_StatusTypeDef status) const;

    /** @brief 将高层配置编码为配置寄存器 0～3。 */
    void BuildRegisters(const Config& config, uint8_t registers[4]) const;

    /** @brief 解码 24 位补码 ADC 数据或左对齐的 14 位温度数据。 */
    void DecodeValue(const uint8_t data[3]);

    /** @brief 将 Gain 枚举转换为实际倍率 1、2、4……128。 */
    uint16_t GetGainValue() const;

    // ADS1220 正向满量程对应 2^23 个码，用于双极性 24 位数据换算。
    static constexpr float AdcPositiveScale_ = 8388608.0f;

    // SPI 句柄由外部持有；本类不取得所有权，也不负责其初始化和销毁。
    SPI_HandleTypeDef* hspi_ = nullptr;

    // 当前配置及最近一次有效测量结果均由对象内部保存，业务层只能只读访问。
    Config  Config_      = {};
    int32_t RawValue_    = 0;
    float   Voltage_     = 0.0f;
    float   Temperature_ = 0.0f;
    Status  LastStatus_  = Status::NotInitialized;
    bool    Initialized_ = false;
    float   GainVoltage_ = 0.0f;
};
