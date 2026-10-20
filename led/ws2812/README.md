# WS2812B 灯带

包名：`SensorLed::WS2812B`。依赖 `memory::DMABuffer` 和 `stm32cubemx`；由宿主 cpkg 集成，不独立构建。[仓库总览](../../README.md)

[WS2812B.hpp](WS2812B.hpp) 提供头文件模板 `sensors::led::WS2812B<N, BufferLen>`。硬件契约和代码级行为以该文件现有 Doxygen 为准；这里保留消费者所需的集成条件，不另述波形展开算法。

## 缓冲区与生命周期

- `N` 为灯珠总数，必须大于零；`BufferLen` 为整个循环 DMA 缓冲区可容纳的灯珠数量，不是单个半区容量。它必须是 4 的倍数，范围为 4～2728，以使两个半区各自占用完整的 32 字节缓存行，并满足 HAL DMA 长度限制。
- 调用方持有 `WS2812B<N, BufferLen>::Buffer`，它是 `memory::DMABuffer<48 * BufferLen>` 的别名，通过引用传入构造函数。波形区占 `48 * BufferLen` 字节（每灯 24 个 `uint16_t` 样本），实例另存 `3 * N` 字节 RGB 数据，不分配堆内存。循环波形缓冲大小固定，不为整条灯带分配展开波形。
- 构造参数依次为 TIM 句柄、通道和 `Buffer&`。构造不访问 HAL/外设，不发送初始帧；当前没有 Cache clean 回调参数或实现。实例使用 Buffer 期间调用方不得访问该缓冲区。
- Buffer 必须位于 DMA 可达且不可缓存或硬件一致性的 SRAM。`DMABuffer` 的 32 字节对齐不保证 DMA 可达或缓存一致；驱动不执行 D-Cache clean，内存屏障也不能替代 clean。经典 H7 默认 DTCM 不可供 DMA1/2 使用，宿主需使用自己的链接段与 MPU 配置，不固定任何消费项目布局。
- 驱动、TIM/DMA 句柄及 Buffer 必须覆盖发送、停止、异步 DMA 中止和全部转发回调的生命周期。析构不会替代停止操作；退出顺序见下文。

## CubeMX 静态配置

所有静态外设参数由宿主 CubeMX 配置；驱动不调用 Init/Config 接口，不切换 OC/PWM 模式，也不自动修复配置：

- TIM 专用、向上计数、边沿对齐、PWM1、高有效、CCR 预装载、初始 Pulse 为 0、无重复计数。
- 所选 **CC** DMA 为内存到外设、Circular、内存递增、外设不递增、双方 Halfword；配置对应 DMA 请求和 IRQ，由宿主 IRQ 调用 HAL DMA 处理函数。
- 时钟、PSC/ARR、GPIO AF/上下拉、空闲与 off-state 电平、以及高级定时器的 break/output 条件均由宿主配置。需要确认实际引脚有输出、实际计数频率符合灯珠数据手册，不能仅由 ARR 数值推定时钟正确。
- 定时器必须配置为 **800 kbit/s**，即 `ARR + 1` 个计数对应一个 1.25 μs 位周期。驱动从 ARR 自动推导脉宽：0 高电平占 24%，1 高电平占 52%（约 300 ns / 650 ns，适用于 WS2812B-V5/W）；无需传入时序参数。复位低电平固定 250 个位周期，约 312.5 μs。
- 宿主须保证单脉冲、从模式、更新事件禁用、CC 控制预装载均未启用，CC DMA 请求由比较事件触发，实例存活期间不从外部修改配置。

现有配置校验使用 HAL getter、Init 字段和 DMA 配置；HAL 没有 getter 的实时配置位不逐位检查，上述条件中存在必须由宿主保证的前提，不能把它们写成驱动已全部校验。硬件运行操作只使用 HAL 函数及官方宏，不直接读写寄存器，仅执行帧开始前计数器归零、更新事件和标志清理，以及 PWM/DMA 启停。若维护中需要新增当前无法完成的校验或绕过 HAL1，须先通过 Ask 确认，而不是新增寄存器替代方案。

## 发送与回调

普通接口由同一个任务调用。`setPixel()`、`fill()`、`clear()` 只修改 RGB 帧；`show()` 非阻塞发送一次 GRB、MSB-first 帧，并自动生成前后复位低电平。发送及 DMA 中止期间，像素修改返回 `false`，再次发送返回 `HAL_BUSY`；调用方应检查返回值。

驱动不覆盖全局 HAL 回调，宿主按实例转发：

| HAL 回调 | 实例入口 |
| --- | --- |
| `HAL_TIM_PWM_PulseFinishedHalfCpltCallback` | `onHalfTransfer(htim)` |
| `HAL_TIM_PWM_PulseFinishedCallback` | `onTransferComplete(htim)` |
| `HAL_TIM_ErrorCallback` | `onError(htim)` |

必须在 DMA 绕回前完成半区续填；每半区期限为 `24 * BufferLen / 2` 个位周期。ISR 延迟和工作量必须满足这一条件，不能以模板缓冲区大小代替运行期时限保证。

## 停止与异常

- 帧间 PWM 通道和 DMA 均停止，不再运行空闲 PWM。每帧开始不写 CCR：CubeMX 的初始 Pulse 为 0，正常帧结束时会消耗完整全零 padding 半区，因此 CCR 保持 0。
- 手动 `stop()` 或错误中止可能留下残余 CCR，下一帧首位可能异常。停发期间 DIN 必须由 CubeMX/板级固定配置保证为低电平；需按所选 TIM/GPIO 确认 idle/off-state 设置及上下拉，不能把停止 PWM 本身当成低电平保证。
- `stop()` 不等于熄灯；熄灯应调用 `clear()` 后 `show()`。
- `stop()` 返回 `HAL_BUSY` 表示异步 DMA 中止尚未结束。此时继续服务 IRQ，保留实例、Buffer 和回调转发直到 `!isBusy()`，销毁前再移除转发。不能以一次 `stop()` 调用返回为资源已释放的保证。
- 错误通过 `getState()` 保持可见，显式 `stop()` 才清除；底层 DMA 未恢复 READY 时不能重新发送。具体状态和返回值以现有 Doxygen 与实现为准。

## STM32 Cortex-M 兼容性边界

以下是已有的官方 HAL/CMSIS 源码兼容性核查记录，区分的是**当前驱动能否直接配合所核查源码编译使用**，不是芯片硬件能否产生 WS2812 波形，不构成逐料号支持承诺，也不是物理波形或上板验证。上游 `master/main` 会变化；历史 Cube 版本和每个料号的请求映射仍需单独检查。宿主使用 C++17 或更新标准。

| 分类 | 系列或既有核查代表型号 | 当前限制 |
| --- | --- | --- |
| Cortex-M0/M0+ 编译不兼容 | F0；C0、G0、L0、U0、WB0、WL3x；双核 WB/WL 的 M0+ 核也在此列 | 已有核查记录中 GNU Arm GCC 16.2.0 下，`std::atomic<State>::is_always_lock_free` 为 `false`，实例化驱动触发断言。这里不把 WB/WL 的 M4 应用核一并判为不兼容。 |
| 经典 HAL 的通道校验宏不兼容 | F1/F2/F4；已有核查使用完整官方头文件编译确认 STM32F103xB、STM32F207xx、STM32F407xx 失败 | 这些代表型号提供 `IS_TIM_CCX_INSTANCE`，但没有驱动使用的 `IS_TIM_CCX_CHANNEL`。不能仅因 TIM/DMA 其他 API 相同就声明可直接使用。 |
| L1 HAL 编译不兼容 | L1；已有核查使用完整官方头文件编译确认 STM32L152xB 失败 | 除校验宏差异外，`TIM_Base_InitTypeDef` 没有 `RepetitionCounter`；运行时条件短路不能绕过 C++ 成员检查。 |
| 新 DMA HAL 需要适配 | H5、H7RS、U3、U5、N6、WBA | GPDMA/HPDMA/LPDMA 的 HAL 使用 `SrcInc`/`DestInc`、`SrcDataWidth`/`DestDataWidth` 等配置，不符合当前 `PeriphInc`/`MemInc`、alignment 字段及 `DMA_CIRCULAR` 校验。循环续填需要适配对应 linked-list/circular 配置与 HT/TC 语义；不能把经典 H7 的结果套用于 H7RS。 |
| HAL2 需要移植 | C5（官方 STM32CubeC5 HAL2） | HAL2 的句柄、状态和传输 API 不同，例如 `hal_dma_state_t` 使用 IDLE/ACTIVE，而非当前依赖的 READY/BUSY 状态契约。 |
| 其余经典 HAL 候选 | F3、F7、G4、L4/L4+、L5、经典 H7、WB/WL 的 M4 应用核 | 不属于上述已识别的不兼容组，但没有逐料号编译和上板认证；必须检查所选版本的宏、API、TIM CC DMA 路由和内存可达性，不能仅凭内核型号宣称兼容。 |

所有候选仍要求：TIM CH1～4 可输出 PWM1 并有对应 **CC** DMA 请求；支持双方 halfword 的循环传输与半/全完成回调；停发后 DIN 为低；续填在半区期限内完成。TIM6/TIM7、无 DMA 请求的通道、仅有互补输出的接法或仅配置 TIM update DMA 都不能直接替代这一契约。DTCM/CCM、D-Cache、TrustZone 和多核外设归属是具体内存/配置限制，不是把整个 M7/M33 系列判为不兼容的理由。

核查依据：

- 经典 HAL：[F1 TIM](https://github.com/STMicroelectronics/stm32f1xx-hal-driver/blob/master/Inc/stm32f1xx_hal_tim.h)、[F103 CMSIS](https://github.com/STMicroelectronics/cmsis-device-f1/blob/master/Include/stm32f103xb.h)、[F2 TIM](https://github.com/STMicroelectronics/stm32f2xx-hal-driver/blob/master/Inc/stm32f2xx_hal_tim.h)、[F4 TIM](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Inc/stm32f4xx_hal_tim.h)、[F407 CMSIS](https://github.com/STMicroelectronics/cmsis-device-f4/blob/master/Include/stm32f407xx.h)、[L1 TIM](https://github.com/STMicroelectronics/stm32l1xx-hal-driver/blob/master/Inc/stm32l1xx_hal_tim.h)。
- 新 DMA HAL：[H5](https://github.com/STMicroelectronics/stm32h5xx-hal-driver/blob/main/Inc/stm32h5xx_hal_dma.h)、[H7RS](https://github.com/STMicroelectronics/stm32h7rsxx-hal-driver/blob/main/Inc/stm32h7rsxx_hal_dma.h)、[U3](https://github.com/STMicroelectronics/stm32u3xx-hal-driver/blob/main/Inc/stm32u3xx_hal_dma.h)、[U5](https://github.com/STMicroelectronics/stm32u5xx-hal-driver/blob/main/Inc/stm32u5xx_hal_dma.h)、[N6](https://github.com/STMicroelectronics/stm32n6xx-hal-driver/blob/main/Inc/stm32n6xx_hal_dma.h)、[WBA](https://github.com/STMicroelectronics/stm32wbaxx-hal-driver/blob/main/Inc/stm32wbaxx_hal_dma.h)。
- HAL2：[C5 DMA](https://github.com/STMicroelectronics/stm32c5xx-drivers/blob/main/hal/stm32c5xx_hal_dma.h)。
