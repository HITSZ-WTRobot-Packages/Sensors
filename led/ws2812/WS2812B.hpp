#ifndef SENSORS_LED_WS2812B_HPP
#define SENSORS_LED_WS2812B_HPP

#include "dma_buffer.hpp"
#include "main.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace sensors::led
{

/**
 * Finite WS2812B frames using a dedicated timer's CC-triggered circular DMA.
 *
 * N is the strip length; BufferLen is the TOTAL waveform capacity in LED units,
 * not the strip length and not the capacity of one half. Only the RGB frame is
 * stored per LED. Each DMA half is refilled in its consumed-half interrupt.
 * The application must service interrupts and finish each refill
 * before DMA returns to that half (24 * BufferLen / 2 bit periods).
 *
 * Hardware contract, checked by show where possible:
 * - Exclusively owned timer, up-counting edge-aligned PWM1, active-high output,
 *   CCR preload enabled, initial Pulse=0, no repetition/slave/one-pulse mode
 *   or CC control preload; update events must remain enabled.
 * - The bit rate MUST be 800 kbit/s. ARR therefore counts one 1.25 us bit;
 *   zero/one high times are derived from ARR as 24%/52% of the period,
 *   i.e. about 300 ns / 650 ns per bit. No timing argument is accepted.
 * - Linked CC DMA: circular, memory-to-peripheral, incrementing halfword memory,
 *   fixed halfword peripheral; CC DMA requests, not update DMA requests.
 * - The application configures the timer clock, PSC/ARR, GPIO AF, DMA request,
 *   DMA IRQ/HAL_DMA_IRQHandler, and any advanced-timer break/output settings.
 *   PWM mode, polarity, preload, GPIO pulls and idle/off-state levels must all
 *   be configured in CubeMX. The driver never reconfigures them. The stopped
 *   output must remain LOW; GPIO/advanced-timer idle settings are board-owned.
 *   It must ensure the actual pin is driven and the actual timing meets the
 *   LED datasheet. Ticks alone do not identify the timer's counter frequency.
 *   Validation uses HAL getters and Init metadata, not direct register reads.
 *   Settings without HAL getters are application preconditions; bypassing HAL
 *   or changing configuration externally is not detected by this driver.
 *
 * Buffer is memory::DMABuffer<48 * BufferLen>, a 32-byte aligned byte array.
 * It must reside in DMA-accessible SRAM (NOT the H7 default DTCM .bss).
 * Place it using the project's linker/section scheme. The buffer must be
 * noncacheable or hardware-coherent: this driver does not clean D-Cache.
 * A barrier alone does not provide cache coherency. No cache policy is changed.
 *
 * One task owns all normal API calls. Only the three explicitly forwarded HAL
 * callbacks run in ISR context. No callbacks are installed globally. Forward:
 * HAL_TIM_PWM_PulseFinishedHalfCpltCallback -> onHalfTransfer;
 * HAL_TIM_PWM_PulseFinishedCallback         -> onTransferComplete;
 * HAL_TIM_ErrorCallback                    -> onError.
 *
 * Construction never touches HAL/peripherals and never sends an initial frame.
 * Driver and Buffer must outlive stop(), asynchronous DMA abort completion
 * (!isBusy()), and any forwarded callback. Detach forwarding before destroying
 * the driver. The destructor deliberately does not invoke uninitialized HAL.
 * Never access the borrowed Buffer while owned by this driver. Do not change
 * timer/DMA configuration during its lifetime. Between frames PWM and DMA are
 * stopped. The project's fixed GPIO/timer configuration must hold DIN LOW.
 */
template <std::size_t N, std::size_t BufferLen> class WS2812B final
{
    static_assert(N > 0, "The strip must contain at least one LED");
    // Check division bounds BEFORE computing the product: HAL's length is u16.
    static_assert(BufferLen <= 65535U / 24U, "Waveform exceeds HAL DMA length");
    // Each half occupies 24*BufferLen bytes, hence whole 32-byte cache lines.
    static_assert(BufferLen >= 4 && BufferLen % 4 == 0,
                  "Each DMA half must occupy independent whole cache lines");

public:
    struct Color
    {
        std::uint8_t r, g, b;
    };
    using RGB = Color;

    static constexpr std::size_t SampleCount     = BufferLen <= 65535U / 24U ? 24U * BufferLen : 0U;
    static constexpr std::size_t HalfSampleCount = SampleCount / 2U;
    static constexpr std::size_t SampleBytes = BufferLen <= 65535U / 24U ? 2U * SampleCount : 0U;
    static constexpr std::size_t HalfSampleBytes = HalfSampleCount * sizeof(std::uint16_t);

    using Buffer = memory::DMABuffer<SampleBytes>;

    // 800 kbit/s -> 1.25 us per bit. At least 300 us of LOW requires 240 bit
    // periods; 250 leaves margin for integer ARR rounding.
    static constexpr std::uint32_t ResetPeriods = 250U;

    using CacheClean = void (*)(const void*, std::size_t);
    enum class State : std::uint8_t
    {
        Idle,
        Sending,
        Stopping,
        Error
    };
    static_assert(std::atomic<State>::is_always_lock_free,
                  "ISR state handoff requires a lock-free atomic");

    WS2812B(TIM_HandleTypeDef* htim, std::uint32_t channel, Buffer& buffer) noexcept :
        htim_(htim), channel_(channel), buffer_(buffer)
    {
    }

    WS2812B(const WS2812B&)            = delete;
    WS2812B& operator=(const WS2812B&) = delete;
    WS2812B(WS2812B&&)                 = delete;
    WS2812B& operator=(WS2812B&&)      = delete;

    [[nodiscard]] bool setPixel(std::size_t index, Color color) noexcept
    {
        if (index >= N || isBusy())
            return false;
        pixels_[index] = color;
        return true;
    }

    [[nodiscard]] bool fill(Color color) noexcept
    {
        if (isBusy())
            return false;
        pixels_.fill(color);
        return true;
    }

    [[nodiscard]] bool clear() noexcept { return fill(Color{ 0, 0, 0 }); }

    [[nodiscard]] State getState() const noexcept
    {
        const State state = state_.load(std::memory_order_acquire);
        if (state == State::Stopping && dmaReady())
            return State::Idle;
        return state;
    }

    [[nodiscard]] bool isBusy() const noexcept
    {
        const State state = state_.load(std::memory_order_acquire);
        return state == State::Sending ||
               ((state == State::Stopping || state == State::Error) && !dmaReady());
    }

    /** HAL_BUSY also covers an asynchronous abort still awaiting its DMA IRQ.
     * Errors remain visible until an explicit stop() clears them after abort.
     */
    [[nodiscard]] HAL_StatusTypeDef show() noexcept
    {
        if (isBusy())
            return HAL_BUSY;
        if (state_.load(std::memory_order_acquire) == State::Error)
            return HAL_ERROR;
        if (!validConfiguration())
        {
            state_.store(State::Error, std::memory_order_release);
            return HAL_ERROR;
        }
        DMA_HandleTypeDef* const dma = selectedDma();
        if (HAL_DMA_GetState(dma) != HAL_DMA_STATE_READY)
            return HAL_BUSY;
        if (HAL_TIM_Base_GetState(htim_) != HAL_TIM_STATE_READY ||
            HAL_TIM_GetChannelState(htim_, channel_) != HAL_TIM_CHANNEL_STATE_READY)
        {
            state_.store(State::Error, std::memory_order_release);
            return HAL_ERROR;
        }

        dma_            = dma;
        phase_          = Phase::LeadingReset;
        resetRemaining_ = ResetPeriods;
        pixelIndex_     = 0;
        bitMask_        = 0;
        fillHalf(0);
        fillHalf(1);

        // No CC DMA requests while clearing preload/pipeline and timer phase.
        __HAL_TIM_DISABLE_DMA(htim_, dmaRequest());
        if (!alignFrameStart())
        {
            state_.store(State::Error, std::memory_order_release);
            stopPwmDma();
            return HAL_ERROR;
        }
        txBarrier();
        // Publish before HAL can enable DMA IRQs; never overwrite ISR results
        // after a successful start (a short frame may finish before return).
        state_.store(State::Sending, std::memory_order_release);
        // HAL's word-pointer signature does not change the halfword DMA width.
        const HAL_StatusTypeDef result =
                HAL_TIM_PWM_Start_DMA(htim_,
                                      channel_,
                                      reinterpret_cast<const std::uint32_t*>(buffer_.data()),
                                      static_cast<std::uint16_t>(SampleCount));
        if (result != HAL_OK)
        {
            state_.store(State::Error, std::memory_order_release);
            stopPwmDma(); // Also releases a channel left BUSY by a failed HAL start.
            return HAL_ERROR;
        }
        return state_.load(std::memory_order_acquire) == State::Error ? HAL_ERROR : HAL_OK;
    }

    /** Request a safe stop, or explicitly clear a latched error.
     * HAL_BUSY means abort is pending: retain this object/Buffer and service
     * the DMA IRQ before reuse/destruction. Does nothing to an unused timer.
     * A mid-frame stop leaves the last CCR value; the next frame may begin
     * with one invalid bit. Normal frame completion ends on all-zero padding.
     */
    [[nodiscard]] HAL_StatusTypeDef stop() noexcept
    {
        State previous = State::Sending;
        if (state_.compare_exchange_strong(previous,
                                           State::Stopping,
                                           std::memory_order_acq_rel,
                                           std::memory_order_acquire))
        {
            if (!stopPwmDma())
            {
                state_.store(State::Error, std::memory_order_release);
                return HAL_ERROR;
            }
        }
        if (!dmaReady())
            return HAL_BUSY;
        state_.store(State::Idle, std::memory_order_release);
        return HAL_OK;
    }

    void onHalfTransfer(TIM_HandleTypeDef* htim) noexcept { consumed(htim, 0); }
    void onTransferComplete(TIM_HandleTypeDef* htim) noexcept { consumed(htim, 1); }

    void onError(TIM_HandleTypeDef* htim) noexcept
    {
        // The timer is dedicated. Some HAL error paths do not preserve Channel.
        if (htim != htim_ || state_.load(std::memory_order_acquire) != State::Sending)
            return;
        state_.store(State::Error, std::memory_order_release);
        stopPwmDma();
    }

private:
    enum class Phase : std::uint8_t
    {
        LeadingReset,
        Pixels,
        TrailingReset,
        Padding
    };

    [[nodiscard]] unsigned channelIndex() const noexcept
    {
        return static_cast<unsigned>(channel_ / 4U);
    }

    [[nodiscard]] DMA_HandleTypeDef* selectedDma() const noexcept
    {
        switch (channel_)
        {
        case TIM_CHANNEL_1:
            return htim_->hdma[TIM_DMA_ID_CC1];
        case TIM_CHANNEL_2:
            return htim_->hdma[TIM_DMA_ID_CC2];
        case TIM_CHANNEL_3:
            return htim_->hdma[TIM_DMA_ID_CC3];
        case TIM_CHANNEL_4:
            return htim_->hdma[TIM_DMA_ID_CC4];
        default:
            return nullptr;
        }
    }

    [[nodiscard]] bool dmaReady() const noexcept
    {
        return dma_ == nullptr || HAL_DMA_GetState(dma_) == HAL_DMA_STATE_READY;
    }

    [[nodiscard]] std::uint32_t dmaRequest() const noexcept
    {
        return TIM_DMA_CC1 << channelIndex();
    }

    // Rounded percentage of the ARR period, independent of the timer clock.
    [[nodiscard]] static constexpr std::uint32_t highTicks(std::uint32_t period,
                                                           std::uint32_t percent) noexcept
    {
        return (period * percent + 50U) / 100U;
    }

    [[nodiscard]] bool validConfiguration() noexcept
    {
        if (htim_ == nullptr || htim_->Instance == nullptr ||
            (channel_ != TIM_CHANNEL_1 && channel_ != TIM_CHANNEL_2 && channel_ != TIM_CHANNEL_3 &&
             channel_ != TIM_CHANNEL_4) ||
            !IS_TIM_CCX_CHANNEL(htim_->Instance, channel_))
            return false;
        const std::uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim_);
        // The timer must be configured for 800 kbit/s. ARR + 1 is one bit.
        if (arr >= 0xFFFFU)
            return false;
        const std::uint32_t period = arr + 1U;
        zeroHighTicks_             = static_cast<std::uint16_t>(highTicks(period, 24U));
        oneHighTicks_              = static_cast<std::uint16_t>(highTicks(period, 52U));
        if (zeroHighTicks_ == 0 || zeroHighTicks_ >= oneHighTicks_ || oneHighTicks_ >= period)
            return false;
        // HAL exposes Init metadata, not getters for all live timer settings.
        // Slave/one-pulse/update/CC-DMA selection remains a CubeMX contract.
        if (htim_->Init.CounterMode != TIM_COUNTERMODE_UP ||
            (IS_TIM_REPETITION_COUNTER_INSTANCE(htim_->Instance) &&
             htim_->Init.RepetitionCounter != 0U))
            return false;
        DMA_HandleTypeDef* const dma = selectedDma();
        return dma != nullptr && dma->Instance != nullptr && dma->Parent == htim_ &&
               dma->Init.Direction == DMA_MEMORY_TO_PERIPH && dma->Init.Mode == DMA_CIRCULAR &&
               dma->Init.PeriphInc == DMA_PINC_DISABLE && dma->Init.MemInc == DMA_MINC_ENABLE &&
               dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_HALFWORD &&
               dma->Init.MemDataAlignment == DMA_MDATAALIGN_HALFWORD;
    }

    [[nodiscard]] bool alignFrameStart() noexcept
    {
        // Runtime frame alignment only. CCR is already zero: CubeMX starts at
        // Pulse=0, and each completed frame ends with an all-zero padding half.
        // An explicit stop/error mid-frame can leave a residual CCR value.
        __HAL_TIM_SET_COUNTER(htim_, 0U);
        if (HAL_TIM_GenerateEvent(htim_, TIM_EVENTSOURCE_UPDATE) != HAL_OK)
            return false;
        __HAL_TIM_CLEAR_FLAG(htim_, TIM_FLAG_UPDATE | (TIM_FLAG_CC1 << channelIndex()));
        return true;
    }

    static void txBarrier() noexcept
    {
#if defined(__CORTEX_M) || defined(__DSB)
        __DSB();
#else
        std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
    }

    // Stop disables the output; board-owned idle levels keep DIN LOW. Do not
    // touch CCR or restart PWM until asynchronous DMA abort has completed.
    bool stopPwmDma() noexcept
    {
        const std::uint32_t     dmaErrors = HAL_DMA_GetError(dma_);
        const HAL_StatusTypeDef stopped   = HAL_TIM_PWM_Stop_DMA(htim_, channel_);
        // Preserve HAL's software error flags if Abort_IT reports no transfer.
        dma_->ErrorCode |= dmaErrors;
        return stopped == HAL_OK;
    }

    void loadPixel() noexcept
    {
        const Color color = pixels_[pixelIndex_];
        pixelWord_        = (static_cast<std::uint32_t>(color.g) << 16U) |
                     (static_cast<std::uint32_t>(color.r) << 8U) | color.b;
        bitMask_ = 1UL << 23U;
    }

    void fillHalf(unsigned half) noexcept
    {
        // True only if EVERY sample was padding after the complete trailing
        // reset. A partially trailing-reset half is deliberately not enough.
        paddingHalf_[half]        = phase_ == Phase::Padding;
        std::uint8_t* const first = buffer_.data() + half * HalfSampleBytes;
        std::size_t i = 0;
        while (i < HalfSampleCount)
        {
            switch (phase_)
            {
            case Phase::LeadingReset:
            case Phase::TrailingReset:
            {
                const std::size_t available = HalfSampleCount - i;
                const std::size_t count = resetRemaining_ < available ? resetRemaining_ : available;
                std::memset(first + i * sizeof(std::uint16_t), 0, count * sizeof(std::uint16_t));
                i += count;
                resetRemaining_ -= count;
                if (resetRemaining_ == 0)
                    phase_ = phase_ == Phase::LeadingReset ? Phase::Pixels : Phase::Padding;
                break;
            }
            case Phase::Pixels:
            {
                if (bitMask_ == 0)
                    loadPixel();
                const std::uint32_t pixel = pixelWord_;
                std::uint32_t mask = bitMask_;
                const std::uint16_t zero = zeroHighTicks_;
                const std::uint16_t one = oneHighTicks_;
                // Keep phase dispatch outside the bit loop; a pixel may span halves.
                do
                {
                    const std::uint16_t sample = (pixel & mask) != 0U ? one : zero;
                    std::memcpy(first + i * sizeof(sample), &sample, sizeof(sample));
                    mask >>= 1U;
                    ++i;
                } while (mask != 0 && i < HalfSampleCount);
                bitMask_ = mask;
                if (mask == 0 && ++pixelIndex_ == N)
                {
                    phase_          = Phase::TrailingReset;
                    resetRemaining_ = ResetPeriods;
                }
                break;
            }
            case Phase::Padding:
                std::memset(first + i * sizeof(std::uint16_t),
                            0,
                            (HalfSampleCount - i) * sizeof(std::uint16_t));
                i = HalfSampleCount;
                break;
            }
        }
        txBarrier();
    }

    void consumed(TIM_HandleTypeDef* htim, unsigned half) noexcept
    {
        if (htim != htim_ || state_.load(std::memory_order_acquire) != State::Sending ||
            static_cast<std::uint32_t>(HAL_TIM_GetActiveChannel(htim)) != (1UL << channelIndex()))
            return;
        if (paddingHalf_[half])
        {
            // An entire extra zero half has now actually traversed DMA, beyond
            // the reset tail and the CCR preload pipeline. Physical frame done.
            state_.store(State::Stopping, std::memory_order_release);
            if (!stopPwmDma())
                state_.store(State::Error, std::memory_order_release);
            return;
        }
        fillHalf(half);
    }

    TIM_HandleTypeDef* const htim_;
    const std::uint32_t      channel_;
    Buffer&                  buffer_;
    std::uint16_t            zeroHighTicks_ = 0;
    std::uint16_t            oneHighTicks_  = 0;
    std::array<Color, N>     pixels_{};
    std::atomic<State>       state_{ State::Idle };
    DMA_HandleTypeDef*       dma_            = nullptr;
    Phase                    phase_          = Phase::LeadingReset;
    std::size_t              resetRemaining_ = 0;
    std::size_t              pixelIndex_     = 0;
    std::uint32_t            pixelWord_      = 0;
    std::uint32_t            bitMask_        = 0;
    bool                     paddingHalf_[2] = { false, false };
};

} // namespace sensors::led

#endif // SENSORS_LED_WS2812B_HPP
