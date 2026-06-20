#include "ADC.h"

#include <PendSV.h>
#include <wiring_private.h>

#ifdef ADC_HAS_D5X_E5X_REGISTERS
#ifndef NVMCTRL_TEMP_LOG
#ifdef NVMCTRL_TEMP_LOG_W0
#define NVMCTRL_TEMP_LOG NVMCTRL_TEMP_LOG_W0
#endif
#endif
#endif

static inline float decToFrac(uint8_t val) {
    // Temperature fuse decimal part is 4 bits only (0-15):
    // 0-9 represent tenths (0.0-0.9), 10-15 represent hundredths (0.10-0.15)
    if (val < 10)
        return static_cast<float>(val) / 10.0f;
    return static_cast<float>(val) / 100.0f;
}

#ifdef ADC_HAS_D5X_E5X_REGISTERS
float analogReadTemperatureC(uint16_t tp, uint16_t tc) {
    const uint32_t roomInt = (*(uint32_t *)FUSES_ROOM_TEMP_VAL_INT_ADDR & FUSES_ROOM_TEMP_VAL_INT_Msk) >>
                             FUSES_ROOM_TEMP_VAL_INT_Pos;
    const uint8_t roomDec = static_cast<uint8_t>((*(uint32_t *)FUSES_ROOM_TEMP_VAL_DEC_ADDR & FUSES_ROOM_TEMP_VAL_DEC_Msk) >>
                            FUSES_ROOM_TEMP_VAL_DEC_Pos);
    const float roomTemp = static_cast<float>(roomInt) + decToFrac(roomDec);

    const uint32_t hotInt = (*(uint32_t *)FUSES_HOT_TEMP_VAL_INT_ADDR & FUSES_HOT_TEMP_VAL_INT_Msk) >>
                            FUSES_HOT_TEMP_VAL_INT_Pos;
    const uint8_t hotDec = static_cast<uint8_t>((*(uint32_t *)FUSES_HOT_TEMP_VAL_DEC_ADDR & FUSES_HOT_TEMP_VAL_DEC_Msk) >>
                           FUSES_HOT_TEMP_VAL_DEC_Pos);
    const float hotTemp = static_cast<float>(hotInt) + decToFrac(hotDec);

    const uint16_t vpl = (*(uint32_t *)FUSES_ROOM_ADC_VAL_PTAT_ADDR & FUSES_ROOM_ADC_VAL_PTAT_Msk) >>
                         FUSES_ROOM_ADC_VAL_PTAT_Pos;
    const uint16_t vph = (*(uint32_t *)FUSES_HOT_ADC_VAL_PTAT_ADDR & FUSES_HOT_ADC_VAL_PTAT_Msk) >>
                         FUSES_HOT_ADC_VAL_PTAT_Pos;
    const uint16_t vcl = (*(uint32_t *)FUSES_ROOM_ADC_VAL_CTAT_ADDR & FUSES_ROOM_ADC_VAL_CTAT_Msk) >>
                         FUSES_ROOM_ADC_VAL_CTAT_Pos;
    const uint16_t vch = (*(uint32_t *)FUSES_HOT_ADC_VAL_CTAT_ADDR & FUSES_HOT_ADC_VAL_CTAT_Msk) >>
                         FUSES_HOT_ADC_VAL_CTAT_Pos;

    return (roomTemp * vph * tc - vpl * hotTemp * tc - roomTemp * vch * tp + hotTemp * vcl * tp) /
           (vcl * tp - vch * tp - vpl * tc + vph * tc);
}
#else
float analogReadTemperatureC(uint16_t adcReading) {
    const uint8_t roomInt =
        static_cast<uint8_t>((*((uint32_t *)FUSES_ROOM_TEMP_VAL_INT_ADDR) & FUSES_ROOM_TEMP_VAL_INT_Msk) >>
                             FUSES_ROOM_TEMP_VAL_INT_Pos);
    const uint8_t roomDec =
        static_cast<uint8_t>((*((uint32_t *)FUSES_ROOM_TEMP_VAL_DEC_ADDR) & FUSES_ROOM_TEMP_VAL_DEC_Msk) >>
                             FUSES_ROOM_TEMP_VAL_DEC_Pos);
    const float roomTemp = static_cast<float>(roomInt) + decToFrac(roomDec);

    const uint8_t hotInt =
        static_cast<uint8_t>((*((uint32_t *)FUSES_HOT_TEMP_VAL_INT_ADDR) & FUSES_HOT_TEMP_VAL_INT_Msk) >>
                             FUSES_HOT_TEMP_VAL_INT_Pos);
    const uint8_t hotDec =
        static_cast<uint8_t>((*((uint32_t *)FUSES_HOT_TEMP_VAL_DEC_ADDR) & FUSES_HOT_TEMP_VAL_DEC_Msk) >>
                             FUSES_HOT_TEMP_VAL_DEC_Pos);
    const float hotTemp = static_cast<float>(hotInt) + decToFrac(hotDec);

    const uint16_t roomAdc =
        static_cast<uint16_t>((*((uint32_t *)FUSES_ROOM_ADC_VAL_ADDR) & FUSES_ROOM_ADC_VAL_Msk) >>
                              FUSES_ROOM_ADC_VAL_Pos);
    const uint16_t hotAdc =
        static_cast<uint16_t>((*((uint32_t *)FUSES_HOT_ADC_VAL_ADDR) & FUSES_HOT_ADC_VAL_Msk) >>
                              FUSES_HOT_ADC_VAL_Pos);

    const int8_t roomInt1vRaw =
        static_cast<int8_t>((*((uint32_t *)FUSES_ROOM_INT1V_VAL_ADDR) & FUSES_ROOM_INT1V_VAL_Msk) >>
                            FUSES_ROOM_INT1V_VAL_Pos);
    const int8_t hotInt1vRaw =
        static_cast<int8_t>((*((uint32_t *)FUSES_HOT_INT1V_VAL_ADDR) & FUSES_HOT_INT1V_VAL_Msk) >>
                            FUSES_HOT_INT1V_VAL_Pos);

    const float roomInt1v = 1.0f - (static_cast<float>(roomInt1vRaw) / 1000.0f);
    const float hotInt1v = 1.0f - (static_cast<float>(hotInt1vRaw) / 1000.0f);

    const float roomVoltageComp = (static_cast<float>(roomAdc) * roomInt1v) / 4095.0f;
    const float hotVoltageComp = (static_cast<float>(hotAdc) * hotInt1v) / 4095.0f;

    const float measurementVoltage = static_cast<float>(adcReading) / 4095.0f;
    const float coarseTemp = roomTemp + (((hotTemp - roomTemp) / (hotVoltageComp - roomVoltageComp)) *
                                         (measurementVoltage - roomVoltageComp));

    const float ref1vAtMeasurement =
        roomInt1v + (((hotInt1v - roomInt1v) * (coarseTemp - roomTemp)) / (hotTemp - roomTemp));
    const float measurementVoltageComp = (static_cast<float>(adcReading) * ref1vAtMeasurement) / 4095.0f;

    return roomTemp + (((hotTemp - roomTemp) / (hotVoltageComp - roomVoltageComp)) *
                       (measurementVoltageComp - roomVoltageComp));
}
#endif

void analogReadCorrection(int offset, uint16_t gain) {
    Adc *adc;
#ifdef ADC_HAS_D5X_E5X_REGISTERS
    adc = ADC0;
#else
    adc = ADC;
#endif

    adc->OFFSETCORR.reg = static_cast<uint16_t>(offset);
    adc->GAINCORR.reg = gain;
    adc->CTRLB.bit.CORREN = 1;

#ifdef ADC_HAS_D5X_E5X_REGISTERS
    while (adc->SYNCBUSY.reg);
#else
    while (adc->STATUS.bit.SYNCBUSY);
#endif
}

namespace {
constexpr uint8_t kInvalidMux = 0xFF;
constexpr uint8_t kMaxAdjres = 4;
constexpr uint8_t kAdcPendSvServiceId = PendSV::kMaxServices - 1;
constexpr uint32_t kAdcNvicPriority = (1u << __NVIC_PRIO_BITS) - 1u;

#ifdef ADC_HAS_D5X_E5X_REGISTERS
#ifdef ADC_EVCTRL_SYNCEI
constexpr uint8_t kAdcEvctrlSynceiBit = ADC_EVCTRL_SYNCEI;
#elif defined(ADC_EVCTRL_FLUSHEI)
constexpr uint8_t kAdcEvctrlSynceiBit = ADC_EVCTRL_FLUSHEI;
#else
constexpr uint8_t kAdcEvctrlSynceiBit = 0u;
#endif
#else
constexpr uint8_t kAdcEvctrlSynceiBit = ADC_EVCTRL_SYNCEI;
#endif

uint8_t adcDmacResrdyTrigger() {
#ifdef ADC_HAS_D5X_E5X_REGISTERS
  return ADC0_DMAC_ID_RESRDY;
#else
  return ADC_DMAC_ID_RESRDY;
#endif
}

inline Adc *adcInstance() {
#ifdef ADC_HAS_D5X_E5X_REGISTERS
  return ADC0;
#else
  return ADC;
#endif
}

// ADC IRQ routing mirrors SERCOM family handling:
// - SAMD2x exposes a single ADC_IRQn vector.
// - SAME/SAMD5x splits ADC0 interrupts across two NVIC lines:
//   ADC0_0 = OVERRUN/WINMON, ADC0_1 = RESRDY.
// Enabling/servicing both split lines ensures window-monitor and data-ready
// callbacks continue to work together on 5x devices.

uint8_t adcIrqCount() {
#ifdef ADC_HAS_D5X_E5X_REGISTERS
    return 2;
#else
    return 1;
#endif
}

IRQn_Type adcIrqAt(uint8_t index) {
#ifdef ADC_HAS_D5X_E5X_REGISTERS
    return (index == 0) ? ADC0_0_IRQn : ADC0_1_IRQn;
#else
    (void)index;
    return ADC_IRQn;
#endif
}

void adcPendSvService(uint8_t serviceId, void *context) {
    (void)serviceId;
    (void)context;
    AdcEngine::instance().onPendSv();
}

uint8_t sampleNumCodeToAdjres(uint8_t sampleNumCode) {
    if (sampleNumCode == 0)
        return 0;
    if (sampleNumCode > kMaxAdjres)
        return kMaxAdjres;
    return sampleNumCode;
}

uint8_t sampleNumToCode(AdcSampleNum sampleNum) {
    return static_cast<uint8_t>(sampleNum);
}

uint8_t pinToMux(uint8_t arduinoPin) {
    if (arduinoPin >= PINS_COUNT)
        return kInvalidMux;

    const EAnalogChannel channel = g_APinDescription[arduinoPin].ulADCChannelNumber;
    if (channel == No_ADC_Channel)
        return kInvalidMux;

    return static_cast<uint8_t>(channel);
}

void configureAnalogPin(uint8_t arduinoPin) {
    if (arduinoPin >= PINS_COUNT)
        return;

    const EAnalogChannel channel = g_APinDescription[arduinoPin].ulADCChannelNumber;
    if (channel == No_ADC_Channel)
        return;

    pinPeripheral(arduinoPin, PIO_ANALOG);
}
} // namespace

uint8_t ADC_MUXPOS_PIN(uint8_t pin) {
    const uint8_t mux = pinToMux(pin);
    if (mux == kInvalidMux)
        return ADC_MUX_SOURCE_INVALID;

    return mux;
}

uint8_t ADC_MUXNEG_PIN(uint8_t pin) {
    const uint8_t mux = pinToMux(pin);
    if (mux == kInvalidMux)
        return ADC_MUX_SOURCE_INVALID;

    return mux;
}

AdcEngine &AdcEngine::instance() {
    static AdcEngine adcEngine;
    return adcEngine;
}

bool AdcEngine::begin() {
    if (initialized_)
        return true;

    queueHead_ = 0;
    queueTail_ = 0;
    queueCount_ = 0;
    activeChannel_ = nullptr;
    activeMonitorMode_ = false;
    pendingChannel_ = nullptr;
    pendingCallback_ = nullptr;
    pendingUserData_ = nullptr;
    pendingResult_ = 0;
    enabledMask_ = 0;
    registeredCount_ = 0;
    monitorCursor_ = 0;

    for (uint8_t i = 0; i < kResultContainerSize; ++i) {
        queue_[i] = nullptr;
        resultContainer_[i] = 0;
        registeredChannels_[i] = nullptr;
    }

    if (!PendSV::instance().registerService(kAdcPendSvServiceId, adcPendSvService, nullptr))
        return false;

    if (dma_.allocate() != DMA_STATUS_OK) {
        PendSV::instance().clearService(kAdcPendSvServiceId);
        return false;
    }

    dma_.setTrigger(adcDmacResrdyTrigger());
    dma_.setAction(DMA_TRIGGER_ACTON_BEAT);
    dma_.setCallback(AdcEngine::dmaDoneCallback, DMA_CALLBACK_TRANSFER_DONE);
    Adc *const adc = adcInstance();

    dmaDescriptor_ =
        dma_.addDescriptor((void *)&adc->RESULT.reg, (void *)&dmaLatestResult_,
                           1, DMA_BEAT_SIZE_HWORD, false, false);
    if (dmaDescriptor_ == nullptr) {
        dma_.free();
        PendSV::instance().clearService(kAdcPendSvServiceId);
        return false;
    }

    for (uint8_t i = 0; i < adcIrqCount(); ++i) {
        IRQn_Type irq = adcIrqAt(i);
        NVIC_DisableIRQ(irq);
        NVIC_ClearPendingIRQ(irq);
        NVIC_SetPriority(irq, kAdcNvicPriority);
        NVIC_EnableIRQ(irq);
    }

    adc->INTFLAG.reg = ADC_INTFLAG_RESRDY | ADC_INTFLAG_WINMON;
    adc->INTENSET.bit.RESRDY = 1;
    initialized_ = true;
    return true;
}

void AdcEngine::end() {
    if (!initialized_)
        return;

    for (uint8_t i = 0; i < adcIrqCount(); ++i) {
        IRQn_Type irq = adcIrqAt(i);
        NVIC_DisableIRQ(irq);
        NVIC_ClearPendingIRQ(irq);
    }
    Adc *const adc = adcInstance();

    adc->INTENCLR.bit.RESRDY = 1;

    if (dmaActive_)
        dma_.abort();

    dma_.free();
    dmaDescriptor_ = nullptr;

    adc->CTRLA.bit.SWRST = 1;
    while (adc->CTRLA.bit.SWRST)
      ;
    waitAdcSync();

    PendSV::instance().clearService(kAdcPendSvServiceId);

    initialized_ = false;
    dmaActive_ = false;
    activeChannel_ = nullptr;
    activeMonitorMode_ = false;
    pendingChannel_ = nullptr;
    pendingCallback_ = nullptr;
    pendingUserData_ = nullptr;
    pendingResult_ = 0;
    enabledMask_ = 0;
    queueHead_ = 0;
    queueTail_ = 0;
    queueCount_ = 0;
    registeredCount_ = 0;
    monitorCursor_ = 0;

    for (uint8_t i = 0; i < kResultContainerSize; ++i)
        registeredChannels_[i] = nullptr;
}

bool AdcEngine::enqueue(ChannelADC *channel) {
    if (!initialized_ || channel == nullptr)
        return false;

    const int8_t index = muxPosToResultIndex(channel->muxPos_);
    if (index < 0 || channel->enqueued_)
        return false;

    if (!pushQueue(channel))
        return false;

    channel->enqueued_ = true;
    enabledMask_ |= (1u << channel->muxPos_);

    if (activeChannel_ == nullptr) {
        ChannelADC *next = popQueue();
        if (next != nullptr && !applyChannelAndStart(next)) {
            next->enqueued_ = false;
            enabledMask_ &= ~(1u << next->muxPos_);
            return false;
        }
    }

    return true;
}

bool AdcEngine::registerChannel(ChannelADC *channel) {
    if (channel == nullptr)
        return false;

    for (uint8_t i = 0; i < registeredCount_; ++i) {
        if (registeredChannels_[i] == channel)
            return true;
    }

    if (registeredCount_ >= kResultContainerSize)
        return false;

    registeredChannels_[registeredCount_++] = channel;
    return true;
}

void AdcEngine::unregisterChannel(ChannelADC *channel) {
    if (channel == nullptr || registeredCount_ == 0)
        return;

    for (uint8_t i = 0; i < registeredCount_; ++i) {
        if (registeredChannels_[i] != channel)
            continue;

        for (uint8_t j = i; j + 1 < registeredCount_; ++j)
            registeredChannels_[j] = registeredChannels_[j + 1];

        registeredChannels_[registeredCount_ - 1] = nullptr;
        --registeredCount_;
        if (monitorCursor_ >= registeredCount_)
            monitorCursor_ = 0;
        return;
    }
}

void AdcEngine::service() {
    if (!initialized_)
        return;

    onPendSv();

    if (activeChannel_ == nullptr) {
        ChannelADC *next = popQueue();
        if (next != nullptr && !applyChannelAndStart(next)) {
            next->enqueued_ = false;
            enabledMask_ &= ~(1u << next->muxPos_);
        }
    }

    if (activeChannel_ == nullptr && queueCount_ == 0)
        startMonitorIfIdle();
}

uint8_t AdcEngine::resultByteForMux(uint8_t muxPos) const {
    const int8_t index = muxPosToResultIndex(muxPos);
    if (index < 0)
        return 0;

    return resultContainer_[index];
}

uint32_t AdcEngine::enabledMask() const {
    return enabledMask_;
}

void AdcEngine::onResrdyIsr() {
  Adc *const adc = adcInstance();
  const uint8_t flags = static_cast<uint8_t>(adc->INTFLAG.reg & 0x0F);

  if ((flags & ADC_INTFLAG_WINMON) != 0u) {
    ChannelADC *channel = activeChannel_;
    if (channel != nullptr && channel->windowEnabled_ &&
        channel->onWindowMonitor_ != nullptr) {
      const uint16_t result = adc->RESULT.reg;
      channel->onWindowMonitor_(channel, result, channel->windowUserData_);
    }
  }

    if ((flags & ADC_INTFLAG_RESRDY) != 0u)
      adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;

    adc->INTFLAG.reg = 0x0F;
}

void AdcEngine::onPendSv() {
    if (!pendSvPending_)
        return;

    pendSvPending_ = false;

    if (pendingCallback_ != nullptr && pendingChannel_ != nullptr)
        pendingCallback_(pendingChannel_, pendingResult_, pendingUserData_);

    pendingCallback_ = nullptr;
    pendingChannel_ = nullptr;
    pendingUserData_ = nullptr;
    pendingResult_ = 0;
}

int8_t AdcEngine::muxPosToResultIndex(uint8_t muxPos) {
    if (muxPos > kMuxMax)
        return -1;

    if (muxPos >= kSkippedMuxStart && muxPos <= kSkippedMuxEnd)
        return -1;

    if (muxPos < kSkippedMuxStart)
        return static_cast<int8_t>(muxPos);

    return static_cast<int8_t>(muxPos - (kSkippedMuxEnd - kSkippedMuxStart + 1));
}

bool AdcEngine::pushQueue(ChannelADC *channel) {
    if (queueCount_ >= kResultContainerSize)
        return false;

    queue_[queueTail_] = channel;
    queueTail_ = static_cast<uint8_t>((queueTail_ + 1) % kResultContainerSize);
    ++queueCount_;
    return true;
}

ChannelADC *AdcEngine::popQueue() {
    if (queueCount_ == 0)
        return nullptr;

    ChannelADC *channel = queue_[queueHead_];
    queue_[queueHead_] = nullptr;
    queueHead_ = static_cast<uint8_t>((queueHead_ + 1) % kResultContainerSize);
    --queueCount_;
    return channel;
}

bool AdcEngine::applyChannelAndStart(ChannelADC *channel, bool monitorMode) {
    if (channel == nullptr)
        return false;

    Adc *const adc = adcInstance();

    adc->CTRLA.bit.ENABLE = 0;
    waitAdcSync();

    const uint8_t refctrlReg = static_cast<uint8_t>(ADC_REFCTRL_REFSEL(channel->refSel_));
    adc->REFCTRL.reg = refctrlReg;

#ifdef ADC_HAS_D5X_E5X_REGISTERS
    const uint16_t inputCtrlReg = static_cast<uint16_t>(
        ADC_INPUTCTRL_MUXPOS(channel->muxPos_) | ADC_INPUTCTRL_MUXNEG(channel->muxNeg_) |
        (channel->differentialMode_ ? ADC_INPUTCTRL_DIFFMODE : 0u));
    adc->INPUTCTRL.reg = inputCtrlReg;

    const uint8_t avgCtrlReg =
        static_cast<uint8_t>(ADC_AVGCTRL_SAMPLENUM(sampleNumToCode(channel->sampleNum_)) |
                             ADC_AVGCTRL_ADJRES(channel->adjres_));
    adc->AVGCTRL.reg = avgCtrlReg;

    uint16_t ctrlaReg = adc->CTRLA.reg;
    ctrlaReg =
        static_cast<uint16_t>((ctrlaReg & ~ADC_CTRLA_PRESCALER_Msk) |
                              ADC_CTRLA_PRESCALER(static_cast<uint8_t>(channel->prescaler_)));
    adc->CTRLA.reg = ctrlaReg;

    const uint16_t ctrlbReg = static_cast<uint16_t>(
        (channel->leftAdjust_ ? ADC_CTRLB_LEFTADJ : 0u) |
        (channel->freeRun_ ? ADC_CTRLB_FREERUN : 0u) |
        (channel->corrEnabled_ ? ADC_CTRLB_CORREN : 0u) |
        ADC_CTRLB_RESSEL(static_cast<uint8_t>(channel->ressel_)) |
        ADC_CTRLB_WINMODE(channel->windowEnabled_
                              ? static_cast<uint8_t>(channel->winMode_)
                              : static_cast<uint8_t>(AdcWinMode::ADC_WINMODE_DISABLE)));
    adc->CTRLB.reg = ctrlbReg;

    const uint8_t evctrlReg =
        static_cast<uint8_t>((channel->evWinmonEo_ ? ADC_EVCTRL_WINMONEO : 0u) |
                             (channel->evResrdyEo_ ? ADC_EVCTRL_RESRDYEO : 0u) |
                             (channel->evSyncei_ ? kAdcEvctrlSynceiBit : 0u) |
                             (channel->evStartei_ ? ADC_EVCTRL_STARTEI : 0u));
    adc->EVCTRL.reg = evctrlReg;

    adc->WINLT.reg = channel->windowLower_;
    adc->WINUT.reg = channel->windowUpper_;

    adc->OFFSETCORR.reg = static_cast<uint16_t>(channel->offsetCorr_);
    adc->GAINCORR.reg = channel->gainCorr_;
#else
    const uint32_t inputCtrlReg = static_cast<uint32_t>(
        ADC_INPUTCTRL_MUXPOS(channel->muxPos_) | ADC_INPUTCTRL_MUXNEG(channel->muxNeg_) |
        ADC_INPUTCTRL_GAIN(static_cast<uint8_t>(channel->gain_)));
    adc->INPUTCTRL.reg = inputCtrlReg;

    const uint8_t avgCtrlReg =
        static_cast<uint8_t>(ADC_AVGCTRL_SAMPLENUM(sampleNumToCode(channel->sampleNum_)) |
                             ADC_AVGCTRL_ADJRES(channel->adjres_));
    adc->AVGCTRL.reg = avgCtrlReg;

    const uint16_t ctrlbReg =
        static_cast<uint16_t>((channel->differentialMode_ ? ADC_CTRLB_DIFFMODE : 0u) |
                              (channel->leftAdjust_ ? ADC_CTRLB_LEFTADJ : 0u) |
                              (channel->freeRun_ ? ADC_CTRLB_FREERUN : 0u) |
                              (channel->corrEnabled_ ? ADC_CTRLB_CORREN : 0u) |
                              ADC_CTRLB_RESSEL(static_cast<uint8_t>(channel->ressel_)) |
                              ADC_CTRLB_PRESCALER(static_cast<uint8_t>(channel->prescaler_)));
    adc->CTRLB.reg = ctrlbReg;

    const uint8_t evctrlReg =
        static_cast<uint8_t>((channel->evWinmonEo_ ? ADC_EVCTRL_WINMONEO : 0u) |
                             (channel->evResrdyEo_ ? ADC_EVCTRL_RESRDYEO : 0u) |
                             (channel->evSyncei_ ? kAdcEvctrlSynceiBit : 0u) |
                             (channel->evStartei_ ? ADC_EVCTRL_STARTEI : 0u));
    adc->EVCTRL.reg = evctrlReg;

    adc->WINLT.reg = channel->windowLower_;
    adc->WINUT.reg = channel->windowUpper_;

    const uint8_t winctrlReg = static_cast<uint8_t>(
        ADC_WINCTRL_WINMODE(channel->windowEnabled_ ? static_cast<uint8_t>(channel->winMode_) : 0));
    adc->WINCTRL.reg = winctrlReg;

    adc->OFFSETCORR.reg = static_cast<uint16_t>(channel->offsetCorr_);
    adc->GAINCORR.reg = channel->gainCorr_;
#endif

    waitAdcSync();

    adc->CTRLA.bit.ENABLE = 1;
    waitAdcSync();

    // The first conversion after changing the mux/reference can be stale.
    // Match Arduino analogRead() by discarding it before arming DMA.
    adc->INTENCLR.reg = 0x0F;
    adc->INTFLAG.reg = 0x0F;
    startConversion();
    while ((adc->INTFLAG.reg & ADC_INTFLAG_RESRDY) == 0u)
      ;
    adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;

    const uint8_t intensetReg =
        static_cast<uint8_t>(ADC_INTENSET_RESRDY |
                             ((monitorMode && channel->windowEnabled_) ? ADC_INTENSET_WINMON : 0u));
    adc->INTENSET.reg = intensetReg;

    adc->INTFLAG.reg = 0x0F;

    if (dmaDescriptor_ == nullptr)
        return false;

    dma_.changeDescriptor(dmaDescriptor_, (void *)&adc->RESULT.reg,
                          (void *)&dmaLatestResult_, 1);
    if (dma_.startJob() != DMA_STATUS_OK)
        return false;

    dmaActive_ = true;
    activeChannel_ = channel;
    activeMonitorMode_ = monitorMode;

    startConversion();
    return true;
}

bool AdcEngine::startMonitorIfIdle() {
    if (!initialized_ || activeChannel_ != nullptr || registeredCount_ == 0)
        return false;

    for (uint8_t i = 0; i < registeredCount_; ++i) {
        const uint8_t idx = static_cast<uint8_t>((monitorCursor_ + i) % registeredCount_);
        ChannelADC *candidate = registeredChannels_[idx];
        if (candidate == nullptr || !candidate->initialized_ || !candidate->monitorConfigured())
            continue;

        monitorCursor_ = static_cast<uint8_t>((idx + 1) % registeredCount_);
        return applyChannelAndStart(candidate, true);
    }

    return false;
}

void AdcEngine::waitAdcSync() const {
  Adc *const adc = adcInstance();
#ifdef ADC_HAS_D5X_E5X_REGISTERS
    while (adc->SYNCBUSY.reg)
      ;
#else
    while (adc->STATUS.bit.SYNCBUSY)
      ;
#endif
}

void AdcEngine::dmaDoneCallback(Adafruit_ZeroDMA *dma) {
    (void)dma;

    AdcEngine &engine = AdcEngine::instance();
    engine.dmaActive_ = false;

    ChannelADC *channel = engine.activeChannel_;
    if (channel == nullptr)
        return;

    const uint16_t result = engine.dmaLatestResult_;
    const int8_t resultIndex = muxPosToResultIndex(channel->muxPos_);
    if (resultIndex >= 0)
        engine.resultContainer_[resultIndex] = static_cast<uint8_t>(result & 0xFF);

    channel->value_ = result;
    channel->hasFreshValue_ = true;

    if (engine.activeMonitorMode_) {
        engine.activeChannel_ = nullptr;
        engine.activeMonitorMode_ = false;
        return;
    }

    channel->enqueued_ = false;
    engine.enabledMask_ &= ~(1u << channel->muxPos_);

    engine.pendingChannel_ = channel;
    engine.pendingResult_ = result;
    engine.pendingCallback_ = channel->onReadComplete_;
    engine.pendingUserData_ = channel->userData_;
    engine.activeChannel_ = nullptr;
    engine.activeMonitorMode_ = false;

    engine.pendSvPending_ = true;
    PendSV::instance().setPending(kAdcPendSvServiceId);
}

bool ChannelADC::setAttachedSources(uint8_t muxPos, uint8_t muxNeg, AdcSampleNum sampleNum) {
    if (muxPos == ADC_MUX_SOURCE_INVALID || muxNeg == ADC_MUX_SOURCE_INVALID)
        return false;

    if (muxPos == kInvalidMux || muxNeg == kInvalidMux)
        return false;

    muxPos_ = muxPos;
    muxNeg_ = muxNeg;
    if (muxNeg_ != static_cast<uint8_t>(AdcMuxNeg::ADC_MUXNEG_GND))
        differentialMode_ = true;
    sampleNum_ = sampleNum;
    adjres_ = sampleNumCodeToAdjres(sampleNumToCode(sampleNum));
    attached_ = true;
    return true;
}

bool ChannelADC::attach(uint8_t pinPos, uint8_t pinNeg, AdcSampleNum sampleNum) {
    AdcEngine &engine = AdcEngine::instance();
    if (!engine.begin())
        return false;

    if (!registered_ && !engine.registerChannel(this))
        return false;

    configureAnalogPin(pinPos);
    configureAnalogPin(pinNeg);

    registered_ = true;
    initialized_ = true;
    return setAttachedSources(ADC_MUXPOS_PIN(pinPos), ADC_MUXNEG_PIN(pinNeg), sampleNum);
}

bool ChannelADC::attach(uint8_t pinPos, AdcMuxNeg muxNeg, AdcSampleNum sampleNum) {
    AdcEngine &engine = AdcEngine::instance();
    if (!engine.begin())
        return false;

    if (!registered_ && !engine.registerChannel(this))
        return false;

    configureAnalogPin(pinPos);

    registered_ = true;
    initialized_ = true;
    return setAttachedSources(ADC_MUXPOS_PIN(pinPos), static_cast<uint8_t>(muxNeg), sampleNum);
}

bool ChannelADC::attach(AdcMuxPos muxPos, uint8_t pinNeg, AdcSampleNum sampleNum) {
    AdcEngine &engine = AdcEngine::instance();
    if (!engine.begin())
        return false;

    if (!registered_ && !engine.registerChannel(this))
        return false;

    configureAnalogPin(pinNeg);

    registered_ = true;
    initialized_ = true;
    return setAttachedSources(static_cast<uint8_t>(muxPos), ADC_MUXNEG_PIN(pinNeg), sampleNum);
}

bool ChannelADC::attach(AdcMuxPos muxPos, AdcMuxNeg muxNeg, AdcSampleNum sampleNum) {
    AdcEngine &engine = AdcEngine::instance();
    if (!engine.begin())
        return false;

    if (!registered_ && !engine.registerChannel(this))
        return false;

    registered_ = true;
    initialized_ = true;
    return setAttachedSources(static_cast<uint8_t>(muxPos), static_cast<uint8_t>(muxNeg),
                              sampleNum);
}

void ChannelADC::setWindow(AdcWinMode mode, uint16_t lower, uint16_t upper) {
    windowLower_ = lower;
    windowUpper_ = upper;
    winMode_ = mode;
    windowEnabled_ = (mode != AdcWinMode::ADC_WINMODE_DISABLE);
}

void ChannelADC::clearWindow() {
    windowEnabled_ = false;
    winMode_ = AdcWinMode::ADC_WINMODE_DISABLE;
}

bool ChannelADC::windowEnabled() const {
    return windowEnabled_;
}

void ChannelADC::setWindowCallback(Callback callback, void *userData) {
    onWindowMonitor_ = callback;
    windowUserData_ = (userData != nullptr) ? userData : this;
}

void ChannelADC::setReadCallback(Callback callback, void *userData) {
    onReadComplete_ = callback;
    userData_ = userData;
}

void ChannelADC::setEventControl(bool winmonEo, bool resrdyEo, bool syncei, bool startei) {
    evWinmonEo_ = winmonEo;
    evResrdyEo_ = resrdyEo;
    evSyncei_ = syncei;
    evStartei_ = startei;
}

void ChannelADC::setCtrlB(AdcResSel resolution, AdcPrescaler prescaler, bool freeRun,
                          bool leftAdjust, bool differentialMode, bool correctionEnable) {
    ressel_ = resolution;
    prescaler_ = prescaler;
    freeRun_ = freeRun;
    leftAdjust_ = leftAdjust;
    differentialMode_ = differentialMode;
    corrEnabled_ = correctionEnable;
}

void ChannelADC::setReference(AdcRefSel reference) {
    refSel_ = static_cast<uint8_t>(reference);
}

void ChannelADC::setGain(AdcGain gain) {
    gain_ = gain;
}

void ChannelADC::setCalibration(uint16_t gainCorr, int16_t offsetCorr, bool enableCorrection) {
    gainCorr_ = gainCorr;
    offsetCorr_ = offsetCorr;
    corrEnabled_ = enableCorrection;
}

bool ChannelADC::monitorConfigured() const {
    return windowEnabled_;
}

bool ChannelADC::read() {
    if (!initialized_ || !attached_)
        return false;

    hasFreshValue_ = false;

    AdcEngine &engine = AdcEngine::instance();
    if (!engine.enqueue(this))
        return false;

    // No callback means caller requested blocking/synchronous behavior.
    if (onReadComplete_ == nullptr) {
        while (!hasFreshValue_)
            engine.service();
    }

    return true;
}

uint16_t ChannelADC::value() const {
    return value_;
}

void ChannelADC::end() {
    initialized_ = false;
    attached_ = false;
    registered_ = false;
    enqueued_ = false;
    hasFreshValue_ = false;
    AdcEngine::instance().unregisterChannel(this);
}

#ifdef ADC_HAS_D5X_E5X_REGISTERS
extern "C" void ADC0_0_Handler(void) {
    AdcEngine::instance().onResrdyIsr();
}

extern "C" void ADC0_1_Handler(void) {
    AdcEngine::instance().onResrdyIsr();
}
#else
extern "C" void ADC_Handler(void) {
    AdcEngine::instance().onResrdyIsr();
}
#endif
