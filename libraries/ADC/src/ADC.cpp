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
#ifdef ADC_HAS_SAME53_E54_REGISTERS
    const uint32_t tempLog0 = *(uint32_t *)(TEMP_LOG_ADDR + 0u);
    const uint32_t tempLog1 = *(uint32_t *)(TEMP_LOG_ADDR + 4u);
    const uint32_t tempLog2 = *(uint32_t *)(TEMP_LOG_ADDR + 8u);

    const uint32_t roomInt =
        (tempLog0 & FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_INT_Msk) >>
        FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_INT_Pos;
    const uint8_t roomDec = static_cast<uint8_t>(
        (tempLog0 & FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_DEC_Msk) >>
        FUSES_TEMP_LOG_WORD_0_ROOM_TEMP_VAL_DEC_Pos);
    const float roomTemp = static_cast<float>(roomInt) + decToFrac(roomDec);

    const uint32_t hotInt =
        (tempLog0 & FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_INT_Msk) >>
        FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_INT_Pos;
    const uint8_t hotDec = static_cast<uint8_t>(
        (tempLog0 & FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_DEC_Msk) >>
        FUSES_TEMP_LOG_WORD_0_HOT_TEMP_VAL_DEC_Pos);
    const float hotTemp = static_cast<float>(hotInt) + decToFrac(hotDec);

    const uint16_t vpl =
        (tempLog1 & FUSES_TEMP_LOG_WORD_1_ROOM_ADC_VAL_PTAT_Msk) >>
        FUSES_TEMP_LOG_WORD_1_ROOM_ADC_VAL_PTAT_Pos;
    const uint16_t vph =
        (tempLog1 & FUSES_TEMP_LOG_WORD_1_HOT_ADC_VAL_PTAT_Msk) >>
        FUSES_TEMP_LOG_WORD_1_HOT_ADC_VAL_PTAT_Pos;
    const uint16_t vcl =
        (tempLog2 & FUSES_TEMP_LOG_WORD_2_ROOM_ADC_VAL_CTAT_Msk) >>
        FUSES_TEMP_LOG_WORD_2_ROOM_ADC_VAL_CTAT_Pos;
    const uint16_t vch =
        (tempLog2 & FUSES_TEMP_LOG_WORD_2_HOT_ADC_VAL_CTAT_Msk) >>
        FUSES_TEMP_LOG_WORD_2_HOT_ADC_VAL_CTAT_Pos;
#else
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
#endif // ADC_HAS_SAME53_E54_REGISTERS

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
#ifdef ADC_HAS_SAME53_E54_REGISTERS
    adc_registers_t *adc = ADC0_REGS;

    adc->ADC_OFFSETCORR = static_cast<uint16_t>(offset);
    adc->ADC_GAINCORR = gain;
    adc->ADC_CTRLB |= ADC_CTRLB_CORREN_Msk;

    while (adc->ADC_SYNCBUSY);
#else
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
#endif // ADC_HAS_SAME53_E54_REGISTERS
}

namespace {
constexpr uint8_t kMaxAdjres = 4;
constexpr uint8_t kAdcPendSvServiceId = PendSVChannels::Adc;
constexpr uint32_t kAdcNvicPriority = (1u << __NVIC_PRIO_BITS) - 1u;

#ifdef ADC_HAS_D5X_E5X_REGISTERS
#ifdef ADC_HAS_SAME53_E54_REGISTERS
constexpr uint16_t kAdcInputctrlDiffmodeBit = ADC_INPUTCTRL_DIFFMODE_Msk;
constexpr uint16_t kAdcCtrlbLeftadjBit = ADC_CTRLB_LEFTADJ_Msk;
constexpr uint16_t kAdcCtrlbFreerunBit = ADC_CTRLB_FREERUN_Msk;
constexpr uint16_t kAdcCtrlbCorrenBit = ADC_CTRLB_CORREN_Msk;
constexpr uint8_t kAdcEvctrlWinmoneoBit = ADC_EVCTRL_WINMONEO_Msk;
constexpr uint8_t kAdcEvctrlResrdyeoBit = ADC_EVCTRL_RESRDYEO_Msk;
constexpr uint8_t kAdcEvctrlStarteiBit = ADC_EVCTRL_STARTEI_Msk;
#else
constexpr uint16_t kAdcInputctrlDiffmodeBit = ADC_INPUTCTRL_DIFFMODE;
constexpr uint16_t kAdcCtrlbLeftadjBit = ADC_CTRLB_LEFTADJ;
constexpr uint16_t kAdcCtrlbFreerunBit = ADC_CTRLB_FREERUN;
constexpr uint16_t kAdcCtrlbCorrenBit = ADC_CTRLB_CORREN;
constexpr uint8_t kAdcEvctrlWinmoneoBit = ADC_EVCTRL_WINMONEO;
constexpr uint8_t kAdcEvctrlResrdyeoBit = ADC_EVCTRL_RESRDYEO;
constexpr uint8_t kAdcEvctrlStarteiBit = ADC_EVCTRL_STARTEI;
#endif

#ifdef ADC_EVCTRL_SYNCEI
constexpr uint8_t kAdcEvctrlSynceiBit = ADC_EVCTRL_SYNCEI;
#elif defined(ADC_EVCTRL_FLUSHEI_Msk)
constexpr uint8_t kAdcEvctrlSynceiBit = ADC_EVCTRL_FLUSHEI_Msk;
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

#ifdef ADC_HAS_SAME53_E54_REGISTERS
inline adc_registers_t *adcInstance() {
    return ADC0_REGS;
}

inline void *adcResultRegister(adc_registers_t *adc) {
    return (void *)&adc->ADC_RESULT;
}

inline uint16_t adcReadResult(adc_registers_t *adc) {
    return adc->ADC_RESULT;
}

inline void adcSetEnable(adc_registers_t *adc, bool enabled) {
    if (enabled)
        adc->ADC_CTRLA |= ADC_CTRLA_ENABLE_Msk;
    else
        adc->ADC_CTRLA &= ~ADC_CTRLA_ENABLE_Msk;
}

inline void adcSoftwareReset(adc_registers_t *adc) {
    adc->ADC_CTRLA |= ADC_CTRLA_SWRST_Msk;
}

inline bool adcSoftwareResetBusy(adc_registers_t *adc) {
    return (adc->ADC_SYNCBUSY & ADC_SYNCBUSY_SWRST_Msk) != 0;
}

inline void adcDisableResultReady(adc_registers_t *adc) {
    adc->ADC_INTENCLR = ADC_INTENCLR_RESRDY_Msk;
}

inline uint8_t adcReadFlags(adc_registers_t *adc) {
    return static_cast<uint8_t>(adc->ADC_INTFLAG & ADC_INTFLAG_Msk);
}

inline void adcClearFlags(adc_registers_t *adc, uint8_t flags) {
    adc->ADC_INTFLAG = flags;
}

inline bool adcFlagWinmon(uint8_t flags) {
    return (flags & ADC_INTFLAG_WINMON_Msk) != 0u;
}

inline bool adcFlagResrdy(uint8_t flags) {
    return (flags & ADC_INTFLAG_RESRDY_Msk) != 0u;
}

inline void adcDisableInterrupts(adc_registers_t *adc, uint8_t mask) {
    adc->ADC_INTENCLR = mask;
}

inline void adcEnableInterrupts(adc_registers_t *adc, uint8_t mask) {
    adc->ADC_INTENSET = mask;
}

inline void adcWriteRefctrl(adc_registers_t *adc, uint8_t value) {
    adc->ADC_REFCTRL = value;
}

inline void adcWriteInputctrl(adc_registers_t *adc, uint16_t value) {
    adc->ADC_INPUTCTRL = value;
}

inline void adcWriteAvgctrl(adc_registers_t *adc, uint8_t value) {
    adc->ADC_AVGCTRL = value;
}

inline void adcWriteSampctrl(adc_registers_t *adc, uint8_t value) {
    adc->ADC_SAMPCTRL = value;
}

inline uint16_t adcReadCtrla(adc_registers_t *adc) {
    return adc->ADC_CTRLA;
}

inline void adcWriteCtrla(adc_registers_t *adc, uint16_t value) {
    adc->ADC_CTRLA = value;
}

inline void adcWriteCtrlb(adc_registers_t *adc, uint16_t value) {
    adc->ADC_CTRLB = value;
}

inline void adcWriteEvctrl(adc_registers_t *adc, uint8_t value) {
    adc->ADC_EVCTRL = value;
}

inline void adcWriteWinlt(adc_registers_t *adc, uint16_t value) {
    adc->ADC_WINLT = value;
}

inline void adcWriteWinut(adc_registers_t *adc, uint16_t value) {
    adc->ADC_WINUT = value;
}

inline void adcWriteOffsetcorr(adc_registers_t *adc, uint16_t value) {
    adc->ADC_OFFSETCORR = value;
}

inline void adcWriteGaincorr(adc_registers_t *adc, uint16_t value) {
    adc->ADC_GAINCORR = value;
}

inline void adcStartConversion(adc_registers_t *adc) {
    adc->ADC_SWTRIG = ADC_SWTRIG_START_Msk;
}
#else
inline Adc *adcInstance() {
#ifdef ADC_HAS_D5X_E5X_REGISTERS
  return ADC0;
#else
  return ADC;
#endif
}

inline void *adcResultRegister(Adc *adc) {
    return (void *)&adc->RESULT.reg;
}

inline uint16_t adcReadResult(Adc *adc) {
    return adc->RESULT.reg;
}

inline void adcSetEnable(Adc *adc, bool enabled) {
    adc->CTRLA.bit.ENABLE = enabled ? 1 : 0;
}

inline void adcSoftwareReset(Adc *adc) {
    adc->CTRLA.bit.SWRST = 1;
}

inline bool adcSoftwareResetBusy(Adc *adc) {
    return adc->CTRLA.bit.SWRST;
}

inline void adcDisableResultReady(Adc *adc) {
    adc->INTENCLR.bit.RESRDY = 1;
}

inline uint8_t adcReadFlags(Adc *adc) {
    return static_cast<uint8_t>(adc->INTFLAG.reg & 0x0F);
}

inline void adcClearFlags(Adc *adc, uint8_t flags) {
    adc->INTFLAG.reg = flags;
}

inline bool adcFlagWinmon(uint8_t flags) {
    return (flags & ADC_INTFLAG_WINMON) != 0u;
}

inline bool adcFlagResrdy(uint8_t flags) {
    return (flags & ADC_INTFLAG_RESRDY) != 0u;
}

inline void adcDisableInterrupts(Adc *adc, uint8_t mask) {
    adc->INTENCLR.reg = mask;
}

inline void adcEnableInterrupts(Adc *adc, uint8_t mask) {
    adc->INTENSET.reg = mask;
}

inline void adcWriteRefctrl(Adc *adc, uint8_t value) {
    adc->REFCTRL.reg = value;
}

inline void adcWriteInputctrl(Adc *adc, uint32_t value) {
    adc->INPUTCTRL.reg = value;
}

inline void adcWriteAvgctrl(Adc *adc, uint8_t value) {
    adc->AVGCTRL.reg = value;
}

inline void adcWriteSampctrl(Adc *adc, uint8_t value) {
#ifdef ADC_HAS_D5X_E5X_REGISTERS
    adc->SAMPCTRL.reg = value;
#else
    (void)adc;
    (void)value;
#endif
}

inline uint16_t adcReadCtrla(Adc *adc) {
    return adc->CTRLA.reg;
}

inline void adcWriteCtrla(Adc *adc, uint16_t value) {
    adc->CTRLA.reg = value;
}

inline void adcWriteCtrlb(Adc *adc, uint16_t value) {
    adc->CTRLB.reg = value;
}

inline void adcWriteEvctrl(Adc *adc, uint8_t value) {
    adc->EVCTRL.reg = value;
}

inline void adcWriteWinlt(Adc *adc, uint16_t value) {
    adc->WINLT.reg = value;
}

inline void adcWriteWinut(Adc *adc, uint16_t value) {
    adc->WINUT.reg = value;
}

inline void adcWriteOffsetcorr(Adc *adc, uint16_t value) {
    adc->OFFSETCORR.reg = value;
}

inline void adcWriteGaincorr(Adc *adc, uint16_t value) {
    adc->GAINCORR.reg = value;
}

inline void adcStartConversion(Adc *adc) {
    adc->SWTRIG.bit.START = 1;
}
#endif // ADC_HAS_SAME53_E54_REGISTERS

#ifdef ADC_HAS_D5X_E5X_REGISTERS
void configureInternalReference(uint8_t refSel) {
    if (refSel != ADC_REFCTRL_REFSEL_INTREF_Val)
        return;

#ifdef ADC_HAS_SAME53_E54_REGISTERS
    SUPC_REGS->SUPC_VREF =
        (SUPC_REGS->SUPC_VREF & ~SUPC_VREF_SEL_Msk) |
        SUPC_VREF_SEL_1V0 |
        SUPC_VREF_VREFOE_Msk;
#else
    SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V0_Val;
    SUPC->VREF.bit.VREFOE = 1;
#endif // ADC_HAS_SAME53_E54_REGISTERS
}

void configureTemperatureSensor(uint8_t muxPos) {
    if (muxPos != ADC_INPUTCTRL_MUXPOS_PTAT_Val &&
        muxPos != ADC_INPUTCTRL_MUXPOS_CTAT_Val)
        return;

#ifdef ADC_HAS_SAME53_E54_REGISTERS
    uint32_t vref = SUPC_REGS->SUPC_VREF;
    vref &= ~(SUPC_VREF_ONDEMAND_Msk | SUPC_VREF_VREFOE_Msk | SUPC_VREF_TSSEL_Msk);
    if (muxPos == ADC_INPUTCTRL_MUXPOS_CTAT_Val)
        vref |= SUPC_VREF_TSSEL_Msk;
    vref |= SUPC_VREF_TSEN_Msk;
    SUPC_REGS->SUPC_VREF = vref;
#else
    SUPC->VREF.bit.ONDEMAND = 0;
    SUPC->VREF.bit.VREFOE = 0;
    SUPC->VREF.bit.TSSEL = (muxPos == ADC_INPUTCTRL_MUXPOS_CTAT_Val) ? 1 : 0;
    SUPC->VREF.bit.TSEN = 1;
#endif // ADC_HAS_SAME53_E54_REGISTERS
}

uint8_t sampleTimeForMux(uint8_t muxPos) {
    if (muxPos == ADC_INPUTCTRL_MUXPOS_PTAT_Val ||
        muxPos == ADC_INPUTCTRL_MUXPOS_CTAT_Val)
        return 0x3Fu;

    return 5u;
}
#endif

// ADC IRQ routing mirrors SERCOM family handling:
// - SAMD2x exposes a single ADC_IRQn vector.
// - SAMD51/E51 splits ADC0 interrupts across ADC0_0/ADC0_1.
// - SAME53/E54 splits ADC0 interrupts across ADC0_OTHER/ADC0_RESRDY.
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
#ifdef ADC_HAS_SAME53_E54_REGISTERS
    return (index == 0) ? ADC0_OTHER_IRQn : ADC0_RESRDY_IRQn;
#elif defined(ADC_HAS_D5X_E5X_REGISTERS)
    return (index == 0) ? ADC0_0_IRQn : ADC0_1_IRQn;
#else
    (void)index;
    return ADC_IRQn;
#endif
}

void adcPendSvService(uint8_t serviceId, void *context) {
    (void)context;
    if (serviceId != kAdcPendSvServiceId)
        return;

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
        return ADC_MUX_SOURCE_INVALID;

    const EAnalogChannel channel = g_APinDescription[arduinoPin].ulADCChannelNumber;
    if (channel == No_ADC_Channel)
        return ADC_MUX_SOURCE_INVALID;

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
    if (mux == ADC_MUX_SOURCE_INVALID)
        return ADC_MUX_SOURCE_INVALID;

    return mux;
}

uint8_t ADC_MUXNEG_PIN(uint8_t pin) {
    const uint8_t mux = pinToMux(pin);
    if (mux == ADC_MUX_SOURCE_INVALID)
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

    if (!PendSVChannels::isAvailable(kAdcPendSvServiceId))
        return false;

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
    pendingStartConversion_ = false;
    conversionState_ = ConversionState::Idle;

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
    auto *const adc = adcInstance();

    dmaDescriptor_ =
        dma_.addDescriptor(adcResultRegister(adc), (void *)&dmaLatestResult_, 1,
                           DMA_BEAT_SIZE_HWORD, false, false);
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

    adcDisableInterrupts(adc, 0x0F);
#ifdef ADC_HAS_SAME53_E54_REGISTERS
    adcClearFlags(adc, ADC_INTFLAG_RESRDY_Msk | ADC_INTFLAG_WINMON_Msk);
#else
    adcClearFlags(adc, ADC_INTFLAG_RESRDY | ADC_INTFLAG_WINMON);
#endif // ADC_HAS_SAME53_E54_REGISTERS
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
    auto *const adc = adcInstance();

    adcDisableResultReady(adc);

    if (dmaActive_)
        dma_.abort();

    dma_.free();
    dmaDescriptor_ = nullptr;

    adcSoftwareReset(adc);
    while (adcSoftwareResetBusy(adc))
      ;
    waitAdcSync();

    PendSV::instance().clearService(kAdcPendSvServiceId);

    initialized_ = false;
    dmaActive_ = false;
    pendingStartConversion_ = false;
    conversionState_ = ConversionState::Idle;
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
  auto *const adc = adcInstance();
  const uint8_t flags = adcReadFlags(adc);

  if (adcFlagWinmon(flags)) {
    ChannelADC *channel = activeChannel_;
    if (channel != nullptr && channel->windowEnabled_ &&
        channel->onWindowMonitor_ != nullptr) {
      const uint16_t result = adcReadResult(adc);
      channel->onWindowMonitor_(channel, result, channel->windowUserData_);
    }
  }

    if (adcFlagResrdy(flags)) {
      if (conversionState_ == ConversionState::Discarding) {
        (void)adcReadResult(adc);
        adcDisableResultReady(adc);
        pendingStartConversion_ = true;
        pendSvPending_ = true;
        PendSV::instance().setPending(kAdcPendSvServiceId);
      } else {
#ifdef ADC_HAS_SAME53_E54_REGISTERS
        adcClearFlags(adc, ADC_INTFLAG_RESRDY_Msk);
#else
        adcClearFlags(adc, ADC_INTFLAG_RESRDY);
#endif // ADC_HAS_SAME53_E54_REGISTERS
      }
    }

    adcClearFlags(adc, 0x0F);
}

void AdcEngine::onPendSv() {
    if (!pendSvPending_)
        return;

    pendSvPending_ = false;

    if (pendingStartConversion_) {
        pendingStartConversion_ = false;
        if (!startActiveDmaConversion()) {
            ChannelADC *channel = activeChannel_;
            if (channel != nullptr) {
                channel->enqueued_ = false;
                enabledMask_ &= ~(1u << channel->muxPos_);
            }
            activeChannel_ = nullptr;
            activeMonitorMode_ = false;
            conversionState_ = ConversionState::Idle;
        }
    }

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

    auto *const adc = adcInstance();

    adcSetEnable(adc, false);
    waitAdcSync();

    const uint8_t refctrlReg = static_cast<uint8_t>(ADC_REFCTRL_REFSEL(channel->refSel_));
    adcWriteRefctrl(adc, refctrlReg);

#ifdef ADC_HAS_D5X_E5X_REGISTERS
    configureInternalReference(channel->refSel_);
    configureTemperatureSensor(channel->muxPos_);

    const uint16_t inputCtrlReg = static_cast<uint16_t>(
        ADC_INPUTCTRL_MUXPOS(channel->muxPos_) | ADC_INPUTCTRL_MUXNEG(channel->muxNeg_) |
        (channel->differentialMode_ ? kAdcInputctrlDiffmodeBit : 0u));
    adcWriteInputctrl(adc, inputCtrlReg);

    const uint8_t avgCtrlReg =
        static_cast<uint8_t>(ADC_AVGCTRL_SAMPLENUM(sampleNumToCode(channel->sampleNum_)) |
                             ADC_AVGCTRL_ADJRES(channel->adjres_));
    adcWriteAvgctrl(adc, avgCtrlReg);
    adcWriteSampctrl(adc, sampleTimeForMux(channel->muxPos_));

    uint16_t ctrlaReg = adcReadCtrla(adc);
    ctrlaReg =
        static_cast<uint16_t>((ctrlaReg & ~ADC_CTRLA_PRESCALER_Msk) |
                              ADC_CTRLA_PRESCALER(static_cast<uint8_t>(channel->prescaler_)));
    adcWriteCtrla(adc, ctrlaReg);

    const uint16_t ctrlbReg = static_cast<uint16_t>(
        (channel->leftAdjust_ ? kAdcCtrlbLeftadjBit : 0u) |
        (channel->freeRun_ ? kAdcCtrlbFreerunBit : 0u) |
        (channel->corrEnabled_ ? kAdcCtrlbCorrenBit : 0u) |
        ADC_CTRLB_RESSEL(static_cast<uint8_t>(channel->ressel_)) |
        ADC_CTRLB_WINMODE(channel->windowEnabled_
                              ? static_cast<uint8_t>(channel->winMode_)
                              : static_cast<uint8_t>(AdcWinMode::ADC_WINMODE_DISABLE)));
    adcWriteCtrlb(adc, ctrlbReg);

    const uint8_t evctrlReg =
        static_cast<uint8_t>((channel->evWinmonEo_ ? kAdcEvctrlWinmoneoBit : 0u) |
                             (channel->evResrdyEo_ ? kAdcEvctrlResrdyeoBit : 0u) |
                             (channel->evSyncei_ ? kAdcEvctrlSynceiBit : 0u) |
                             (channel->evStartei_ ? kAdcEvctrlStarteiBit : 0u));
    adcWriteEvctrl(adc, evctrlReg);

    adcWriteWinlt(adc, channel->windowLower_);
    adcWriteWinut(adc, channel->windowUpper_);

    adcWriteOffsetcorr(adc, static_cast<uint16_t>(channel->offsetCorr_));
    adcWriteGaincorr(adc, channel->gainCorr_);
#else
    const uint32_t inputCtrlReg = static_cast<uint32_t>(
        ADC_INPUTCTRL_MUXPOS(channel->muxPos_) | ADC_INPUTCTRL_MUXNEG(channel->muxNeg_) |
        ADC_INPUTCTRL_GAIN(static_cast<uint8_t>(channel->gain_)));
    adcWriteInputctrl(adc, inputCtrlReg);

    const uint8_t avgCtrlReg =
        static_cast<uint8_t>(ADC_AVGCTRL_SAMPLENUM(sampleNumToCode(channel->sampleNum_)) |
                             ADC_AVGCTRL_ADJRES(channel->adjres_));
    adcWriteAvgctrl(adc, avgCtrlReg);

    const uint16_t ctrlbReg =
        static_cast<uint16_t>((channel->differentialMode_ ? ADC_CTRLB_DIFFMODE : 0u) |
                              (channel->leftAdjust_ ? ADC_CTRLB_LEFTADJ : 0u) |
                              (channel->freeRun_ ? ADC_CTRLB_FREERUN : 0u) |
                              (channel->corrEnabled_ ? ADC_CTRLB_CORREN : 0u) |
                              ADC_CTRLB_RESSEL(static_cast<uint8_t>(channel->ressel_)) |
                              ADC_CTRLB_PRESCALER(static_cast<uint8_t>(channel->prescaler_)));
    adcWriteCtrlb(adc, ctrlbReg);

    const uint8_t evctrlReg =
        static_cast<uint8_t>((channel->evWinmonEo_ ? ADC_EVCTRL_WINMONEO : 0u) |
                             (channel->evResrdyEo_ ? ADC_EVCTRL_RESRDYEO : 0u) |
                             (channel->evSyncei_ ? kAdcEvctrlSynceiBit : 0u) |
                             (channel->evStartei_ ? ADC_EVCTRL_STARTEI : 0u));
    adcWriteEvctrl(adc, evctrlReg);

    adcWriteWinlt(adc, channel->windowLower_);
    adcWriteWinut(adc, channel->windowUpper_);

    const uint8_t winctrlReg = static_cast<uint8_t>(
        ADC_WINCTRL_WINMODE(channel->windowEnabled_ ? static_cast<uint8_t>(channel->winMode_) : 0));
    adc->WINCTRL.reg = winctrlReg;

    adcWriteOffsetcorr(adc, static_cast<uint16_t>(channel->offsetCorr_));
    adcWriteGaincorr(adc, channel->gainCorr_);
#endif

    waitAdcSync();

    adcSetEnable(adc, true);
    waitAdcSync();

    adcDisableInterrupts(adc, 0x0F);
    adcClearFlags(adc, 0x0F);

#ifdef ADC_HAS_SAME53_E54_REGISTERS
    const uint8_t intensetReg =
        static_cast<uint8_t>(ADC_INTENSET_RESRDY_Msk |
                             ((monitorMode && channel->windowEnabled_) ? ADC_INTENSET_WINMON_Msk : 0u));
#else
    const uint8_t intensetReg =
        static_cast<uint8_t>(ADC_INTENSET_RESRDY |
                             ((monitorMode && channel->windowEnabled_) ? ADC_INTENSET_WINMON : 0u));
#endif // ADC_HAS_SAME53_E54_REGISTERS
    adcEnableInterrupts(adc, intensetReg);

    adcClearFlags(adc, 0x0F);

    activeChannel_ = channel;
    activeMonitorMode_ = monitorMode;
    conversionState_ = ConversionState::Discarding;

    startConversion();
    return true;
}

bool AdcEngine::startActiveDmaConversion() {
    ChannelADC *channel = activeChannel_;
    if (channel == nullptr)
        return false;

    auto *const adc = adcInstance();
    adcDisableResultReady(adc);
    adcClearFlags(adc, 0x0F);

    if (dmaDescriptor_ == nullptr)
        return false;

    dma_.changeDescriptor(dmaDescriptor_, adcResultRegister(adc),
                          (void *)&dmaLatestResult_, 1);
    if (dma_.startJob() != DMA_STATUS_OK)
        return false;

    dmaActive_ = true;
    conversionState_ = ConversionState::Sampling;

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
  auto *const adc = adcInstance();
#ifdef ADC_HAS_D5X_E5X_REGISTERS
    while (
#ifdef ADC_HAS_SAME53_E54_REGISTERS
        adc->ADC_SYNCBUSY
#else
        adc->SYNCBUSY.reg
#endif // ADC_HAS_SAME53_E54_REGISTERS
    )
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
    engine.conversionState_ = ConversionState::Idle;

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

void ChannelADC::onSyncReadComplete(ChannelADC *channel, uint16_t result, void *userData) {
    (void)result;

    ChannelADC *target = static_cast<ChannelADC *>(userData);
    if (target == nullptr)
        target = channel;

    if (target != nullptr)
        target->syncReadDone_ = true;
}

bool ChannelADC::read() {
    if (!initialized_ || !attached_)
        return false;

    hasFreshValue_ = false;
    syncReadDone_ = false;

    AdcEngine &engine = AdcEngine::instance();
    const bool syncRead = (onReadComplete_ == nullptr);
    Callback savedCallback = onReadComplete_;
    void *savedUserData = userData_;

    if (syncRead) {
        onReadComplete_ = &ChannelADC::onSyncReadComplete;
        userData_ = this;
    }

    if (!engine.enqueue(this)) {
        if (syncRead) {
            onReadComplete_ = savedCallback;
            userData_ = savedUserData;
        }
        return false;
    }

    if (syncRead) {
        const uint32_t startMs = millis();
        while (!syncReadDone_) {
            engine.service();
            if ((millis() - startMs) > 50u) {
                onReadComplete_ = savedCallback;
                userData_ = savedUserData;
                return false;
            }
        }

        onReadComplete_ = savedCallback;
        userData_ = savedUserData;
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

#ifdef ADC_HAS_SAME53_E54_REGISTERS
extern "C" void ADC0_OTHER_Handler(void) {
    AdcEngine::instance().onResrdyIsr();
}

extern "C" void ADC0_RESRDY_Handler(void) {
    AdcEngine::instance().onResrdyIsr();
}
#elif defined(ADC_HAS_D5X_E5X_REGISTERS)
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
#endif // ADC_HAS_SAME53_E54_REGISTERS / ADC_HAS_D5X_E5X_REGISTERS
