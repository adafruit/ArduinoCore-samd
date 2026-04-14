#pragma once

#include <Arduino.h>
#include <stdint.h>

#include <Adafruit_ZeroDMA.h>
#include <SERCOM_PinMux.h>

static constexpr uint8_t ADC_MUX_SOURCE_INVALID = 0xFF;

void analogReadCorrection(int offset, uint16_t gain);
#ifdef FAMILY_SAMD5X
float analogReadTemperatureC(uint16_t ptatReading, uint16_t ctatReading);
#else
float analogReadTemperatureC(uint16_t adcReading);
#endif

enum class AdcSampleNum : uint8_t {
    ADC_SAMPLENUM_1 = ADC_AVGCTRL_SAMPLENUM_1_Val,
    ADC_SAMPLENUM_2 = ADC_AVGCTRL_SAMPLENUM_2_Val,
    ADC_SAMPLENUM_4 = ADC_AVGCTRL_SAMPLENUM_4_Val,
    ADC_SAMPLENUM_8 = ADC_AVGCTRL_SAMPLENUM_8_Val,
    ADC_SAMPLENUM_16 = ADC_AVGCTRL_SAMPLENUM_16_Val,
    ADC_SAMPLENUM_32 = ADC_AVGCTRL_SAMPLENUM_32_Val,
    ADC_SAMPLENUM_64 = ADC_AVGCTRL_SAMPLENUM_64_Val,
    ADC_SAMPLENUM_128 = ADC_AVGCTRL_SAMPLENUM_128_Val,
    ADC_SAMPLENUM_256 = ADC_AVGCTRL_SAMPLENUM_256_Val,
    ADC_SAMPLENUM_512 = ADC_AVGCTRL_SAMPLENUM_512_Val,
    ADC_SAMPLENUM_1024 = ADC_AVGCTRL_SAMPLENUM_1024_Val,
};

enum class AdcWinMode : uint8_t {
#ifdef FAMILY_SAMD5X
    ADC_WINMODE_DISABLE = ADC_CTRLB_WINMODE_DISABLE_Val, ///< No window mode (disabled)
    ADC_WINMODE_MODE1 = ADC_CTRLB_WINMODE_MODE1_Val,     ///< Window mode 1: RESULT > WINLT.
    ADC_WINMODE_MODE2 = ADC_CTRLB_WINMODE_MODE2_Val,     ///< Window mode 2: RESULT < WINUT.
    ADC_WINMODE_MODE3 = ADC_CTRLB_WINMODE_MODE3_Val,     ///< Window mode 3: WINLT < RESULT < WINUT.
    ADC_WINMODE_MODE4 = ADC_CTRLB_WINMODE_MODE4_Val, ///< Window mode 4: !(WINLT < RESULT < WINUT).
#else
    ADC_WINMODE_DISABLE = ADC_WINCTRL_WINMODE_DISABLE_Val, ///< No window mode (disabled)
    ADC_WINMODE_MODE1 = ADC_WINCTRL_WINMODE_MODE1_Val,     ///< Window mode 1: RESULT > WINLT.
    ADC_WINMODE_MODE2 = ADC_WINCTRL_WINMODE_MODE2_Val,     ///< Window mode 2: RESULT < WINUT.
    ADC_WINMODE_MODE3 = ADC_WINCTRL_WINMODE_MODE3_Val, ///< Window mode 3: WINLT < RESULT < WINUT.
    ADC_WINMODE_MODE4 =
        ADC_WINCTRL_WINMODE_MODE4_Val, ///< Window mode 4: !(WINLT < RESULT < WINUT).
#endif
};

enum class AdcRefSel : uint8_t {
#ifdef FAMILY_SAMD5X
    ADC_REFSEL_INT1V = ADC_REFCTRL_REFSEL_INTREF_Val,
#else
    ADC_REFSEL_INT1V = ADC_REFCTRL_REFSEL_INT1V_Val,
#endif
    ADC_REFSEL_INTVCC0 = ADC_REFCTRL_REFSEL_INTVCC0_Val,
    ADC_REFSEL_INTVCC1 = ADC_REFCTRL_REFSEL_INTVCC1_Val,
    ADC_REFSEL_AREFA = ADC_REFCTRL_REFSEL_AREFA_Val,
    ADC_REFSEL_AREFB = ADC_REFCTRL_REFSEL_AREFB_Val,
};

enum class AdcGain : uint8_t {
#ifdef FAMILY_SAMD5X
    ADC_GAIN_1X = 0x0,
    ADC_GAIN_2X = 0x1,
    ADC_GAIN_4X = 0x2,
    ADC_GAIN_8X = 0x3,
    ADC_GAIN_16X = 0x4,
    ADC_GAIN_1_DIV_2 = 0xF,
#else
    ADC_GAIN_1X = ADC_INPUTCTRL_GAIN_1X_Val,
    ADC_GAIN_2X = ADC_INPUTCTRL_GAIN_2X_Val,
    ADC_GAIN_4X = ADC_INPUTCTRL_GAIN_4X_Val,
    ADC_GAIN_8X = ADC_INPUTCTRL_GAIN_8X_Val,
    ADC_GAIN_16X = ADC_INPUTCTRL_GAIN_16X_Val,
    ADC_GAIN_1_DIV_2 = ADC_INPUTCTRL_GAIN_DIV2_Val,
#endif
};

enum class AdcResSel : uint8_t {
    ADC_RESSEL_12BIT = ADC_CTRLB_RESSEL_12BIT_Val,
    ADC_RESSEL_16BIT = ADC_CTRLB_RESSEL_16BIT_Val,
    ADC_RESSEL_10BIT = ADC_CTRLB_RESSEL_10BIT_Val,
    ADC_RESSEL_8BIT = ADC_CTRLB_RESSEL_8BIT_Val,
};

enum class AdcPrescaler : uint8_t {
#ifdef FAMILY_SAMD5X
    ADC_PRESCALER_DIV4 = ADC_CTRLA_PRESCALER_DIV4_Val,
    ADC_PRESCALER_DIV8 = ADC_CTRLA_PRESCALER_DIV8_Val,
    ADC_PRESCALER_DIV16 = ADC_CTRLA_PRESCALER_DIV16_Val,
    ADC_PRESCALER_DIV32 = ADC_CTRLA_PRESCALER_DIV32_Val,
    ADC_PRESCALER_DIV64 = ADC_CTRLA_PRESCALER_DIV64_Val,
    ADC_PRESCALER_DIV128 = ADC_CTRLA_PRESCALER_DIV128_Val,
    ADC_PRESCALER_DIV256 = ADC_CTRLA_PRESCALER_DIV256_Val,
#else
    ADC_PRESCALER_DIV4 = ADC_CTRLB_PRESCALER_DIV4_Val,
    ADC_PRESCALER_DIV8 = ADC_CTRLB_PRESCALER_DIV8_Val,
    ADC_PRESCALER_DIV16 = ADC_CTRLB_PRESCALER_DIV16_Val,
    ADC_PRESCALER_DIV32 = ADC_CTRLB_PRESCALER_DIV32_Val,
    ADC_PRESCALER_DIV64 = ADC_CTRLB_PRESCALER_DIV64_Val,
    ADC_PRESCALER_DIV128 = ADC_CTRLB_PRESCALER_DIV128_Val,
    ADC_PRESCALER_DIV256 = ADC_CTRLB_PRESCALER_DIV256_Val,
    ADC_PRESCALER_DIV512 = ADC_CTRLB_PRESCALER_DIV512_Val,
#endif
};

enum class AdcMuxPos : uint8_t {
#ifdef FAMILY_SAMD5X
    ADC_MUXPOS_PIN0 = ADC_INPUTCTRL_MUXPOS_AIN0_Val,
    ADC_MUXPOS_PIN1 = ADC_INPUTCTRL_MUXPOS_AIN1_Val,
    ADC_MUXPOS_PIN2 = ADC_INPUTCTRL_MUXPOS_AIN2_Val,
    ADC_MUXPOS_PIN3 = ADC_INPUTCTRL_MUXPOS_AIN3_Val,
    ADC_MUXPOS_PIN4 = ADC_INPUTCTRL_MUXPOS_AIN4_Val,
    ADC_MUXPOS_PIN5 = ADC_INPUTCTRL_MUXPOS_AIN5_Val,
    ADC_MUXPOS_PIN6 = ADC_INPUTCTRL_MUXPOS_AIN6_Val,
    ADC_MUXPOS_PIN7 = ADC_INPUTCTRL_MUXPOS_AIN7_Val,
    ADC_MUXPOS_PIN8 = ADC_INPUTCTRL_MUXPOS_AIN8_Val,
    ADC_MUXPOS_PIN9 = ADC_INPUTCTRL_MUXPOS_AIN9_Val,
    ADC_MUXPOS_PIN10 = ADC_INPUTCTRL_MUXPOS_AIN10_Val,
    ADC_MUXPOS_PIN11 = ADC_INPUTCTRL_MUXPOS_AIN11_Val,
    ADC_MUXPOS_PIN12 = ADC_INPUTCTRL_MUXPOS_AIN12_Val,
    ADC_MUXPOS_PIN13 = ADC_INPUTCTRL_MUXPOS_AIN13_Val,
    ADC_MUXPOS_PIN14 = ADC_INPUTCTRL_MUXPOS_AIN14_Val,
    ADC_MUXPOS_PIN15 = ADC_INPUTCTRL_MUXPOS_AIN15_Val,
    ADC_MUXPOS_PIN16 = ADC_INPUTCTRL_MUXPOS_AIN16_Val,
    ADC_MUXPOS_PIN17 = ADC_INPUTCTRL_MUXPOS_AIN17_Val,
    ADC_MUXPOS_PIN18 = ADC_INPUTCTRL_MUXPOS_AIN18_Val,
    ADC_MUXPOS_PIN19 = ADC_INPUTCTRL_MUXPOS_AIN19_Val,
    ADC_MUXPOS_TEMP = ADC_INPUTCTRL_MUXPOS_PTAT_Val,
    ADC_MUXPOS_TEMP_CTAT = ADC_INPUTCTRL_MUXPOS_CTAT_Val,
#else
    ADC_MUXPOS_PIN0 = ADC_INPUTCTRL_MUXPOS_PIN0_Val,
    ADC_MUXPOS_PIN1 = ADC_INPUTCTRL_MUXPOS_PIN1_Val,
    ADC_MUXPOS_PIN2 = ADC_INPUTCTRL_MUXPOS_PIN2_Val,
    ADC_MUXPOS_PIN3 = ADC_INPUTCTRL_MUXPOS_PIN3_Val,
    ADC_MUXPOS_PIN4 = ADC_INPUTCTRL_MUXPOS_PIN4_Val,
    ADC_MUXPOS_PIN5 = ADC_INPUTCTRL_MUXPOS_PIN5_Val,
    ADC_MUXPOS_PIN6 = ADC_INPUTCTRL_MUXPOS_PIN6_Val,
    ADC_MUXPOS_PIN7 = ADC_INPUTCTRL_MUXPOS_PIN7_Val,
    ADC_MUXPOS_PIN8 = ADC_INPUTCTRL_MUXPOS_PIN8_Val,
    ADC_MUXPOS_PIN9 = ADC_INPUTCTRL_MUXPOS_PIN9_Val,
    ADC_MUXPOS_PIN10 = ADC_INPUTCTRL_MUXPOS_PIN10_Val,
    ADC_MUXPOS_PIN11 = ADC_INPUTCTRL_MUXPOS_PIN11_Val,
    ADC_MUXPOS_PIN12 = ADC_INPUTCTRL_MUXPOS_PIN12_Val,
    ADC_MUXPOS_PIN13 = ADC_INPUTCTRL_MUXPOS_PIN13_Val,
    ADC_MUXPOS_PIN14 = ADC_INPUTCTRL_MUXPOS_PIN14_Val,
    ADC_MUXPOS_PIN15 = ADC_INPUTCTRL_MUXPOS_PIN15_Val,
    ADC_MUXPOS_PIN16 = ADC_INPUTCTRL_MUXPOS_PIN16_Val,
    ADC_MUXPOS_PIN17 = ADC_INPUTCTRL_MUXPOS_PIN17_Val,
    ADC_MUXPOS_PIN18 = ADC_INPUTCTRL_MUXPOS_PIN18_Val,
    ADC_MUXPOS_PIN19 = ADC_INPUTCTRL_MUXPOS_PIN19_Val,
    ADC_MUXPOS_TEMP = ADC_INPUTCTRL_MUXPOS_TEMP_Val,
#endif
    ADC_MUXPOS_BANDGAP = ADC_INPUTCTRL_MUXPOS_BANDGAP_Val,
    ADC_MUXPOS_SCALEDCOREVCC = ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC_Val,
    ADC_MUXPOS_SCALEDIOVCC = ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val,
    ADC_MUXPOS_DAC = ADC_INPUTCTRL_MUXPOS_DAC_Val,
};

enum class AdcMuxNeg : uint8_t {
#ifdef FAMILY_SAMD5X
    ADC_MUXNEG_PIN0 = ADC_INPUTCTRL_MUXNEG_AIN0_Val,
    ADC_MUXNEG_PIN1 = ADC_INPUTCTRL_MUXNEG_AIN1_Val,
    ADC_MUXNEG_PIN2 = ADC_INPUTCTRL_MUXNEG_AIN2_Val,
    ADC_MUXNEG_PIN3 = ADC_INPUTCTRL_MUXNEG_AIN3_Val,
    ADC_MUXNEG_PIN4 = ADC_INPUTCTRL_MUXNEG_AIN4_Val,
    ADC_MUXNEG_PIN5 = ADC_INPUTCTRL_MUXNEG_AIN5_Val,
    ADC_MUXNEG_PIN6 = ADC_INPUTCTRL_MUXNEG_AIN6_Val,
    ADC_MUXNEG_PIN7 = ADC_INPUTCTRL_MUXNEG_AIN7_Val,
    ADC_MUXNEG_IOGND = 0x19,
#else
    ADC_MUXNEG_PIN0 = ADC_INPUTCTRL_MUXNEG_PIN0_Val,
    ADC_MUXNEG_PIN1 = ADC_INPUTCTRL_MUXNEG_PIN1_Val,
    ADC_MUXNEG_PIN2 = ADC_INPUTCTRL_MUXNEG_PIN2_Val,
    ADC_MUXNEG_PIN3 = ADC_INPUTCTRL_MUXNEG_PIN3_Val,
    ADC_MUXNEG_PIN4 = ADC_INPUTCTRL_MUXNEG_PIN4_Val,
    ADC_MUXNEG_PIN5 = ADC_INPUTCTRL_MUXNEG_PIN5_Val,
    ADC_MUXNEG_PIN6 = ADC_INPUTCTRL_MUXNEG_PIN6_Val,
    ADC_MUXNEG_PIN7 = ADC_INPUTCTRL_MUXNEG_PIN7_Val,
    ADC_MUXNEG_IOGND = ADC_INPUTCTRL_MUXNEG_IOGND_Val,
#endif
    ADC_MUXNEG_GND = ADC_INPUTCTRL_MUXNEG_GND_Val,
};

class ChannelADC;

class AdcEngine {
  public:
    static constexpr uint8_t kMuxMin = 0x00;
    static constexpr uint8_t kMuxMax = 0x1C;
    static constexpr uint8_t kSkippedMuxStart = 0x14;
    static constexpr uint8_t kSkippedMuxEnd = 0x17;
    static constexpr uint8_t kResultContainerSize = 25;

    static AdcEngine &instance();

    bool begin();
    void end();

    bool enqueue(ChannelADC *channel);
    bool registerChannel(ChannelADC *channel);
    void unregisterChannel(ChannelADC *channel);
    void service();

    uint8_t resultByteForMux(uint8_t muxPos) const;
    uint32_t enabledMask() const;

    void onResrdyIsr();
    void onPendSv();

  private:
    AdcEngine() = default;

    static int8_t muxPosToResultIndex(uint8_t muxPos);

    bool pushQueue(ChannelADC *channel);
    ChannelADC *popQueue();

    bool applyChannelAndStart(ChannelADC *channel, bool monitorMode = false);
    bool startMonitorIfIdle();
    void waitAdcSync() const;
    inline void startConversion() const { ADC->SWTRIG.bit.START = 1;}
    static void dmaDoneCallback(Adafruit_ZeroDMA *dma);

    ChannelADC *queue_[kResultContainerSize]{};
    uint8_t queueHead_ = 0;
    uint8_t queueTail_ = 0;
    uint8_t queueCount_ = 0;

    volatile uint8_t resultContainer_[kResultContainerSize]{};
    volatile uint32_t enabledMask_ = 0;

    ChannelADC *activeChannel_ = nullptr;
    bool activeMonitorMode_ = false;

    ChannelADC *registeredChannels_[kResultContainerSize]{};
    uint8_t registeredCount_ = 0;
    uint8_t monitorCursor_ = 0;

    ChannelADC *pendingChannel_ = nullptr;
    volatile uint16_t pendingResult_ = 0;
    void (*pendingCallback_)(ChannelADC *channel, uint16_t result, void *userData) = nullptr;
    void *pendingUserData_ = nullptr;

    Adafruit_ZeroDMA dma_;
    DmacDescriptor *dmaDescriptor_ = nullptr;
    volatile uint16_t dmaLatestResult_ = 0;
    bool dmaActive_ = false;
    bool initialized_ = false;
    volatile bool pendSvPending_ = false;
};

class ChannelADC {
  public:
    using Callback = void (*)(ChannelADC *channel, uint16_t result, void *userData);

    bool attach(uint8_t pinPos, uint8_t pinNeg,
                AdcSampleNum sampleNum = AdcSampleNum::ADC_SAMPLENUM_16);
    bool attach(uint8_t pinPos, AdcMuxNeg muxNeg = AdcMuxNeg::ADC_MUXNEG_GND,
                AdcSampleNum sampleNum = AdcSampleNum::ADC_SAMPLENUM_16);
    bool attach(AdcMuxPos muxPos, uint8_t pinNeg,
                AdcSampleNum sampleNum = AdcSampleNum::ADC_SAMPLENUM_16);
    bool attach(AdcMuxPos muxPos, AdcMuxNeg muxNeg = AdcMuxNeg::ADC_MUXNEG_GND,
                AdcSampleNum sampleNum = AdcSampleNum::ADC_SAMPLENUM_16);

    bool read();
    uint16_t value() const;

    void setWindow(AdcWinMode mode, uint16_t lower, uint16_t upper);
    void clearWindow();
    bool windowEnabled() const;

    void setWindowCallback(Callback callback, void *userData = nullptr);
    void setReadCallback(Callback callback, void *userData = nullptr);
    void setEventControl(bool winmonEo, bool resrdyEo, bool syncei, bool startei);
    void setCtrlB(AdcResSel resolution, AdcPrescaler prescaler, bool freeRun = false,
                  bool leftAdjust = false, bool differentialMode = false,
                  bool correctionEnable = false);
    void setReference(AdcRefSel reference);
    void setGain(AdcGain gain);
    void setCalibration(uint16_t gainCorr, int16_t offsetCorr, bool enableCorrection = true);
    void end();

  private:
    friend class AdcEngine;

    bool setAttachedSources(uint8_t encodedPos, uint8_t encodedNeg, AdcSampleNum sampleNum);
    bool monitorConfigured() const;

    uint8_t muxPos_ = 0;
    uint8_t muxNeg_ = 0x18;
    AdcSampleNum sampleNum_ = AdcSampleNum::ADC_SAMPLENUM_16;
    uint8_t adjres_ = 4;
    uint8_t refSel_ = static_cast<uint8_t>(AdcRefSel::ADC_REFSEL_INTVCC1);
    AdcResSel ressel_ = AdcResSel::ADC_RESSEL_16BIT;
    AdcPrescaler prescaler_ = AdcPrescaler::ADC_PRESCALER_DIV4;
    bool freeRun_ = false;
    bool leftAdjust_ = false;
    bool differentialMode_ = false;
    bool enqueued_ = false;
    bool registered_ = false;
    Callback onReadComplete_ = nullptr;
    void *userData_ = nullptr;

    bool windowEnabled_ = false;
    uint16_t windowLower_ = 0;
    uint16_t windowUpper_ = 0xFFFF;
    AdcWinMode winMode_ = AdcWinMode::ADC_WINMODE_DISABLE;
    Callback onWindowMonitor_ = nullptr;
    void *windowUserData_ = nullptr;

    bool evWinmonEo_ = false;
    bool evResrdyEo_ = false;
    bool evSyncei_ = false;
    bool evStartei_ = false;

    bool corrEnabled_ = false;
    AdcGain gain_ = AdcGain::ADC_GAIN_1_DIV_2;
    uint16_t gainCorr_ = 0;
    int16_t offsetCorr_ = 0;

    uint16_t value_ = 0;
    bool hasFreshValue_ = false;
    bool attached_ = false;
    bool initialized_ = false;
};
