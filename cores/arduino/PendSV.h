#pragma once

#include "PendSVChannelMap.h"
#include "sam.h"

#include <array>
#include <stdint.h>

#if defined(SERCOM0) || defined(SERCOM0_REGS)
#define PENDSV_HAS_SERCOM0 true
#else
#define PENDSV_HAS_SERCOM0 false
#endif
#if defined(SERCOM1) || defined(SERCOM1_REGS)
#define PENDSV_HAS_SERCOM1 true
#else
#define PENDSV_HAS_SERCOM1 false
#endif
#if defined(SERCOM2) || defined(SERCOM2_REGS)
#define PENDSV_HAS_SERCOM2 true
#else
#define PENDSV_HAS_SERCOM2 false
#endif
#if defined(SERCOM3) || defined(SERCOM3_REGS)
#define PENDSV_HAS_SERCOM3 true
#else
#define PENDSV_HAS_SERCOM3 false
#endif
#if defined(SERCOM4) || defined(SERCOM4_REGS)
#define PENDSV_HAS_SERCOM4 true
#else
#define PENDSV_HAS_SERCOM4 false
#endif
#if defined(SERCOM5) || defined(SERCOM5_REGS)
#define PENDSV_HAS_SERCOM5 true
#else
#define PENDSV_HAS_SERCOM5 false
#endif
#if defined(SERCOM6) || defined(SERCOM6_REGS)
#define PENDSV_HAS_SERCOM6 true
#else
#define PENDSV_HAS_SERCOM6 false
#endif
#if defined(SERCOM7) || defined(SERCOM7_REGS)
#define PENDSV_HAS_SERCOM7 true
#else
#define PENDSV_HAS_SERCOM7 false
#endif

#if defined(USB) || defined(USB_REGS)
#define PENDSV_HAS_USB true
#else
#define PENDSV_HAS_USB false
#endif

#if defined(GMAC) || defined(GMAC_REGS)
#define PENDSV_HAS_GMAC true
#define PENDSV_HAS_PHY true
#else
#define PENDSV_HAS_GMAC false
#define PENDSV_HAS_PHY false
#endif
#if defined(ADC) || defined(ADC_REGS) || defined(ADC0) || defined(ADC0_REGS)
#define PENDSV_HAS_ADC true
#else
#define PENDSV_HAS_ADC false
#endif
#if defined(AES) || defined(AES_REGS)
#define PENDSV_HAS_AES true
#else
#define PENDSV_HAS_AES false
#endif
#if defined(PUKCC) || defined(PUKCC_REGS) || defined(ID_PUKCC) ||             \
    defined(PUKCC_INSTANCE_ID) || defined(PUKCC_IRQn)
#define PENDSV_HAS_PUKCC true
#else
#define PENDSV_HAS_PUKCC false
#endif
#if defined(TRNG) || defined(TRNG_REGS)
#define PENDSV_HAS_TRNG true
#else
#define PENDSV_HAS_TRNG false
#endif
#if defined(RTC) || defined(RTC_REGS)
#define PENDSV_HAS_RTC true
#else
#define PENDSV_HAS_RTC false
#endif

using PendSVChannels = PendSVChannelMap<
    PENDSV_HAS_SERCOM0, PENDSV_HAS_SERCOM1, PENDSV_HAS_SERCOM2,
    PENDSV_HAS_SERCOM3, PENDSV_HAS_SERCOM4, PENDSV_HAS_SERCOM5,
    PENDSV_HAS_SERCOM6, PENDSV_HAS_SERCOM7, PENDSV_HAS_USB, PENDSV_HAS_GMAC,
    PENDSV_HAS_PHY, PENDSV_HAS_ADC, PENDSV_HAS_AES, PENDSV_HAS_PUKCC,
    PENDSV_HAS_TRNG, PENDSV_HAS_RTC>;

class PendSV {
  public:
    using ServiceFn = void (*)(uint8_t serviceId, void *context);
    static constexpr uint8_t kMaxServices = PendSVChannels::kMaxServices;
    static constexpr uint8_t kDispatchBudget = 16;
    static_assert(kMaxServices <= 32, "PendSV pending mask supports at most 32 services");
    static_assert(PendSVChannels::Count <= kMaxServices,
                  "PendSV channel allocation exceeds dispatcher capacity");

    static PendSV &instance();
    static bool initializeCoreServices();

    bool registerService(uint8_t serviceId, ServiceFn fn, void *context = nullptr);
    // Cancels queued work that has not started dispatching. If a callback was
    // already copied for dispatch, its context must remain valid until it returns.
    void clearService(uint8_t serviceId);
    void dispatchPending();
    void setPending(uint8_t serviceId);
    void setPendingOnce(uint8_t serviceId);

  private:
    static uint32_t enterCritical();
    static void exitCritical(uint32_t primask);

    struct ServiceEntry {
      ServiceFn fn = nullptr;
      void *context = nullptr;
    };

    std::array<ServiceEntry, kMaxServices> services_{};
    std::array<uint16_t, kMaxServices> pendingCount_{};
    volatile uint32_t pendingMask_ = 0;
};
