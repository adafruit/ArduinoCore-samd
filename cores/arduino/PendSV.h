#pragma once

#include <sam.h>

#include <array>
#include <stdint.h>

#if defined(SERCOM0) || defined(SERCOM0_REGS)
#define SIMIO_PENDSV_HAS_SERCOM0 true
#else
#define SIMIO_PENDSV_HAS_SERCOM0 false
#endif
#if defined(SERCOM1) || defined(SERCOM1_REGS)
#define SIMIO_PENDSV_HAS_SERCOM1 true
#else
#define SIMIO_PENDSV_HAS_SERCOM1 false
#endif
#if defined(SERCOM2) || defined(SERCOM2_REGS)
#define SIMIO_PENDSV_HAS_SERCOM2 true
#else
#define SIMIO_PENDSV_HAS_SERCOM2 false
#endif
#if defined(SERCOM3) || defined(SERCOM3_REGS)
#define SIMIO_PENDSV_HAS_SERCOM3 true
#else
#define SIMIO_PENDSV_HAS_SERCOM3 false
#endif
#if defined(SERCOM4) || defined(SERCOM4_REGS)
#define SIMIO_PENDSV_HAS_SERCOM4 true
#else
#define SIMIO_PENDSV_HAS_SERCOM4 false
#endif
#if defined(SERCOM5) || defined(SERCOM5_REGS)
#define SIMIO_PENDSV_HAS_SERCOM5 true
#else
#define SIMIO_PENDSV_HAS_SERCOM5 false
#endif
#if defined(SERCOM6) || defined(SERCOM6_REGS)
#define SIMIO_PENDSV_HAS_SERCOM6 true
#else
#define SIMIO_PENDSV_HAS_SERCOM6 false
#endif
#if defined(SERCOM7) || defined(SERCOM7_REGS)
#define SIMIO_PENDSV_HAS_SERCOM7 true
#else
#define SIMIO_PENDSV_HAS_SERCOM7 false
#endif
#if defined(DMAC) || defined(DMAC_REGS)
#define SIMIO_PENDSV_HAS_DMAC true
#else
#define SIMIO_PENDSV_HAS_DMAC false
#endif
#if defined(AES) || defined(AES_REGS)
#define SIMIO_PENDSV_HAS_AES true
#else
#define SIMIO_PENDSV_HAS_AES false
#endif
#if defined(ICM) || defined(ICM_REGS)
#define SIMIO_PENDSV_HAS_ICM true
#else
#define SIMIO_PENDSV_HAS_ICM false
#endif
#if defined(PUKCC) || defined(PUKCC_REGS) || defined(ID_PUKCC) || defined(PUKCC_INSTANCE_ID) || \
    defined(PUKCC_IRQn)
#define SIMIO_PENDSV_HAS_PUKCC true
#else
#define SIMIO_PENDSV_HAS_PUKCC false
#endif
#if defined(TRNG) || defined(TRNG_REGS)
#define SIMIO_PENDSV_HAS_TRNG true
#else
#define SIMIO_PENDSV_HAS_TRNG false
#endif
#if defined(GMAC) || defined(GMAC_REGS)
#define SIMIO_PENDSV_HAS_GMAC true
#define SIMIO_PENDSV_HAS_PHY true
#else
#define SIMIO_PENDSV_HAS_GMAC false
#define SIMIO_PENDSV_HAS_PHY false
#endif

template <bool HasSercom0, bool HasSercom1, bool HasSercom2, bool HasSercom3, bool HasSercom4,
          bool HasSercom5, bool HasSercom6, bool HasSercom7, bool HasDmac, bool HasAes,
          bool HasIcm, bool HasPukcc, bool HasTrng, bool HasGmac, bool HasPhy>
struct PendSVChannelMap {
  static constexpr uint8_t kUnavailable = 0xFF;

private:
  static constexpr uint8_t assign(bool present, uint8_t next) {
    return present ? next : kUnavailable;
  }
  static constexpr uint8_t advance(bool present, uint8_t next) {
    return present ? static_cast<uint8_t>(next + 1) : next;
  }

  static constexpr uint8_t kBase = 0;
  static constexpr uint8_t kAfterSercom0 = advance(HasSercom0, kBase);
  static constexpr uint8_t kAfterSercom1 = advance(HasSercom1, kAfterSercom0);
  static constexpr uint8_t kAfterSercom2 = advance(HasSercom2, kAfterSercom1);
  static constexpr uint8_t kAfterSercom3 = advance(HasSercom3, kAfterSercom2);
  static constexpr uint8_t kAfterSercom4 = advance(HasSercom4, kAfterSercom3);
  static constexpr uint8_t kAfterSercom5 = advance(HasSercom5, kAfterSercom4);
  static constexpr uint8_t kAfterSercom6 = advance(HasSercom6, kAfterSercom5);
  static constexpr uint8_t kAfterSercom7 = advance(HasSercom7, kAfterSercom6);
  static constexpr uint8_t kAfterRtc = advance(true, kAfterSercom7);
  static constexpr uint8_t kAfterUsb = advance(true, kAfterRtc);
  static constexpr uint8_t kAfterAdc = advance(true, kAfterUsb);
  static constexpr uint8_t kAfterDmac = advance(HasDmac, kAfterAdc);
  static constexpr uint8_t kAfterAes = advance(HasAes, kAfterDmac);
  static constexpr uint8_t kAfterIcm = advance(HasIcm, kAfterAes);
  static constexpr uint8_t kAfterPukcc = advance(HasPukcc, kAfterIcm);
  static constexpr uint8_t kAfterTrng = advance(HasTrng, kAfterPukcc);
  static constexpr uint8_t kAfterGmac = advance(HasGmac, kAfterTrng);
  static constexpr uint8_t kAfterPhy = advance(HasPhy, kAfterGmac);
  static constexpr uint8_t kAfterUsbProtocol = advance(true, kAfterPhy);
  static constexpr uint8_t kAfterDiscreteInput = advance(true, kAfterUsbProtocol);

public:
  static constexpr uint8_t Sercom0 = assign(HasSercom0, kBase);
  static constexpr uint8_t Sercom1 = assign(HasSercom1, kAfterSercom0);
  static constexpr uint8_t Sercom2 = assign(HasSercom2, kAfterSercom1);
  static constexpr uint8_t Sercom3 = assign(HasSercom3, kAfterSercom2);
  static constexpr uint8_t Sercom4 = assign(HasSercom4, kAfterSercom3);
  static constexpr uint8_t Sercom5 = assign(HasSercom5, kAfterSercom4);
  static constexpr uint8_t Sercom6 = assign(HasSercom6, kAfterSercom5);
  static constexpr uint8_t Sercom7 = assign(HasSercom7, kAfterSercom6);
  static constexpr uint8_t Rtc = kAfterSercom7;
  static constexpr uint8_t Usb = kAfterRtc;
  static constexpr uint8_t Adc = kAfterUsb;
  static constexpr uint8_t Dmac = assign(HasDmac, kAfterAdc);
  static constexpr uint8_t Aes = assign(HasAes, kAfterDmac);
  static constexpr uint8_t Icm = assign(HasIcm, kAfterAes);
  static constexpr uint8_t Pukcc = assign(HasPukcc, kAfterIcm);
  static constexpr uint8_t Trng = assign(HasTrng, kAfterPukcc);
  static constexpr uint8_t Gmac = assign(HasGmac, kAfterTrng);
  static constexpr uint8_t Phy = assign(HasPhy, kAfterGmac);
  static constexpr uint8_t UsbProtocol = kAfterPhy;
  static constexpr uint8_t DiscreteInput = kAfterUsbProtocol;
  static constexpr uint8_t Count = kAfterDiscreteInput;

  static constexpr bool isAvailable(uint8_t channel) { return channel != kUnavailable; }
  static constexpr uint8_t sercom(uint8_t index) {
    return index == 0 ? Sercom0 : index == 1 ? Sercom1 : index == 2 ? Sercom2
         : index == 3 ? Sercom3 : index == 4 ? Sercom4 : index == 5 ? Sercom5
         : index == 6 ? Sercom6 : index == 7 ? Sercom7 : kUnavailable;
  }
  static constexpr uint8_t sercomIndex(uint8_t channel) {
    return channel == Sercom0 ? 0 : channel == Sercom1 ? 1 : channel == Sercom2 ? 2
         : channel == Sercom3 ? 3 : channel == Sercom4 ? 4 : channel == Sercom5 ? 5
         : channel == Sercom6 ? 6 : channel == Sercom7 ? 7 : kUnavailable;
  }
};

using PendSVChannels = PendSVChannelMap<
    SIMIO_PENDSV_HAS_SERCOM0, SIMIO_PENDSV_HAS_SERCOM1, SIMIO_PENDSV_HAS_SERCOM2,
    SIMIO_PENDSV_HAS_SERCOM3, SIMIO_PENDSV_HAS_SERCOM4, SIMIO_PENDSV_HAS_SERCOM5,
    SIMIO_PENDSV_HAS_SERCOM6, SIMIO_PENDSV_HAS_SERCOM7, SIMIO_PENDSV_HAS_DMAC,
    SIMIO_PENDSV_HAS_AES, SIMIO_PENDSV_HAS_ICM, SIMIO_PENDSV_HAS_PUKCC,
    SIMIO_PENDSV_HAS_TRNG, SIMIO_PENDSV_HAS_GMAC, SIMIO_PENDSV_HAS_PHY>;

class PendSV {
public:
  using ServiceFn = void (*)(uint8_t serviceId, void *context);
  static constexpr uint8_t kMaxServices = PendSVChannels::Count;
  static constexpr uint8_t kDispatchBudget = 16;
  static_assert(kMaxServices <= 32, "PendSV pending mask supports at most 32 services");

  static PendSV &instance();
  static bool initializeCoreServices();

  bool registerService(uint8_t serviceId, ServiceFn fn, void *context = nullptr);
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
