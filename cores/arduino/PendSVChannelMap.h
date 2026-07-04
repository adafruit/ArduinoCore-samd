#pragma once

#include <stdint.h>

template <bool HasSercom0, bool HasSercom1, bool HasSercom2, bool HasSercom3,
          bool HasSercom4, bool HasSercom5, bool HasSercom6, bool HasSercom7,
          bool HasUsb, bool HasGmac, bool HasPhy, bool HasAdc, bool HasAes,
          bool HasPukcc, bool HasTrng, bool HasRtc>
struct PendSVChannelMap {
  static constexpr uint8_t kMaxServices = 32;
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
  static constexpr uint8_t kAfterUsb = advance(HasUsb, kAfterSercom7);
  static constexpr uint8_t kAfterGmac = advance(HasGmac, kAfterUsb);
  static constexpr uint8_t kAfterPhy = advance(HasPhy, kAfterGmac);
  static constexpr uint8_t kAfterAdc = advance(HasAdc, kAfterPhy);
  static constexpr uint8_t kAfterAes = advance(HasAes, kAfterAdc);
  static constexpr uint8_t kAfterPukcc = advance(HasPukcc, kAfterAes);
  static constexpr uint8_t kAfterTrng = advance(HasTrng, kAfterPukcc);

public:
  static constexpr uint8_t Sercom0 = assign(HasSercom0, kBase);
  static constexpr uint8_t Sercom1 = assign(HasSercom1, kAfterSercom0);
  static constexpr uint8_t Sercom2 = assign(HasSercom2, kAfterSercom1);
  static constexpr uint8_t Sercom3 = assign(HasSercom3, kAfterSercom2);
  static constexpr uint8_t Sercom4 = assign(HasSercom4, kAfterSercom3);
  static constexpr uint8_t Sercom5 = assign(HasSercom5, kAfterSercom4);
  static constexpr uint8_t Sercom6 = assign(HasSercom6, kAfterSercom5);
  static constexpr uint8_t Sercom7 = assign(HasSercom7, kAfterSercom6);
  static constexpr uint8_t Usb = assign(HasUsb, kAfterSercom7);
  static constexpr uint8_t Gmac = assign(HasGmac, kAfterUsb);
  static constexpr uint8_t Phy = assign(HasPhy, kAfterGmac);
  static constexpr uint8_t Adc = assign(HasAdc, kAfterPhy);
  static constexpr uint8_t Aes = assign(HasAes, kAfterAdc);
  static constexpr uint8_t Pukcc = assign(HasPukcc, kAfterAes);
  static constexpr uint8_t Trng = assign(HasTrng, kAfterPukcc);
  static constexpr uint8_t Rtc = assign(HasRtc, kAfterTrng);
  static constexpr uint8_t Count = advance(HasRtc, kAfterTrng);

  static constexpr bool isAvailable(uint8_t channel) {
    return channel != kUnavailable;
  }
};
