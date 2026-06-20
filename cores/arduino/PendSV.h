#pragma once

#include <array>
#include <stdint.h>

class PendSV {
  public:
    using ServiceFn = void (*)(uint8_t serviceId, void *context);
    static constexpr uint8_t kMaxServices = 32;
    static constexpr uint8_t kDispatchBudget = 16;
    static_assert(kMaxServices <= 32, "PendSV pending mask supports at most 32 services");

    static PendSV &instance();

    bool registerService(uint8_t serviceId, ServiceFn fn, void *context = nullptr);
    // Cancels queued work that has not started dispatching. If a callback was
    // already copied for dispatch, its context must remain valid until it returns.
    void clearService(uint8_t serviceId);
    void dispatchPending();
    void setPending(uint8_t serviceId);

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
