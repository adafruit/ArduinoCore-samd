#ifndef SERCOM_WIRE_BUS_ERROR_POLICY_H
#define SERCOM_WIRE_BUS_ERROR_POLICY_H

#include <stdint.h>

namespace simio {
namespace wire {

enum class BusState : uint8_t {
  Unknown = 0,
  Idle = 1,
  Owner = 2,
  Busy = 3,
};

enum class BusErrorAction : uint8_t {
  ClearSlaveWarning,
  RestartQueueHead,
  WaitForBusState,
  WaitForArbitrationRelease,
  WaitAtCommandPoint,
  EstablishIdle,
  ResetPeripheralAndRestart,
  RetireAmbiguous,
  RetireTerminal,
};

struct BusErrorContext {
  bool master;
  BusState busState;
  bool arbitrationLost;
  bool commandReady;
  bool terminalCondition;
  bool replayAllowed;
  bool transactionActive;
};

constexpr BusErrorAction decideBusErrorAction(BusErrorContext context) {
  return !context.master
             ? BusErrorAction::ClearSlaveWarning
         : !context.transactionActive || !context.replayAllowed
             ? BusErrorAction::RetireAmbiguous
         : context.terminalCondition
             ? BusErrorAction::RetireTerminal
         : context.busState == BusState::Busy ||
                   context.busState == BusState::Idle
             ? BusErrorAction::RestartQueueHead
         : context.busState == BusState::Unknown
             ? BusErrorAction::EstablishIdle
         : context.arbitrationLost
             ? BusErrorAction::WaitForArbitrationRelease
         : context.commandReady ? BusErrorAction::WaitAtCommandPoint
                                : BusErrorAction::WaitForBusState;
}

constexpr uint32_t kBusErrorRecoveryDeadlineUs = 100000u;

constexpr bool busErrorDeadlineExpired(uint32_t startedUs, uint32_t nowUs,
                                       bool active) {
  return active &&
         static_cast<uint32_t>(nowUs - startedUs) >=
             kBusErrorRecoveryDeadlineUs;
}

constexpr BusErrorAction decideBusErrorWaitAction(BusState busState,
                                                   bool timedOut) {
  return busState == BusState::Busy || busState == BusState::Idle
             ? BusErrorAction::RestartQueueHead
         : timedOut ? BusErrorAction::ResetPeripheralAndRestart
                    : BusErrorAction::WaitForBusState;
}

} // namespace wire
} // namespace simio

#endif
