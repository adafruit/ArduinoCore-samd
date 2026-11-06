#pragma once
#include <stdint.h>
// Observes an existing DMA channel; its owner retains descriptors and callbacks.
class DmacCrc {
public:
  enum class Polynomial : uint8_t { Crc16, Crc32 };
  enum class Status : uint8_t { Ok, Busy, InvalidArgument, NotConfigured, NotOwner, TransferFailed };
  // Call after allocation/configuration, before enabling the DMA channel.
  static Status begin(uint8_t channel, Polynomial polynomial, uint32_t seed);
  // Call after completion or abort. Busy retains ownership until the channel stops.
  // Only the successful DMA completion path may pass transferSucceeded=true.
  static Status finish(uint8_t channel, bool transferSucceeded, uint32_t& checksum);
  // Uses the hardware CRC input register when bytes are transferred without DMA.
  // Enables the DMAC clock without allocating a DMA channel; owner retains its token.
  static Status beginInput(uint8_t owner, Polynomial polynomial, uint32_t seed);
  static Status writeInput(uint8_t owner, uint8_t byte);
  static Status finishInput(uint8_t owner, bool transferSucceeded, uint32_t& checksum);
};
