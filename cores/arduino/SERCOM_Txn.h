#ifndef _SERCOM_TXN_H_
#define _SERCOM_TXN_H_

#include <stddef.h>
#include <stdint.h>

struct SercomTxn {
  uint16_t config;     // common + protocol-specific flags/fields
  uint16_t address;    // I2C addr, SPI CS/baud, UART RTS/CTS
  size_t length;
  const uint8_t* txPtr;
  uint8_t* rxPtr;
  void (*onComplete)(void* user, int status);
  void* user;
};

// I2C config flags and helpers
enum : uint16_t {
  I2C_CFG_READ    = 1u << 0,
  I2C_CFG_STOP    = 1u << 1,
  I2C_CFG_CRC     = 1u << 2,
  I2C_CFG_10BIT   = 1u << 3,
  I2C_CFG_RESTART = 1u << 4,
};

static inline uint16_t I2C_ADDR(uint16_t addr10) { return addr10 & 0x03FFu; }
static inline uint8_t I2C_ADDR7(uint16_t addr) { return (uint8_t)(addr & 0x7Fu); }

// SPI config flags and helpers
enum : uint16_t {
  SPI_CFG_READ    = 1u << 0,
  SPI_CFG_STOP    = 1u << 1,
  SPI_CFG_CRC     = 1u << 2,
  SPI_CFG_CPOL    = 1u << 3,
  SPI_CFG_CPHA    = 1u << 4,
  SPI_CFG_DORD    = 1u << 5,
  SPI_CFG_CHAR9   = 1u << 6,
  SPI_CFG_CS_HOLD = 1u << 7,
  SPI_CFG_DUMMY   = 1u << 8,
  SPI_CFG_TX_ONLY = 1u << 9,
};

static inline uint16_t SPI_ADDR_CS(uint8_t cs) { return (uint16_t)cs; }
static inline uint16_t SPI_ADDR_BAUD(uint8_t baud) { return (uint16_t)baud << 8; }
static inline uint8_t SPI_ADDR_GET_CS(uint16_t addr) { return (uint8_t)(addr & 0x00FFu); }
static inline uint8_t SPI_ADDR_GET_BAUD(uint16_t addr) { return (uint8_t)(addr >> 8); }

// UART config flags and helpers
enum : uint16_t {
  UART_CFG_READ    = 1u << 0,
  UART_CFG_STOP    = 1u << 1,
  UART_CFG_CRC     = 1u << 2,
  UART_CFG_BREAK   = 1u << 3,
  UART_CFG_MARK    = 1u << 4,
  UART_CFG_TX_ONLY = 1u << 5,
  UART_CFG_RX_ONLY = 1u << 6,
};

static inline uint16_t UART_ADDR_RTS(uint8_t pin) { return (uint16_t)pin; }
static inline uint16_t UART_ADDR_CTS(uint8_t pin) { return (uint16_t)pin << 8; }
static inline uint8_t UART_ADDR_GET_RTS(uint16_t addr) { return (uint8_t)(addr & 0x00FFu); }
static inline uint8_t UART_ADDR_GET_CTS(uint16_t addr) { return (uint8_t)(addr >> 8); }

static constexpr uint8_t SERCOM_PIN_NONE = 0xFFu;

#endif
