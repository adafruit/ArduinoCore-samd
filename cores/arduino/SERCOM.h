/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _SERCOM_CLASS_
#define _SERCOM_CLASS_

#include "sam.h"
#include "SERCOM_Txn.h"
#include "RingBuffer.h"
#include <array>

#ifdef USE_ZERODMA
class Adafruit_ZeroDMA;
#endif

// SAMD51 has configurable MAX_SPI, else use peripheral clock default.
// Update: changing MAX_SPI via compiler flags is DEPRECATED, because
// this affects ALL SPI peripherals including some that should NOT be
// changed (e.g. anything using SD card). Instead, use setClockSource().
// This is left here for compatibility w/interim MAX_SPI-dependent code:
#if defined(MAX_SPI)
  #define SERCOM_SPI_FREQ_REF (MAX_SPI * 2)
#else
  #define SERCOM_SPI_FREQ_REF 48000000ul
#endif
// Other SERCOM peripherals always use the 48 MHz clock
#define SERCOM_FREQ_REF       48000000ul
#define SERCOM_NVIC_PRIORITY  ((1<<__NVIC_PRIO_BITS) - 1)

#ifndef SERCOM_QUEUE_LENGTH
#define SERCOM_QUEUE_LENGTH 8
#endif

typedef enum
{
	UART_EXT_CLOCK = 0,
	UART_INT_CLOCK = 0x1u
} SercomUartMode;

typedef enum
{
	SPI_SLAVE_OPERATION = 0x2u,
	SPI_MASTER_OPERATION = 0x3u
} SercomSpiMode;

typedef enum
{
	I2C_SLAVE_OPERATION = 0x4u,
	I2C_MASTER_OPERATION = 0x5u
} SercomI2CMode;

typedef enum
{
	SERCOM_EVEN_PARITY = 0,
	SERCOM_ODD_PARITY,
	SERCOM_NO_PARITY
} SercomParityMode;

typedef enum
{
	SERCOM_STOP_BIT_1 = 0,
	SERCOM_STOP_BITS_2
} SercomNumberStopBit;

typedef enum
{
	MSB_FIRST = 0,
	LSB_FIRST
} SercomDataOrder;

typedef enum
{
	UART_CHAR_SIZE_8_BITS = 0,
	UART_CHAR_SIZE_9_BITS,
	UART_CHAR_SIZE_5_BITS = 0x5u,
	UART_CHAR_SIZE_6_BITS,
	UART_CHAR_SIZE_7_BITS
} SercomUartCharSize;

typedef enum
{
	SERCOM_RX_PAD_0 = 0,
	SERCOM_RX_PAD_1,
	SERCOM_RX_PAD_2,
	SERCOM_RX_PAD_3
} SercomRXPad;

typedef enum
{
	UART_TX_PAD_0 = 0x0ul,  // Only for UART
	UART_TX_PAD_2 = 0x1ul,  // Only for UART
	UART_TX_RTS_CTS_PAD_0_2_3 = 0x2ul,  // Only for UART with TX on PAD0, RTS on PAD2 and CTS on PAD3
} SercomUartTXPad;

typedef enum
{
	SAMPLE_RATE_x16 = 0x1,  // Fractional
	SAMPLE_RATE_x8  = 0x3,  // Fractional
} SercomUartSampleRate;

typedef enum
{
	SERCOM_SPI_MODE_0 = 0, // CPOL : 0 | CPHA : 0
	SERCOM_SPI_MODE_1,     // CPOL : 0 | CPHA : 1
	SERCOM_SPI_MODE_2,     // CPOL : 1 | CPHA : 0
	SERCOM_SPI_MODE_3      // CPOL : 1 | CPHA : 1
} SercomSpiClockMode;

typedef enum
{
	SPI_PAD_0_SCK_1 = 0,
	SPI_PAD_2_SCK_3,
	SPI_PAD_3_SCK_1,
	SPI_PAD_0_SCK_3
} SercomSpiTXPad;

typedef enum
{
	SPI_CHAR_SIZE_8_BITS = 0x0ul,
	SPI_CHAR_SIZE_9_BITS
} SercomSpiCharSize;

typedef enum
{
	WIRE_UNKNOWN_STATE = 0x0ul,
	WIRE_IDLE_STATE,
	WIRE_OWNER_STATE,
	WIRE_BUSY_STATE
} SercomWireBusState;

typedef enum
{
	WIRE_WRITE_FLAG = 0x0ul,
	WIRE_READ_FLAG
} SercomWireReadWriteFlag;

typedef enum
{
	WIRE_MASTER_ACT_NO_ACTION = 0,
	WIRE_MASTER_ACT_REPEAT_START,
	WIRE_MASTER_ACT_READ,
	WIRE_MASTER_ACT_STOP
} SercomMasterCommandWire;

typedef enum
{
	WIRE_MASTER_ACK_ACTION = 0,
	WIRE_MASTER_NACK_ACTION
} SercomMasterAckActionWire;

// SERCOM clock source override is available only on SAMD51 (not 21)
// but the enumeration is made regardless so user code doesn't need
// ifdefs or lengthy comments explaining the different situations --
// the clock-sourcing functions just compile to nothing on SAMD21.
typedef enum {
  SERCOM_CLOCK_SOURCE_FCPU,     // F_CPU clock (GCLK0)
  SERCOM_CLOCK_SOURCE_48M,      // 48 MHz peripheral clock (GCLK1) (standard)
  SERCOM_CLOCK_SOURCE_100M,     // 100 MHz peripheral clock (GCLK2)
  SERCOM_CLOCK_SOURCE_32K,      // XOSC32K clock (GCLK3)
  SERCOM_CLOCK_SOURCE_12M,      // 12 MHz peripheral clock (GCLK4)
  SERCOM_CLOCK_SOURCE_NO_CHANGE // Leave clock source setting unchanged
} SercomClockSource;

class SERCOM
{
	public:
		SERCOM(Sercom* s) ;
		void resetSERCOM( void ) ;
		inline void enableSERCOM( void ) ;
		inline void disableSERCOM( void ) ;
		inline void disableInterrupts(uint8_t mask) { sercom->I2CM.INTENCLR.reg = mask; }
		inline void enableInterrupts(uint8_t mask) { sercom->I2CM.INTENSET.reg = mask; }
		inline uint8_t getINTFLAG( void ) const { return sercom->I2CM.INTFLAG.reg; }
		inline uint16_t getSTATUS( void ) const { return sercom->I2CM.STATUS.reg; }
		inline void clearINTFLAG( void ) { sercom->I2CM.INTFLAG.reg = 0xFF; }

		/* ========== UART ========== */
		void initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate=0) ;
		void initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits) ;
		void initPads(SercomUartTXPad txPad, SercomRXPad rxPad) ;

		void resetUART( void ) ;
		void enableUART( void ) { enableSERCOM(); }
		void disableUART( void ) { disableSERCOM(); }
		void flushUART( void ) ;
		void clearStatusUART( void ) ;
		bool availableDataUART( void ) ;
		bool isBufferOverflowErrorUART( void ) ;
		bool isFrameErrorUART( void ) ;
		void clearFrameErrorUART( void ) ;
		bool isParityErrorUART( void ) ;
		bool isDataRegisterEmptyUART( void ) ;
		uint8_t readDataUART( void ) ;
		int writeDataUART(uint8_t data) ;
		bool isUARTError() ;
		void acknowledgeUARTError() ;
		void enableDataRegisterEmptyInterruptUART();
		void disableDataRegisterEmptyInterruptUART();
		bool enqueueUART(SercomTxn* txn);
		bool startTransmissionUART(void);
		SercomTxn* stopTransmissionUART(void);
		SercomTxn* stopTransmissionUART(SercomUartError error);
		void deferStopUART(SercomUartError error);

		/* ========== SPI ========== */
		void initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder) ;
		void initSPIClock(SercomSpiClockMode clockMode, uint32_t baudrate) ;
		void resetSPI( void ) ;
		void enableSPI( void ) { enableSERCOM(); }
		void disableSPI( void ) { disableSERCOM(); }
		void setDataOrderSPI(SercomDataOrder dataOrder) ;
		SercomDataOrder getDataOrderSPI( void ) ;
		void setBaudrateSPI(uint8_t divider) ;
		void setClockModeSPI(SercomSpiClockMode clockMode) ;
		uint8_t transferDataSPI(uint8_t data) ;
		bool isBufferOverflowErrorSPI( void ) ;
		bool isDataRegisterEmptySPI( void ) ;
		bool isTransmitCompleteSPI( void ) ;
		bool isReceiveCompleteSPI( void ) ;
		bool enqueueSPI(SercomTxn* txn);
		bool startTransmissionSPI(void);
		void deferStopSPI(SercomSpiError error);
		SercomTxn* stopTransmissionSPI(void);
		SercomTxn* stopTransmissionSPI(SercomSpiError error);
		inline SercomTxn* getCurrentTxnSPI(void) { return _spi.currentTxn; }
		inline const SercomTxn* getCurrentTxnSPI(void) const { return _spi.currentTxn; }
		inline size_t getTxnIndexSPI(void) const { return _spi.index; }
		inline size_t getTxnLengthSPI(void) const { return _spi.length; }
		inline bool isActiveSPI(void) const { return _spi.active; }
		inline void setTxnIndexSPI(size_t index) { _spi.index = index; }
		inline void setReturnValueSPI(SercomSpiError err) { _spi.returnValue = err; }
		inline bool sendDataSPI(void);
		inline bool readDataSPI(void);

		/* ========== WIRE ========== */
		void initSlaveWIRE(uint8_t address, bool enableGeneralCall = false, uint8_t speed = 0x0) ;
		void initSlaveWIRE(uint16_t address, bool enableGeneralCall = false, uint8_t speed = 0x0, bool enable10Bit = false) ;
		void initMasterWIRE(uint32_t baudrate) ;
		inline void setTxnWIRE(SercomTxn* txn);
		inline void setDmaWIRE(bool useDma) { _wire.useDma = useDma; }
		inline bool isDmaWIRE(void) const { return _wire.useDma; }
		void registerReceiveWIRE(void (*cb)(void* user, int length), void* user);
		void deferReceiveWIRE(int length);
		void setSlaveWIRE( void ) ;
		void setMasterWIRE( void ) ;

		void resetWIRE( void ) ;
		void clearQueueWIRE( void ) ;
		inline void enableWIRE( void ) ;
		inline void disableWIRE( void ) { disableSERCOM(); }
		void setBaudrateWIRE(uint32_t baudrate) ;
        inline void prepareNackBitWIRE( void ) { sercom->I2CM.CTRLB.bit.ACKACT = 1; }
		inline void prepareAckBitWIRE( void ) { sercom->I2CM.CTRLB.bit.ACKACT = 0; }
        inline void prepareCommandBitsWIRE(uint8_t cmd) ;
		SercomTxn* startTransmissionWIRE( void ) ;
		bool startTransmissionWIRE( uint8_t address, SercomWireReadWriteFlag flag ) = delete ;
		SercomTxn* stopTransmissionWIRE( void ) ;
		SercomTxn* stopTransmissionWIRE( SercomWireError error ) ;
		bool enqueueWIRE(SercomTxn* txn);
		void deferStopWIRE(SercomWireError error);

		inline bool sendDataWIRE( void ) ;
		inline bool isMasterWIRE( void ) { return sercom->I2CM.CTRLA.bit.MODE == I2C_MASTER_OPERATION; }
		inline bool isSlaveWIRE( void ) { return sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION; }
		inline bool isBusIdleWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE; }
		inline bool isBusOwnerWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE; }
		inline bool isBusUnknownWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_UNKNOWN_STATE; }
		inline bool isArbLostWIRE( void ) { return sercom->I2CM.STATUS.bit.ARBLOST == 1; }
		inline bool isBusBusyWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_BUSY_STATE; }
		inline bool isDataReadyWIRE( void ) { return sercom->I2CS.INTFLAG.bit.DRDY; }
		inline bool isStopDetectedWIRE( void ) { return sercom->I2CS.INTFLAG.bit.PREC; }
		inline bool isRestartDetectedWIRE( void ) { return sercom->I2CS.STATUS.bit.SR; }
		inline bool isAddressMatch( void ) { return sercom->I2CS.INTFLAG.bit.AMATCH; }
		inline bool isMasterReadOperationWIRE( void ) { return sercom->I2CS.STATUS.bit.DIR; }
        inline bool isRXNackReceivedWIRE( void ) { return sercom->I2CM.STATUS.bit.RXNACK; }
		inline int availableWIRE( void ) { return isMasterWIRE() ? sercom->I2CM.INTFLAG.bit.SB : sercom->I2CS.INTFLAG.bit.DRDY; }
		inline bool readDataWIRE( void );
		inline SercomTxn* getCurrentTxnWIRE(void) { return _wire.currentTxn; }
		inline const SercomTxn* getCurrentTxnWIRE(void) const { return _wire.currentTxn; }
		inline size_t getTxnIndexWIRE(void) const { return _wire.txnIndex; }
		inline size_t getTxnLengthWIRE(void) const { return _wire.txnLength; }

		inline bool isDBGSTOP( void ) const { return sercom->I2CM.DBGCTRL.bit.DBGSTOP; }
		inline void setDBGSTOP( bool stop ) { sercom->I2CM.DBGCTRL.bit.DBGSTOP = stop; }
		inline Sercom* getSercom() const { return sercom; }
		int8_t getSercomIndex(void) ;
        uint32_t getSercomFreqRef(void) ;

#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)
		// SERCOM clock source override is only available on
		// SAMD51 (not 21) ... but these functions are declared
		// regardless so user code doesn't need ifdefs or lengthy
		// comments explaining the different situations -- these
		// just compile to nothing on SAMD21.
		void setClockSource(int8_t idx, SercomClockSource src, bool core);
		SercomClockSource getClockSource(void) { return clockSource; };
		uint32_t getFreqRef(void) { return freqRef; };
#else
		// The equivalent SAMD21 dummy functions...
		void setClockSource(int8_t idx, SercomClockSource src, bool core) { (void)idx; (void)src; (void)core; };
		SercomClockSource getClockSource(void) { return SERCOM_CLOCK_SOURCE_FCPU; };
		uint32_t getFreqRef(void) { return F_CPU; };
#endif // SAMD51 vs 21

		enum class Role : uint8_t { None = 0, UART, SPI, I2C };
		using ServiceFn = SercomTxn* (SERCOM::*)();

		static bool claim(uint8_t sercomId, Role role);
		static void release(uint8_t sercomId);
		static bool registerService(uint8_t sercomId, ServiceFn fn);
		static void setPending(uint8_t sercomId);
		static void dispatchPending(void);

#ifdef USE_ZERODMA
		using DmaCallback = void (*)(Adafruit_ZeroDMA*);
		enum class DmaStatus : uint8_t {
			Ok = 0,
			NotConfigured,
			NullPtr,
			ZeroLen,
			AllocateFailed,
			DescriptorFailed,
			StartFailed
		};

		DmaStatus dmaInit(int8_t sercomId, uint8_t beatSize = 0); // beatSize: 0=byte (default), 1=halfword, 2=word
		void dmaSetCallbacks(DmaCallback txCb, DmaCallback rxCb);
		DmaStatus dmaStartTx(const void* src, volatile void* dstReg, size_t len);
		DmaStatus dmaStartRx(void* dst, volatile void* srcReg, size_t len);
		DmaStatus dmaStartDuplex(const void* txSrc, void* rxDst, volatile void* txReg, volatile void* rxReg, size_t len,
		                    const uint8_t* dummyTx = nullptr);
		void dmaRelease();
		void dmaResetDescriptors();
		void dmaAbortTx();
		void dmaAbortRx();
		bool dmaTxBusy() const;
		bool dmaRxBusy() const;
		DmaStatus dmaLastError() const;
		// DMA callbacks are protocol-owned (Wire/SPI/UART) and registered via dmaSetCallbacks().
		static inline SERCOM* findDmaOwner(Adafruit_ZeroDMA* dma, bool tx);

		// --- WIRE DMA callbacks (ISR-safe, PendSV-only completion) ---
		static inline void dmaTxCallbackWIRE(Adafruit_ZeroDMA* dma);
		static inline void dmaRxCallbackWIRE(Adafruit_ZeroDMA* dma);
		// --- SPI DMA callbacks (protocol-owned) ---
		static inline void dmaTxCallbackSPI(Adafruit_ZeroDMA* dma);
		static inline void dmaRxCallbackSPI(Adafruit_ZeroDMA* dma);
		// --- UART DMA callbacks (protocol-owned) ---
		static inline void dmaTxCallbackUART(Adafruit_ZeroDMA* dma);
		static inline void dmaRxCallbackUART(Adafruit_ZeroDMA* dma);
#endif // USE_ZERODMA

#ifdef SERCOM_STRICT_PADS
		enum class PadFunc : uint8_t {
			None = 0,
			UartTx,
			UartRx,
			SpiMosi,
			SpiMiso,
			SpiSck,
			SpiSs,
			WireSda,
			WireScl
		};

		static bool registerPads(uint8_t sercomId, const PadFunc (&pads)[4], bool muxFunctionD);
		static void clearPads(uint8_t sercomId);
#endif // SERCOM_STRICT_PADS

    private:
        Sercom *sercom;
        uint32_t freqRef = 48000000ul; // Frequency corresponding to clockSource
#if defined(__SAMD51__) || defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)
        SercomClockSource clockSource;
#endif

#if defined(SERCOM_INST_NUM) && (SERCOM_INST_NUM > 0)
		static constexpr size_t kSercomCount = SERCOM_INST_NUM;
#else
		#pragma message("SERCOM_INST_NUM not defined; SERCOM support disabled.")
		static constexpr size_t kSercomCount = 0;
#endif // SERCOM_INST_NUM

		struct SercomState {
			Role role = Role::None;
			ServiceFn service = nullptr;
#ifdef SERCOM_STRICT_PADS
			PadFunc pads[4] = { PadFunc::None, PadFunc::None, PadFunc::None, PadFunc::None };
			bool padsConfigured = false;
			bool muxFunctionD = false;
#endif // SERCOM_STRICT_PADS
		};

		static std::array<SercomState, kSercomCount> s_states;
		static std::array<SERCOM*, kSercomCount> s_instances;
		static volatile uint32_t s_pendingMask;
		uint8_t calculateBaudrateSynchronous(uint32_t baudrate) ;
		uint32_t division(uint32_t dividend, uint32_t divisor) ;
		void initClockNVIC( void ) ;
		void initWIRE(void) ;

		// Cached I2C master/slave configuration for fast role switching.
		// This can be expanded to support additional configuration options
		// as needed in the future. For now, it just provides default support
		// for (hs) mode and DMA.
		struct WireConfig {
            uint32_t ctrla = 0x00000000;             // default CTRLA value: auto ENABLE
			uint32_t ctrlb = SERCOM_I2CM_CTRLB_SMEN; // default CTRLB value: SMEN
			uint32_t baud  = 0x000000FF;             // default to lowest supported speed
			uint32_t addr  = 0x00000000;             // default address no GCEN, no ADDRMASK, 7-bit address only
			uint8_t masterSpeed = 0x0;               // default to lowest speed
			uint8_t slaveSpeed = 0x0;                // default to lowest speed
			bool inited = false;		             // whether initMaster/SlaveWIRE has been called
			bool useDma = false;		             // per transaction DMA use flag for Host/Client modes
			bool active = false;                     // active transaction in progress
			uint8_t retryCount = 0;                  // retry count for recoverable bus errors
			SercomWireError returnValue = SercomWireError::SUCCESS;
			SercomTxn* currentTxn = nullptr;
			size_t txnIndex = 0;
			size_t txnLength = 0;
		} _wire;

		struct SpiConfig {
			bool active = false;
			bool useDma = false;
			bool dmaNeedTx = false;
			bool dmaNeedRx = false;
			bool dmaTxDone = false;
			bool dmaRxDone = false;
			size_t index = 0;
			size_t length = 0;
			SercomTxn* currentTxn = nullptr;
			SercomSpiError returnValue = SercomSpiError::SUCCESS;
		} _spi;

		struct UartConfig {
			bool active = false;
			bool useDma = false;
			bool dmaNeedTx = false;
			bool dmaNeedRx = false;
			bool dmaTxDone = false;
			bool dmaRxDone = false;
			size_t index = 0;
			size_t length = 0;
			SercomTxn* currentTxn = nullptr;
			SercomUartError returnValue = SercomUartError::SUCCESS;
		} _uart;

		RingBufferN<SERCOM_QUEUE_LENGTH, SercomTxn*> _txnQueue;
		void (*_wireDeferredCb)(void* user, int length) = nullptr;
		void* _wireDeferredUser = nullptr;
		int _wireDeferredLength = 0;
		bool _wireDeferredPending = false;

#ifdef USE_ZERODMA
		Adafruit_ZeroDMA* _dmaTx = nullptr;
		Adafruit_ZeroDMA* _dmaRx = nullptr;
		DmacDescriptor* _dmaTxDesc = nullptr;
		DmacDescriptor* _dmaRxDesc = nullptr;
		DmaCallback _dmaTxCb = nullptr;
		DmaCallback _dmaRxCb = nullptr;
		uint8_t _dmaDummy = 0;
		uint8_t _dmaTxTrigger = 0;
		uint8_t _dmaRxTrigger = 0;
		bool _dmaConfigured = false;
		bool _dmaTxActive = false;
		bool _dmaRxActive = false;
		DmaStatus _dmaLastError = DmaStatus::Ok;
#endif
};

#include "SERCOM_inline.h"

#endif
