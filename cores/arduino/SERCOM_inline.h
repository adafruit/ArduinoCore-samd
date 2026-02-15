#ifndef SERCOM_INLINE_H
#define SERCOM_INLINE_H

#ifdef __cplusplus

inline void SERCOM::enableSERCOM( void )
{
	// UART, SPI, I2CS, and I2CM use the same enable bit
	sercom->I2CM.CTRLA.bit.ENABLE = 1;
	while (sercom->I2CM.SYNCBUSY.bit.ENABLE != 0) ;
}

inline void SERCOM::disableSERCOM( void )
{
	// UART, SPI, I2CS, and I2CM use the same enable bit
	sercom->I2CM.CTRLA.bit.ENABLE = 0;
	while (sercom->I2CM.SYNCBUSY.bit.ENABLE != 0) ;
}

inline void SERCOM::enableWIRE( void )
{
	enableSERCOM();

	// Setting bus idle mode
	sercom->I2CM.STATUS.bit.BUSSTATE = 1;
	while (sercom->I2CM.SYNCBUSY.bit.SYSOP != 0) ;
}

inline void SERCOM::deferStopWIRE(SercomWireError error)
{
	_wire.returnValue = error;
	setPending((uint8_t)getSercomIndex());
}

inline bool SERCOM::sendDataWIRE( void )
{
	SercomTxn* txn = _wire.currentTxn;
	if (txn == nullptr || txn->txPtr == nullptr) return false;
	
#ifdef USE_ZERODMA
	if (isDmaWIRE()) {
		DmaStatus value  = DmaStatus::StartFailed;
		if (!_dmaTxActive && !_dmaRxActive)
			value = dmaStartTx(txn->txPtr, &sercom->I2CM.DATA.reg, _wire.txnLength);
		return value == DmaStatus::Ok;
	}
#endif

	sercom->I2CM.DATA.reg = txn->txPtr[_wire.txnIndex++];

	if(isMasterWIRE())
		while (sercom->I2CM.SYNCBUSY.bit.SYSOP) ; // Wait for DATA to sync and clear MB
		
	// Return false when the last byte has been consumed so the caller can
	// issue STOP / complete the transaction without waiting for another SB.
	return (_wire.txnIndex < _wire.txnLength);
}

inline void SERCOM::prepareCommandBitsWIRE(uint8_t cmd)
{
	sercom->I2CM.CTRLB.bit.CMD = cmd;
	if (isMasterWIRE())
		while (sercom->I2CM.SYNCBUSY.bit.SYSOP) ; // Waiting for synchronization
}

inline bool SERCOM::readDataWIRE( void )
{
	SercomTxn* txn = _wire.currentTxn;
	if (txn == nullptr || txn->rxPtr == nullptr) return false;

#ifdef USE_ZERODMA
	if (isDmaWIRE()) {
		DmaStatus value = DmaStatus::StartFailed;
		if (!_dmaRxActive && !_dmaTxActive)
			value = dmaStartRx(txn->rxPtr, &sercom->I2CM.DATA.reg, _wire.txnLength);
		return value == DmaStatus::Ok;
	}
#endif

	bool isMaster = isMasterWIRE();

	if (isMaster) {
		if (_wire.txnIndex == (_wire.txnLength - 1)) {
			uint8_t cmd = txn->config & I2C_CFG_STOP ? WIRE_MASTER_ACT_STOP : WIRE_MASTER_ACT_NO_ACTION;
			sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT | SERCOM_I2CM_CTRLB_CMD(cmd); // NACK the last byte and send STOP if requested
			if (cmd == WIRE_MASTER_ACT_STOP)
				while (sercom->I2CM.SYNCBUSY.bit.SYSOP) ; // Wait for CMD to sync and clear SB
		}
		else
			prepareAckBitWIRE(); // ACK bytes otherwise for non-SCLSM mode
	}

	// Read DATA register (clears SB in Smart Mode)
	txn->rxPtr[_wire.txnIndex++] = sercom->I2CM.DATA.reg;

	if(isMaster)
		while (sercom->I2CM.SYNCBUSY.bit.SYSOP) ; // Wait for DATA to sync and clear SB

	return (_wire.txnIndex < _wire.txnLength);
}

inline bool SERCOM::sendDataSPI(void)
{
	SercomTxn* txn = _spi.currentTxn;
	if (txn == nullptr || txn->txPtr == nullptr) return false;

#ifdef USE_ZERODMA
	if (_spi.useDma) {
		DmaStatus value = DmaStatus::StartFailed;
		if (!_dmaTxActive && !_dmaRxActive)
			value = dmaStartTx(txn->txPtr, &sercom->SPI.DATA.reg, _spi.length);
		return value == DmaStatus::Ok;
	}
#endif

	// Byte-by-byte: Write DATA register
	sercom->SPI.DATA.bit.DATA = txn->txPtr[_spi.index++];

	// Return false when last byte consumed so caller can complete transaction
	return (_spi.index < _spi.length);
}

inline bool SERCOM::readDataSPI(void)
{
	SercomTxn* txn = _spi.currentTxn;
	if (txn == nullptr || txn->rxPtr == nullptr) return false;

#ifdef USE_ZERODMA
	if (_spi.useDma) {
		DmaStatus value = DmaStatus::StartFailed;
		if (!_dmaRxActive && !_dmaTxActive)
			value = dmaStartRx(txn->rxPtr, &sercom->SPI.DATA.reg, _spi.length);
		return value == DmaStatus::Ok;
	}
#endif

	// Byte-by-byte: Read DATA register
	txn->rxPtr[_spi.index - 1] = sercom->SPI.DATA.bit.DATA;

	// Return false when all bytes consumed
	return (_spi.index < _spi.length);
}

inline void SERCOM::setTxnWIRE(SercomTxn* txn, size_t length, bool useDma)
{
	_wire.currentTxn = txn;
	_wire.txnLength = length;
	_wire.txnIndex = 0;
	_wire.useDma = useDma;
}

#ifdef USE_ZERODMA
inline SERCOM* SERCOM::findDmaOwner(Adafruit_ZeroDMA* dma, bool tx)
{
	if (dma == nullptr) return nullptr;

	for (size_t i = 0; i < kSercomCount; ++i)
	{
		SERCOM* inst = s_instances[i];
		if (inst == nullptr)
			continue;
		if (tx)
		{
			if (inst->_dmaTx == dma)
				return inst;
		}
		else
		{
			if (inst->_dmaRx == dma)
				return inst;
		}
	}

	return nullptr;
}

inline void SERCOM::dmaTxCallbackWIRE(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, true);
	if (!inst) return;

	// When using ADDR.LENEN mode, the hardware automatically generates STOP
	// after ADDR.LEN bytes are transferred (datasheet §28.6.4.1.2).
	// If a NACK TOPis received by the client for a host write transaction before
	// ADDR.LEN bytes, a STOP will be automatically generated and the length error
	// (STATUS.LENERR) will be raised along with the INTFLAG.ERROR interrupt.re

	inst->_wire.txnIndex = inst->_wire.txnLength;
	inst->_dmaTxActive = false;
	inst->deferStopWIRE(SercomWireError::SUCCESS);
}

inline void SERCOM::dmaRxCallbackWIRE(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, false);
	if (!inst) return;
	
	// When using ADDR.LENEN mode, the hardware automatically generates NACK+STOP
	// after ADDR.LEN bytes are transferred (datasheet §28.6.4.1.2).
	// Do NOT issue a manual STOP command - it conflicts with the automatic sequence.
	// The STOP is generated by hardware, not by software CMD write.
	
	inst->_wire.txnIndex = inst->_wire.txnLength;
	inst->_dmaRxActive = false;
	inst->deferStopWIRE(SercomWireError::SUCCESS);
}

inline void SERCOM::dmaTxCallbackSPI(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, true);
	if (!inst) return;
	inst->_spi.dmaTxDone = true;
	if (inst->_spi.dmaNeedRx && !inst->_spi.dmaRxDone)
		return;
	inst->_dmaTxActive = false;
	inst->_spi.returnValue = SercomSpiError::SUCCESS;
	SERCOM::setPending((uint8_t)inst->getSercomIndex());
}

inline void SERCOM::dmaRxCallbackSPI(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, false);
	if (!inst) return;
	inst->_spi.dmaRxDone = true;
	if (inst->_spi.dmaNeedTx && !inst->_spi.dmaTxDone)
		return;
	inst->_dmaRxActive = false;
	inst->_spi.returnValue = SercomSpiError::SUCCESS;
	SERCOM::setPending((uint8_t)inst->getSercomIndex());
}

inline void SERCOM::dmaTxCallbackUART(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, true);
	if (!inst) return;
	inst->_uart.dmaTxDone = true;
	if (inst->_uart.dmaNeedRx && !inst->_uart.dmaRxDone)
		return;
	inst->_dmaTxActive = false;
	inst->_uart.returnValue = SercomUartError::SUCCESS;
	SERCOM::setPending((uint8_t)inst->getSercomIndex());
}

inline void SERCOM::dmaRxCallbackUART(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, false);
	if (!inst) return;
	inst->_uart.dmaRxDone = true;
	if (inst->_uart.dmaNeedTx && !inst->_uart.dmaTxDone)
		return;
	inst->_dmaRxActive = false;
	inst->_uart.returnValue = SercomUartError::SUCCESS;
	SERCOM::setPending((uint8_t)inst->getSercomIndex());
}
#endif

#endif // __cplusplus

#endif // SERCOM_INLINE_H
