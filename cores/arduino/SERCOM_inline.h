#ifndef SERCOM_INLINE_H
#define SERCOM_INLINE_H

#ifdef __cplusplus

inline void SERCOM::waitSyncBusyEnable( void )
{
#if defined(__SAME53__) || defined(__SAME54__)
	while (sercom->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_ENABLE_Msk) ;
#else
	while (sercom->I2CM.SYNCBUSY.bit.ENABLE != 0) ;
#endif // __SAME53__ / __SAME54__
}

inline void SERCOM::waitSyncBusySwrst( void )
{
#if defined(__SAME53__) || defined(__SAME54__)
	while ((sercom->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_SWRST_Msk) ||
	       (sercom->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SWRST_Msk)) ;
#else
	while (sercom->I2CM.CTRLA.bit.SWRST || sercom->I2CM.SYNCBUSY.bit.SWRST) ;
#endif // __SAME53__ / __SAME54__
}

inline void SERCOM::waitSyncBusySysOp( void )
{
#if defined(__SAME53__) || defined(__SAME54__)
	while (sercom->I2CM.SERCOM_SYNCBUSY & SERCOM_I2CM_SYNCBUSY_SYSOP_Msk) ;
#else
	while (sercom->I2CM.SYNCBUSY.bit.SYSOP != 0) ;
#endif // __SAME53__ / __SAME54__
}

inline void SERCOM::waitSyncBusyCtrlB( void )
{
#if defined(__SAME53__) || defined(__SAME54__)
	while (sercom->SPIM.SERCOM_SYNCBUSY & SERCOM_SPIM_SYNCBUSY_CTRLB_Msk) ;
#else
	while (sercom->SPI.SYNCBUSY.bit.CTRLB != 0) ;
#endif // __SAME53__ / __SAME54__
}

inline void SERCOM::enableSERCOM( void )
{
	// UART, SPI, I2CS, and I2CM use the same enable bit
#if defined(__SAME53__) || defined(__SAME54__)
	sercom->I2CM.SERCOM_CTRLA |= SERCOM_I2CM_CTRLA_ENABLE_Msk;
#else
	sercom->I2CM.CTRLA.bit.ENABLE = 1;
#endif // __SAME53__ / __SAME54__
	waitSyncBusyEnable();
}

inline void SERCOM::disableSERCOM( void )
{
	// UART, SPI, I2CS, and I2CM use the same enable bit
#if defined(__SAME53__) || defined(__SAME54__)
	sercom->I2CM.SERCOM_CTRLA &= ~SERCOM_I2CM_CTRLA_ENABLE_Msk;
#else
	sercom->I2CM.CTRLA.bit.ENABLE = 0;
#endif // __SAME53__ / __SAME54__
	waitSyncBusyEnable();
}

inline void SERCOM::enableWIRE( void )
{
	enableSERCOM();

	// Setting bus idle mode
#if defined(__SAME53__) || defined(__SAME54__)
	sercom->I2CM.SERCOM_STATUS =
		(sercom->I2CM.SERCOM_STATUS & ~SERCOM_I2CM_STATUS_BUSSTATE_Msk) |
		SERCOM_I2CM_STATUS_BUSSTATE(WIRE_IDLE_STATE);
#else
	sercom->I2CM.STATUS.bit.BUSSTATE = 1;
#endif // __SAME53__ / __SAME54__
	waitSyncBusySysOp();
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
#if defined(__SAME53__) || defined(__SAME54__)
			value = dmaStartTx(txn->txPtr, &sercom->I2CM.SERCOM_DATA, _wire.txnLength);
#else
			value = dmaStartTx(txn->txPtr, &sercom->I2CM.DATA.reg, _wire.txnLength);
#endif // __SAME53__ / __SAME54__
		return value == DmaStatus::Ok;
	}
#endif // USE_ZERODMA

	// Wait for DATA to sync out of the ISR and clear MB
#if defined(__SAME53__) || defined(__SAME54__)
	sercom->I2CM.SERCOM_DATA = txn->txPtr[_wire.txnIndex++];
#else
	sercom->I2CM.DATA.reg = txn->txPtr[_wire.txnIndex++];
#endif // __SAME53__ / __SAME54__

	// Return false when the last byte has been consumed so the caller can
	// issue STOP / complete the transaction without waiting for another SB.
	return (_wire.txnIndex < _wire.txnLength);
}

inline void SERCOM::prepareCommandBitsWIRE(uint8_t cmd)
{
	if (isMasterWIRE() && cmd == WIRE_MASTER_ACT_STOP)
		markBusReleasePendingWIRE();
#if defined(__SAME53__) || defined(__SAME54__)
	sercom->I2CM.SERCOM_CTRLB =
		(sercom->I2CM.SERCOM_CTRLB & ~SERCOM_I2CM_CTRLB_CMD_Msk) |
		SERCOM_I2CM_CTRLB_CMD(cmd);
#else
	sercom->I2CM.CTRLB.bit.CMD = cmd;
#endif // __SAME53__ / __SAME54__
	if (isMasterWIRE())
		waitSyncBusySysOp();
}

inline void SERCOM::prepareSlaveCommandBitsWIRE(uint8_t cmd)
{
#if defined(__SAME53__) || defined(__SAME54__)
	sercom->I2CS.SERCOM_CTRLB =
		(sercom->I2CS.SERCOM_CTRLB & ~SERCOM_I2CS_CTRLB_CMD_Msk) |
		SERCOM_I2CS_CTRLB_CMD(cmd);
#else
	sercom->I2CS.CTRLB.bit.CMD = cmd;
#endif // __SAME53__ / __SAME54__
}

inline bool SERCOM::readDataWIRE( void )
{
	SercomTxn* txn = _wire.currentTxn;
	if (txn == nullptr || txn->rxPtr == nullptr) return false;

#ifdef USE_ZERODMA
		if (isDmaWIRE()) {
			DmaStatus value = DmaStatus::StartFailed;
			if (!_dmaRxActive && !_dmaTxActive)
#if defined(__SAME53__) || defined(__SAME54__)
				value = dmaStartRx(txn->rxPtr, &sercom->I2CM.SERCOM_DATA, _wire.txnLength);
#else
				value = dmaStartRx(txn->rxPtr, &sercom->I2CM.DATA.reg, _wire.txnLength);
#endif // __SAME53__ / __SAME54__
			return value == DmaStatus::Ok;
		}
#endif // USE_ZERODMA

	bool isMaster = isMasterWIRE();

	if (isMaster) {
		if (_wire.txnIndex == (_wire.txnLength - 1)) {
			uint8_t cmd = txn->config & I2C_CFG_STOP ? WIRE_MASTER_ACT_STOP : WIRE_MASTER_ACT_NO_ACTION;
			if (cmd == WIRE_MASTER_ACT_STOP)
				markBusReleasePendingWIRE();
#if defined(__SAME53__) || defined(__SAME54__)
			sercom->I2CM.SERCOM_CTRLB =
				sercom->I2CM.SERCOM_CTRLB |
				SERCOM_I2CM_CTRLB_ACKACT_Msk |
				SERCOM_I2CM_CTRLB_CMD(cmd);
#else
			sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT | SERCOM_I2CM_CTRLB_CMD(cmd); // NACK the last byte and send STOP if requested
#endif // __SAME53__ / __SAME54__
		}
		else
			prepareAckBitWIRE(); // ACK bytes otherwise for non-SCLSM mode
	}
	else {
		// Slave mode: set ACKACT BEFORE reading DATA (SMEN auto-receives based on ACKACT)
		if (_wire.txnIndex >= _wire.txnLength)
			prepareNackBitWIRE(); // NACK if buffer full
		else
			prepareAckBitWIRE();  // ACK if room available
	}

	// Read DATA register (accesses auto-trigger bus operation based on ACKACT/SMEN)
#if defined(__SAME53__) || defined(__SAME54__)
	txn->rxPtr[_wire.txnIndex++] = sercom->I2CM.SERCOM_DATA;
#else
	txn->rxPtr[_wire.txnIndex++] = sercom->I2CM.DATA.reg;
#endif // __SAME53__ / __SAME54__

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
#if defined(__SAME53__) || defined(__SAME54__)
			value = dmaStartTx(txn->txPtr, &sercom->SPIM.SERCOM_DATA, _spi.length);
#else
			value = dmaStartTx(txn->txPtr, &sercom->SPI.DATA.reg, _spi.length);
#endif // __SAME53__ / __SAME54__
		return value == DmaStatus::Ok;
	}
#endif // USE_ZERODMA

	// Byte-by-byte: Write DATA register
#if defined(__SAME53__) || defined(__SAME54__)
	sercom->SPIM.SERCOM_DATA = txn->txPtr[_spi.index++];
#else
	sercom->SPI.DATA.bit.DATA = txn->txPtr[_spi.index++];
#endif // __SAME53__ / __SAME54__

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
#if defined(__SAME53__) || defined(__SAME54__)
			value = dmaStartRx(txn->rxPtr, &sercom->SPIM.SERCOM_DATA, _spi.length);
#else
			value = dmaStartRx(txn->rxPtr, &sercom->SPI.DATA.reg, _spi.length);
#endif // __SAME53__ / __SAME54__
		return value == DmaStatus::Ok;
	}
#endif // USE_ZERODMA

	// Byte-by-byte: Read DATA register
#if defined(__SAME53__) || defined(__SAME54__)
	txn->rxPtr[_spi.index - 1] = sercom->SPIM.SERCOM_DATA;
#else
	txn->rxPtr[_spi.index - 1] = sercom->SPI.DATA.bit.DATA;
#endif // __SAME53__ / __SAME54__

	// Return false when all bytes consumed
	return (_spi.index < _spi.length);
}

inline void SERCOM::setTxnWIRE(SercomTxn* txn)
{
	_wire.currentTxn = txn;
	_wire.txnIndex = 0;
	_wire.returnValue = SercomWireError::SUCCESS;

	if (txn)
		_wire.txnLength = txn->length;
#ifdef USE_ZERODMA
#if defined(__SAME53__) || defined(__SAME54__)
	const bool sclsm =
	    (sercom->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_SCLSM_Msk) != 0;
#else
	const bool sclsm = sercom->I2CM.CTRLA.bit.SCLSM;
#endif // __SAME53__ / __SAME54__
	const size_t length = txn ? txn->length : 0u;
	const bool dmaEligible = length > 0u && length <= 255u;
	setDmaWIRE(dmaEligible);
	if (sclsm && !dmaEligible)
		_wire.returnValue = SercomWireError::DATA_TOO_LONG;

	if (isDmaWIRE())
		_wire.txnLength = (_wire.txnLength < 255u) ? _wire.txnLength : 255u;
#else
	_wire.useDma = false;

#if defined(__SAME53__) || defined(__SAME54__)
	if (sercom->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_SCLSM_Msk)
#else
	if (sercom->I2CM.CTRLA.bit.SCLSM)
#endif // __SAME53__ / __SAME54__
		_wire.returnValue = SercomWireError::DATA_TOO_LONG;
#endif // USE_ZERODMA
	if (!_wire.active)
		_wire.retryCount = 0;

	_wire.active = true;
}

#if defined(__SAME53__) || defined(__SAME54__)

inline void SERCOM::prepareNackBitWIRE( void ) { sercom->I2CM.SERCOM_CTRLB |= SERCOM_I2CM_CTRLB_ACKACT_Msk; }
inline void SERCOM::prepareAckBitWIRE( void ) { sercom->I2CM.SERCOM_CTRLB &= ~SERCOM_I2CM_CTRLB_ACKACT_Msk; }
inline bool SERCOM::isMasterWIRE( void ) { return (sercom->I2CM.SERCOM_CTRLA & SERCOM_I2CM_CTRLA_MODE_Msk) == SERCOM_I2CM_CTRLA_MODE_I2C_MASTER; }
inline bool SERCOM::isSlaveWIRE( void ) { return (sercom->I2CS.SERCOM_CTRLA & SERCOM_I2CS_CTRLA_MODE_Msk) == SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE; }
inline bool SERCOM::isBusIdleWIRE( void ) { return ((sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >> SERCOM_I2CM_STATUS_BUSSTATE_Pos) == WIRE_IDLE_STATE; }
inline bool SERCOM::isBusOwnerWIRE( void ) { return ((sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >> SERCOM_I2CM_STATUS_BUSSTATE_Pos) == WIRE_OWNER_STATE; }
inline bool SERCOM::isBusUnknownWIRE( void ) { return ((sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >> SERCOM_I2CM_STATUS_BUSSTATE_Pos) == WIRE_UNKNOWN_STATE; }
inline bool SERCOM::isArbLostWIRE( void ) { return (sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_ARBLOST_Msk) != 0; }
inline bool SERCOM::isBusBusyWIRE( void ) { return ((sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_BUSSTATE_Msk) >> SERCOM_I2CM_STATUS_BUSSTATE_Pos) == WIRE_BUSY_STATE; }
inline bool SERCOM::isDataReadyWIRE( void ) { return (sercom->I2CS.SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_DRDY_Msk) != 0; }
inline bool SERCOM::isStopDetectedWIRE( void ) { return (sercom->I2CS.SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_PREC_Msk) != 0; }
inline bool SERCOM::isRestartDetectedWIRE( void ) { return (sercom->I2CS.SERCOM_STATUS & SERCOM_I2CS_STATUS_SR_Msk) != 0; }
inline bool SERCOM::isAddressMatch( void ) { return (sercom->I2CS.SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_AMATCH_Msk) != 0; }
inline bool SERCOM::isMasterReadOperationWIRE( void ) { return (sercom->I2CS.SERCOM_STATUS & SERCOM_I2CS_STATUS_DIR_Msk) != 0; }
inline bool SERCOM::isRXNackReceivedWIRE( void ) { return (sercom->I2CM.SERCOM_STATUS & SERCOM_I2CM_STATUS_RXNACK_Msk) != 0; }
inline int SERCOM::availableWIRE( void ) { return isMasterWIRE() ? ((sercom->I2CM.SERCOM_INTFLAG & SERCOM_I2CM_INTFLAG_SB_Msk) != 0) : ((sercom->I2CS.SERCOM_INTFLAG & SERCOM_I2CS_INTFLAG_DRDY_Msk) != 0); }
inline bool SERCOM::isDBGSTOP( void ) const { return (sercom->I2CM.SERCOM_DBGCTRL & SERCOM_I2CM_DBGCTRL_DBGSTOP_Msk) != 0; }
inline void SERCOM::setDBGSTOP( bool stop ) { if (stop) sercom->I2CM.SERCOM_DBGCTRL |= SERCOM_I2CM_DBGCTRL_DBGSTOP_Msk; else sercom->I2CM.SERCOM_DBGCTRL &= ~SERCOM_I2CM_DBGCTRL_DBGSTOP_Msk; }

#else

inline void SERCOM::prepareNackBitWIRE( void ) { sercom->I2CM.CTRLB.bit.ACKACT = 1; }
inline void SERCOM::prepareAckBitWIRE( void ) { sercom->I2CM.CTRLB.bit.ACKACT = 0; }
inline bool SERCOM::isMasterWIRE( void ) { return sercom->I2CM.CTRLA.bit.MODE == I2C_MASTER_OPERATION; }
inline bool SERCOM::isSlaveWIRE( void ) { return sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION; }
inline bool SERCOM::isBusIdleWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE; }
inline bool SERCOM::isBusOwnerWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE; }
inline bool SERCOM::isBusUnknownWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_UNKNOWN_STATE; }
inline bool SERCOM::isArbLostWIRE( void ) { return sercom->I2CM.STATUS.bit.ARBLOST == 1; }
inline bool SERCOM::isBusBusyWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_BUSY_STATE; }
inline bool SERCOM::isDataReadyWIRE( void ) { return sercom->I2CS.INTFLAG.bit.DRDY; }
inline bool SERCOM::isStopDetectedWIRE( void ) { return sercom->I2CS.INTFLAG.bit.PREC; }
inline bool SERCOM::isRestartDetectedWIRE( void ) { return sercom->I2CS.STATUS.bit.SR; }
inline bool SERCOM::isAddressMatch( void ) { return sercom->I2CS.INTFLAG.bit.AMATCH; }
inline bool SERCOM::isMasterReadOperationWIRE( void ) { return sercom->I2CS.STATUS.bit.DIR; }
inline bool SERCOM::isRXNackReceivedWIRE( void ) { return sercom->I2CM.STATUS.bit.RXNACK; }
inline int SERCOM::availableWIRE( void ) { return isMasterWIRE() ? sercom->I2CM.INTFLAG.bit.SB : sercom->I2CS.INTFLAG.bit.DRDY; }
inline bool SERCOM::isDBGSTOP( void ) const { return sercom->I2CM.DBGCTRL.bit.DBGSTOP; }
inline void SERCOM::setDBGSTOP( bool stop ) { sercom->I2CM.DBGCTRL.bit.DBGSTOP = stop; }

#endif // __SAME53__ / __SAME54__

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
	if (!inst->_wire.active || inst->_wire.currentTxn == nullptr ||
			!inst->_wire.useDma) return;

	// When using ADDR.LENEN mode, the hardware automatically generates STOP
	// after ADDR.LEN bytes are transferred (datasheet §28.6.4.1.2).
	// If the slave NACKs a master-write transaction before ADDR.LEN bytes, STOP
	// is generated automatically and STATUS.LENERR is raised with INTFLAG.ERROR.

	inst->_wire.txnIndex = inst->_wire.txnLength;
	inst->_dmaTxActive = false;
	if (inst->isSlaveWIRE()) {
		// DMA has written the final DATA byte, but the master's ACK/NACK is
		// reported at the following DRDY boundary. Restore that one interrupt
		// so onService() can release the slave transaction physically.
#if defined(__SAME53__) || defined(__SAME54__)
		inst->enableInterrupts(SERCOM_I2CS_INTENSET_DRDY_Msk);
#else
		inst->enableInterrupts(SERCOM_I2CS_INTENSET_DRDY);
#endif
	} else {
		inst->deferStopWIRE(SercomWireError::SUCCESS);
	}
}

inline void SERCOM::dmaRxCallbackWIRE(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, false);
	if (!inst) return;
	if (!inst->_wire.active || inst->_wire.currentTxn == nullptr ||
			!inst->_wire.useDma) return;

	// When using ADDR.LENEN mode, the hardware automatically generates NACK+STOP
	// after ADDR.LEN bytes are transferred (datasheet §28.6.4.1.2).
	// Do NOT issue a manual STOP command - it conflicts with the automatic sequence.
	// The STOP is generated by hardware, not by software CMD write.

	inst->_wire.txnIndex = inst->_wire.txnLength;
	inst->_dmaRxActive = false;
	if (inst->isSlaveWIRE())
		inst->deferStopWIRE(SercomWireError::SUCCESS);
	else
		inst->deferStopWIRE(SercomWireError::SUCCESS);
}

inline void SERCOM::dmaErrorCallbackWIRE(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, true);
	if (!inst)
		inst = findDmaOwner(dma, false);
	if (!inst) return;
	if (!inst->_wire.active || inst->_wire.currentTxn == nullptr ||
			!inst->_wire.useDma) return;

	inst->dmaAbortTx();
	inst->dmaAbortRx();
	inst->abortWIRE(SercomWireError::DMA_ERROR);
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
#endif // USE_ZERODMA

#endif // __cplusplus

#endif // SERCOM_INLINE_H
