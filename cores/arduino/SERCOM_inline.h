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
	if (txn && txn->txPtr == nullptr) return false;
	
#ifdef USE_ZERODMA
	if (isWireDma()) {
		if (!_dmaTxActive)
			dmaStartTx(txn->txPtr, &sercom->I2CM.DATA.reg, _wire.txnLength);
		return true;
	}
#endif
	if (_wire.txnIndex < _wire.txnLength) {
		sercom->I2CM.DATA.bit.DATA = txn->txPtr[_wire.txnIndex++];
		return true;
	}

	return false;
}

inline bool SERCOM::isMasterWIRE( void ) { return sercom->I2CM.CTRLA.bit.MODE == I2C_MASTER_OPERATION; }
inline bool SERCOM::isSlaveWIRE( void ) { return sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION; }

inline bool SERCOM::isBusIdleWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE; }
inline bool SERCOM::isBusOwnerWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE; }
inline bool SERCOM::isBusUnknownWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_UNKNOWN_STATE; }
inline bool SERCOM::isArbLostWIRE( void ) { return sercom->I2CM.STATUS.bit.ARBLOST == 1; }
inline bool SERCOM::isBusBusyWIRE( void ) { return sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_BUSY_STATE; }
inline bool SERCOM::isDataReadyWIRE( void ) { return sercom->I2CS.INTFLAG.bit.DRDY; }
inline bool SERCOM::isStopDetectedWIRE( void ) { return sercom->I2CS.INTFLAG.bit.PREC; }
inline bool SERCOM::isRestartDetectedWIRE( void ) { return sercom->I2CS.INTFLAG.bit.SR; }
inline bool SERCOM::isAddressMatch( void ) { return sercom->I2CS.INTFLAG.bit.AMATCH; }

inline void SERCOM::prepareNackBitWIRE( void ) { sercom->I2CM.CTRLB.bit.ACKACT = 1; }
inline void SERCOM::prepareAckBitWIRE( void ) { sercom->I2CM.CTRLB.bit.ACKACT = 0; }
inline void SERCOM::prepareCommandBitsWire(uint8_t cmd)
{
	if (isMasterWIRE()) {
		sercom->I2CM.CTRLB.bit.CMD = cmd;
		while (sercom->I2CM.SYNCBUSY.bit.SYSOP)
		{
			// Waiting for synchronization
		}
	} else {
		sercom->I2CS.CTRLB.bit.CMD = cmd;
	}
}
inline int SERCOM::availableWIRE( void ) { return isMasterWIRE() ? sercom->I2CM.INTFLAG.bit.SB : sercom->I2CS.INTFLAG.bit.DRDY; }

inline bool SERCOM::readDataWIRE( void )
{
	SercomTxn* txn = _wire.currentTxn;
	if (txn && txn->rxPtr == nullptr) return false;

	if (isMasterWIRE() && (_wire.txnIndex + 1 >= _wire.txnLength))
		sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT;
	else
		sercom->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;

#ifdef USE_ZERODMA
	if (isWireDma()) {
		if (!_dmaRxActive)
			dmaStartRx(txn->rxPtr, &sercom->I2CM.DATA.reg, _wire.txnLength);
		return true;
	}
#endif

	if ( _wire.txnIndex >= _wire.txnLength)
		return false;

	txn->rxPtr[_wire.txnIndex++] = sercom->I2CM.DATA.bit.DATA;
	return true;
}

inline uint8_t SERCOM::getINTFLAG( void ) const { return sercom->I2CM.INTFLAG.reg; }
inline uint16_t SERCOM::getSTATUS( void ) const { return sercom->I2CM.STATUS.reg; }
inline void SERCOM::clearINTFLAG( void ) { sercom->I2CM.INTFLAG.reg = 0xFF; }

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

	if (inst->isMasterWIRE())
		inst->sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(WIRE_MASTER_ACT_STOP);
	else
		inst->sercom->I2CS.CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x3);
	inst->_wire.returnValue = SercomWireError::SUCCESS;
	inst->_dmaTxActive = false;
	SERCOM::setPending((uint8_t)inst->getSercomIndex());
}

inline void SERCOM::dmaRxCallbackWIRE(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, false);
	if (!inst) return; 

	if (inst->isMasterWIRE())
		inst->sercom->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_CMD(WIRE_MASTER_ACT_STOP);
	else
		inst->sercom->I2CS.CTRLB.reg |= SERCOM_I2CS_CTRLB_CMD(0x3);
	inst->_wire.txnIndex = inst->_wire.txnLength;
	inst->_wire.returnValue = SercomWireError::SUCCESS;
	inst->_dmaRxActive = false;
	SERCOM::setPending((uint8_t)inst->getSercomIndex());
}

inline void SERCOM::dmaTxCallbackSPI(Adafruit_ZeroDMA* dma)
{
	SERCOM* inst = findDmaOwner(dma, true);
	if (!inst) return;
	inst->_spi.dmaTxDone = true;
	if (inst->_spi.dmaNeedRx && !inst->_spi.dmaRxDone)
		return;
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
	inst->_spi.returnValue = SercomSpiError::SUCCESS;
	SERCOM::setPending((uint8_t)inst->getSercomIndex());
}
#endif

#endif // __cplusplus

#endif // SERCOM_INLINE_H
