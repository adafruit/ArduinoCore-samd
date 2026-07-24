/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  SAMD51 support added by Adafruit - Copyright (c) 2018 Dean Miller for Adafruit Industries

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

#pragma once

#include <Arduino.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

typedef uint8_t ep_t;

class USBDevice_SAMD21G18x {
public:
#if defined(__SAME53__) || defined(__SAME54__)
	USBDevice_SAMD21G18x() : usb(USB_REGS->DEVICE) {
		// Empty
	}

	// USB Device function mapping
	// ---------------------------

	// Reset USB Device
	void reset();

	// Enable
	inline void enable() {
		usb.USB_CTRLA |= USB_CTRLA_ENABLE_Msk;
		while (usb.USB_SYNCBUSY & USB_SYNCBUSY_ENABLE_Msk);
	}
	inline void disable() {
		usb.USB_CTRLA &= ~USB_CTRLA_ENABLE_Msk;
		while (usb.USB_SYNCBUSY & USB_SYNCBUSY_ENABLE_Msk);
	}

	// USB mode (device/host)
	inline void setUSBDeviceMode() { usb.USB_CTRLA &= ~USB_CTRLA_MODE_Msk; }
	inline void setUSBHostMode() { usb.USB_CTRLA |= USB_CTRLA_MODE_Msk; }

	inline void runInStandby() { usb.USB_CTRLA |= USB_CTRLA_RUNSTDBY_Msk; }
	inline void noRunInStandby() { usb.USB_CTRLA &= ~USB_CTRLA_RUNSTDBY_Msk; }
	inline void wakeupHost() { usb.USB_CTRLB |= USB_DEVICE_CTRLB_UPRSM_Msk; }

	// USB speed
	inline void setFullSpeed() { setSpeedConfiguration(USB_DEVICE_CTRLB_SPDCONF_FS_Val); }
	inline void setLowSpeed()  { setSpeedConfiguration(USB_DEVICE_CTRLB_SPDCONF_LS_Val); }
	inline void setHiSpeed() { setFullSpeed(); }
	inline void setHiSpeedTestMode() { setFullSpeed(); }

	// Authorize attach if Vbus is present
	inline void attach() { usb.USB_CTRLB &= ~USB_DEVICE_CTRLB_DETACH_Msk; }
	inline void detach() { usb.USB_CTRLB |= USB_DEVICE_CTRLB_DETACH_Msk; }

	// USB Interrupts
	inline bool isEndOfResetInterrupt() { return usb.USB_INTFLAG & USB_DEVICE_INTFLAG_EORST_Msk; }
	inline void ackEndOfResetInterrupt() { usb.USB_INTFLAG = USB_DEVICE_INTFLAG_EORST_Msk; }
	inline void enableEndOfResetInterrupt() { usb.USB_INTENSET = USB_DEVICE_INTENSET_EORST_Msk; }
	inline void disableEndOfResetInterrupt() { usb.USB_INTENCLR = USB_DEVICE_INTENCLR_EORST_Msk; }

	inline bool isStartOfFrameInterrupt() { return usb.USB_INTFLAG & USB_DEVICE_INTFLAG_SOF_Msk; }
	inline void ackStartOfFrameInterrupt() { usb.USB_INTFLAG = USB_DEVICE_INTFLAG_SOF_Msk; }
	inline void enableStartOfFrameInterrupt() { usb.USB_INTENSET = USB_DEVICE_INTENSET_SOF_Msk; }
	inline void disableStartOfFrameInterrupt() { usb.USB_INTENCLR = USB_DEVICE_INTENCLR_SOF_Msk; }

	// USB Address
	inline void setAddress(uint32_t addr) { usb.USB_DADD = USB_DEVICE_DADD_DADD(addr) | USB_DEVICE_DADD_ADDEN_Msk; }
	inline void unsetAddress() { usb.USB_DADD = 0; }

	// Frame number
	inline uint16_t frameNumber() { return (usb.USB_FNUM & USB_DEVICE_FNUM_FNUM_Msk) >> USB_DEVICE_FNUM_FNUM_Pos; }

#else
	USBDevice_SAMD21G18x() : usb(USB->DEVICE) {
		// Empty
	}

	// USB Device function mapping
	// ---------------------------

	// Reset USB Device
	void reset();

	// Enable
	inline void enable() {
		usb.CTRLA.bit.ENABLE = 1;
#if defined(__SAMD51__) || defined(__SAME51__)
		while (usb.SYNCBUSY.reg & USB_SYNCBUSY_ENABLE); // Wait for sync
#endif
	}
	inline void disable() {
		usb.CTRLA.bit.ENABLE = 0;
#if defined(__SAMD51__) || defined(__SAME51__)
		while (usb.SYNCBUSY.reg & USB_SYNCBUSY_ENABLE); // Wait for sync
#endif
	}

	// USB mode (device/host)
	inline void setUSBDeviceMode() { usb.CTRLA.bit.MODE = USB_CTRLA_MODE_DEVICE_Val; }
	inline void setUSBHostMode() { usb.CTRLA.bit.MODE = USB_CTRLA_MODE_HOST_Val; }

	inline void runInStandby() { usb.CTRLA.bit.RUNSTDBY = 1; }
	inline void noRunInStandby() { usb.CTRLA.bit.RUNSTDBY = 0; }
	inline void wakeupHost() { usb.CTRLB.bit.UPRSM = 1; }

	// USB speed
	inline void setFullSpeed() { setSpeedConfiguration(USB_DEVICE_CTRLB_SPDCONF_FS_Val); }
	inline void setLowSpeed() { setSpeedConfiguration(USB_DEVICE_CTRLB_SPDCONF_LS_Val); }
	inline void setHiSpeed() { setSpeedConfiguration(USB_DEVICE_CTRLB_SPDCONF_HS_Val); }
	inline void setHiSpeedTestMode() { setSpeedConfiguration(USB_DEVICE_CTRLB_SPDCONF_HSTM_Val); }

	// Authorize attach if Vbus is present
	inline void attach() { usb.CTRLB.bit.DETACH = 0; }
	inline void detach() { usb.CTRLB.bit.DETACH = 1; }

	// USB Interrupts
	inline bool isEndOfResetInterrupt() { return usb.INTFLAG.bit.EORST; }
	inline void ackEndOfResetInterrupt() { usb.INTFLAG.reg = USB_DEVICE_INTFLAG_EORST; }
	inline void enableEndOfResetInterrupt() { usb.INTENSET.bit.EORST = 1; }
	inline void disableEndOfResetInterrupt() { usb.INTENCLR.bit.EORST = 1; }

	inline bool isStartOfFrameInterrupt() { return usb.INTFLAG.bit.SOF; }
	inline void ackStartOfFrameInterrupt() { usb.INTFLAG.reg = USB_DEVICE_INTFLAG_SOF; }
	inline void enableStartOfFrameInterrupt() { usb.INTENSET.bit.SOF = 1; }
	inline void disableStartOfFrameInterrupt() { usb.INTENCLR.bit.SOF = 1; }

	// USB Address
	inline void setAddress(uint32_t addr) { usb.DADD.bit.DADD = addr; usb.DADD.bit.ADDEN = 1; }
	inline void unsetAddress() { usb.DADD.bit.DADD = 0; usb.DADD.bit.ADDEN = 0; }

	// Frame number
	inline uint16_t frameNumber() { return usb.FNUM.bit.FNUM; }
#endif

	// Load calibration values
	void calibrate();

	// USB Device Endpoints function mapping
	// -------------------------------------

#if defined(__SAME53__) || defined(__SAME54__)
	// Config
	inline void epBank0SetType(ep_t ep, uint8_t type) {
		usb.DEVICE_ENDPOINT[ep].USB_EPCFG =
		    (usb.DEVICE_ENDPOINT[ep].USB_EPCFG & ~USB_DEVICE_EPCFG_EPTYPE0_Msk) |
		    USB_DEVICE_EPCFG_EPTYPE0(type);
	}
	inline void epBank1SetType(ep_t ep, uint8_t type) {
		usb.DEVICE_ENDPOINT[ep].USB_EPCFG =
		    (usb.DEVICE_ENDPOINT[ep].USB_EPCFG & ~USB_DEVICE_EPCFG_EPTYPE1_Msk) |
		    USB_DEVICE_EPCFG_EPTYPE1(type);
	}

	// Interrupts
	inline uint16_t epInterruptSummary() { return usb.USB_EPINTSMRY; }
	inline bool epBank0IsSetupReceived(ep_t ep) { return usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_RXSTP_Msk; }
	inline bool epBank0IsStalled(ep_t ep) { return usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_STALL0_Msk; }
	inline bool epBank1IsStalled(ep_t ep) { return usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_STALL1_Msk; }
	inline bool epBank0IsTransferComplete(ep_t ep) { return usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_TRCPT0_Msk; }
	inline bool epBank1IsTransferComplete(ep_t ep) { return usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG & USB_DEVICE_EPINTFLAG_TRCPT1_Msk; }
	inline void epBank0AckSetupReceived(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_RXSTP_Msk; }
	inline void epBank0AckStalled(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_STALL0_Msk; }
	inline void epBank1AckStalled(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_STALL1_Msk; }
	inline void epBank0AckTransferComplete(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT0_Msk; }
	inline void epBank1AckTransferComplete(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTFLAG = USB_DEVICE_EPINTFLAG_TRCPT1_Msk; }
	inline void epBank0EnableSetupReceived(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENSET = USB_DEVICE_EPINTENSET_RXSTP_Msk; }
	inline void epBank0EnableStalled(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENSET = USB_DEVICE_EPINTENSET_STALL0_Msk; }
	inline void epBank1EnableStalled(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENSET = USB_DEVICE_EPINTENSET_STALL1_Msk; }
	inline void epBank0EnableTransferComplete(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRCPT0_Msk; }
	inline void epBank1EnableTransferComplete(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENSET = USB_DEVICE_EPINTENSET_TRCPT1_Msk; }
	inline void epBank0DisableSetupReceived(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_RXSTP_Msk; }
	inline void epBank0DisableStalled(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_STALL0_Msk; }
	inline void epBank1DisableStalled(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_STALL1_Msk; }
	inline void epBank0DisableTransferComplete(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_TRCPT0_Msk; }
	inline void epBank1DisableTransferComplete(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPINTENCLR = USB_DEVICE_EPINTENCLR_TRCPT1_Msk; }

	// Status
	inline bool epBank0IsReady(ep_t ep) { return usb.DEVICE_ENDPOINT[ep].USB_EPSTATUS & USB_DEVICE_EPSTATUS_BK0RDY_Msk; }
	inline bool epBank1IsReady(ep_t ep) { return usb.DEVICE_ENDPOINT[ep].USB_EPSTATUS & USB_DEVICE_EPSTATUS_BK1RDY_Msk; }
	inline void epBank0SetReady(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK0RDY_Msk; }
	inline void epBank1SetReady(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_BK1RDY_Msk; }
	inline void epBank0ResetReady(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK0RDY_Msk; }
	inline void epBank1ResetReady(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_BK1RDY_Msk; }
	inline void epBank0SetStallReq(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_STALLRQ0_Msk; }
	inline void epBank1SetStallReq(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPSTATUSSET = USB_DEVICE_EPSTATUSSET_STALLRQ1_Msk; }
	inline void epBank0ResetStallReq(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ0_Msk; }
	inline void epBank1ResetStallReq(ep_t ep) { usb.DEVICE_ENDPOINT[ep].USB_EPSTATUSCLR = USB_DEVICE_EPSTATUSCLR_STALLRQ1_Msk; }

	// Packet
	inline uint16_t epBank0ByteCount(ep_t ep) { return (EP[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE & USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) >> USB_DEVICE_PCKSIZE_BYTE_COUNT_Pos; }
	inline uint16_t epBank1ByteCount(ep_t ep) { return (EP[ep].DEVICE_DESC_BANK[1].USB_PCKSIZE & USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) >> USB_DEVICE_PCKSIZE_BYTE_COUNT_Pos; }
	inline void epBank0SetByteCount(ep_t ep, uint16_t bc) { EP[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE = (EP[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) | USB_DEVICE_PCKSIZE_BYTE_COUNT(bc); }
	inline void epBank1SetByteCount(ep_t ep, uint16_t bc) { EP[ep].DEVICE_DESC_BANK[1].USB_PCKSIZE = (EP[ep].DEVICE_DESC_BANK[1].USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_BYTE_COUNT_Msk) | USB_DEVICE_PCKSIZE_BYTE_COUNT(bc); }
	inline void epBank0SetMultiPacketSize(ep_t ep, uint16_t s) { EP[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE = (EP[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(s); }
	inline void epBank1SetMultiPacketSize(ep_t ep, uint16_t s) { EP[ep].DEVICE_DESC_BANK[1].USB_PCKSIZE = (EP[ep].DEVICE_DESC_BANK[1].USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE_Msk) | USB_DEVICE_PCKSIZE_MULTI_PACKET_SIZE(s); }
	inline void epBank0SetAddress(ep_t ep, void *addr) { EP[ep].DEVICE_DESC_BANK[0].USB_ADDR = (uint32_t)addr; }
	inline void epBank1SetAddress(ep_t ep, void *addr) { EP[ep].DEVICE_DESC_BANK[1].USB_ADDR = (uint32_t)addr; }
	inline void epBank0SetSize(ep_t ep, uint16_t size) { EP[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE = (EP[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_SIZE_Msk) | USB_DEVICE_PCKSIZE_SIZE(EP_PCKSIZE_SIZE(size)); }
	inline void epBank1SetSize(ep_t ep, uint16_t size) { EP[ep].DEVICE_DESC_BANK[1].USB_PCKSIZE = (EP[ep].DEVICE_DESC_BANK[1].USB_PCKSIZE & ~USB_DEVICE_PCKSIZE_SIZE_Msk) | USB_DEVICE_PCKSIZE_SIZE(EP_PCKSIZE_SIZE(size)); }
	inline uint8_t EP_PCKSIZE_SIZE(uint16_t size) {
		switch (size) {
		case 8: return 0; case 16: return 1; case 32: return 2; case 64: return 3;
		case 128: return 4; case 256: return 5; case 512: return 6; case 1023: return 7;
		default: return 0;
		}
	}
	inline void epBank0DisableAutoZLP(ep_t ep) { EP[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP_Msk; }
	inline void epBank1DisableAutoZLP(ep_t ep) { EP[ep].DEVICE_DESC_BANK[1].USB_PCKSIZE &= ~USB_DEVICE_PCKSIZE_AUTO_ZLP_Msk; }
	inline void epBank0EnableAutoZLP(ep_t ep) { EP[ep].DEVICE_DESC_BANK[0].USB_PCKSIZE |= USB_DEVICE_PCKSIZE_AUTO_ZLP_Msk; }
	inline void epBank1EnableAutoZLP(ep_t ep) { EP[ep].DEVICE_DESC_BANK[1].USB_PCKSIZE |= USB_DEVICE_PCKSIZE_AUTO_ZLP_Msk; }
#else
	// Config
	inline void epBank0SetType(ep_t ep, uint8_t type) { usb.DeviceEndpoint[ep].EPCFG.bit.EPTYPE0 = type; }
	inline void epBank1SetType(ep_t ep, uint8_t type) { usb.DeviceEndpoint[ep].EPCFG.bit.EPTYPE1 = type; }

	// Interrupts
	inline uint16_t epInterruptSummary() { return usb.EPINTSMRY.reg; }

	inline bool epBank0IsSetupReceived(ep_t ep)     { return usb.DeviceEndpoint[ep].EPINTFLAG.bit.RXSTP; }
	inline bool epBank0IsStalled(ep_t ep)           { return usb.DeviceEndpoint[ep].EPINTFLAG.bit.STALL0; }
	inline bool epBank1IsStalled(ep_t ep)           { return usb.DeviceEndpoint[ep].EPINTFLAG.bit.STALL1; }
	inline bool epBank0IsTransferComplete(ep_t ep)  { return usb.DeviceEndpoint[ep].EPINTFLAG.bit.TRCPT0; }
	inline bool epBank1IsTransferComplete(ep_t ep)  { return usb.DeviceEndpoint[ep].EPINTFLAG.bit.TRCPT1; }

	inline void epBank0AckSetupReceived(ep_t ep)    { usb.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_RXSTP; }
	inline void epBank0AckStalled(ep_t ep)          { usb.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL(1); }
	inline void epBank1AckStalled(ep_t ep)          { usb.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_STALL(2); }
	inline void epBank0AckTransferComplete(ep_t ep) { usb.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT(1); }
	inline void epBank1AckTransferComplete(ep_t ep) { usb.DeviceEndpoint[ep].EPINTFLAG.reg = USB_DEVICE_EPINTFLAG_TRCPT(2); }

	inline void epBank0EnableSetupReceived(ep_t ep)    { usb.DeviceEndpoint[ep].EPINTENSET.bit.RXSTP = 1; }
	inline void epBank0EnableStalled(ep_t ep)          { usb.DeviceEndpoint[ep].EPINTENSET.bit.STALL0 = 1; }
	inline void epBank1EnableStalled(ep_t ep)          { usb.DeviceEndpoint[ep].EPINTENSET.bit.STALL1 = 1; }
	inline void epBank0EnableTransferComplete(ep_t ep) { usb.DeviceEndpoint[ep].EPINTENSET.bit.TRCPT0 = 1; }
	inline void epBank1EnableTransferComplete(ep_t ep) { usb.DeviceEndpoint[ep].EPINTENSET.bit.TRCPT1 = 1; }

	inline void epBank0DisableSetupReceived(ep_t ep)    { usb.DeviceEndpoint[ep].EPINTENCLR.bit.RXSTP = 1; }
	inline void epBank0DisableStalled(ep_t ep)          { usb.DeviceEndpoint[ep].EPINTENCLR.bit.STALL0 = 1; }
	inline void epBank1DisableStalled(ep_t ep)          { usb.DeviceEndpoint[ep].EPINTENCLR.bit.STALL1 = 1; }
	inline void epBank0DisableTransferComplete(ep_t ep) { usb.DeviceEndpoint[ep].EPINTENCLR.bit.TRCPT0 = 1; }
	inline void epBank1DisableTransferComplete(ep_t ep) { usb.DeviceEndpoint[ep].EPINTENCLR.bit.TRCPT1 = 1; }

	// Status
	inline bool epBank0IsReady(ep_t ep)    { return usb.DeviceEndpoint[ep].EPSTATUS.bit.BK0RDY; }
	inline bool epBank1IsReady(ep_t ep)    { return usb.DeviceEndpoint[ep].EPSTATUS.bit.BK1RDY; }
	inline void epBank0SetReady(ep_t ep)   { usb.DeviceEndpoint[ep].EPSTATUSSET.bit.BK0RDY = 1; }
	inline void epBank1SetReady(ep_t ep)   { usb.DeviceEndpoint[ep].EPSTATUSSET.bit.BK1RDY = 1; }
	inline void epBank0ResetReady(ep_t ep) { usb.DeviceEndpoint[ep].EPSTATUSCLR.bit.BK0RDY = 1; }
	inline void epBank1ResetReady(ep_t ep) { usb.DeviceEndpoint[ep].EPSTATUSCLR.bit.BK1RDY = 1; }

	inline void epBank0SetStallReq(ep_t ep)   { usb.DeviceEndpoint[ep].EPSTATUSSET.bit.STALLRQ0 = 1; }
	inline void epBank1SetStallReq(ep_t ep)   { usb.DeviceEndpoint[ep].EPSTATUSSET.bit.STALLRQ1 = 1; }
	inline void epBank0ResetStallReq(ep_t ep) { usb.DeviceEndpoint[ep].EPSTATUSCLR.bit.STALLRQ0 = 1; }
	inline void epBank1ResetStallReq(ep_t ep) { usb.DeviceEndpoint[ep].EPSTATUSCLR.bit.STALLRQ1 = 1; }

	// Packet
	inline uint16_t epBank0ByteCount(ep_t ep) { return EP[ep].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT; }
	inline uint16_t epBank1ByteCount(ep_t ep) { return EP[ep].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT; }
	inline void epBank0SetByteCount(ep_t ep, uint16_t bc) { EP[ep].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = bc; }
	inline void epBank1SetByteCount(ep_t ep, uint16_t bc) { EP[ep].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = bc; }
	inline void epBank0SetMultiPacketSize(ep_t ep, uint16_t s) { EP[ep].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = s; }
	inline void epBank1SetMultiPacketSize(ep_t ep, uint16_t s) { EP[ep].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = s; }

	inline void epBank0SetAddress(ep_t ep, void *addr) { EP[ep].DeviceDescBank[0].ADDR.reg = (uint32_t)addr; }
	inline void epBank1SetAddress(ep_t ep, void *addr) { EP[ep].DeviceDescBank[1].ADDR.reg = (uint32_t)addr; }
	inline void epBank0SetSize(ep_t ep, uint16_t size) { EP[ep].DeviceDescBank[0].PCKSIZE.bit.SIZE = EP_PCKSIZE_SIZE(size); }
	inline void epBank1SetSize(ep_t ep, uint16_t size) { EP[ep].DeviceDescBank[1].PCKSIZE.bit.SIZE = EP_PCKSIZE_SIZE(size); }
	inline uint8_t EP_PCKSIZE_SIZE(uint16_t size) {
		switch (size) {
		case 8:    return 0;
		case 16:   return 1;
		case 32:   return 2;
		case 64:   return 3;
		case 128:  return 4;
		case 256:  return 5;
		case 512:  return 6;
		case 1023: return 7;
		default:   return 0;
		}
	}

	inline void epBank0DisableAutoZLP(ep_t ep) { EP[ep].DeviceDescBank[0].PCKSIZE.bit.AUTO_ZLP = 0; }
	inline void epBank1DisableAutoZLP(ep_t ep) { EP[ep].DeviceDescBank[1].PCKSIZE.bit.AUTO_ZLP = 0; }
	inline void epBank0EnableAutoZLP(ep_t ep)  { EP[ep].DeviceDescBank[0].PCKSIZE.bit.AUTO_ZLP = 1; }
	inline void epBank1EnableAutoZLP(ep_t ep)  { EP[ep].DeviceDescBank[1].PCKSIZE.bit.AUTO_ZLP = 1; }
#endif
private:
#if defined(__SAME53__) || defined(__SAME54__)
	inline void setSpeedConfiguration(uint8_t speed) {
		usb.USB_CTRLB = (usb.USB_CTRLB & ~USB_DEVICE_CTRLB_SPDCONF_Msk) |
		                  USB_DEVICE_CTRLB_SPDCONF(speed);
	}

	// USB Device registers
	usb_device_registers_t &usb;

	// Endpoints descriptors table
	__attribute__((__aligned__(4))) usb_descriptor_device_registers_t EP[USB_EPT_NUM];
#else
	inline void setSpeedConfiguration(uint8_t speed) {
		usb.CTRLB.bit.SPDCONF = speed;
	}

	// USB Device registers
	UsbDevice &usb;

	// Endpoints descriptors table
	__attribute__((__aligned__(4)))	UsbDeviceDescriptor EP[USB_EPT_NUM];
#endif
};

/*
 * Synchronization primitives.
 * TODO: Move into a separate header file and make an API out of it
 */

class __Guard {
public:
	__Guard() : primask(__get_PRIMASK()), loops(1) {
		__disable_irq();
	}
	~__Guard() {
		if (primask == 0) {
			__enable_irq();
			// http://infocenter.arm.com/help/topic/com.arm.doc.dai0321a/BIHBFEIB.html
			__ISB();
		}
	}
	uint32_t enter() { return loops--; }
private:
	uint32_t primask;
	uint32_t loops;
};

#define synchronized for (__Guard __guard; __guard.enter(); )

/*
 * USB EP generic handlers.
 */

class EPHandler {
public:
	virtual void handleEndpoint() = 0;
	virtual uint32_t recv(void *_data, uint32_t len) = 0;
	virtual uint32_t available() const = 0;

	virtual void init() = 0;
};

class DoubleBufferedEPOutHandler : public EPHandler {
public:
	DoubleBufferedEPOutHandler(USBDevice_SAMD21G18x &usbDev, uint32_t endPoint, uint32_t bufferSize) :
		usbd(usbDev),
		ep(endPoint), size(bufferSize),
		current(0), incoming(0),
		first0(0), last0(0), ready0(false),
		first1(0), last1(0), ready1(false),
		notify(false)
	{
		data0 = reinterpret_cast<uint8_t *>(malloc(size));
		data1 = reinterpret_cast<uint8_t *>(malloc(size));

		usbd.epBank0SetSize(ep, 64);
		usbd.epBank0SetType(ep, 3); // BULK OUT

		usbd.epBank0SetAddress(ep, const_cast<uint8_t *>(data0));

		release();
	}

	virtual ~DoubleBufferedEPOutHandler() {
		free((void*)data0);
		free((void*)data1);
	}
	void init() {};

	virtual uint32_t recv(void *_data, uint32_t len)
	{
		uint8_t *data = reinterpret_cast<uint8_t *>(_data);

		// R/W: current, first0/1, ready0/1, notify
		// R  : last0/1, data0/1
		if (current == 0) {
			synchronized {
				if (!ready0) {
					return 0;
				}
			}
			// when ready0==true the buffer is not being filled and last0 is constant
			uint32_t i;
			for (i=0; i<len && first0 < last0; i++) {
				data[i] = data0[first0++];
			}
			if (first0 == last0) {
				first0 = 0;
				current = 1;
				synchronized {
					ready0 = false;
					if (notify) {
						notify = false;
						release();
					}
				}
			}
			return i;
		} else {
			synchronized {
				if (!ready1) {
					return 0;
				}
			}
			// when ready1==true the buffer is not being filled and last1 is constant
			uint32_t i;
			for (i=0; i<len && first1 < last1; i++) {
				data[i] = data1[first1++];
			}
			if (first1 == last1) {
				first1 = 0;
				current = 0;
				synchronized {
					ready1 = false;
					if (notify) {
						notify = false;
						release();
					}
				}
			}
			return i;
		}
	}

	virtual void handleEndpoint()
	{
		// R/W : incoming, ready0/1
		//   W : last0/1, notify
		if (usbd.epBank0IsTransferComplete(ep))
		{
			// Ack Transfer complete
			usbd.epBank0AckTransferComplete(ep);
			//usbd.epBank0AckTransferFailed(ep); // XXX

			// Update counters and swap banks for non-ZLP's
			if (incoming == 0) {
				last0 = usbd.epBank0ByteCount(ep);
				if (last0 != 0) {
					incoming = 1;
					usbd.epBank0SetAddress(ep, const_cast<uint8_t *>(data1));
					synchronized {
						ready0 = true;
						if (ready1) {
							notify = true;
							return;
						}
						notify = false;
					}
				}
			} else {
				last1 = usbd.epBank0ByteCount(ep);
				if (last1 != 0) {
					incoming = 0;
					usbd.epBank0SetAddress(ep, const_cast<uint8_t *>(data0));
					synchronized {
						ready1 = true;
						if (ready0) {
							notify = true;
							return;
						}
						notify = false;
					}
				}
			}
			release();
		}
	}

	// Returns how many bytes are stored in the buffers
	virtual uint32_t available() const {
		if (current == 0) {
			bool ready = false;
			synchronized {
				ready = ready0;
			}
			return ready ? (last0 - first0) : 0;
		} else {
			bool ready = false;
			synchronized {
				ready = ready1;
			}
			return ready ? (last1 - first1) : 0;
		}
	}

	void release() {
		// Release OUT EP
		usbd.epBank0EnableTransferComplete(ep);
		usbd.epBank0SetMultiPacketSize(ep, size);
		usbd.epBank0SetByteCount(ep, 0);
		usbd.epBank0ResetReady(ep);
	}

private:
	USBDevice_SAMD21G18x &usbd;

	const uint32_t ep;
	const uint32_t size;
	uint32_t current, incoming;

	volatile uint8_t *data0;
	uint32_t first0;
	volatile uint32_t last0;
	volatile bool ready0;

	volatile uint8_t *data1;
	uint32_t first1;
	volatile uint32_t last1;
	volatile bool ready1;

	volatile bool notify;
};
