/*
 * SAMD21_USBDevice.cpp
 *
 *  Created on: Feb 21, 2018
 *      Author: deanm
 */

#ifndef USE_TINYUSB

#include "SAMD21_USBDevice.h"

void USBDevice_SAMD21G18x::reset() {
#if defined(__SAME53__) || defined(__SAME54__)
    usb.USB_CTRLA |= USB_CTRLA_SWRST_Msk;
    memset(EP, 0, sizeof(EP));
    while (usb.USB_SYNCBUSY & (USB_SYNCBUSY_SWRST_Msk | USB_SYNCBUSY_ENABLE_Msk)) {}
    usb.USB_DESCADD = (uint32_t)(&EP);
#else
    usb.CTRLA.bit.SWRST = 1;
    memset(EP, 0, sizeof(EP));
    while (usb.SYNCBUSY.bit.SWRST || usb.SYNCBUSY.bit.ENABLE) {}
    usb.DESCADD.reg = (uint32_t)(&EP);
#endif
}

void USBDevice_SAMD21G18x::calibrate() {
    // Load Pad Calibration data from non-volatile memory
#if defined(__SAME53__) || defined(__SAME54__)
    const uint32_t calibration = *(const uint32_t *)(SW0_ADDR + 4u);
    uint32_t pad_transn = (calibration & FUSES_SW0_WORD_1_USB_TRANSN_Msk) >>
                          FUSES_SW0_WORD_1_USB_TRANSN_Pos;
    uint32_t pad_transp = (calibration & FUSES_SW0_WORD_1_USB_TRANSP_Msk) >>
                          FUSES_SW0_WORD_1_USB_TRANSP_Pos;
    uint32_t pad_trim = (calibration & FUSES_SW0_WORD_1_USB_TRIM_Msk) >>
                        FUSES_SW0_WORD_1_USB_TRIM_Pos;
#else
    uint32_t *pad_transn_p = (uint32_t *) USB_FUSES_TRANSN_ADDR;
    uint32_t *pad_transp_p = (uint32_t *) USB_FUSES_TRANSP_ADDR;
    uint32_t *pad_trim_p   = (uint32_t *) USB_FUSES_TRIM_ADDR;

    uint32_t pad_transn = (*pad_transn_p & USB_FUSES_TRANSN_Msk) >> USB_FUSES_TRANSN_Pos;
    uint32_t pad_transp = (*pad_transp_p & USB_FUSES_TRANSP_Msk) >> USB_FUSES_TRANSP_Pos;
    uint32_t pad_trim   = (*pad_trim_p   & USB_FUSES_TRIM_Msk  ) >> USB_FUSES_TRIM_Pos;
#endif

    if (pad_transn == 0x1F)  // maximum value (31)
        pad_transn = 5;
    if (pad_transp == 0x1F)  // maximum value (31)
        pad_transp = 29;
    if (pad_trim == 0x7)     // maximum value (7)
        pad_trim = 3;

#if defined(__SAME53__) || defined(__SAME54__)
    usb.USB_PADCAL = USB_PADCAL_TRANSN(pad_transn) |
                     USB_PADCAL_TRANSP(pad_transp) |
                     USB_PADCAL_TRIM(pad_trim);
#else
    usb.PADCAL.bit.TRANSN = pad_transn;
    usb.PADCAL.bit.TRANSP = pad_transp;
    usb.PADCAL.bit.TRIM   = pad_trim;
#endif
}

#endif // USE_TINYUSB
