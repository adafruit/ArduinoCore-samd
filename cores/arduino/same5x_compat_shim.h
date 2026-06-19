#pragma once

#if defined(__GNUC__)
#pragma GCC system_header
#endif

#if defined(__SAME51__) || defined(__SAME53__) || defined(__SAME54__)

#include "sam.h"

#ifndef _Ul
#define _Ul(x) x##U
#endif

#ifndef SIMIO_SAME5X_COMPAT_ROREG8
#define SIMIO_SAME5X_COMPAT_ROREG8 1
typedef volatile const uint8_t RoReg8;
#endif

#if !defined(NVMCTRL_SW0) && defined(SW0_ADDR)
#define NVMCTRL_SW0 SW0_ADDR
#elif !defined(NVMCTRL_SW0) && defined(SW0_FUSES_BASE_ADDRESS)
#define NVMCTRL_SW0 SW0_FUSES_BASE_ADDRESS
#endif

#if defined(__has_include)
#if __has_include("samd51/include/component/sercom.h")
#include "samd51/include/component/sercom.h"
#include "samd51/include/component/usb.h"
#include "samd51/include/component/port.h"
#include "samd51/include/component/gclk.h"
#include "samd51/include/component/mclk.h"
#include "samd51/include/component/tc.h"
#include "samd51/include/component/tcc.h"
#include "samd51/include/component/nvmctrl.h"
#include "samd51/include/component/dmac.h"
#include "samd51/include/component/eic.h"
#include "samd51/include/component/osc32kctrl.h"
#include "samd51/include/component/oscctrl.h"
#include "samd51/include/component/adc.h"
#include "samd51/include/component/dac.h"
#include "samd51/include/component/supc.h"
#include "samd51/include/component/ac.h"
#include "samd51/include/component/cmcc.h"
#else
#error "SAME5x compatibility shim requires CMSIS-Atmel SAMD51 component headers on the include path."
#endif
#else
#include "samd51/include/component/sercom.h"
#include "samd51/include/component/usb.h"
#include "samd51/include/component/port.h"
#include "samd51/include/component/gclk.h"
#include "samd51/include/component/mclk.h"
#include "samd51/include/component/tc.h"
#include "samd51/include/component/tcc.h"
#include "samd51/include/component/nvmctrl.h"
#include "samd51/include/component/dmac.h"
#include "samd51/include/component/eic.h"
#include "samd51/include/component/osc32kctrl.h"
#include "samd51/include/component/oscctrl.h"
#include "samd51/include/component/adc.h"
#include "samd51/include/component/dac.h"
#include "samd51/include/component/supc.h"
#include "samd51/include/component/ac.h"
#include "samd51/include/component/cmcc.h"
#endif

#if !defined(GCLK) && defined(GCLK_REGS)
#define GCLK ((Gclk *)GCLK_REGS)
#endif
#if !defined(MCLK) && defined(MCLK_REGS)
#define MCLK ((Mclk *)MCLK_REGS)
#endif
#if !defined(PORT) && defined(PORT_REGS)
#define PORT ((Port *)PORT_REGS)
#endif
#if !defined(USB) && defined(USB_REGS)
#define USB ((Usb *)USB_REGS)
#endif
#if !defined(EIC) && defined(EIC_REGS)
#define EIC ((Eic *)EIC_REGS)
#endif
#if !defined(DMAC) && defined(DMAC_REGS)
#define DMAC ((Dmac *)DMAC_REGS)
#endif
#if !defined(NVMCTRL) && defined(NVMCTRL_REGS)
#define NVMCTRL ((Nvmctrl *)NVMCTRL_REGS)
#endif
#if !defined(OSC32KCTRL) && defined(OSC32KCTRL_REGS)
#define OSC32KCTRL ((Osc32kctrl *)OSC32KCTRL_REGS)
#endif
#if !defined(OSCCTRL) && defined(OSCCTRL_REGS)
#define OSCCTRL ((Oscctrl *)OSCCTRL_REGS)
#endif
#if !defined(ADC0) && defined(ADC0_REGS)
#define ADC0 ((Adc *)ADC0_REGS)
#endif
#if !defined(ADC1) && defined(ADC1_REGS)
#define ADC1 ((Adc *)ADC1_REGS)
#endif
#if !defined(DAC) && defined(DAC_REGS)
#define DAC ((Dac *)DAC_REGS)
#endif
#if !defined(SUPC) && defined(SUPC_REGS)
#define SUPC ((Supc *)SUPC_REGS)
#endif
#if !defined(AC) && defined(AC_REGS)
#define AC ((Ac *)AC_REGS)
#endif
#if !defined(CMCC) && defined(CMCC_REGS)
#define CMCC ((Cmcc *)CMCC_REGS)
#endif

#if !defined(SERCOM0) && defined(SERCOM0_REGS)
#define SERCOM0 ((Sercom *)SERCOM0_REGS)
#endif
#if !defined(SERCOM1) && defined(SERCOM1_REGS)
#define SERCOM1 ((Sercom *)SERCOM1_REGS)
#endif
#if !defined(SERCOM2) && defined(SERCOM2_REGS)
#define SERCOM2 ((Sercom *)SERCOM2_REGS)
#endif
#if !defined(SERCOM3) && defined(SERCOM3_REGS)
#define SERCOM3 ((Sercom *)SERCOM3_REGS)
#endif
#if !defined(SERCOM4) && defined(SERCOM4_REGS)
#define SERCOM4 ((Sercom *)SERCOM4_REGS)
#endif
#if !defined(SERCOM5) && defined(SERCOM5_REGS)
#define SERCOM5 ((Sercom *)SERCOM5_REGS)
#endif
#if !defined(SERCOM6) && defined(SERCOM6_REGS)
#define SERCOM6 ((Sercom *)SERCOM6_REGS)
#endif
#if !defined(SERCOM7) && defined(SERCOM7_REGS)
#define SERCOM7 ((Sercom *)SERCOM7_REGS)
#endif

#if !defined(TC0) && defined(TC0_REGS)
#define TC0 ((Tc *)TC0_REGS)
#endif
#if !defined(TC1) && defined(TC1_REGS)
#define TC1 ((Tc *)TC1_REGS)
#endif
#if !defined(TC2) && defined(TC2_REGS)
#define TC2 ((Tc *)TC2_REGS)
#endif
#if !defined(TC3) && defined(TC3_REGS)
#define TC3 ((Tc *)TC3_REGS)
#endif
#if !defined(TC4) && defined(TC4_REGS)
#define TC4 ((Tc *)TC4_REGS)
#endif
#if !defined(TC5) && defined(TC5_REGS)
#define TC5 ((Tc *)TC5_REGS)
#endif
#if !defined(TC6) && defined(TC6_REGS)
#define TC6 ((Tc *)TC6_REGS)
#endif
#if !defined(TC7) && defined(TC7_REGS)
#define TC7 ((Tc *)TC7_REGS)
#endif

#if !defined(TCC0) && defined(TCC0_REGS)
#define TCC0 ((Tcc *)TCC0_REGS)
#endif
#if !defined(TCC1) && defined(TCC1_REGS)
#define TCC1 ((Tcc *)TCC1_REGS)
#endif
#if !defined(TCC2) && defined(TCC2_REGS)
#define TCC2 ((Tcc *)TCC2_REGS)
#endif
#if !defined(TCC3) && defined(TCC3_REGS)
#define TCC3 ((Tcc *)TCC3_REGS)
#endif
#if !defined(TCC4) && defined(TCC4_REGS)
#define TCC4 ((Tcc *)TCC4_REGS)
#endif

#ifndef TC_INST_NUM
#define TC_INST_NUM 8
#endif
#ifndef TCC_INST_NUM
#define TCC_INST_NUM 5
#endif
#ifndef SERCOM_INST_NUM
#define SERCOM_INST_NUM 8
#endif

#ifndef EIC_IRQn
#define EIC_IRQn EIC_EXTINT_0_IRQn
#endif
#ifndef EIC_0_IRQn
#define EIC_0_IRQn EIC_EXTINT_0_IRQn
#endif
#ifndef USB_0_IRQn
#define USB_0_IRQn USB_OTHER_IRQn
#endif
#ifndef USB_1_IRQn
#define USB_1_IRQn USB_SOF_HSOF_IRQn
#endif
#ifndef USB_2_IRQn
#define USB_2_IRQn USB_TRCPT0_IRQn
#endif
#ifndef USB_3_IRQn
#define USB_3_IRQn USB_TRCPT1_IRQn
#endif
#ifndef DMAC_4_IRQn
#define DMAC_4_IRQn DMAC_3_IRQn
#endif
#ifndef SERCOM0_3_IRQn
#define SERCOM0_3_IRQn SERCOM0_2_IRQn
#endif
#ifndef SERCOM1_3_IRQn
#define SERCOM1_3_IRQn SERCOM1_2_IRQn
#endif
#ifndef SERCOM2_3_IRQn
#define SERCOM2_3_IRQn SERCOM2_2_IRQn
#endif
#ifndef SERCOM3_3_IRQn
#define SERCOM3_3_IRQn SERCOM3_2_IRQn
#endif
#ifndef SERCOM4_3_IRQn
#define SERCOM4_3_IRQn SERCOM4_2_IRQn
#endif
#ifndef SERCOM5_3_IRQn
#define SERCOM5_3_IRQn SERCOM5_2_IRQn
#endif
#ifndef SERCOM6_3_IRQn
#define SERCOM6_3_IRQn SERCOM6_2_IRQn
#endif
#ifndef SERCOM7_3_IRQn
#define SERCOM7_3_IRQn SERCOM7_2_IRQn
#endif

#endif
