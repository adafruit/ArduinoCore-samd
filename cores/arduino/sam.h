#pragma once

#pragma push_macro("LITTLE_ENDIAN")
#undef LITTLE_ENDIAN

#if defined(ARDUINO_SAME54_XPLAINED_PRO)
#ifndef __SAME54P20A__
#define __SAME54P20A__
#endif
#ifndef __SAME54__
#define __SAME54__
#endif
#include <same54p20a.h>
#else
#include_next <sam.h>
#endif

#pragma pop_macro("LITTLE_ENDIAN")
