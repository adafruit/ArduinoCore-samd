#pragma once

// The board's build configuration selects the exact CMSIS device. Keep that
// selection authoritative, then expose only the family distinctions needed by
// the Arduino core.
#if defined(__SAME53J18A__) || defined(__ATSAME53J18A__)
#include_next <same53j18a.h>
#elif defined(__SAME53J19A__) || defined(__ATSAME53J19A__)
#include_next <same53j19a.h>
#elif defined(__SAME53J20A__) || defined(__ATSAME53J20A__)
#include_next <same53j20a.h>
#elif defined(__SAME53N19A__) || defined(__ATSAME53N19A__)
#include_next <same53n19a.h>
#elif defined(__SAME53N20A__) || defined(__ATSAME53N20A__)
#include_next <same53n20a.h>
#elif defined(__SAME54N19A__) || defined(__ATSAME54N19A__)
#include_next <same54n19a.h>
#elif defined(__SAME54N20A__) || defined(__ATSAME54N20A__)
#include_next <same54n20a.h>
#elif defined(__SAME54P19A__) || defined(__ATSAME54P19A__)
#include_next <same54p19a.h>
#elif defined(__SAME54P20A__) || defined(__ATSAME54P20A__)
#include_next <same54p20a.h>
#else
#include_next <sam.h>
#endif // SAME53/E54 device selection

#if defined(__SAMD51__) || defined(__SAME51__)
#define ARDUINO_SAMD51_E51
#elif defined(__SAME53__) || defined(__SAME54__)
#define ARDUINO_SAME53_E54
typedef dmac_descriptor_registers_t DmacDescriptor;
#endif
