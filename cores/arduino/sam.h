#pragma once

// The board's build configuration selects the exact CMSIS device. Keep that
// selection authoritative, then expose only the family distinctions needed by
// the Arduino core.
#include_next <sam.h>

#if defined(__SAMD51__) || defined(__SAME51__)
#define ARDUINO_SAMD51_E51
#elif defined(__SAME53__) || defined(__SAME54__)
#define ARDUINO_SAME53_E54
#endif
