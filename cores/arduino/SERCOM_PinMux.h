#pragma once

/**
 * @file SERCOM_PinMux.h
 * @brief SERCOM pin-route validation APIs.
 *
 * This header exposes small, shared declarations used by SPI/Wire to validate
 * that variant-assigned pins can be routed to the selected SERCOM instance.
 *
 * @note Function definitions live in each library translation unit:
 *       - libraries/SPI/SPI.cpp
 *       - libraries/Wire/Wire.cpp
 */

#include "sam.h"
#include <stdint.h>

/** @brief Shared helpers for validating SERCOM pin routing. */
namespace sercomPinMux {

/**
 * @brief Check whether a variant pin can be used for I2C on a SERCOM instance.
 * @param arduinoPin Arduino pin index into g_APinDescription.
 * @param sercomIndex Zero-based SERCOM instance index.
 * @return true if the pin has an I2C mapping for the requested SERCOM.
 */
bool wirePinValidForSercom(uint8_t arduinoPin, uint8_t sercomIndex);

/**
 * @brief Check whether a MOSI pin matches an SPI route for SERCOM+pad.
 * @param arduinoPin Arduino pin index into g_APinDescription.
 * @param sercomIndex Zero-based SERCOM instance index.
 * @param mosiPad SERCOM MOSI pad index (0, 2, or 3 depending on TX pad mode).
 * @return true if the pin supports the requested SERCOM/pad route.
 */
bool spiMosiPinValidForRoute(uint8_t arduinoPin, uint8_t sercomIndex,
                             uint8_t mosiPad);

} // namespace sercomPinMux
