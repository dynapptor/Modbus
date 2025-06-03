/**
 * @file ModbusUtility.h
 * @brief Utility functions for Modbus RTU/TCP communication.
 * @details Provides functions for timing, endianness handling, and buffer printing.
 */

#pragma once
#include <utils.h>

/**
 * @brief Detects platform endianness at runtime.
 * @return bool True if big-endian, false if little-endian.
 */
extern bool setIsBigEndian();

/**
 * @brief Overrides platform endianness manually.
 * @details Use cautiously, primarily for testing purposes.
 * @param value True for big-endian, false for little-endian.
 */
extern void setIsBigEndian(bool value);

/**
 * @var isBigEndian
 * @brief Global flag indicating platform endianness.
 * @details Set by setIsBigEndian().
 */
extern bool isBigEndian;

/**
 * @brief Prints a buffer to the serial output.
 * @param buffer Pointer to the byte buffer.
 * @param len Length of the buffer.
 */
extern void printBuffer(uint8_t *buffer, uint16_t len);