/**
 * @file Crc16.h
 * @brief CRC-16 (IBM/Modbus) calculation for Modbus RTU communication.
 * @details Provides functions to compute, set, and verify CRC-16 checksums for Modbus RTU messages.
 * @note Platform-dependent macro for reading CRC tables (PROGMEM on AVR, direct access otherwise).
 */

#pragma once
#include <Arduino.h>

/**
 * @def PROGMEM_READ_BYTE(x)
 * @brief Platform-dependent macro for reading CRC tables.
 * @details Uses pgm_read_byte_near on AVR platforms to access PROGMEM, otherwise direct access.
 */
#ifdef __AVR__
#define PROGMEM_READ_BYTE(x) pgm_read_byte_near(x)
#else
#define PROGMEM_READ_BYTE(x) (*(x))
#endif

/**
 * @var tableCrcHi
 * @brief CRC-16 high byte lookup table (256 entries).
 * @details Stores precomputed high byte values for CRC-16 calculation.
 */
extern const uint8_t tableCrcHi[];

/**
 * @var tableCrcLo
 * @brief CRC-16 low byte lookup table (256 entries).
 * @details Stores precomputed low byte values for CRC-16 calculation.
 */
extern const uint8_t tableCrcLo[];

/**
 * @brief Computes CRC-16 checksum for a byte buffer.
 * @param buffer Input byte array to compute CRC for.
 * @param length Length of the input buffer.
 * @return uint16_t CRC-16 checksum, or 0xFFFF if buffer is null or length is 0.
 */
extern uint16_t crc16(const uint8_t* buffer, uint16_t length);

/**
 * @brief Verifies the CRC-16 checksum of a received Modbus RTU message.
 * @param msg Input message including CRC (last 2 bytes).
 * @param length Total length of the message (including CRC).
 * @return bool True if CRC is valid, false otherwise.
 */
extern bool crc16Check(const uint8_t* msg, uint16_t length);

/**
 * @brief Appends CRC-16 checksum to a Modbus RTU message.
 * @param msg Input/output message buffer (must have space for 2 extra bytes).
 * @param length Length of the message excluding CRC.
 * @return uint8_t* Pointer to the modified message with appended CRC.
 */
extern uint8_t* crc16Set(uint8_t* msg, uint16_t length);