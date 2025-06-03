/**
 * @file ModbusDef.h
 * @brief Definitions and constants for Modbus RTU/TCP communication.
 * @details Includes function codes, diagnostic subfunction codes, error codes, timeouts, and protocol limits.
 * @note User-defined constants (PDU_SIZE, QUEUE_SIZE, CLIENT_COUNT) must be defined before including this header.
 */

#pragma once

/**
 * @defgroup UserConfig User-Defined Configuration
 * @brief Constants that must be defined by the user before including this header.
 * @{
 */

/**
 * @def PDU_SIZE
 * @brief Maximum PDU size in bytes (default: 253).
 * @details Must be between 8 and 253 bytes per Modbus protocol limits.
 */
/*#ifndef PDU_SIZE
#define PDU_SIZE 253
#endif
#if PDU_SIZE > 253
#error "PDU_SIZE cannot exceed 253 bytes (Modbus protocol limit)"
#endif
#if PDU_SIZE < 8
#error "PDU_SIZE cannot be less than 8 bytes"
#endif*/

/**
 * @def QUEUE_SIZE
 * @brief Maximum ADU queue size (default: 5).
 */
/*#ifndef QUEUE_SIZE
#define QUEUE_SIZE 5
#endif*/

/**
 * @def CLIENT_COUNT
 * @brief Maximum number of TCP clients (default: 5).
 */
/*#ifndef CLIENT_COUNT
#define CLIENT_COUNT 5
#endif*/
/** @} */

/**
 * @defgroup FunctionCodes Modbus Function Codes
 * @brief Function codes defined by the Modbus Application Protocol Specification.
 * @{
 */
#define MB_FC_READ_COILS 0x01                  ///< Read multiple coils (binary outputs).
#define MB_FC_READ_DISCRETE_INPUTS 0x02        ///< Read multiple discrete inputs (binary inputs).
#define MB_FC_READ_HOLDING_REGISTERS 0x03      ///< Read multiple holding registers (16-bit).
#define MB_FC_READ_INPUT_REGISTERS 0x04        ///< Read multiple input registers (16-bit).
#define MB_FC_WRITE_SINGLE_COIL 0x05           ///< Write a single coil (binary output).
#define MB_FC_WRITE_SINGLE_REGISTER 0x06       ///< Write a single holding register (16-bit).
#define MB_FC_READ_EXCEPTION_STATUS 0x07       ///< Read exception status (serial line only, 8 bits).
#define MB_FC_DIAGNOSTICS 0x08                 ///< Perform diagnostic operations (serial line only).
#define MB_FC_GET_COMM_EVENT_COUNTER 0x0B      ///< Get communication event counter (serial line only).
#define MB_FC_GET_COMM_EVENT_LOG 0x0C          ///< Get communication event log (serial line only).
#define MB_FC_WRITE_MULTIPLE_COILS 0x0F        ///< Write multiple coils (binary outputs).
#define MB_FC_WRITE_MULTIPLE_REGISTERS 0x10    ///< Write multiple holding registers (16-bit).
#define MB_FC_REPORT_SERVER_ID 0x11            ///< Report server ID (serial line only).
#define MB_FC_READ_FILE_RECORD 0x14            ///< Read file record (rarely used in RTU).
#define MB_FC_WRITE_FILE_RECORD 0x15           ///< Write file record (rarely used in RTU).
#define MB_FC_MASK_WRITE_REGISTER 0x16         ///< Mask write holding register (bit-level manipulation).
#define MB_FC_READ_AND_WRITE_REGISTERS 0x17    ///< Read/write multiple registers in one transaction.
#define MB_FC_READ_FIFO_QUEUE 0x18             ///< Read FIFO queue (rarely used in RTU).
#define MB_FC_READ_DEVICE_IDENTIFICATION 0x2B  ///< Read device identification (rarely used in RTU).
/** @} */

/**
 * @defgroup DiagnosticSubCodes Modbus Diagnostic Subfunction Codes
 * @brief Subfunction codes for Function Code 0x08 (Diagnostics).
 * @{
 */
#define MB_FC_SUB_QUERY_DATA 0x00                              ///< Echo back request data.
#define MB_FC_SUB_RESTART_COMMUNICATION_OPTION 0x01            ///< Restart communication, clear logs/counters.
#define MB_FC_SUB_DIAGNOSTIC_REGISTER 0x02                     ///< Return diagnostic register contents.
#define MB_FC_SUB_CHANGE_ASCII_INPUT_DELIMITER 0x03            ///< Change ASCII input delimiter (not used in RTU).
#define MB_FC_SUB_FORCE_LISTEN_ONLY_MODE 0x04                  ///< Force slave into listen-only mode.
#define MB_FC_SUB_CLEAR_COUNTERS_AND_DIAGNOSTIC_REGISTER 0x0A  ///< Clear counters and diagnostic register.
#define MB_FC_SUB_BUS_MESSAGE_COUNT 0x0B                       ///< Return bus message count.
#define MB_FC_SUB_BUS_COMMUNICATION_ERROR_COUNT 0x0C           ///< Return bus communication error count.
#define MB_FC_SUB_BUS_EXCEPTION_ERROR_COUNT 0x0D               ///< Return bus exception error count.
#define MB_FC_SUB_SERVER_MESSAGE_COUNT 0x0E                    ///< Return server message count.
#define MB_FC_SUB_SERVER_NO_RESPONSE_COUNT 0x0F                ///< Return server no response count.
#define MB_FC_SUB_SERVER_NAK_COUNT 0x10                        ///< Return server NAK count.
#define MB_FC_SUB_SERVER_BUSY_COUNT 0x11                       ///< Return server busy count.
#define MB_FC_SUB_BUS_CHARACTER_OVERRUN_COUNT 0x12             ///< Return bus character overrun count.
#define MB_FC_SUB_CLEAR_OVERRUN_CHARACTER_AND_FLAG 0x14        ///< Clear overrun counter and flag.
/** @} */

/**
 * @defgroup ErrorCodes Modbus Error Codes
 * @brief Standard Modbus error codes (1-10) and library-specific error codes (12+).
 * @{
 */
#define MB_EX_SUCCESS 0                                     ///< Operation successful.
#define MB_EX_ILLEGAL_FUNCTION 1                            ///< Function code not recognized by slave.
#define MB_EX_ILLEGAL_DATA_ADDRESS 2                        ///< Data address not allowed or does not exist.
#define MB_EX_ILLEGAL_DATA_VALUE 3                          ///< Value not accepted by slave.
#define MB_EX_SLAVE_DEVICE_ERROR 4                          ///< Unrecoverable error in slave.
#define MB_EX_ACKNOWLEDGE 5                                 ///< Slave accepted request, processing takes time.
#define MB_EX_SLAVE_DEVICE_BUSY 6                           ///< Slave busy with long-duration command.
#define MB_EX_NEGATIVE_ACKNOWLEDGE 7                        ///< Slave cannot perform programming functions.
#define MB_EX_MEMORY_PARITY_ERROR 8                         ///< Slave detected memory parity error.
#define MB_EX_GATEWAY_PATH_UNAVAILABLE 9                    ///< Gateway misconfigured.
#define MB_EX_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND 10    ///< Slave failed to respond (gateway-specific).
#define MB_EX_LIB_TOO_MANY_DATA 12                          ///< Too many data bytes requested.
#define MB_EX_LIB_TOO_FEW_DATA 13                           ///< Too few data bytes received.
#define MB_EX_LIB_RESPONSE_TIMEOUT 14                       ///< Response timeout exceeded.
#define MB_EX_LIB_CONN_RESET_BY_PEER 15                     ///< Connection reset by peer (network).
#define MB_EX_LIB_CONN_REFUSED 16                           ///< Connection refused (network).
#define MB_EX_LIB_INVALID_SLAVE 17                          ///< Invalid slave ID in response.
#define MB_EX_LIB_INVALID_FUNCTION 18                       ///< Invalid function code in response.
#define MB_EX_LIB_INVALID_SUB_FUNCTION 19                   ///< Invalid diagnostic subfunction code.
#define MB_EX_LIB_INVALID_ADDRESS 20                        ///< Invalid address in response.
#define MB_EX_LIB_INVALID_DATA 21                           ///< Invalid data value in response.
#define MB_EX_LIB_INVALID_DATA_QUANTITY 22                  ///< Invalid quantity in response.
#define MB_EX_LIB_INVALID_BYTE_LENGTH 23                    ///< Invalid byte length in response.
#define MB_EX_LIB_INVALID_EXCEPTION_CODE 24                 ///< Invalid exception code in response.
#define MB_EX_LIB_CRC 25                                    ///< CRC checksum error in response.
#define MB_EX_LIB_INVALID_ARGUMENT 26                       ///< Invalid argument provided to function.
#define MB_EX_LIB_INVALID_SOURCE_SIZE 27                    ///< Source data size not aligned.
#define MB_EX_LIB_NOT_SUPPORTED 28                          ///< Operation not supported by library.
#define MB_EX_LIB_QUEUE_FULL 29                             ///< Queue buffer is full.
#define MB_EX_LIB_TCP_SENT_BUFFER_FULL 30                   ///< TCP send buffer is full.
#define MB_EX_LIB_TCP_NO_CLIENT_AVAILABLE_FOR_THE_SLAVE 31  ///< No client available for the slave.
#define MB_EX_LIB_NO_MORE_FREE_ADU 32                       ///< No more free ADUs available.
#define MB_EX_LIB_BUFFER_IS_TOO_SMALL 33                    ///< Buffer too small for operation.
#define MB_EX_LIB_INVALID_MBAP_HEADER 40                    ///< Invalid MBAP header.
#define MB_EX_LIB_INVALID_MBAP_TRANSACTION_ID 41            ///< Invalid MBAP transaction ID.
#define MB_EX_LIB_INVALID_MBAP_PROTOCOL_ID 42               ///< Invalid MBAP protocol ID.
#define MB_EX_LIB_INVALID_MBAP_LENGTH 43                    ///< Invalid MBAP length.
#define MB_EX_LIB_INVALID_MBAP_UNIT_ID 44                   ///< Invalid MBAP unit ID.
/** @} */

/**
 * @defgroup Timeouts Response Timeouts
 * @brief Configurable timeout values for Modbus communication.
 * @{
 */
#define MB_RESPONSE_TIMEOUT (uint32_t)3000      ///< Default RTU response timeout (µs).
#define MB_TCP_RESPONSE_TIMEOUT (uint32_t)2000  ///< Default TCP response timeout (ms).
#define MB_RECONNECT 100                        ///< TCP reconnect interval (ms).
/** @} */

/**
 * @defgroup ProtocolLimits Modbus Protocol Limits
 * @brief Maximum limits for Modbus data and packet sizes.
 * @{
 */
#define MB_MAX_READ_COILS 2000             ///< Max coils to read (FC 0x01, 0x02).
#define MB_MAX_READ_COILS_IN_BYTE 250      ///< Max coil data bytes (2000 coils / 8).
#define MB_MAX_WRITE_COILS 1968            ///< Max coils to write (FC 0x0F).
#define MB_MAX_WRITE_COILS_IN_BYTES 246    ///< Max coil data bytes (1968 coils / 8).
#define MB_MAX_READ_REGISTERS 125          ///< Max registers to read (FC 0x03, 0x04).
#define MB_MAX_READ_REGISTERS_IN_BYTE 250  ///< Max register data bytes (125 * 2).
#define MB_MAX_WRITE_REGISTERS 123         ///< Max registers to write (FC 0x10).
#define MB_MAX_WRITE_READ_REGISTERS 121    ///< Max registers for read/write (FC 0x17).
#define MB_ADU_RTU_HEADER_LEN 1            ///< RTU header length (slave ID).
#define MB_ADU_RTU_CRC_LEN 2               ///< RTU CRC length.
#define MB_ADU_MBAP_LEN 7                  ///< TCP MBAP header length.
#define MB_PDU_MAX_RESPONSE_LEN 7          ///< Max PDU response length (excluding data).
#define MB_PDU_ERR_LEN 2                   ///< PDU error response length.
#define MB_MAX_SLAVE_ID 247                ///< Max Modbus slave ID.
#define MB_PDU_MAX_SIZE 253                ///< Max Modbus PDU size.
                                           /** @} */