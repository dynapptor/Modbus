/**
 * @file ModbusCallbackTypes.h
 * @brief Defines callback types for Modbus PDU response handling.
 * @details Provides the modbusCallback type for asynchronous PDU response processing.
 */

#pragma once

class PDU;
template <typename RET, typename... ARGS>
class Callback;

/**
 * @typedef modbusCallback
 * @brief Callback type for Modbus PDU response handling.
 * @details Takes a PDU reference and has no return value, used for asynchronous response processing.
 */
using modbusCallback = Callback<void, PDU&>;