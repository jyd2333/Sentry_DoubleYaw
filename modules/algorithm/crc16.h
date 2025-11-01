#ifndef __CRC16_H
#define __CRC16_H
#include "main.h"

#define CRC_START_16 0xFFFF
#define CRC_START_MODBUS 0xFFFF
#define CRC_POLY_16 0xA001

// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.


#include <stdint.h>


/**
  * @brief CRC16 Verify function
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return : True or False (CRC Verify Result)
  */
uint32_t Verify_CRC16_Check_Sum_1(const uint8_t * pchMessage, uint32_t dwLength);

/**
  * @brief Append CRC16 value to the end of the buffer
  * @param[in] pchMessage : Data to Verify,
  * @param[in] dwLength : Stream length = Data + checksum
  * @return none
  */
void Append_CRC16_Check_Sum_1(uint8_t * pchMessage, uint32_t dwLength);

  // namespace crc16

#endif  // RM_SERIAL_DRIVER__CRC_HPP_
