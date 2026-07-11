/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/**
 * @file vl53l5cx_platform.h
 * @brief Platform abstraction layer for VL53L5CX on CYT4BB (Seekfree soft_iic)
 *
 * This file adapts the ST VL53L5CX ULD API to the CYT4BB platform using
 * the Seekfree software I2C library (zf_driver_soft_iic.h).
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _VL53L5CX_PLATFORM_H_
#define _VL53L5CX_PLATFORM_H_

#include <stdint.h>
#include <string.h>
#include "zf_driver_soft_iic.h"

/**
 * @brief Platform structure for VL53L5CX on CYT4BB.
 *
 * Contains the software I2C interface pointer needed by the ST API.
 * User must set the 'iic' field before calling any VL53L5CX functions.
 */
typedef struct
{
    soft_iic_info_struct *iic;      /**< Pointer to Seekfree soft I2C struct */
    uint16_t address;               /**< I2C device address (used by ST API) */
} VL53L5CX_Platform;

/**
 * @brief Maximum I2C clock speed supported by VL53L5CX.
 */
#define VL53L5CX_MAX_CLK_SPEED 1000000

/*
 * @brief The macro below is used to define the number of target per zone sent
 * through I2C. This value can be changed by user, in order to tune I2C
 * transaction, and also the total memory size (a lower number of target per
 * zone means a lower RAM). The value must be between 1 and 4.
 */

#ifndef CONFIG_VL53L5CX_NB_TARGET_PER_ZONE
#define VL53L5CX_NB_TARGET_PER_ZONE 1U
#else
#define VL53L5CX_NB_TARGET_PER_ZONE CONFIG_VL53L5CX_NB_TARGET_PER_ZONE
#endif

/*
 * @brief All macro below are used to configure the sensor output. User can
 * define some macros if he wants to disable selected output, in order to reduce
 * I2C access.
 */

// #define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
// #define VL53L5CX_DISABLE_NB_SPADS_ENABLED
// #define VL53L5CX_DISABLE_NB_TARGET_DETECTED
// #define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
// #define VL53L5CX_DISABLE_RANGE_SIGMA_MM
// #define VL53L5CX_DISABLE_DISTANCE_MM
// #define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
// #define VL53L5CX_DISABLE_TARGET_STATUS
// #define VL53L5CX_DISABLE_MOTION_INDICATOR

/**
 * @brief Read one single byte from VL53L5CX via I2C.
 * @param p_platform : Pointer to VL53L5CX platform structure.
 * @param RegisterAdress : I2C register address (16-bit).
 * @param p_value : Pointer to store the read byte.
 * @return 0 if OK
 */
uint8_t VL53L5CX_RdByte(
        VL53L5CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_value);

/**
 * @brief Write one single byte to VL53L5CX via I2C.
 * @param p_platform : Pointer to VL53L5CX platform structure.
 * @param RegisterAdress : I2C register address (16-bit).
 * @param value : Byte to write.
 * @return 0 if OK
 */
uint8_t VL53L5CX_WrByte(
        VL53L5CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t value);

/**
 * @brief Read multiple bytes from VL53L5CX via I2C.
 * @param p_platform : Pointer to VL53L5CX platform structure.
 * @param RegisterAdress : I2C register address (16-bit).
 * @param p_values : Buffer to store read bytes.
 * @param size : Number of bytes to read.
 * @return 0 if OK
 */
uint8_t VL53L5CX_RdMulti(
        VL53L5CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size);

/**
 * @brief Write multiple bytes to VL53L5CX via I2C.
 * @param p_platform : Pointer to VL53L5CX platform structure.
 * @param RegisterAdress : I2C register address (16-bit).
 * @param p_values : Buffer of bytes to write.
 * @param size : Number of bytes to write.
 * @return 0 if OK
 */
uint8_t VL53L5CX_WrMulti(
        VL53L5CX_Platform *p_platform,
        uint16_t RegisterAdress,
        uint8_t *p_values,
        uint32_t size);

/**
 * @brief Optional: Hardware reset the VL53L5CX sensor.
 * Uses the PWR_EN and LPn pins (platform-specific).
 * @param p_platform : Pointer to VL53L5CX platform structure.
 * @return 0 if OK
 */
uint8_t VL53L5CX_Reset_Sensor(
        VL53L5CX_Platform *p_platform);

/**
 * @brief Swap byte order in a buffer (32-bit word swap).
 * Required by ST API for endianness conversion.
 * @param buffer : Buffer to swap (multiple of 4 bytes).
 * @param size : Buffer size in bytes.
 */
void VL53L5CX_SwapBuffer(
        uint8_t *buffer,
        uint16_t size);

/**
 * @brief Wait for a specified time in milliseconds.
 * @param p_platform : Pointer to VL53L5CX platform structure.
 * @param TimeMs : Time to wait in milliseconds.
 * @return 0 if wait is finished.
 */
uint8_t VL53L5CX_WaitMs(
        VL53L5CX_Platform *p_platform,
        uint32_t TimeMs);

#endif /* _VL53L5CX_PLATFORM_H_ */

#ifdef __cplusplus
}
#endif
