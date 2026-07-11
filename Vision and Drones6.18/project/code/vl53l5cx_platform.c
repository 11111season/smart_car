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
 * @file vl53l5cx_platform.c
 * @brief Platform implementation for VL53L5CX on CYT4BB (Seekfree soft_iic)
 *
 * Implements the I2C read/write, reset, swap, and wait functions using
 * the Seekfree software I2C and GPIO libraries.
 */

#include <stdlib.h>
#include "vl53l5cx_platform.h"
#include "zf_driver_gpio.h"
#include "zf_driver_delay.h"
#include "zf_common_debug.h"

/* VL53L5CX control pins (from VL53L5CX.h) */
#ifndef VL53L5CX_PWR_EN_PIN
#define VL53L5CX_PWR_EN_PIN     (P13_0)
#endif
#ifndef VL53L5CX_LPn_PIN
#define VL53L5CX_LPn_PIN        (P13_1)
#endif

/*
 * @brief The VL53L5CX supports I2C write operations of various sizes.
 * Minimum I2C write is 1 byte, maximum is 32800 bytes (for FW download).
 * Minimum I2C read is 1 byte, maximum is 3100 bytes (for results read).
 */

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform,
        uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
{
    /*
     * Write data to VL53L5CX via I2C.
     * For large transfers (e.g. 32KB firmware download), split into
     * smaller chunks to avoid malloc() on embedded systems with limited heap.
     * Each chunk sends: START + addr(W) + regH + regL + data + STOP.
     * The VL53L5CX firmware loader auto-increments its internal address
     * on each write to register 0.
     */
    uint8_t buf[258]; /* [regH, regL] + up to 256 data bytes */
    uint32_t offset = 0;
    uint32_t chunk_size;
    uint16_t addr;

    while (offset < size)
    {
        chunk_size = size - offset;
        if (chunk_size > 256)
        {
            chunk_size = 256;
        }

        /* Use progressive register address so each chunk goes to the correct offset.
         * This is critical for firmware download (large writes to address 0):
         * the firmware loader uses the register address as the memory offset
         * within the page. Without this, each chunk would overwrite the previous one. */
        addr = RegisterAdress + offset;
        buf[0] = (uint8_t)(addr >> 8);
        buf[1] = (uint8_t)(addr & 0xFF);
        memcpy(buf + 2, p_values + offset, chunk_size);
        soft_iic_write_8bit_array(p_platform->iic, buf, 2 + chunk_size);

        offset += chunk_size;
    }

    return 0;
}

uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform,
        uint16_t RegisterAdress, uint8_t value)
{
    return VL53L5CX_WrMulti(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform,
        uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
{
    /*
     * I2C read sequence:
     *   START + addr(W) + regH + regL + (re)START + addr(R) + data[0..size-1] + STOP
     * We use soft_iic_transfer_8bit_array with:
     *   write_data = [regH, regL], write_len = 2
     *   read_data = p_values, read_len = size
     */
    uint8_t addr_buf[2];

    addr_buf[0] = (uint8_t)(RegisterAdress >> 8);
    addr_buf[1] = (uint8_t)(RegisterAdress & 0xFF);

    soft_iic_transfer_8bit_array(p_platform->iic,
            addr_buf, 2, p_values, size);

    return 0;
}

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform,
        uint16_t RegisterAdress, uint8_t *p_value)
{
    return VL53L5CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform)
{
    /*
     * Hardware reset sequence for VL53L5CX:
     * 1. Pull PWR_EN low  (power off)
     * 2. Wait 2ms
     * 3. Pull PWR_EN high (power on)
     * 4. Wait 2ms
     * 5. Pull LPn low     (hardware reset)
     * 6. Wait 2ms
     * 7. Pull LPn high    (exit reset, sensor starts booting)
     * 8. Wait 50ms (let firmware boot)
     *
     * Note: This function is optional - the ST API can also do a soft reset.
     * Pins must have been initialized as outputs by the user before calling this.
     */
    (void)p_platform; /* Platform struct not needed for pin access */

    gpio_low(VL53L5CX_PWR_EN_PIN);
    system_delay_ms(2);
    gpio_high(VL53L5CX_PWR_EN_PIN);
    system_delay_ms(2);
    gpio_low(VL53L5CX_LPn_PIN);
    system_delay_ms(2);
    gpio_high(VL53L5CX_LPn_PIN);
    system_delay_ms(50);

    return 0;
}

void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
    /*
     * Swap byte order for 32-bit words (little-endian <-> big-endian).
     * Required by ST API for DCI data exchange with the sensor.
     */
    uint32_t i;
    uint8_t tmp[4];

    for (i = 0; i < size; i += 4)
    {
        tmp[0] = buffer[i + 3];
        tmp[1] = buffer[i + 2];
        tmp[2] = buffer[i + 1];
        tmp[3] = buffer[i];
        memcpy(&(buffer[i]), tmp, 4);
    }
}

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs)
{
    (void)p_platform;
    system_delay_ms(TimeMs);
    return 0;
}
