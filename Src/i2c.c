/*
  i2c.c - I2C support for EEPROM, keypad and Trinamic plugins

  Part of grblHAL driver for STM32G0xx

  Copyright (c) 2018-2025 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <main.h>

#include "i2c.h"

#ifdef I2C_PORT

#include "grbl/grbl.h"

#if KEYPAD_ENABLE == 1
#include "keypad/keypad.h"
#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C
#define I2C_ADR_I2CBRIDGE 0x47
#endif

#define I2Cport(p) I2CportI(p)
#define I2CportI(p) I2C ## p

#define I2CPORT I2Cport(I2C_PORT)

static uint8_t keycode = 0;
static keycode_callback_ptr keypad_callback = NULL;

// I2C timing configuration for 100kHz at 64MHz system clock
// Formula: PRESC = 0, SCLDEL = 4, SDADEL = 2, SCLH = 0x13, SCLL = 0x13
// This gives ~100kHz I2C clock frequency
#define I2C_TIMING_100KHZ   0x00420F13U

static I2C_HandleTypeDef i2c_port = {
    .Instance = I2CPORT,
    .Init.Timing = I2C_TIMING_100KHZ,
    .Init.OwnAddress1 = 0,
    .Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    .Init.DualAddressMode = I2C_DUALADDRESS_DISABLE,
    .Init.OwnAddress2 = 0,
    .Init.OwnAddress2Masks = I2C_OA2_NOMASK,
    .Init.GeneralCallMode = I2C_GENERALCALL_DISABLE,
    .Init.NoStretchMode = I2C_NOSTRETCH_DISABLE
};

#if I2C_PORT == 1
void I2C1_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&i2c_port);
  HAL_I2C_ER_IRQHandler(&i2c_port);
}
#else
void I2C2_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&i2c_port);
  HAL_I2C_ER_IRQHandler(&i2c_port);
}
#endif

static inline __attribute__((always_inline)) bool wait_ready (void)
{
    while(i2c_port.State != HAL_I2C_STATE_READY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    return true;
}

bool i2c_probe (i2c_address_t i2cAddr)
{
    return wait_ready() && HAL_I2C_IsDeviceReady(&i2c_port, i2cAddr << 1, 4, 10) == HAL_OK;
}

bool i2c_get_keycode (i2c_address_t i2cAddr, keycode_callback_ptr callback)
{
    bool ok;

    if((ok = wait_ready() && HAL_I2C_Master_Receive_IT(&i2c_port, i2cAddr << 1, &keycode, 1) == HAL_OK)) {
        keycode = 0;
        keypad_callback = callback;
    }

    return ok;
}

bool i2c_transfer (i2c_transfer_t *i2c, bool read)
{
    if(!wait_ready())
        return false;

    HAL_StatusTypeDef ret;

    if(read)
        ret = HAL_I2C_Mem_Read(&i2c_port, i2c->address << 1, i2c->word_addr, i2c->word_addr_bytes == 1 ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT, i2c->data, i2c->count, 100);
    else
        ret = HAL_I2C_Mem_Write(&i2c_port, i2c->address << 1, i2c->word_addr, i2c->word_addr_bytes == 1 ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT, i2c->data, i2c->count, 100);

    return ret == HAL_OK;;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(keypad_callback && keycode != 0) {
        keypad_callback(keycode);
        keypad_callback = NULL;
    }
}

#if TRINAMIC_ENABLE && TRINAMIC_I2C

static uint16_t axis = 0xFF;
static const uint16_t tmc_addr = I2C_ADR_I2CBRIDGE << 1;

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    uint8_t buffer[5] = {0};
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        buffer[0] = driver.axis | 0x80;
        HAL_I2C_Mem_Write(&i2c_port, tmc_addr, axis, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);

        axis = driver.axis;
    }

    HAL_I2C_Mem_Read(&i2c_port, tmc_addr, (uint16_t)reg->addr.idx, I2C_MEMADD_SIZE_8BIT, buffer, 5, 100);

    status = buffer[0];
    reg->payload.value = buffer[4];
    reg->payload.value |= buffer[3] << 8;
    reg->payload.value |= buffer[2] << 16;
    reg->payload.value |= buffer[1] << 24;

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    uint8_t buffer[5] = {0};;
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        buffer[0] = driver.axis | 0x80;
        HAL_I2C_Mem_Write(&i2c_port, tmc_addr, axis, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);

        axis = driver.axis;
    }

    buffer[0] = (reg->payload.value >> 24) & 0xFF;
    buffer[1] = (reg->payload.value >> 16) & 0xFF;
    buffer[2] = (reg->payload.value >> 8) & 0xFF;
    buffer[3] = reg->payload.value & 0xFF;

    reg->addr.write = 1;
    HAL_I2C_Mem_Write(&i2c_port, tmc_addr, (uint16_t)reg->addr.value, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);
    reg->addr.write = 0;

    return status;
}

#endif

i2c_cap_t i2c_start (void)
{
    static i2c_cap_t cap = {};

    if(cap.started)
        return cap;

#if I2C_PORT == 1
    // Configure GPIO pins for I2C1: PB8 (SCL), PB9 (SDA)
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = GPIO_PIN_8 | GPIO_PIN_9,  // PB8 (SCL) | PB9 (SDA)
        .Mode = GPIO_MODE_AF_OD,         // Alternate function open-drain
        .Pull = GPIO_PULLUP,             // Pull-up enabled
        .Speed = GPIO_SPEED_FREQ_HIGH,   // High speed
        .Alternate = GPIO_AF6_I2C1       // Alternate function 6 for I2C1
    };
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Enable I2C1 clock
    __HAL_RCC_I2C1_CLK_ENABLE();

    // Initialize I2C peripheral
    HAL_I2C_Init(&i2c_port);

    // Enable I2C1 event and error interrupts
    HAL_NVIC_SetPriority(I2C1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
#endif

#if I2C_PORT == 2
    // Configure GPIO pins for I2C2: PB10 (SCL), PB11 (SDA) - alternative pins
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = GPIO_PIN_10 | GPIO_PIN_11,  // PB10 (SCL) | PB11 (SDA)
        .Mode = GPIO_MODE_AF_OD,           // Alternate function open-drain
        .Pull = GPIO_PULLUP,               // Pull-up enabled
        .Speed = GPIO_SPEED_FREQ_HIGH,     // High speed
        .Alternate = GPIO_AF6_I2C2         // Alternate function 6 for I2C2
    };
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Enable I2C2 clock
    __HAL_RCC_I2C2_CLK_ENABLE();

    // Initialize I2C peripheral
    HAL_I2C_Init(&i2c_port);

    // Enable I2C2 event and error interrupts
    HAL_NVIC_SetPriority(I2C2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(I2C2_IRQn);
#endif

    cap.started = On;

    return cap;
}

#endif // I2C_PORT