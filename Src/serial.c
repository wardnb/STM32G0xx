/*

  serial.c - low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include <string.h>

#include "driver.h"
#include "serial.h"

#include "grbl/hal.h"
#include "grbl/protocol.h"

// Forward declarations for serial functions
int16_t serialGetC (void);
bool serialPutC (const char c);
void serialWriteS (const char *data);
void serialWrite(const char *data, uint16_t length);
uint16_t serialTxCount (void);
uint16_t serialRxCount (void);
uint16_t serialRxFree (void);
void serialRxFlush (void);
void serialTxFlush (void);
void serialRxCancel (bool disable);

static stream_block_tx_buffer_t txbuf = {0};
static char rxbuf[RX_BUFFER_SIZE];
static stream_rx_buffer_t rxbuffer = {0}, rxbackup;

static void uart_interrupt_handler (void);

UART_HandleTypeDef huart1;

static io_stream_t serial = {
    .type = StreamType_Serial,
    .state = {0},
    .read = serialGetC,
    .write = serialWriteS,
    .write_n = serialWrite,
    .write_char = serialPutC,
    .get_rx_buffer_free = serialRxFree,
    .get_rx_buffer_count = serialRxCount,
    .get_tx_buffer_count = serialTxCount,
    .reset_write_buffer = serialTxFlush,
    .reset_read_buffer = serialRxFlush,
    .disable_rx = serialRxCancel
};

const io_stream_t *serialInit(void)
{
    // Initialize UART1
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
        Error_Handler();
    }
    
    return &serial;
}

io_stream_t *serialInit0 (uint32_t baud_rate)
{
    txbuf.s = txbuf.data;
    txbuf.max_length = sizeof(txbuf.data);

    rxbuffer.tail = rxbuffer.head = rxbuffer.backup = rxbuffer.data;
    // rxbuffer.max_length = RX_BUFFER_SIZE;  // TODO: Fix buffer structure compatibility
    rxbuffer.overflow = false;

    serialInit();

    // Enable UART receive interrupt
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    
    NVIC_EnableIRQ(USART1_IRQn);

    hal.stream = serial;  // grblHAL expects the stream structure directly

    return &serial;
}

//
// serialGetC - returns -1 if no data available
//
int16_t serialGetC (void)
{
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    char data = rxbuffer.data[bptr++];               // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);   // and update pointer

    return (int16_t)data;
}

bool serialPutC (const char c)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&c, 1, HAL_MAX_DELAY);
    return true;
}

void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

void serialWrite(const char *data, uint16_t length)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, length, HAL_MAX_DELAY);
}

bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

uint16_t serialTxCount (void)
{
    return 0; // TODO: implement proper TX buffer count
}

uint16_t serialRxCount (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t serialRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - serialRxCount();
}

void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = false;
}

void serialTxFlush (void)
{
    // TODO: implement TX buffer flush
}

void serialRxCancel (bool disable)
{
    if(disable)
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    else
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void USART1_IRQHandler(void)
{
    uart_interrupt_handler();
}

static void uart_interrupt_handler (void)
{
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        
        uint8_t data = (uint8_t)(huart1.Instance->RDR & 0xFF);
        uint16_t next_head = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);

        if(next_head == rxbuffer.tail) {
            rxbuffer.overflow = true;
        } else {
            rxbuffer.data[rxbuffer.head] = data;
            rxbuffer.head = next_head;
        }

        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
    }
}

// HAL MSP Init function for UART
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    if(huart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        
        /**USART1 GPIO Configuration    
        PA9     ------> USART1_TX
        PA10     ------> USART1_RX 
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);  // For Cortex-M0+, only priority matters
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}