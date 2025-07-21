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

#define SERIAL_TX_BUFFER_SIZE 256
#define SERIAL_ERROR_TIMEOUT_MS 1000
#define SERIAL_TX_TIMEOUT_MS 100
#define SERIAL_RECONNECT_DELAY_MS 50

// Serial connection state management
typedef struct {
    bool connected;
    bool tx_busy;
    bool error_state;
    uint32_t last_rx_time;
    uint32_t last_tx_time;
    uint32_t error_timestamp;
    uint32_t tx_error_count;
    uint32_t rx_error_count;
    uint32_t overrun_count;
    uint32_t framing_error_count;
    uint32_t parity_error_count;
    uint32_t noise_error_count;
    uint32_t reconnect_count;
} serial_state_t;

// Enhanced transmit buffer for flow control  
typedef struct {
    char data[SERIAL_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile bool overflow;
    volatile bool busy;
} serial_tx_buffer_t;

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

// Serial state management functions
static void serialUpdateState(void);
static bool serialIsHealthy(void);
static void serialHandleError(void);
static void serialRecoverFromError(void);
static bool serialTxBufferPutC(char c);
static void serialFlushTxBuffer(void);

static stream_block_tx_buffer_t txbuf = {0};
static stream_rx_buffer_t rxbuffer = {0};
static serial_tx_buffer_t tx_buffer = {0};
static volatile serial_state_t serial_state = {0};

static void uart_interrupt_handler (void);

UART_HandleTypeDef huart1;

//
// Check if UART is healthy and ready for communication
//
static bool serialIsHealthy(void)
{
    return serial_state.connected && !serial_state.error_state;
}

//
// Update serial connection state based on UART status and timing
//
static void serialUpdateState(void)
{
    uint32_t now = HAL_GetTick();
    
    // Check UART peripheral state - simplified for STM32G0
    bool uart_ready = (huart1.ErrorCode == 0);
                     
    if (uart_ready && serial_state.error_state) {
        // Recovery from error state
        if ((now - serial_state.error_timestamp) > SERIAL_ERROR_TIMEOUT_MS) {
            serial_state.error_state = false;
            serial_state.connected = true;
            serial_state.reconnect_count++;
        }
    } else if (!uart_ready && !serial_state.error_state) {
        // Enter error state
        serialHandleError();
    }
    
    // Update activity timestamps
    if (serialRxCount() > 0) {
        serial_state.last_rx_time = now;
    }
}

//
// Handle UART error condition
//
static void serialHandleError(void)
{
    serial_state.error_state = true;
    serial_state.connected = false;
    serial_state.error_timestamp = HAL_GetTick();
    
    // Clear transmit buffer on error
    tx_buffer.head = tx_buffer.tail = 0;
    tx_buffer.overflow = false;
    tx_buffer.busy = false;
    
    // Start recovery process
    serialRecoverFromError();
}

//
// Attempt to recover from UART error
//
static void serialRecoverFromError(void)
{
    // Reset UART error code
    huart1.ErrorCode = 0;
    
    // Re-enable receive interrupt (keep it simple for STM32G0)
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

//
// Get transmit buffer count
//
static uint16_t serialGetTxCount(void)
{
    uint16_t tail = tx_buffer.tail;
    return BUFCOUNT(tx_buffer.head, tail, SERIAL_TX_BUFFER_SIZE);
}

//
// Get transmit buffer free space
//
static uint16_t serialGetTxFree(void)
{
    return (SERIAL_TX_BUFFER_SIZE - 1) - serialGetTxCount();
}

//
// Add character to transmit buffer
//
static bool serialTxBufferPutC(char c)
{
    uint16_t next_head = (tx_buffer.head + 1) & (SERIAL_TX_BUFFER_SIZE - 1);
    
    if (next_head == tx_buffer.tail) {
        tx_buffer.overflow = true;
        return false; // Buffer full
    }
    
    tx_buffer.data[tx_buffer.head] = c;
    tx_buffer.head = next_head;
    return true;
}

//
// Flush transmit buffer to UART
//
static void serialFlushTxBuffer(void)
{
    if (!serialIsHealthy() || tx_buffer.busy) {
        return;
    }
    
    uint16_t count = serialGetTxCount();
    if (count == 0) {
        return;
    }
    
    // Determine how much we can transmit
    uint16_t tx_size = (count > 64) ? 64 : count; // Limit to reasonable packet size
    static uint8_t tx_packet[64];
    uint16_t tail = tx_buffer.tail;
    
    // Copy data to transmission packet
    for (uint16_t i = 0; i < tx_size; i++) {
        tx_packet[i] = tx_buffer.data[tail];
        tail = (tail + 1) & (SERIAL_TX_BUFFER_SIZE - 1);
    }
    
    // Attempt non-blocking transmission
    if (HAL_UART_Transmit_IT(&huart1, tx_packet, tx_size) == HAL_OK) {
        // Success - update buffer tail and mark busy
        tx_buffer.tail = tail;
        tx_buffer.busy = true;
        serial_state.last_tx_time = HAL_GetTick();
    } else {
        // Failed transmission
        serial_state.tx_error_count++;
    }
}

//
// Callback for UART transmit completion
//
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        tx_buffer.busy = false;
        
        // Try to send more data if available
        if (serialGetTxCount() > 0) {
            serialFlushTxBuffer();
        }
    }
}

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
    // Initialize UART1 with enhanced configuration
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
    // Initialize legacy buffer structures
    txbuf.s = txbuf.data;
    txbuf.max_length = sizeof(txbuf.data);

    rxbuffer.tail = rxbuffer.head = rxbuffer.backup = rxbuffer.data;
    // Note: max_length is handled by grblHAL core buffer management
    rxbuffer.overflow = false;

    // Initialize enhanced transmit buffer
    tx_buffer.head = tx_buffer.tail = 0;
    tx_buffer.overflow = false;
    tx_buffer.busy = false;

    // Initialize serial state
    serial_state.connected = false;
    serial_state.tx_busy = false;
    serial_state.error_state = false;
    serial_state.last_rx_time = 0;
    serial_state.last_tx_time = 0;
    serial_state.error_timestamp = 0;
    serial_state.tx_error_count = 0;
    serial_state.rx_error_count = 0;
    serial_state.overrun_count = 0;
    serial_state.framing_error_count = 0;
    serial_state.parity_error_count = 0;
    serial_state.noise_error_count = 0;
    serial_state.reconnect_count = 0;

    // Initialize UART hardware
    serialInit();

    // Enable UART receive interrupt
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    
    NVIC_EnableIRQ(USART1_IRQn);

    // Mark as connected after successful initialization
    serial_state.connected = true;

    hal.stream = serial;  // grblHAL expects the stream structure directly

    return &serial;
}

//
// serialGetC - returns -1 if no data available
//
int16_t serialGetC (void)
{
    // Update connection state periodically
    serialUpdateState();
    
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available

    char data = rxbuffer.data[bptr++];               // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);   // and update pointer

    return (int16_t)data;
}

bool serialPutC (const char c)
{
    // Update connection state
    serialUpdateState();
    
    // Try direct transmission first if buffer is empty and UART is healthy
    if (serialGetTxCount() == 0 && serialIsHealthy() && !tx_buffer.busy) {
        if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)&c, 1) == HAL_OK) {
            tx_buffer.busy = true;
            serial_state.last_tx_time = HAL_GetTick();
            return true;
        }
    }
    
    // Fall back to buffering
    bool ok = serialTxBufferPutC(c);
    
    // Try to flush buffer if it's getting full or on line end
    if (c == '\n' || serialGetTxFree() < 8) {
        serialFlushTxBuffer();
    }
    
    return ok;
}

void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

void serialWrite(const char *data, uint16_t length)
{
    // Update connection state
    serialUpdateState();
    
    // For larger writes, use buffering system for reliability
    char *ptr = (char *)data;
    while(length--) {
        serialPutC(*ptr++);
    }
}

bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

uint16_t serialTxCount (void)
{
    return serialGetTxCount();
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
    serialUpdateState();
    serialFlushTxBuffer();
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
    // Use HAL interrupt handler for better STM32G0 compatibility
    HAL_UART_IRQHandler(&huart1);
    
    // Simple receive data handling - HAL handles most error processing
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint8_t data = (uint8_t)(huart1.Instance->RDR & 0xFF);
        uint16_t next_head = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);

        if(next_head == rxbuffer.tail) {
            rxbuffer.overflow = true;
            serial_state.overrun_count++;
        } else {
            rxbuffer.data[rxbuffer.head] = data;
            rxbuffer.head = next_head;
            
            // Update activity timestamp
            serial_state.last_rx_time = HAL_GetTick();
        }

        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);
    }
    
    // Check for any error conditions (simplified for STM32G0)
    if (huart1.ErrorCode != 0) {
        serial_state.rx_error_count++;
        
        // Handle severe errors by triggering recovery
        if (serial_state.rx_error_count > 10) {
            serialHandleError();
        }
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

        /* USART1 interrupt Init - High priority for reliable communication */
        HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);  // Priority 2 for UART (lower than USB but still high)
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

//
// HAL UART Error Callback
//
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        // Count HAL-level errors
        serial_state.tx_error_count++;
        
        // Handle the error through our error management system
        serialHandleError();
    }
}