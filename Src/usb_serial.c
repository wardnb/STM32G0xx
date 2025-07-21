/*
  usb_serial.c - USB serial I/O stream for STM32G0xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <string.h>

#include "usb_serial.h"
#include "grbl/protocol.h"

// Forward declaration to avoid HAL include issues
extern uint32_t HAL_GetTick(void);

#if USB_SERIAL_CDC
#include "usb_device.h"
#include "usbd_cdc_if.h"
#endif

#if USB_SERIAL_CDC
// Use actual USB middleware definitions
#else
// Minimal USB placeholder definitions for compilation when USB is disabled
typedef enum {
    USBD_OK = 0,
    USBD_BUSY = 1,
    USBD_FAIL = 2
} USBD_StatusTypeDef;

// USB function prototypes for placeholder mode
void MX_USB_DEVICE_Init(void);
USBD_StatusTypeDef CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
#endif

#define USB_SER_RX_BUFFER_SIZE 256
#define USB_SER_TX_BUFFER_SIZE 128
#define USB_CONNECT_TIMEOUT_MS 5000
#define USB_RECONNECT_DELAY_MS 100

// USB connection state management
typedef struct {
    bool connected;
    bool enumerated;
    bool tx_busy;
    uint32_t connect_time;
    uint32_t disconnect_time;
    uint32_t last_tx_time;
    uint32_t tx_timeout_count;
    uint32_t reconnect_count;
} usb_state_t;

// USB transmit buffer for flow control
typedef struct {
    char data[USB_SER_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile bool overflow;
} usb_tx_buffer_t;

static stream_rx_buffer_t rxbuf = {0};
static char rxdata[USB_SER_RX_BUFFER_SIZE];
static usb_tx_buffer_t txbuf = {0};
static volatile usb_state_t usb_state = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

volatile usb_linestate_t usb_linestate = {0};

// USB state management functions
static void usbUpdateConnectionState(void);
static bool usbIsReady(void);
static void usbHandleDisconnect(void);
static void usbFlushTxBuffer(void);

//
// Check if USB is connected and enumerated properly
//
static bool usbIsReady(void)
{
    return usb_state.connected && usb_state.enumerated && usb_linestate.pin.dtr;
}

//
// Update USB connection state based on line state and timing
//
static void usbUpdateConnectionState(void)
{
    uint32_t now = HAL_GetTick();
    bool currently_connected = usb_linestate.pin.dtr;
    
    // Detect connection state changes
    if (currently_connected && !usb_state.connected) {
        // Connection established
        usb_state.connected = true;
        usb_state.connect_time = now;
        usb_state.enumerated = false; // Wait for enumeration to complete
        usb_state.reconnect_count++;
    } else if (!currently_connected && usb_state.connected) {
        // Connection lost
        usbHandleDisconnect();
    }
    
    // Check for enumeration completion
    if (usb_state.connected && !usb_state.enumerated) {
        // Consider enumerated after connection has been stable for a short time
        if ((now - usb_state.connect_time) > USB_RECONNECT_DELAY_MS) {
            usb_state.enumerated = true;
        }
    }
    
    // Handle connection timeout
    if (usb_state.connected && !usb_state.enumerated) {
        if ((now - usb_state.connect_time) > USB_CONNECT_TIMEOUT_MS) {
            // Enumeration timeout - force disconnect and retry
            usbHandleDisconnect();
        }
    }
}

//
// Handle USB disconnect event
//
static void usbHandleDisconnect(void)
{
    usb_state.connected = false;
    usb_state.enumerated = false;
    usb_state.tx_busy = false;
    usb_state.disconnect_time = HAL_GetTick();
    
    // Clear transmit buffer on disconnect
    txbuf.head = txbuf.tail = 0;
    txbuf.overflow = false;
}

//
// Returns number of characters in USB serial input buffer
//
static uint16_t usbRxCount (void)
{
    uint16_t tail = rxbuf.tail;
    return BUFCOUNT(rxbuf.head, tail, USB_SER_RX_BUFFER_SIZE);
}

//
// Flushes the USB serial input buffer
//
static void usbRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the USB serial input buffer
//
static void usbRxCancel (void)
{
    rxdata[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = (rxbuf.tail + 1) & (USB_SER_RX_BUFFER_SIZE - 1);
}

//
// Returns number of characters in TX buffer
//
static uint16_t usbTxCount(void)
{
    uint16_t tail = txbuf.tail;
    return BUFCOUNT(txbuf.head, tail, USB_SER_TX_BUFFER_SIZE);
}

//
// Returns free space in TX buffer  
//
static uint16_t usbTxFree(void)
{
    return (USB_SER_TX_BUFFER_SIZE - 1) - usbTxCount();
}

//
// Add character to transmit buffer
//
static bool usbTxBufferPutC(char c)
{
    uint16_t next_head = (txbuf.head + 1) & (USB_SER_TX_BUFFER_SIZE - 1);
    
    if (next_head == txbuf.tail) {
        txbuf.overflow = true;
        return false; // Buffer full
    }
    
    txbuf.data[txbuf.head] = c;
    txbuf.head = next_head;
    return true;
}

//
// Flush transmit buffer to USB
//
static void usbFlushTxBuffer(void)
{
    if (!usbIsReady() || usb_state.tx_busy) {
        return;
    }
    
    uint16_t count = usbTxCount();
    if (count == 0) {
        return;
    }
    
    // Prepare data for transmission
    static uint8_t tx_packet[64]; // USB CDC packet size
    uint16_t packet_size = (count > sizeof(tx_packet)) ? sizeof(tx_packet) : count;
    uint16_t tail = txbuf.tail;
    
    for (uint16_t i = 0; i < packet_size; i++) {
        tx_packet[i] = txbuf.data[tail];
        tail = (tail + 1) & (USB_SER_TX_BUFFER_SIZE - 1);
    }
    
    // Attempt transmission
    if (CDC_Transmit_FS(tx_packet, packet_size) == USBD_OK) {
        // Success - update buffer tail
        txbuf.tail = tail;
        usb_state.tx_busy = true;
        usb_state.last_tx_time = HAL_GetTick();
    } else {
        // Failed - increment timeout counter
        usb_state.tx_timeout_count++;
    }
}

//
// Attempt to send a character bypassing buffering (for high priority data)
//
static bool usbPutCDirect(char c)
{
    if (!usbIsReady() || usb_state.tx_busy) {
        return false;
    }
    
    return CDC_Transmit_FS((uint8_t *)&c, 1) == USBD_OK;
}

//
// Writes a character to the USB serial output stream with buffering
//
static bool usbPutC(char c)
{
    // Update connection state
    usbUpdateConnectionState();
    
    // Try direct transmission first for immediate response
    if (usbTxCount() == 0 && usbPutCDirect(c)) {
        usb_state.tx_busy = true;
        usb_state.last_tx_time = HAL_GetTick();
        return true;
    }
    
    // Fall back to buffering
    bool ok = usbTxBufferPutC(c);
    
    // Try to flush buffer if it's getting full or on line end
    if (c == ASCII_LF || usbTxFree() < 8) {
        usbFlushTxBuffer();
    }
    
    return ok;
}

//
// Writes a string to the USB serial output stream, blocks if buffer full
//
static void usbWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        usbPutC(c);
}

//
// Writes a number of characters from string to the USB serial output stream, blocks if buffer full
//
static void usbWrite (const char *data, uint16_t length)
{
    char *ptr = (char *)data;

    while(length--)
        usbPutC(*ptr++);
}

//
// Flushes the USB serial output stream
//
__attribute__((unused)) static void usbTxFlush(void)
{
    usbUpdateConnectionState();
    usbFlushTxBuffer();
}

//
// Returns number of characters pending transmission
//
static uint16_t usbGetTxCount(void)
{
    return usbTxCount();
}

//
// serialGetC - returns -1 if no data available
//
static int16_t usbGetC(void)
{
    // Update connection state periodically
    usbUpdateConnectionState();
    
    uint16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return -1; // no data available

    char data = rxdata[bptr++];                 // Get next character, increment tmp pointer
    rxbuf.tail = bptr & (USB_SER_RX_BUFFER_SIZE - 1);  // and update pointer

    return (int16_t)data;
}

static bool usbSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usbEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usbSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler != NULL)
        enqueue_realtime_command = handler;

    return prev;
}

static const io_stream_t usb_stream = {
    .type = StreamType_Serial,
    .state.is_usb = On,
    .read = usbGetC,
    .write = usbWriteS,
    .write_n = usbWrite,
    .enqueue_rt_command = usbEnqueueRtCommand,
    .get_rx_buffer_count = usbRxCount,
    .get_tx_buffer_count = usbGetTxCount,
    .reset_read_buffer = usbRxFlush,
    .cancel_read_buffer = usbRxCancel,
    .suspend_read = usbSuspendInput,
    .set_enqueue_rt_handler = usbSetRtHandler
};

void usbBufferInput(uint8_t *data, uint32_t length)
{
    while(length--) {
        // Only buffer input if USB is properly connected and enumerated
        if(usbIsReady() || enqueue_realtime_command != protocol_enqueue_realtime_command) {

            uint16_t next_head = (rxbuf.head + 1) & (USB_SER_RX_BUFFER_SIZE - 1);   // Get next head pointer

            if(next_head == rxbuf.tail)                                             // If buffer full
                rxbuf.overflow = On;                                                // flag overflow
            else {
                rxdata[rxbuf.head] = *data;                                     // Add data to buffer
                rxbuf.head = next_head;                                             // and update pointer
            }
        }

        data++;
    }
}

//
// "dummy" version of stream buffering function
//
__attribute__((unused)) static uint16_t usbRxFree (void)
{
    return (USB_SER_RX_BUFFER_SIZE - 1) - usbRxCount();
}

bool usbIsConnected(void)
{
    usbUpdateConnectionState();
    return usbIsReady();
}

//
// Callback for USB transmit completion
//
void usbTransmitComplete(void)
{
    usb_state.tx_busy = false;
    
    // Try to send more data if available
    if (usbTxCount() > 0) {
        usbFlushTxBuffer();
    }
}

const io_stream_t *usbInit(void)
{
    // Initialize buffers
    rxbuf.head = rxbuf.tail = 0;
    rxbuf.overflow = Off;
    txbuf.head = txbuf.tail = 0;
    txbuf.overflow = false;
    
    // Initialize USB state
    usb_state.connected = false;
    usb_state.enumerated = false;
    usb_state.tx_busy = false;
    usb_state.connect_time = 0;
    usb_state.disconnect_time = 0;
    usb_state.last_tx_time = 0;
    usb_state.tx_timeout_count = 0;
    usb_state.reconnect_count = 0;

#if USB_SERIAL_CDC
    // Initialize USB device with actual implementation
    MX_USB_DEVICE_Init();
#endif

    return &usb_stream;
}

#if !USB_SERIAL_CDC
// Placeholder implementations for compilation when USB is disabled
void MX_USB_DEVICE_Init(void) {
    // Placeholder - USB not enabled
}

USBD_StatusTypeDef CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) {
    // Placeholder - USB not enabled
    return USBD_OK;
}
#endif