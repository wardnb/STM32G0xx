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

// Minimal USB placeholder definitions for compilation
// Full USB implementation would require complete STM32 USB middleware
typedef enum {
    USBD_OK = 0,
    USBD_BUSY = 1,
    USBD_FAIL = 2
} USBD_StatusTypeDef;

// USB function prototypes
void MX_USB_DEVICE_Init(void);
USBD_StatusTypeDef CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#define USB_SER_RX_BUFFER_SIZE 256

static stream_rx_buffer_t rxbuf = {0};
static char rxdata[USB_SER_RX_BUFFER_SIZE];
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

volatile usb_linestate_t usb_linestate = {0};

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
// Attempt to send a character bypassing buffering
//
static bool usbPutCDirect (char c)
{
    return CDC_Transmit_FS((uint8_t *)&c, 1) == USBD_OK;
}

//
// Writes a character to the USB serial output stream
//
static bool usbPutC (char c)
{
    static uint8_t buf[64];
    static uint8_t len = 0;

    bool ok;

    if((ok = len == 0 && usbPutCDirect(c)))
        return true;

    if(len < sizeof(buf))
        buf[len++] = c;

    if(len == sizeof(buf) || c == ASCII_LF) {
        if((ok = CDC_Transmit_FS(buf, len) == USBD_OK))
            len = 0;
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
static void usbTxFlush (void)
{
    // USB handles flushing automatically
}

//
// Returns number of characters pending transmission
//
static uint16_t usbTxCount (void)
{
    // USB handles transmission automatically
    return 0;
}

//
// serialGetC - returns -1 if no data available
//
static int16_t usbGetC (void)
{
    uint16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return -1; // no data available else EOF

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
    .get_tx_buffer_count = usbTxCount,
    .reset_read_buffer = usbRxFlush,
    .cancel_read_buffer = usbRxCancel,
    .suspend_read = usbSuspendInput,
    .set_enqueue_rt_handler = usbSetRtHandler
};

void usbBufferInput (uint8_t *data, uint32_t length)
{
    while(length--) {

        if(usb_linestate.pin.dtr || enqueue_realtime_command != protocol_enqueue_realtime_command) {

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
static uint16_t usbRxFree (void)
{
    return (USB_SER_RX_BUFFER_SIZE - 1) - usbRxCount();
}

bool usbIsConnected (void)
{
    return usb_linestate.pin.dtr;
}

const io_stream_t *usbInit (void)
{
    rxbuf.head = rxbuf.tail = 0;
    rxbuf.overflow = Off;

    // TODO: Initialize USB peripheral when middleware is complete
    // MX_USB_DEVICE_Init();

    return &usb_stream;
}

// Placeholder implementations for compilation
void MX_USB_DEVICE_Init(void) {
    // TODO: Initialize USB device when middleware is complete
}

USBD_StatusTypeDef CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) {
    // TODO: Transmit data via USB CDC when middleware is complete
    return USBD_OK;
}