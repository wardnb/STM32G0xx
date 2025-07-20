/*
  usb_serial.h - USB serial I/O stream for STM32G0xx ARM processors

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

#ifndef _USB_SERIAL_H_
#define _USB_SERIAL_H_

#include "grbl/hal.h"

typedef struct {
    uint32_t timestamp;
    union {
        uint8_t value;
        serial_linestate_t pin;
    };
} usb_linestate_t;

extern volatile usb_linestate_t usb_linestate;

const io_stream_t *usbInit (void);
void usbBufferInput (uint8_t *data, uint32_t length);

#endif