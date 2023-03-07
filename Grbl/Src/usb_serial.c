/*

  usb_serial.c - USB serial port implementation for STM32F103C8 ARM processors

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

#include "driver.h"

#if USB_SERIAL_CDC

#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "main.h"
//#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "bsp_usb_vom.h"
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"

#include "usb_device_descriptor.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_block_tx_buffer2_t txbuf = {0};
//static stream_block_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;
extern usb_cdc_vcom_struct_t s_cdcVcom;
//
// Returns number of free characters in the input buffer
//
static uint16_t usb_serialRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;
	return (uint16_t)((RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE));
}

//
// Flushes the input buffer
//
static void usb_serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the input buffer
//
static void usb_serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes current buffer to the USB output stream, swaps buffers
//
//static inline bool _usb_write(void)
//{
//    size_t length, txfree;
//    txbuf.s = txbuf.data;
//    while (txbuf.length)
//    {
//        if ((txfree = usb_serial_write_buffer_free()) > 10)
//        {
//			length = txfree < txbuf.length ? txfree : txbuf.length;
//			USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, (uint8_t *)txbuf.s, txbuf.length);
//			txbuf.length -= length;
//			txbuf.s += length;
//        }
//        if (txbuf.length && !hal.stream_blocking_callback())
//        {
//            txbuf.length = 0;
//            txbuf.s = txbuf.data;
//            return false;
//        }
//    }
//    txbuf.s = txbuf.data;
//    return true;
//}
static inline bool _usb_write (void)
{
    static uint8_t dummy = 0;
    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;
	while(USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, (uint8_t *)txbuf.s, txbuf.length) != kStatus_USB_Success) {
        if(!hal.stream_blocking_callback())
            return false;
    }
    if(txbuf.length % 64 == 0) {
        while(USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, &dummy, 0) != kStatus_USB_Success) {
            if(!hal.stream_blocking_callback())
                return false;
        }
    }
    txbuf.use_tx2data = !txbuf.use_tx2data;
    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;
    txbuf.length = 0;
    return true;
}

//
// Writes a single character to the USB output stream, blocks if buffer full
//
static bool usb_serialPutC (const char c)
{
    static uint8_t buf[1];

    *buf = c;

    while(USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, buf, 1) != kStatus_USB_Success) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    return true;
}

//
// Writes a number of characters from string to the USB output stream, blocks if buffer full
//
static void usb_serialWrite (const char *s, uint16_t length)
{
    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!_usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!_usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        _usb_write();
    }
}
//
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//
static void usb_serialWriteS(const char *s)
{
    if (*s == '\0')
        return;
    size_t length = strlen(s);
    if ((length + txbuf.length) < BLOCK_TX_BUFFER_SIZE)
    {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;

        if (s[length - 1] == ASCII_LF || txbuf.length > txbuf.max_length)
        {
            if (!_usb_write())
                return;
        }
    }
    else
        usb_serialWrite(s, (uint16_t)length);
}

//static void usb_serialWriteS (const char *s)
//{
//    size_t length = strlen(s);

//    if(length == 0)
//        return;

//    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
//        if(!_usb_write())
//            return;
//    }

//    while(length > txbuf.max_length) {
//        txbuf.length = txbuf.max_length;
//        memcpy(txbuf.s, s, txbuf.length);
//        if(!_usb_write())
//            return;
//        length -= txbuf.max_length;
//        s += txbuf.max_length;
//    }

//    if(length) {
//        memcpy(txbuf.s, s, length);
//        txbuf.length += length;
//        txbuf.s += length;
//        if(s[length - 1] == ASCII_LF)
//            _usb_write();
//    }
//}

//
// usb_serialGetC - returns -1 if no data available
//
static int16_t usb_serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

    return (int16_t)data;
}

static bool usb_serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usb_serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usb_serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

#define BLOCK_RX_BUFFER_SIZE 20
uint32_t millis();
uint32_t USB_VCOM_Available();
uint32_t USB_VCOM_Read(uint8_t *pBuf,uint16_t len);
void USB_VCOM_Task(void);
//
// This function get called from the foregorund process,
// used here to get characters off the USB serial input stream and buffer
// them for processing by the core. Real time command characters are stripped out
// and submitted for realtime processing.
//
void usb_execute_realtime(sys_state_t state)
{
    static volatile bool lock = false;
    static volatile uint32_t last_millis = 0;
    static uint8_t tmpbuf[BLOCK_RX_BUFFER_SIZE];
	
    // if(lock)
    //     return;

    uint32_t current_micros;
    if (lock || ((current_micros = millis()) - last_millis) < 1)
        return;

    uint8_t c, *dp;
    int avail, free;

    lock = true;
    last_millis = current_micros;
	USB_VCOM_Task();
    if ((avail = USB_VCOM_Available()))//获取接收缓冲区的可用数据长度
    {
        dp = tmpbuf;
        free = usb_serialRxFree();
        free = free > BLOCK_RX_BUFFER_SIZE ? BLOCK_RX_BUFFER_SIZE : free;
        avail = USB_VCOM_Read(tmpbuf, avail > free ? free : avail);
        while (avail--)
        {
            c = *dp++;
            if (!enqueue_realtime_command(c))
            {
                uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf); // Get next head pointer
                if (next_head == rxbuf.tail)                          // If buffer full
                    rxbuf.overflow = On;                              // flag overflow,
                else
                {
                    rxbuf.data[rxbuf.head] = c; // else add character data to buffer
                    rxbuf.head = next_head;     // and update pointer
                }
            }
        }
    }
    lock = false;
}

extern void USB_VCOM_Init(void);
// NOTE: USB interrupt priority should be set lower than stepper/step timer to avoid jitter
// It is set in HAL_PCD_MspInit() in usbd_conf.c
const io_stream_t *usb_serialInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.is_usb = On,
        .read = usb_serialGetC,
        .write = usb_serialWriteS,
        .write_char = usb_serialPutC,
        .write_n = usb_serialWrite,
        .enqueue_rt_command = usb_serialEnqueueRtCommand,
        .get_rx_buffer_free = usb_serialRxFree,
        .reset_read_buffer = usb_serialRxFlush,
        .cancel_read_buffer = usb_serialRxCancel,
        .suspend_read = usb_serialSuspendInput,
        .set_enqueue_rt_handler = usb_serialSetRtHandler
    };
	
	USB_VCOM_Init();
    txbuf.s = txbuf.data;
    txbuf.max_length = BLOCK_TX_BUFFER_SIZE;
	grbl.on_execute_realtime = usb_execute_realtime;
    return &stream;
}

// NOTE: add a call to this function as the first line CDC_Receive_FS() in usbd_cdc_if.c
void usbBufferInput (uint8_t *data, uint32_t length)
{
	static uint8_t temp = 0;
    while(length--) 
	{
        if(!enqueue_realtime_command(*data)) 
		{                  // Check and strip realtime commands,
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                         // If buffer full
                rxbuf.overflow = 1;                             // flag overflow
            else 
			{
                rxbuf.data[rxbuf.head] = *data;                 // if not add data to buffer
                rxbuf.head = next_head;                         // and update pointer
            }
        }
        data++;                                                 // next...
    }
	USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, &temp, 0);
}

#endif
