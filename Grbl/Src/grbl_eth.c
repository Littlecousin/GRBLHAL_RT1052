
#include "driver.h"

#if GRBL_ETH

#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "main.h"
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "tcpecho_raw.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_block_tx_buffer2_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;
//extern eth_cdc_vcom_struct_t s_cdcVcom;
extern struct tcp_pcb *tcpecho_raw_pcb_test;

int tcpecho_main(void);
//
// Returns number of free characters in the input buffer
//
static uint16_t streamRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the input buffer
//
static void streamRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the input buffer
//
static void websocketd_RxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes current buffer to the USB output stream, swaps buffers
//
static inline bool eth_write (void)
{
    static uint8_t dummy = 0;

    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;

	tcp_write(tcpecho_raw_pcb_test, (uint8_t *)txbuf.s, txbuf.length, 1);
//	while(tcp_write(tcpecho_raw_pcb, (uint8_t *)txbuf.s, txbuf.length, 1) != ERR_OK) 
//	{
//		if(!hal.stream_blocking_callback())
//            return false;
//	}
    txbuf.use_tx2data = !txbuf.use_tx2data;
    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;
    txbuf.length = 0;

    return true;
}

//
// Writes a single character to the USB output stream, blocks if buffer full
//
static bool streamPutC (const char c)
{
    static uint8_t buf[1];
    *buf = c;
	tcp_write(tcpecho_raw_pcb_test, buf, 1, 1);
//	while(tcp_write(tcpecho_raw_pcb, buf, 1, 1) != ERR_OK) 
//	{
//		if(!hal.stream_blocking_callback())
//            return false;
//	}
    return true;
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//
static void streamWriteS (const char *s)
{
    size_t length = strlen(s);

    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!eth_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!eth_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        if(s[length - 1] == ASCII_LF)
            eth_write();
    }
}

//
// Writes a number of characters from string to the USB output stream, blocks if buffer full
//
static void streamWrite (const char *s, uint16_t length)
{
    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!eth_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!eth_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        eth_write();
    }
}

//
// streamGetC - returns -1 if no data available
//
static int16_t streamGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

    return (int16_t)data;
}

static bool streamSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool streamEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr streamSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}
bool tcpecho_check();
// NOTE: USB interrupt priority should be set lower than stepper/step timer to avoid jitter
// It is set in HAL_PCD_MspInit() in usbd_conf.c
const io_stream_t *eth_serialInit (void)
{
//    static const io_stream_t stream = {
//        .type = StreamType_Serial,
//        .state.is_usb = On,
//        .read = streamGetC,
//        .write = streamWriteS,
//        .write_char = streamPutC,
//        .write_n = streamWrite,
//        .enqueue_rt_command = streamEnqueueRtCommand,
//        .get_rx_buffer_free = streamRxFree,
//        .reset_read_buffer = streamRxFlush,
//        .cancel_read_buffer = websocketd_RxCancel,
//        .suspend_read = streamSuspendInput,
//        .set_enqueue_rt_handler = streamSetRtHandler
//    };
	
    static const io_stream_t eth_stream = {
        .type = StreamType_WebSocket,
        .state.connected = true,
//        .state.webui_connected = true,
        .read = streamGetC,
        .write = streamWriteS,
        .write_char = streamPutC,
		.write_n = streamWrite,
        .enqueue_rt_command = streamEnqueueRtCommand,
        .get_rx_buffer_free = streamRxFree,
        .reset_read_buffer = streamRxFlush,
        .cancel_read_buffer = websocketd_RxCancel,
        .suspend_read = streamSuspendInput,
        .set_enqueue_rt_handler = streamSetRtHandler
    };
	tcpecho_main();
	while(tcpecho_check() != 1)
	{
		eth_calling();
	}
    txbuf.s = txbuf.data;
    txbuf.max_length = BLOCK_TX_BUFFER_SIZE;
    return &eth_stream;
}

void ethBufferInput (uint8_t *data, uint32_t length)
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
//	USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, &temp, 0);
}

#endif
