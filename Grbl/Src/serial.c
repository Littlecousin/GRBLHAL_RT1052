/*

  serial.c - serial port implementation for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2022 Terje Io

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

#include <string.h>

#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "main.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;


#ifndef SERIAL_MOD
#define SERIAL_MOD 1
#endif

#define USART 				usart(SERIAL_MOD)//(LPUART_Type *)
#define USART_IRQ 			usartINT(SERIAL_MOD)
#define USART_IRQHandler 	usartHANDLER(SERIAL_MOD)

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serialInit
    },
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    stream_register_streams(&streams);
}

//
// Returns number of free characters in serial input buffer
//
static uint16_t serialRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serialRxCount (void)
{
    uint32_t tail = rxbuf.tail, head = rxbuf.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a character to the serial output stream
//
static bool serialPutC (const char c)
{
    uint16_t next_head = BUFNEXT(txbuf.head, txbuf);    // Get pointer to next free slot in buffer

    while(txbuf.tail == next_head) {                    // While TX buffer full
        if(!hal.stream_blocking_callback())             // check if blocking for space,
            return false;                               // exit if not (leaves TX buffer in an inconsistent state)
    }
    txbuf.data[txbuf.head] = c;                         // Add data to buffer,
    txbuf.head = next_head;                             // update head pointer and
	
//    LPUART_EnableInterrupts(USART, kLPUART_TxDataRegEmptyInterruptEnable);// enable TX interrupts

	uint_fast16_t tail = txbuf.tail;            // Get buffer pointer
	LPUART_WriteByte( USART, txbuf.data[tail]);
	txbuf.tail = tail = BUFNEXT(tail, txbuf);   // and increment pointer
	while (!(USART->STAT & LPUART_STAT_TDRE_MASK))
	{

	}
    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serialWriteS (const char *s)
{
    char c, *ptr = (char *)s;
    while((c = *ptr++) != '\0')
        serialPutC(c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
static void serialWrite (const char *s, uint16_t length)
{
    char *ptr = (char *)s;
    while(length--)
        serialPutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serialTxFlush (void)
{
	// Disable TX interrupts
//	LPUART_DisableInterrupts(USART, kLPUART_TxDataRegEmptyInterruptEnable);
    txbuf.tail = txbuf.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serialTxCount (void)
{
    // uint8_t rx_isempty = 0;
    uint32_t tail = txbuf.tail, head = txbuf.head;
    // rx_isempty = (USART->STAT & LPUART_STAT_TDRE_MASK);
    // //0：没有发送完
    // //1：发送完了
    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + (USART->STAT & LPUART_STAT_TDRE_MASK ? 0 : 1);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer
    if(tail == rxbuf.head)
        return -1; // no data available
    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer
    return (int16_t)data;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool serialSetBaudRate (uint32_t baud_rate)
{
	LPUART_SetBaudRate(USART,baud_rate,BOARD_DebugConsoleSrcFreq());
    return true;
}

static bool serialDisable (bool disable)
{
    if(disable)

		LPUART_EnableRx(USART,0);
    else

		LPUART_EnableRx(USART,1);

    return true;
}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

ring_fifo_t *serial_fifo_handle;
uint8_t ring_buffer[8192] = {0};
static uint32_t rx_count = 0;
//待测试的fifo类型
#define TEST_FIFO_TYPE RF_TYPE_STREAM//RF_TYPE_STREAM RF_TYPE_FRAME

const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = On,
        .read = serialGetC,
        .write = serialWriteS,
        .write_n =  serialWrite,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .get_tx_buffer_count = serialTxCount,
        .reset_write_buffer = serialTxFlush,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .disable_rx = serialDisable,
        .set_baud_rate = serialSetBaudRate,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed)
        return NULL;

    serial[0].flags.claimed = On;

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART1,
        .port = GPIO1,
        .pin = 9,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "UART1"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART1,
        .port = GPIO1,
        .pin = 10,
        .mode = { .mask = PINMODE_NONE },
        .description = "UART1"
    };
	
	hal.periph_port.register_pin(&rx);
	hal.periph_port.register_pin(&tx);
	uint32_t status;
	UART_ModeConfig();
	status = ring_fifo_init(&serial_fifo_handle,ring_buffer,sizeof(ring_buffer),TEST_FIFO_TYPE);
	if(status != RF_SUCCESS)
    {
        while(1)
		{
			
		}
    }
    return &stream;
}

uint32_t RxOverrunFlag;

void USART_IRQHandler (void)
{
	QUEUE_DATA_TYPE *data_p;
    uint8_t data;
    /* If new data arrived. */
    if ((kLPUART_RxDataRegFullFlag)&LPUART_GetStatusFlags(USART))//接收完成中断
    {
		//接收数据中断
        data = LPUART_ReadByte(USART);
		ring_fifo_write(serial_fifo_handle,&data,1);
		if (!enqueue_realtime_command((char)data))
        {                                                    // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf); // Get and increment buffer pointer
            if (next_head == rxbuf.tail)                     // If buffer full
                rxbuf.overflow = 1;                          // flag overflow
            else
            {
                rxbuf.data[rxbuf.head] = (char)data; // if not add data to buffer
                rxbuf.head = next_head;              // and update pointer
            }
        }
    }
	if ((kLPUART_RxOverrunFlag)&LPUART_GetStatusFlags(USART))//接收完成中断
    {
		RxOverrunFlag++;
		LPUART_ClearStatusFlags(DEBUG_UARTx, kLPUART_RxOverrunFlag);   
	}
	SDK_ISR_EXIT_BARRIER;
}
