/*
 * slip.h
 *
 *  Created on: 19.12.2016
 *      Author: Tobias
 */

#ifndef SRC_UTIL_SLIP_SLIP_H_
#define SRC_UTIL_SLIP_SLIP_H_

#include <zephyr/sys/ring_buffer.h>
#include <zephyr/kernel.h>

/* SLIP special character codes
 */
#define SLIP_END             ((uint8_t)0300)    /* indicates end of packet */
#define SLIP_ESC             ((uint8_t)0333)    /* indicates byte stuffing */
#define SLIP_ESC_END         ((uint8_t)0334)    /* ESC ESC_END means END data byte */
#define SLIP_ESC_ESC         ((uint8_t)0335)    /* ESC ESC_ESC means ESC data byte */

// Configurations

#define SLIP_READ_PACKETS_MAX_QTY 2

struct slipBuffer{
	struct ring_buf ringBuf;
	uint8_t last; 						// last received character.
	struct k_sem ring_buffer_sem;	// Semaphore to protect ring buffer
	struct k_sem packet_ready_sem; 	// Semaphore that signals number of ready to read packets
} ;

/**
 * @brief 
 * 
 * @param slip_buf 
 * @param buf Ring buffer data area.
 * @param size Ring buffer size in bytes
 */
int init_slip_buffer(struct slipBuffer* slip_buf, uint8_t* buf, int size);

void slip_uart_putc(struct slipBuffer* slip_buf, char c);

/* slip_recv_packet: reads a packet from buf into the buffer
 * located at "p". If more than len bytes are received, the
 * packet will be truncated.
 * Returns the number of bytes stored in the buffer.
 * Returns 0 if the buffer does not contain a full packet.
 */
int slip_read_packet(struct slipBuffer* slip_buf, uint8_t *p, int len);

void slip_send_packet(uint8_t *p, int len, void (*send_char)(char c));
void slip_encode(const uint8_t* p, int len, void (* send_char)(char c));

// int slipmux_read_packet(volatile slipBuffer_t* buf, uint8_t *p, int len, uint8_t* type);
// void slipmux_send_packet(const uint8_t* p, int len, uint8_t type, void (* send_char)(char c));
// void slipmux_setSemaphores(SemaphoreHandle_t rxSem, SemaphoreHandle_t txSem);

#endif /* SRC_UTIL_SLIP_SLIP_H_ */
