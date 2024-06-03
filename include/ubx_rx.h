#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Receive buffer: must fit uBloxPacket + trailing checksum (see ubx.h). */
#ifndef UBX_RX_BUFFER_BYTES
#define UBX_RX_BUFFER_BYTES 1024
#endif

typedef struct ubx_rx {
	uint8_t sync;
	uint16_t buffer_pos;
	uint8_t buf[UBX_RX_BUFFER_BYTES];
} ubx_rx_t;

void ubx_rx_init(ubx_rx_t *rx);
/** Returns true when a full UBX frame is in rx->buf (same layout as firmware uBloxPacket + checksum). */
bool ubx_rx_feed_byte(ubx_rx_t *rx, uint8_t data);
