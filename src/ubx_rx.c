#include "ubx_rx.h"

#include <string.h>

#define UBX_HEADER_BYTES 6
#define UBX_CKSUM_BYTES 2

void ubx_rx_init(ubx_rx_t *rx) {
	memset(rx, 0, sizeof(*rx));
}

bool ubx_rx_feed_byte(ubx_rx_t *rx, uint8_t data) {
	const uint16_t incoming_cap = (uint16_t)UBX_RX_BUFFER_BYTES;

	if (!rx->sync) {
		if (!rx->buffer_pos && data == 0xB5) {
			rx->buffer_pos = 1;
			rx->buf[0] = data;
		} else if (rx->buffer_pos == 1 && data == 0x62) {
			rx->sync = 1;
			rx->buffer_pos = 2;
			rx->buf[1] = data;
		} else {
			rx->buffer_pos = 0;
		}
		return false;
	}

	if (rx->buffer_pos >= incoming_cap) {
		rx->buffer_pos = 0;
		rx->sync = 0;
		return false;
	}

	rx->buf[rx->buffer_pos] = data;

	uint16_t payload_size = (uint16_t)rx->buf[4] | ((uint16_t)rx->buf[5] << 8);
	if ((rx->buffer_pos >= UBX_HEADER_BYTES - 1)
			&& (rx->buffer_pos - 1 == (payload_size + UBX_HEADER_BYTES + UBX_CKSUM_BYTES - 2))) {
		rx->buffer_pos = 0;
		rx->sync = 0;
		return true;
	}

	rx->buffer_pos++;
	if (rx->buffer_pos >= incoming_cap) {
		rx->buffer_pos = 0;
		rx->sync = 0;
	}
	return false;
}
