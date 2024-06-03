/**
 * Host test: links only src/ubx_rx.c (no Pico SDK, no ubx.h).
 *   cmake -S tests -B build-host && cmake --build build-host && ctest --test-dir build-host
 */

#include "ubx_rx.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Same Fletcher checksum as include/ubx.h ubx_calculate_checksum. */
static void append_ubx_checksum(uint8_t *frame, size_t header_plus_payload) {
	uint8_t msg_class = frame[2];
	uint8_t msg_id = frame[3];
	uint16_t payload_len = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	ck_a += msg_class;
	ck_b += ck_a;
	ck_a += msg_id;
	ck_b += ck_a;
	ck_a += (uint8_t)(payload_len & 0xffU);
	ck_b += ck_a;
	ck_a += (uint8_t)(payload_len >> 8U);
	ck_b += ck_a;
	for (uint16_t i = 0; i < payload_len; i++) {
		ck_a += frame[6 + i];
		ck_b += ck_a;
	}
	frame[header_plus_payload] = ck_a;
	frame[header_plus_payload + 1] = ck_b;
}

static void feed_expect(ubx_rx_t *rx, const uint8_t *want, size_t want_len) {
	size_t i = 0;
	for (; i + 1 < want_len; i++) {
		assert(!ubx_rx_feed_byte(rx, want[i]));
	}
	assert(ubx_rx_feed_byte(rx, want[want_len - 1]));
	assert(memcmp(rx->buf, want, want_len) == 0);
}

int main(void) {
	ubx_rx_t rx;
	ubx_rx_init(&rx);

	uint8_t tp_payload[16] = {
			0xf8, 0x97, 0x50, 0x01, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	uint8_t tp_frame[6 + 16 + 2];
	memcpy(tp_frame, (uint8_t[]){0xb5, 0x62, 0x0d, 0x01, 0x10, 0x00}, 6);
	memcpy(tp_frame + 6, tp_payload, 16);
	append_ubx_checksum(tp_frame, 6 + 16);
	feed_expect(&rx, tp_frame, sizeof tp_frame);

	ubx_rx_init(&rx);
	uint8_t nc_payload[20];
	uint32_t itow = 22075000;
	int32_t clk_b = -15617;
	int32_t clk_d = 647;
	uint32_t t_acc = 52;
	uint32_t f_acc = 2585;
	memcpy(nc_payload + 0, &itow, 4);
	memcpy(nc_payload + 4, &clk_b, 4);
	memcpy(nc_payload + 8, &clk_d, 4);
	memcpy(nc_payload + 12, &t_acc, 4);
	memcpy(nc_payload + 16, &f_acc, 4);
	uint8_t nc_frame[6 + 20 + 2];
	memcpy(nc_frame, (uint8_t[]){0xb5, 0x62, 0x01, 0x22, 0x14, 0x00}, 6);
	memcpy(nc_frame + 6, nc_payload, 20);
	append_ubx_checksum(nc_frame, 6 + 20);
	feed_expect(&rx, nc_frame, sizeof nc_frame);

	(void)printf("ubx_rx_test: ok\n");
	return 0;
}
