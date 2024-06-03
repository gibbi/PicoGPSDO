#include "ubx_clean.h"

#include "hardware/uart.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "ubx_rx.h"

#include <assert.h>
#include <stddef.h>

#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_RX_PIN 17
#define UART_TX_PIN 16

#define UBX_LOG(...)         \
	do {                     \
		printf(__VA_ARGS__); \
		stdio_flush();       \
	} while (0)

static inline void ubx_emit_hex_uart(const uint8_t* buf, size_t len) {
	printf("UBX:");
	for (size_t i = 0; i < len; i++) {
		printf("%s0x%02x", i ? "," : "", (unsigned)buf[i]);
	}
	printf("\n");
	stdio_flush();
}

static inline uint16_t ubx_le_u16(const uint8_t* p) {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t ubx_le_u32(const uint8_t* p) {
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline int32_t ubx_le_i32(const uint8_t* p) {
	return (int32_t)ubx_le_u32(p);
}

static void uart_send(uint8_t c) {
	uart_putc(uart0, c);
}

_Static_assert(sizeof(uBloxPacket) + sizeof(uBloxChecksum) <= UBX_RX_BUFFER_BYTES, "UBX_RX_BUFFER_BYTES");

static uBloxChecksum ubx_calculate_checksum(const uint8_t msgClass, const uint8_t msgId, const uint8_t* message, uint16_t size) {
	uBloxChecksum ck = {0, 0};
	ck.ck_a += msgClass;
	ck.ck_b += ck.ck_a;
	ck.ck_a += msgId;
	ck.ck_b += ck.ck_a;
	ck.ck_a += size & 0xffU;
	ck.ck_b += ck.ck_a;
	ck.ck_a += size >> 8U;
	ck.ck_b += ck.ck_a;
	for (uint16_t i = 0; i < size; i++) {
		ck.ck_a += message[i];
		ck.ck_b += ck.ck_a;
	}
	return ck;
}

static uBloxPacket msgcfgrst = {
		.header = {0xb5, 0x62, .messageClass = 0x06, .messageId = 0x04, .payloadSize = sizeof(uBloxCFGRSTPayload)},
		.data.cfgrst = {.navBbrMask = 0xffff, .resetMode = 1, .reserved1 = 0},
};

static uBloxPacket msgcfgprt = {
		.header = {0xb5, 0x62, .messageClass = 0x06, .messageId = 0x00, .payloadSize = sizeof(uBloxCFGPRTPayload)},
		.data.cfgprt = {.portID = 1, .reserved1 = 0, .txReady = 0, .mode = 0b0000100011000000, .baudRate = 38400, .inProtoMask = 1, .outProtoMask = 1, .flags = 0, .reserved2 = {0, 0}},
};

static uBloxPacket msgcfgrxm = {
		.header = {0xb5, 0x62, .messageClass = 0x06, .messageId = 0x11, .payloadSize = sizeof(uBloxCFGRXMPayload)},
		.data.cfgrxm = {.reserved1 = 8, .lpMode = 0}};

static uBloxPacket msgcfgmsg = {
		.header = {0xb5, 0x62, .messageClass = 0x06, .messageId = 0x01, .payloadSize = sizeof(uBloxCFGMSGPayload)},
		.data.cfgmsg = {.msgClass = 0x01, .msgID = 0x02, .rate = 1}};

static uBloxPacket msgcfgrate = {
		.header = {0xb5, 0x62, .messageClass = 0x06, .messageId = 0x08, .payloadSize = sizeof(uBloxCFGRATEPayload)},
		.data.cfgrate = {.measRate = 1000, .navRate = 1, .timeRef = 0}};

static uBloxPacket msgcfgnav5 = {
		.header = {0xb5, 0x62, .messageClass = 0x06, .messageId = 0x24, .payloadSize = sizeof(uBloxCFGNAV5Payload)},
		.data.cfgnav5 = {.mask = 0b0000001111111111, .dynModel = 2, .fixMode = 3, .fixedAlt = 0, .fixedAltVar = 10000, .minElev = 5, .drLimit = 0, .pDop = 25, .tDop = 25, .pAcc = 100, .tAcc = 300, .staticHoldThresh = 0, .dgpsTimeOut = 2, .reserved2 = 0, .reserved3 = 0, .reserved4 = 0},
};

gps_data current_gps_data;
volatile bool ack_received = false;
volatile bool nack_received = false;

static void ubx_handle_packet(uBloxPacket* pkt) {
	if (pkt->header.payloadSize > UBX_MAX_PAYLOAD_BYTES) {
		current_gps_data.bad_packets += 1;
		return;
	}

	uBloxChecksum cksum = ubx_calculate_checksum(pkt->header.messageClass, pkt->header.messageId, (const uint8_t*)&pkt->data, pkt->header.payloadSize);
	uBloxChecksum* checksum = (uBloxChecksum*)(((uint8_t*)&pkt->data) + pkt->header.payloadSize);
	if (cksum.ck_a != checksum->ck_a || cksum.ck_b != checksum->ck_b) {
		current_gps_data.bad_packets += 1;
		return;
	}

	if (pkt->header.messageClass == 0x01 && pkt->header.messageId == 0x07) {
		current_gps_data.ok_packets += 1;
		current_gps_data.time_of_week_millis = pkt->data.navpvt.iTOW;
		current_gps_data.year = pkt->data.navpvt.year;
		current_gps_data.month = pkt->data.navpvt.month;
		current_gps_data.day = pkt->data.navpvt.day;
		current_gps_data.hours = pkt->data.navpvt.hour;
		current_gps_data.minutes = pkt->data.navpvt.min;
		current_gps_data.seconds = pkt->data.navpvt.sec;
		current_gps_data.fix = pkt->data.navpvt.fixType;
		current_gps_data.latitude_degrees_1000000 = pkt->data.navpvt.lat;
		current_gps_data.longitude_degrees_1000000 = pkt->data.navpvt.lon;
		current_gps_data.altitude_mm = pkt->data.navpvt.hMSL;
		current_gps_data.satellites_visible = pkt->data.navpvt.numSV;
		current_gps_data.ground_speed_cm_per_second = pkt->data.navpvt.gSpeed;
		current_gps_data.heading_degrees_100000 = pkt->data.navpvt.headMot;
		current_gps_data.climb_cm_per_second = -pkt->data.navpvt.velD;
		current_gps_data.updated = true;
	} else if (pkt->header.messageClass == 0x01 && pkt->header.messageId == 0x12) {
		current_gps_data.ok_packets += 1;
		current_gps_data.time_of_week_millis = pkt->data.navvelned.iTOW;
		current_gps_data.ground_speed_cm_per_second = pkt->data.navvelned.gSpeed;
		current_gps_data.heading_degrees_100000 = pkt->data.navvelned.headMot;
		current_gps_data.climb_cm_per_second = -pkt->data.navvelned.velD;
		current_gps_data.updated = true;
	} else if (pkt->header.messageClass == 0x01 && pkt->header.messageId == 0x02) {
		current_gps_data.ok_packets += 1;
		current_gps_data.time_of_week_millis = pkt->data.navposllh.iTOW;
		current_gps_data.latitude_degrees_1000000 = pkt->data.navposllh.lat;
		current_gps_data.longitude_degrees_1000000 = pkt->data.navposllh.lon;
		current_gps_data.altitude_mm = pkt->data.navposllh.hMSL;
		current_gps_data.updated = true;
	} else if (pkt->header.messageClass == 0x01 && pkt->header.messageId == 0x06) {
		current_gps_data.time_of_week_millis = pkt->data.navsol.iTOW;
		current_gps_data.week = pkt->data.navsol.week;
		current_gps_data.fix = pkt->data.navsol.gpsFix;
		current_gps_data.satellites_visible = pkt->data.navsol.numSV;
		current_gps_data.updated = true;
	} else if (pkt->header.messageClass == 0x01 && pkt->header.messageId == 0x20) {
		current_gps_data.time_of_week_millis = pkt->data.navtimegps.iTOW;
		current_gps_data.week = pkt->data.navtimegps.week;
		current_gps_data.updated = true;
	} else if (pkt->header.messageClass == 0x01 && pkt->header.messageId == 0x21) {
		current_gps_data.year = pkt->data.navtimeutc.year;
		current_gps_data.month = pkt->data.navtimeutc.month;
		current_gps_data.day = pkt->data.navtimeutc.day;
		current_gps_data.hours = pkt->data.navtimeutc.hour;
		current_gps_data.minutes = pkt->data.navtimeutc.min;
		current_gps_data.seconds = pkt->data.navtimeutc.sec;
		current_gps_data.updated = true;
	} else if (pkt->header.messageClass == 0x01 && pkt->header.messageId == 0x22) {
		current_gps_data.lastQerr = (uint32_t)current_gps_data.clkB;
		current_gps_data.iTOW = pkt->data.navclock.iTOW;
		current_gps_data.clkB = pkt->data.navclock.clkB;
		current_gps_data.clkD = pkt->data.navclock.clkD;
		current_gps_data.tAcc = pkt->data.navclock.tAcc;
		current_gps_data.fAcc = pkt->data.navclock.fAcc;
		current_gps_data.updated = true;
		current_gps_data.newBias = true;
		printf("[NC] iT=%u cB=%d\n", (unsigned)current_gps_data.iTOW, (int)current_gps_data.clkB);
		stdio_flush();
	} else if (pkt->header.messageClass == 0x0d && pkt->header.messageId == 0x01) {
		if (pkt->header.payloadSize < 16) {
			return;
		}
		const uint8_t* tp = (const uint8_t*)&pkt->data;
		const uint32_t towMS = ubx_le_u32(tp + 0);
		const uint32_t towSubMS = ubx_le_u32(tp + 4);
		const int32_t qErr = ubx_le_i32(tp + 8);
		const uint16_t week = ubx_le_u16(tp + 12);
		const uint8_t flags = tp[14];

		current_gps_data.qAge = (towMS - current_gps_data.towMS) / 1000.0f;
		if (current_gps_data.qAge > 20.0f) {
			current_gps_data.qAge = 20.0f;
		}
		current_gps_data.towMS = towMS;
		current_gps_data.towSubMS = towSubMS;
		current_gps_data.qErr = qErr;
		current_gps_data.week_nr = week;
		current_gps_data.flags = flags;
		current_gps_data.updated = true;
		current_gps_data.newQerr = true;
		printf("[TP] tMS=%u q=%d\n", (unsigned)current_gps_data.towMS, (int)current_gps_data.qErr);
		stdio_flush();
	} else if (pkt->header.messageClass == 0x05 && pkt->header.messageId == 0x01) {
		ack_received = true;
	} else if (pkt->header.messageClass == 0x05 && pkt->header.messageId == 0x00) {
		nack_received = true;
	}
}

static ubx_rx_t ubx_rx_state;

void ubx_handle_byte(uint8_t data) {
	if (!ubx_rx_feed_byte(&ubx_rx_state, data)) {
		return;
	}
	uBloxPacket* incoming_packet = (uBloxPacket*)ubx_rx_state.buf;
	if (g_ubx_uart_hex_dump) {
		uint16_t psz = incoming_packet->header.payloadSize;
		if (psz <= UBX_MAX_PAYLOAD_BYTES) {
			size_t wire_len = (size_t)sizeof(uBloxHeader) + (size_t)psz + sizeof(uBloxChecksum);
			if (wire_len <= (size_t)UBX_RX_BUFFER_BYTES) {
				ubx_emit_hex_uart(ubx_rx_state.buf, wire_len);
			}
		}
	}
	ubx_handle_packet(incoming_packet);
}

static void ubx_clear_ack(void) {
	ack_received = false;
	nack_received = false;
}

typedef enum {
	UBX_WAIT_ACK = 0,
	UBX_WAIT_NACK = 1,
	UBX_WAIT_TIMEOUT = 2,
} ubx_ack_wait_t;

static ubx_ack_wait_t ubx_wait_for_ack_result(void) {
	uint16_t timeout = 1500;
	while (!ack_received && !nack_received && timeout-- > 0) {
#ifdef UARTFIFO
		ubx_poll();
#endif
		sleep_ms(1);
	}
	if (ack_received) return UBX_WAIT_ACK;
	if (nack_received) return UBX_WAIT_NACK;
	return UBX_WAIT_TIMEOUT;
}

void ubx_send_command(uint8_t msgClass, uint8_t msgId, uint8_t* payload, uint16_t payloadSize) {
	ubx_clear_ack();
	uBloxChecksum chksum = ubx_calculate_checksum(msgClass, msgId, payload, payloadSize);
	uart_send(0xB5);
	uart_send(0x62);
	uart_send(msgClass);
	uart_send(msgId);
	uart_send((uint8_t)(payloadSize & 0xFFU));
	uart_send((uint8_t)(payloadSize >> 8U));
	for (uint16_t i = 0; i < payloadSize; ++i) {
		uart_send(payload[i]);
	}
	uart_send(chksum.ck_a);
	uart_send(chksum.ck_b);
}

void ubx_send_packet(uBloxPacket* packet) {
	ubx_send_command(packet->header.messageClass, packet->header.messageId, (uint8_t*)&packet->data, packet->header.payloadSize);
}

bool ubx_send_packet_and_wait_for_ack(uBloxPacket* packet, const char* step_name) {
	const int max_attempts = 5;
	for (int attempt = 1; attempt <= max_attempts; attempt++) {
		ubx_send_packet(packet);
		ubx_ack_wait_t w = ubx_wait_for_ack_result();
		if (w == UBX_WAIT_ACK) {
			UBX_LOG("[UBX] %s: ACK (class 0x%02x id 0x%02x, attempt %d)\n", step_name, packet->header.messageClass, packet->header.messageId, attempt);
			return true;
		}
		if (w == UBX_WAIT_NACK) {
			UBX_LOG("[UBX] %s: NACK — receiver rejected (class 0x%02x id 0x%02x, attempt %d)\n", step_name, packet->header.messageClass, packet->header.messageId, attempt);
		} else {
			UBX_LOG("[UBX] %s: no ACK (~1.5s timeout) (class 0x%02x id 0x%02x, attempt %d)\n", step_name, packet->header.messageClass, packet->header.messageId, attempt);
		}
		if (attempt < max_attempts) {
			sleep_ms(120);
		}
	}
	UBX_LOG("[UBX] %s: FAILED after %d attempts\n", step_name, max_attempts);
	return false;
}

void ubx_poll(void) {
	while (uart_is_readable(uart0)) {
		ubx_handle_byte(uart_getc(uart0));
	}
}

void uart_init_ubx(uint32_t rate) {
	int uart_irq = UART0_IRQ;
	uart_init(uart0, rate);
	uart_set_format(uart0, DATA_BITS, STOP_BITS, PARITY);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	uart_set_hw_flow(uart0, false, false);
#ifndef UARTFIFO
	uart_set_fifo_enabled(uart0, true);
	irq_set_exclusive_handler(uart_irq, ubx_poll);
	irq_set_enabled(uart_irq, true);
	uart_set_irq_enables(uart0, true, false);
#endif
}

bool ubx_init(void) {
	bool success;
	UBX_LOG("[UBX] === bring-up start (expect reset/reconfig each boot) ===\n");
	UBX_LOG("[UBX] UART 38400 (probe / first CFG-PRT)\n");
	uart_init_ubx(38400);
	sleep_ms(100);
	UBX_LOG("[UBX] CFG-RST cold start + controlled SW reset (no ACK expected)\n");
	ubx_send_packet(&msgcfgrst);
	sleep_ms(1000);
	UBX_LOG("[UBX] UART 9600 (module default after reset)\n");
	uart_init_ubx(9600);
	sleep_ms(100);
	UBX_LOG("[UBX] CFG-RST again after baud change\n");
	ubx_send_packet(&msgcfgrst);
	sleep_ms(1000);
	UBX_LOG("[UBX] CFG-PRT: set 38400 8N1 UBX in/out (TX only, no ACK wait)\n");
	ubx_send_packet(&msgcfgprt);
	sleep_ms(100);
	UBX_LOG("[UBX] UART 38400 (match module)\n");
	uart_init_ubx(38400);
	UBX_LOG("[UBX] waiting 10s for receiver (1s heartbeat; USB CDC may buffer without flush)…\n");
	for (int s = 0; s < 10; s++) {
		sleep_ms(1000);
		UBX_LOG("[UBX] post-reset settle %d/10 s\n", s + 1);
	}
	UBX_LOG("[UBX] CFG-RXM (lpMode=%u max performance)\n", (unsigned)msgcfgrxm.data.cfgrxm.lpMode);
	success = ubx_send_packet_and_wait_for_ack(&msgcfgrxm, "CFG-RXM power");
	if (!success) return false;
	msgcfgmsg.data.cfgmsg.msgClass = 0x01;
	msgcfgmsg.data.cfgmsg.msgID = 0x02;
	msgcfgmsg.data.cfgmsg.rate = 1;
	success = ubx_send_packet_and_wait_for_ack(&msgcfgmsg, "CFG-MSG rate NAV-POSLLH 1 Hz");
	if (!success) return false;
	UBX_LOG("[UBX] CFG-RATE meas=%u ms nav=%u timeRef=%u\n", msgcfgrate.data.cfgrate.measRate, msgcfgrate.data.cfgrate.navRate, msgcfgrate.data.cfgrate.timeRef);
	success = ubx_send_packet_and_wait_for_ack(&msgcfgrate, "CFG-RATE meas/nav");
	if (!success) return false;
	msgcfgmsg.data.cfgmsg.msgID = 0x6;
	msgcfgmsg.data.cfgmsg.rate = 1;
	success = ubx_send_packet_and_wait_for_ack(&msgcfgmsg, "CFG-MSG rate NAV-SOL 1 Hz");
	if (!success) return false;
	msgcfgmsg.data.cfgmsg.msgClass = 0x0d;
	msgcfgmsg.data.cfgmsg.msgID = 0x01;
	msgcfgmsg.data.cfgmsg.rate = 1;
	success = ubx_send_packet_and_wait_for_ack(&msgcfgmsg, "CFG-MSG rate TIM-TP 1 Hz");
	if (!success) return false;
	msgcfgmsg.data.cfgmsg.msgClass = 0x01;
	msgcfgmsg.data.cfgmsg.msgID = 0x22;
	msgcfgmsg.data.cfgmsg.rate = 1;
	success = ubx_send_packet_and_wait_for_ack(&msgcfgmsg, "CFG-MSG rate NAV-CLOCK 1 Hz");
	if (!success) return false;
	msgcfgmsg.data.cfgmsg.msgID = 0x21;
	msgcfgmsg.data.cfgmsg.rate = 1;
	success = ubx_send_packet_and_wait_for_ack(&msgcfgmsg, "CFG-MSG rate NAV-TIMEUTC 1 Hz");
	if (!success) return false;
	msgcfgmsg.data.cfgmsg.msgID = 0x12;
	msgcfgmsg.data.cfgmsg.rate = 1;
	success = ubx_send_packet_and_wait_for_ack(&msgcfgmsg, "CFG-MSG rate NAV-VELNED 1 Hz");
	if (!success) return false;
	UBX_LOG("[UBX] CFG-NAV5 dynModel=%u fixMode=%u mask=0x%04x\n", msgcfgnav5.data.cfgnav5.dynModel, msgcfgnav5.data.cfgnav5.fixMode, msgcfgnav5.data.cfgnav5.mask);
	success = ubx_send_packet_and_wait_for_ack(&msgcfgnav5, "CFG-NAV5 engine");
	if (!success) return false;
	sleep_ms(100);
	UBX_LOG("[UBX] === bring-up done; streaming NAV/TIM for DAC loop ===\n");
	return true;
}
