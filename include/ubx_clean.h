#pragma once

#include <stdbool.h>
#include <stdint.h>

extern bool g_ubx_uart_hex_dump;

typedef struct _gps_data {
	bool updated;
	uint32_t towMS;
	float qAge;
	uint32_t towSubMS;
	int32_t qErr;
	uint16_t week_nr;
	uint8_t flags;
	uint32_t iTOW;
	uint32_t lastChangedBiasTOW;
	uint32_t lastQerr;
	int32_t clkB;
	int32_t clkD;
	float clkD_bias;
	uint32_t tAcc;
	uint32_t fAcc;
	uint32_t time_of_week_millis;
	int16_t week;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	int32_t latitude_degrees_1000000;
	int32_t longitude_degrees_1000000;
	int32_t altitude_mm;
	uint32_t ground_speed_cm_per_second;
	int32_t heading_degrees_100000;
	int32_t climb_cm_per_second;
	uint8_t satellites_visible;
	uint8_t fix;
	uint16_t ok_packets;
	uint16_t bad_packets;
	uint32_t ticks;
	uint32_t dac;
	bool adjustDac;
	bool newBias;
	bool newQerr;
} gps_data;

typedef struct __attribute__((packed)) {
	uint8_t sc1;
	uint8_t sc2;
	uint8_t messageClass;
	uint8_t messageId;
	uint16_t payloadSize;
} uBloxHeader;

typedef struct { uint8_t ck_a, ck_b; } uBloxChecksum;
typedef struct { uint32_t iTOW; uint16_t year; uint8_t month, day, hour, min, sec, valid; uint32_t tAcc; int32_t nano; uint8_t fixType, flags, flags2, numSV; int32_t lon, lat, height, hMSL; uint32_t hAcc, vAcc; int32_t velN, velE, velD, gSpeed, headMot; uint32_t sAcc, headAcc; uint16_t pDOP; uint8_t reserved1[6]; int32_t headVeh; uint8_t reserved2[4]; } uBloxNAVPVTPayload;
typedef struct { uint32_t iTOW; int32_t lon, lat, height, hMSL; uint32_t hAcc, vAcc; } uBloxNAVPOSLLHPayload;
typedef struct { uint32_t towMS, towSubMS; int32_t qErr; uint16_t week; uint8_t flags, reserved1; } uBloxTIMTPPayload;
typedef struct { uint32_t iTOW; int32_t clkB, clkD; uint32_t tAcc, fAcc; } uBloxNAVCLOCKPayload;
typedef struct { uint32_t iTOW; int32_t fTOW; int16_t week; uint8_t gpsFix, flags; int32_t ecefX, ecefY, ecefZ; uint32_t pAcc; int32_t ecefVX, ecefVY, ecefVZ; uint32_t sAcc; uint16_t pDOP; uint8_t reserved1, numSV; uint32_t reserved2; } uBloxNAVSOLPayload;
typedef struct { uint32_t iTOW, fTOW; int16_t week; int8_t leapS; uint8_t valid; uint32_t tAcc; } uBloxNAVTIMEGPSPayload;
typedef struct { uint32_t iTOW, tAcc; int32_t nano; uint16_t year; uint8_t month, day, hour, min, sec, valid; } uBloxNAVTIMEUTCPayload;
typedef struct { uint32_t iTOW; int32_t velN, velE, velD; uint32_t speed, gSpeed; int32_t headMot; uint32_t sAcc, headAcc; } uBloxNAVVELNEDPayload;
typedef struct { uint8_t portID, reserved1; uint16_t txReady; uint32_t mode, baudRate; uint16_t inProtoMask, outProtoMask, flags; uint8_t reserved2[2]; } uBloxCFGPRTPayload;
typedef struct { uint8_t clsID, msgID, ck_a, ck_b; } uBloxACKACKayload;
typedef struct { uint8_t msgClass, msgID; } uBloxCFGMSGPOLLPayload;
typedef struct { uint8_t msgClass, msgID, rate; } uBloxCFGMSGPayload;
typedef struct { uint32_t interval, length; int8_t status; uint8_t timeRef, flags, reserved1; uint16_t antennaCableDelay, rfGroupDelay; uint32_t userDelay; } uBloxCFGTPPayload;
typedef struct { uint16_t navBbrMask; uint8_t resetMode, reserved1; } uBloxCFGRSTPayload;
typedef struct { uint16_t mask; uint8_t dynModel, fixMode; int32_t fixedAlt; uint32_t fixedAltVar; int8_t minElev; uint8_t drLimit; uint16_t pDop, tDop, pAcc, tAcc; uint8_t staticHoldThresh, dgpsTimeOut; uint32_t reserved2, reserved3, reserved4; } uBloxCFGNAV5Payload;
typedef struct { uint8_t reserved1, lpMode; } uBloxCFGRXMPayload;
typedef struct { uint16_t measRate, navRate, timeRef; } uBloxCFGRATEPayload;

typedef union {
	uBloxTIMTPPayload timtp;
	uBloxNAVCLOCKPayload navclock;
	uBloxNAVPVTPayload navpvt;
	uBloxCFGPRTPayload cfgprt;
	uBloxCFGMSGPOLLPayload cfgmsgpoll;
	uBloxCFGMSGPayload cfgmsg;
	uBloxCFGNAV5Payload cfgnav5;
	uBloxCFGTPPayload cfgtp;
	uBloxNAVPOSLLHPayload navposllh;
	uBloxNAVSOLPayload navsol;
	uBloxNAVTIMEGPSPayload navtimegps;
	uBloxNAVTIMEUTCPayload navtimeutc;
	uBloxNAVVELNEDPayload navvelned;
	uBloxACKACKayload ackack;
	uBloxCFGRSTPayload cfgrst;
	uBloxCFGRXMPayload cfgrxm;
	uBloxCFGRATEPayload cfgrate;
} ubloxPacketData;

typedef struct __attribute__((packed)) {
	uBloxHeader header;
	ubloxPacketData data;
} uBloxPacket;

#define UBX_MAX_PAYLOAD_BYTES ((uint16_t)sizeof(ubloxPacketData))

extern gps_data current_gps_data;
extern volatile bool ack_received;
extern volatile bool nack_received;

void ubx_handle_byte(uint8_t data);
void ubx_poll(void);
void uart_init_ubx(uint32_t rate);
void ubx_send_command(uint8_t msgClass, uint8_t msgId, uint8_t* payload, uint16_t payloadSize);
void ubx_send_packet(uBloxPacket* packet);
bool ubx_send_packet_and_wait_for_ack(uBloxPacket* packet, const char* step_name);
bool ubx_init(void);
