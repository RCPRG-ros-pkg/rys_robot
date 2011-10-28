/*
 * common.hh
 *
 *  Created on: Jun 9, 2011
 *      Author: dseredyn
 */

#ifndef COMMON_HH_
#define COMMON_HH_

#include <inttypes.h>

#pragma pack(1)

typedef struct tMSG
{
	uint16_t header;
	uint8_t adr;
	uint8_t id;
	uint8_t d[12];
	uint16_t crc;
} MSG;

#define MSG_HEADER 0xAA12


class Crc {
public:
	static uint16_t Crc16Byte(uint8_t* buffer,uint16_t length, uint16_t initial);
	static uint16_t Crc16(const MSG &msg);
private:
	static uint16_t crc16(uint8_t buffer, uint16_t polynom, uint16_t initial);
	static void GenerateTableCrc16(uint16_t poly);

	static uint16_t CrcTable[256];
	static bool initialised;
};

#endif // COMMON_HH_
