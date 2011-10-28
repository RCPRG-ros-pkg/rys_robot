/*
 * hbridge.cc
 *
 *  Created on: Jun 9, 2011
 *      Author: dseredyn
 */

#include <math.h>
#include "common.h"

bool Crc::initialised = false;
uint16_t Crc::CrcTable[256];

uint16_t Crc::crc16(uint8_t buffer, uint16_t polynom, uint16_t initial)
{
        uint16_t i,j;
        uint16_t result = initial;

        result = result ^ ((uint16_t)buffer << 8);
        for (j=0; j<7; ++j)
                if ((result & 0x8000)!=0)
                        result = (result <<1) ^ polynom;
                else
                        result = result << 1;

        return result;
}

void Crc::GenerateTableCrc16(uint16_t poly)
{
        for (int i=0;i<256;++i)
        {
                CrcTable[i] = crc16(i,poly,0);
        }
}

uint16_t Crc::Crc16(const MSG &msg) {
    return Crc16Byte((uint8_t*)&msg,sizeof(MSG),0);
}

uint16_t Crc::Crc16Byte(uint8_t* buffer,uint16_t length, uint16_t initial)
{
        int i;
        uint16_t result = initial;

        if (!initialised) {
            GenerateTableCrc16(0x8005);
            initialised = true;
        }

        for (i=0; i<length; ++i)
                result = (result<<8) ^ CrcTable[(buffer[i] ^ (result>>8)) & 0xFF];

        return ((result&0xFF)<<8) | ((result)>>8);
}
