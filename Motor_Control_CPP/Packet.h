#ifndef Packet_H
#define Packet_H

#include <cstdint>

class Packet
{
	public:
		uint8_t PacketType;
		uint8_t Ch_Status;
		uint8_t DataLength;
		uint8_t* Data = new uint8_t[6 + DataLength];
		uint8_t CRC8;
};


#endif