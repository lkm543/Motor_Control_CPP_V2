#ifndef __Serial_PortPacket_H__
#define __Serial_PortPacket_H__

#include <string> 
#include <iostream>
#include <windows.h>
#include <cstdint>
#include "Packet.h"

class Packet;

class SerialPort{

public:
	static const int Size_Buffer = 10000;
	char szBuff[Size_Buffer];//Size_Buffer
	int MAX_PACKET_SIZE = 50;
	std::string serial_port_name;
	DWORD BaudRate;
	DWORD Read_Ptr=0;
	COMMTIMEOUTS timeouts;
	DCB dcbSerialParams;
	HANDLE hSerial;
	void Open_SerialPort();
	void SetParam_SerialPort();
	void SetTimeout_SerialPort();
	void Close_SerialPort();
	bool Write_into_SerialPort(Packet AHRSPacket);
	void flush();
};


#endif