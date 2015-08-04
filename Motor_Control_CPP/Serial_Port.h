#include "stdafx.h"
#include <string> 
#include <iostream>
#include <windows.h>
#include <cstdint>

#ifndef HEADER_Packet
#define HEADER_Packet

#include "Packet.h"
class Packet;

class SerialPort{

public:
	std::string serial_port_name;
	DWORD BaudRate;
	COMMTIMEOUTS timeouts;
	DCB dcbSerialParams;
	HANDLE hSerial;
	void Open_SerialPort();
	void SetParam_SerialPort();
	void SetTimeout_SerialPort();
	void Close_SerialPort();
	bool Write_into_SerialPort(Packet AHRSPacket);
};


void SerialPort::Open_SerialPort(){
	hSerial = CreateFile(std::wstring(serial_port_name.begin(), serial_port_name.end()).c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);
	if (hSerial == INVALID_HANDLE_VALUE){
		if (GetLastError() == ERROR_FILE_NOT_FOUND){
			std::cout << "Serial Port Not Exist!" << std::endl;
			//serial port does not exist. Inform user.
		}
		else{
			std::cout << "Serial Port Failed!" << std::endl;
			//some other error occurred. Inform user.
		}
	}
};

void SerialPort::SetParam_SerialPort(){
	dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams)) {
		//error getting state
	}
	dcbSerialParams.BaudRate = BaudRate;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (!SetCommState(hSerial, &dcbSerialParams)){
		//error setting serial port state
	}
};

void SerialPort::SetTimeout_SerialPort(){
	timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	if (!SetCommTimeouts(hSerial, &timeouts)){
		//error occureed. Inform user
	}
};

void SerialPort::Close_SerialPort(){
	CloseHandle(hSerial);
}

bool SerialPort::Write_into_SerialPort(Packet AHRSPacket){
	LPDWORD byteswritten = (LPDWORD) 6 + (LPDWORD)AHRSPacket.DataLength;
	int length = 0;
	uint8_t* packet = new uint8_t[6 + AHRSPacket.DataLength];
	packet[0] = (uint8_t)'E';
	packet[1] = (uint8_t)'C';
	packet[2] = (uint8_t)'S';
	packet[3] = AHRSPacket.PacketType;
	packet[4] = AHRSPacket.Ch_Status; // channle/status
	packet[5] = AHRSPacket.DataLength;
	// Fill data section
	int i = 0;
	for (i = 0; i < (AHRSPacket.DataLength - 1); i++)
	{
		packet[6 + i] = AHRSPacket.Data[i];
	}
	packet[6 + i] = AHRSPacket.CRC8;

	return WriteFile(hSerial, packet, (DWORD)byteswritten, (LPDWORD)0, NULL);
	//return true;
}

#endif