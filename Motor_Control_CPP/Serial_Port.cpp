#pragma once

#include "stdafx.h"
#include "Serial_Port.h"


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
			std::cout << "Open Serial Port Failed!" << std::endl;
		}
	}
};

void SerialPort::SetParam_SerialPort(){
	dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	if (!GetCommState(hSerial, &dcbSerialParams)) {
		std::cout << "error getting state" << std::endl;
	}
	dcbSerialParams.BaudRate = BaudRate;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (!SetCommState(hSerial, &dcbSerialParams)){
		std::cout << "error setting state" << std::endl;
	}
};

void SerialPort::SetTimeout_SerialPort(){
	timeouts = { 0 };
	/*
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
	*/
	if (!SetCommTimeouts(hSerial, &timeouts)){
		std::cout << "Set params of Timeouts Failed" << std::endl;
		//error occureed. Inform user
	}
};

void SerialPort::Close_SerialPort(){
	CloseHandle(hSerial);
}

bool SerialPort::Write_into_SerialPort(Packet AHRSPacket){
	DWORD byteswritten = 6 + AHRSPacket.DataLength;
	DWORD NumberOfBytesWritten;
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
	if (AHRSPacket.DataLength > 1){
		for (i = 0; i < (AHRSPacket.DataLength - 1); i++)
		{
			packet[6 + i] = AHRSPacket.Data[i];
		}
	}
	packet[6 + i] = AHRSPacket.CRC8;

	return WriteFile(hSerial, packet, byteswritten, &NumberOfBytesWritten, NULL);
}

void SerialPort::flush(){
	DWORD   dwErrors;
	COMSTAT *comStat = new COMSTAT();
	ClearCommError(hSerial, &dwErrors, comStat);
	DWORD Read_Ptr_temp;
	while (comStat->cbInQue!=0)
	{
		ClearCommError(hSerial, &dwErrors, comStat);
		if (comStat->cbInQue == 0)
			break;
		ReadFile(hSerial, szBuff, comStat->cbInQue, &Read_Ptr_temp, NULL);
	};
};