#include "stdafx.h"
#include <string> 
#include <iostream>
#include <windows.h>
#include <cstdint>
#ifndef HEADER_Packet
#define HEADER_Packet

#include "Packet.h"
#include "AHRS.h"
class Packet;

class SerialPort{

public:
	static const int Size_Buffer = 30000;
	char szBuff[Size_Buffer];//Size_Buffer
	int MAX_PACKET_SIZE = 100;
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
	bool Read_Data();
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
	for (i = 0; i < (AHRSPacket.DataLength - 1); i++)
	{
		packet[6 + i] = AHRSPacket.Data[i];
	}
	packet[6 + i] = AHRSPacket.CRC8;

	return WriteFile(hSerial, packet, byteswritten, &NumberOfBytesWritten , NULL);
}

bool SerialPort::Read_Data(){

	int continue_parsing = 1;
	//int bytes_to_read;
	bool found_packet;
	int packet_start_index;
	int packet_index;
	DWORD   dwErrors;
	COMSTAT comStat;
	AHRS *AHRS_Class = new AHRS();

	//取得Receive但尚未Read的數目(comStat.cbInQue)
	ClearCommError(hSerial, &dwErrors, &comStat);
	//std::cout << dwErrors << ":::" << comStat.cbInQue << std::endl;
	
	
	if (ReadFile(hSerial, szBuff, comStat.cbInQue, &Read_Ptr, NULL)){
		//std::cout << szBuff << std::endl;
		//std::cout << dwErrors << ":::" << comStat.cbInQue << std::endl;


		while (continue_parsing == 1)//RXbufPtr >= 8 &&
		{
			// Search for the packet start sequence
			found_packet = false;
			packet_start_index = 0;
			for (packet_index = 0; packet_index < comStat.cbInQue; packet_index++)
			{
				if (szBuff[packet_index] == 'E' && szBuff[packet_index + 1] == 'C' && szBuff[packet_index + 2] == 'S')
				{
					found_packet = true;
					packet_start_index = packet_index;

					break;
				}
			}

			// If start sequence found, try to recover all the data in the packet
			if (found_packet && ((comStat.cbInQue - packet_start_index) >= 8))
			{
				int i;
				Packet *DataPacket = new Packet();
				DataPacket->PacketType = szBuff[packet_start_index + 3];
				DataPacket->Ch_Status = szBuff[packet_start_index + 4];
				DataPacket->DataLength = szBuff[packet_start_index + 5];
				DataPacket->Data = new byte[DataPacket->DataLength-1];

				// Only process packet if data_size is not too large.
				if (DataPacket->DataLength <= MAX_PACKET_SIZE)
				{

					// If a full packet has been received, then the full packet size should be
					// 3 + 1 + 1 + 1 + [data_size] + 1
					// that is, 3 bytes for the start sequence, 1 byte for type, 1 byte for status, 1 byte for data length, 
					// data_size bytes for packet data inculde 1 bytes for the CRC-8.
					// If enough data has been received, go ahead and recover the packet.  If not, wait until the
					// rest of the data arrives
					int buffer_length = (comStat.cbInQue - packet_start_index);
					int packet_length = (6 + DataPacket->DataLength);
					if (buffer_length >= packet_length)
					{
						if (DataPacket->DataLength == 0)
						{
							// this packet length is wrong!!!
						}
						else
						{
							// A full packet has been received.  Retrieve the data.
							for (i = 0; i < (DataPacket->DataLength - 1); i++)
							{
								DataPacket->Data[i] = szBuff[packet_start_index + 6 + i];
							}
							DataPacket->CRC8 = szBuff[packet_start_index + 6 + i];
							AHRS_Class->receive_packet(DataPacket);
							
						}

						// Copy all received bytes that weren't part of this packet into the beginning of the
						// buffer.  Then, reset RXbufPtr.

						for (int index = 0; index < (buffer_length - packet_length); index++)
						{
							szBuff[index] = szBuff[(packet_start_index + packet_length) + index];
						}
						comStat.cbInQue = (buffer_length - packet_length);
					}
					else
					{
						continue_parsing = 0;
					}
				}
				else
				{
					// data_size was too large - the packet data is invalid.  Clear the RX buffer.
					comStat.cbInQue = 0;
					continue_parsing = 0;
					//PacketReceivedEvent(PName.CMD_OVER_DATA_LENGTH, -1, DataPacket.Ch_Status);
				}
			}
			else
			{
				continue_parsing = 0;
			}
		}


		//std::cout << sizeof(szBuff) << std::endl;
		//error occurred. Report to user.
		return true;
	}
	else{
		return false;
	}
}

#endif