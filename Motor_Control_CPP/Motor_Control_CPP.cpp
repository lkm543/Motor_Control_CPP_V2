#include "stdafx.h"
#include "Serial_Port.h"
#include "Visual.h"
#include <string> 
#include <iostream>
#include "Packet.h"

using namespace std;
int _tmain(int argc, _TCHAR* argv[])
{
	SerialPort *serial = new SerialPort();
	serial->serial_port_name = "COM4";
	serial->BaudRate = 4900000;
	serial->Open_SerialPort();
	Packet packet;
	packet.PacketType = 0x12;
	packet.Ch_Status = 0x00;
	packet.DataLength = 0x02;
	packet.Data[0] = 0x0f;
	packet.CRC8 = 0xf2;
	serial->Write_into_SerialPort(packet);

	system("pause");
	return 0;
}

