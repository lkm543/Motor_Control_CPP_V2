#include "stdafx.h"
#include "Serial_Port.h"
#include "Visual.h"
#include <string> 
#include <iostream>
#include <thread>
#include "Packet.h"

using namespace std;
void data_receive();
SerialPort *serial = new SerialPort();
//View *view = new View();
Packet packet;
Data *data = new Data();

int _tmain(int argc, _TCHAR* argv[])
{
	serial->serial_port_name = "\\\\.\\COM4";
	serial->BaudRate = 4900000;
	serial->Open_SerialPort();

	packet.PacketType = 0x12;
	packet.Ch_Status = 0x00;
	packet.DataLength = 0x02;
	packet.Data[0] = 0x0f;
	packet.CRC8 = 0xf2;
	serial->Write_into_SerialPort(packet);
	thread mThread(data_receive);
	while (true){
		//view->refresh(data);
	}
	
	
	std::terminate();
	cout << "¥ô·NÁäµ²§ô" << endl;
	system("pause");
	return 0;
}

void data_receive(){
	while (true){
		//serial->Write_into_SerialPort(packet);
		serial->Read_Data();
	}
}