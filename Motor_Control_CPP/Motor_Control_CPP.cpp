#include "stdafx.h"
#include <string> 
#include <iostream>
#include <thread>
#include "AHRS.h"

using namespace std;

void data_receive();
AHRS *ahrs;

int _tmain(int argc, _TCHAR* argv[])
{
	ahrs = new AHRS();
	//thread mThread(data_receive);
	ahrs->kick_off();
	/*
	packet.PacketType = 0x12;
	packet.Ch_Status = 0x00;
	packet.DataLength = 0x02;
	packet.Data[0] = 0x0f;
	packet.CRC8 = 0xf2;
	serial->Write_into_SerialPort(packet);
	*/
	
	while (true){
		ahrs->read_packet();
	}
	//cout << "¥ô·NÁäµ²§ô" << endl;
	system("pause");
	std::terminate();
	return 0;
}


void data_receive(){
	while (true){
		ahrs->read_packet();
		//serial->Write_into_SerialPort(packet);
		//serial->Read_Data();
	}
};