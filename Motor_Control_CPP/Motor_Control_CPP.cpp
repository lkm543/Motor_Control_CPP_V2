#include "stdafx.h"
#include <string> 
#include <iostream>
#include <thread>
#include "AHRS.h"
#include "conio.h"

using namespace std;

void data_receive();
void refresh();
void tracking_motor(int tracked, int tracker);
AHRS *ahrs;

int _tmain(int argc, _TCHAR* argv[])
{
	ahrs = new AHRS();
	ahrs->kick_off();
	int tracker, tracked;

	cout << "Please enter the number of motor being tracked [1-7]" << endl;
	cin >> tracked;
	cout << "Please enter the number of motor to tracker [1-7]" << endl;
	cin >> tracker;

	thread receiveThread(data_receive);
	thread refreshThread(refresh);
	thread trackingThread(tracking_motor,tracked,tracker);

	while (true){
		if (_kbhit()) {
			//cout << "keyboard"<<_getch() << endl;
			if (_getch() == 'q' || getchar() == 'Q')
				break;
			if (_getch() == 'r' || getchar() == 'R')
			{
				cout << "Reset PID Param" << endl;
				ahrs->set_PID_param();
			}
		}
	}

	//system("pause");
	std::terminate();
	return 0;

}


void data_receive(){
	while (true){
		ahrs->read_packet();
	}

};
void refresh(){
	while (true){
		ahrs->view->refresh(ahrs->data);
		Sleep(200);
	}
};
void tracking_motor(int tracked,int tracker){
	while (true){
		ahrs->tracking_motor(tracked,tracker);
		Sleep(50);
	}
};
