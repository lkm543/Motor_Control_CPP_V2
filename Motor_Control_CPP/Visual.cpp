#pragma once

#include "stdafx.h"
#include "Visual.h"
#include "time.h"
#include "AHRS.h"

void View::refresh(Data *data){
	clock_t start, finish;
	start = clock();
	count = count + 1;
	int data_per_frames=100;
	if (count > data_per_frames){
		count = 1;
		system("cls");
		cout << "     Motor\t\t1\t2\t3\t4\t5\t6\t7" << endl;
		cout << "     Count\t\t" << (int)data->Controller_Status[0][0]
			<< "\t" << (int)data->Controller_Status[1][0]
			<< "\t" << (int)data->Controller_Status[2][0]
			<< "\t" << (int)data->Controller_Status[3][0]
			<< "\t" << (int)data->Controller_Status[4][0]
			<< "\t" << (int)data->Controller_Status[5][0]
			<< "\t" << (int)data->Controller_Status[6][0]
			<< endl;
		cout << "Position Target\t\t" << (int)data->Controller_Status[0][1]
			<< "\t" << (int)data->Controller_Status[1][1]
			<< "\t" << (int)data->Controller_Status[2][1]
			<< "\t" << (int)data->Controller_Status[3][1]
			<< "\t" << (int)data->Controller_Status[4][1]
			<< "\t" << (int)data->Controller_Status[5][1]
			<< "\t" << (int)data->Controller_Status[6][1]
			<< endl;
		cout << "Motor Position\t\t" << (int)data->Controller_Status[0][2]
			<< "\t" << (int)data->Controller_Status[1][2]
			<< "\t" << (int)data->Controller_Status[2][2]
			<< "\t" << (int)data->Controller_Status[3][2]
			<< "\t" << (int)data->Controller_Status[4][2]
			<< "\t" << (int)data->Controller_Status[5][2]
			<< "\t" << (int)data->Controller_Status[6][2]
			<< endl;
		
		cout << "Velocity External\t" << (int)data->Controller_Status[0][3]
			<< "\t" << (int)data->Controller_Status[1][3]
			<< "\t" << (int)data->Controller_Status[2][3]
			<< "\t" << (int)data->Controller_Status[3][3]
			<< "\t" << (int)data->Controller_Status[4][3]
			<< "\t" << (int)data->Controller_Status[5][3]
			<< "\t" << (int)data->Controller_Status[6][3]
			<< endl;
		cout << "Velocity Internal\t" << (int)data->Controller_Status[0][4]
			<< "\t" << (int)data->Controller_Status[1][4]
			<< "\t" << (int)data->Controller_Status[2][4]
			<< "\t" << (int)data->Controller_Status[3][4]
			<< "\t" << (int)data->Controller_Status[4][4]
			<< "\t" << (int)data->Controller_Status[5][4]
			<< "\t" << (int)data->Controller_Status[6][4]
			<< endl;
		cout << "Motor Velocity\t\t" << (int)data->Controller_Status[0][5]
			<< "\t" << (int)data->Controller_Status[1][5]
			<< "\t" << (int)data->Controller_Status[2][5]
			<< "\t" << (int)data->Controller_Status[3][5]
			<< "\t" << (int)data->Controller_Status[4][5]
			<< "\t" << (int)data->Controller_Status[5][5]
			<< "\t" << (int)data->Controller_Status[6][5]
			<< endl;
		cout << "Torque External\t\t" << (int)data->Controller_Status[0][6]
			<< "\t" << (int)data->Controller_Status[1][6]
			<< "\t" << (int)data->Controller_Status[2][6]
			<< "\t" << (int)data->Controller_Status[3][6]
			<< "\t" << (int)data->Controller_Status[4][6]
			<< "\t" << (int)data->Controller_Status[5][6]
			<< "\t" << (int)data->Controller_Status[6][6]
			<< endl;
		cout << "Torque Internal\t\t" << (int)data->Controller_Status[0][7]
			<< "\t" << (int)data->Controller_Status[1][7]
			<< "\t" << (int)data->Controller_Status[2][7]
			<< "\t" << (int)data->Controller_Status[3][7]
			<< "\t" << (int)data->Controller_Status[4][7]
			<< "\t" << (int)data->Controller_Status[5][7]
			<< "\t" << (int)data->Controller_Status[6][7]
			<< endl;
		cout << "Mortor Torque\t\t" << (int)data->Controller_Status[0][8]
			<< "\t" << (int)data->Controller_Status[1][8]
			<< "\t" << (int)data->Controller_Status[2][8]
			<< "\t" << (int)data->Controller_Status[3][8]
			<< "\t" << (int)data->Controller_Status[4][8]
			<< "\t" << (int)data->Controller_Status[5][8]
			<< "\t" << (int)data->Controller_Status[6][8]
			<< endl;
		cout << "Duty Cycle\t\t" << (int)data->Controller_Status[0][9]
			<< "\t" << (int)data->Controller_Status[1][9]
			<< "\t" << (int)data->Controller_Status[2][9]
			<< "\t" << (int)data->Controller_Status[3][9]
			<< "\t" << (int)data->Controller_Status[4][9]
			<< "\t" << (int)data->Controller_Status[5][9]
			<< "\t" << (int)data->Controller_Status[6][9]
			<< endl;
			
		finish = clock();
		double duration = (double)(finish - start) / CLOCKS_PER_SEC;
		cout << "Time consuming in cout: " << duration << "s" << endl;
		cout << "Number of data in buffer: " << AHRS::number_bytes_perframe << endl;
		cout << "Number of data in this frame: " << data_per_frames << endl;
		cout << "Time consuming in handling packets: " << AHRS::time_per_frame <<"s"<< endl;
		AHRS::number_bytes_perframe = 0;
		AHRS::time_per_frame = 0.0;
	}
}
/*
*********Controller_Status*********
Count	1
Position Target	4
Motor position	4
Velocity External	2
Velocity internal	2
Motor velocity	2
torque external	2
torque internal	2
Mortor torque	2
dutycycle	1
*/

/*
*********PID_Status********* (All of them are 4 bytes)
Kp for position
Ki for position
Kd for position
integrator saturation for position
Kp for velocity
Ki for velocity
Kd for velocity
integrator saturation for velocity
Kp for torque
Ki for torque
Kd for torque
integrator saturation for torque
*/
