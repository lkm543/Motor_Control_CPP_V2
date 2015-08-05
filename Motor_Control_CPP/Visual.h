#include <iostream>


#ifndef HEADER_Data
#define HEADER_Data
#include "Data.h"
using namespace std;
class Data;

class View{
	public:
		void refresh(Data *data);
};
void View::refresh(Data *data){
	system("cls");
	cout << "     Motor\t\t1\t2\t3\t4\t5\t6\t7"<< endl;
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
	cout << "Motor Position\t\t" << (int)data->Controller_Status[0][1]
		<< "\t" << (int)data->Controller_Status[1][1]
		<< "\t" << (int)data->Controller_Status[2][1]
		<< "\t" << (int)data->Controller_Status[3][1]
		<< "\t" << (int)data->Controller_Status[4][1]
		<< "\t" << (int)data->Controller_Status[5][1]
		<< "\t" << (int)data->Controller_Status[6][1]
		<< endl;
	cout << "Velocity External\t" << (int)data->Controller_Status[0][1]
		<< "\t" << (int)data->Controller_Status[1][1]
		<< "\t" << (int)data->Controller_Status[2][1]
		<< "\t" << (int)data->Controller_Status[3][1]
		<< "\t" << (int)data->Controller_Status[4][1]
		<< "\t" << (int)data->Controller_Status[5][1]
		<< "\t" << (int)data->Controller_Status[6][1]
		<< endl;
	cout << "Velocity Internal\t" << (int)data->Controller_Status[0][1]
		<< "\t" << (int)data->Controller_Status[1][1]
		<< "\t" << (int)data->Controller_Status[2][1]
		<< "\t" << (int)data->Controller_Status[3][1]
		<< "\t" << (int)data->Controller_Status[4][1]
		<< "\t" << (int)data->Controller_Status[5][1]
		<< "\t" << (int)data->Controller_Status[6][1]
		<< endl;
	cout << "Motor Velocity\t\t" << (int)data->Controller_Status[0][1]
		<< "\t" << (int)data->Controller_Status[1][1]
		<< "\t" << (int)data->Controller_Status[2][1]
		<< "\t" << (int)data->Controller_Status[3][1]
		<< "\t" << (int)data->Controller_Status[4][1]
		<< "\t" << (int)data->Controller_Status[5][1]
		<< "\t" << (int)data->Controller_Status[6][1]
		<< endl;
	cout << "Torque External\t\t" << (int)data->Controller_Status[0][1]
		<< "\t" << (int)data->Controller_Status[1][1]
		<< "\t" << (int)data->Controller_Status[2][1]
		<< "\t" << (int)data->Controller_Status[3][1]
		<< "\t" << (int)data->Controller_Status[4][1]
		<< "\t" << (int)data->Controller_Status[5][1]
		<< "\t" << (int)data->Controller_Status[6][1]
		<< endl;
	cout << "Torque Internal\t\t" << (int)data->Controller_Status[0][1]
		<< "\t" << (int)data->Controller_Status[1][1]
		<< "\t" << (int)data->Controller_Status[2][1]
		<< "\t" << (int)data->Controller_Status[3][1]
		<< "\t" << (int)data->Controller_Status[4][1]
		<< "\t" << (int)data->Controller_Status[5][1]
		<< "\t" << (int)data->Controller_Status[6][1]
		<< endl;
	cout << "Mortor Torque\t\t" << (int)data->Controller_Status[0][1]
		<< "\t" << (int)data->Controller_Status[1][1]
		<< "\t" << (int)data->Controller_Status[2][1]
		<< "\t" << (int)data->Controller_Status[3][1]
		<< "\t" << (int)data->Controller_Status[4][1]
		<< "\t" << (int)data->Controller_Status[5][1]
		<< "\t" << (int)data->Controller_Status[6][1]
		<< endl;
	cout << "Duty Cycle\t\t" << (int)data->Controller_Status[0][1]
		<< "\t" << (int)data->Controller_Status[1][1]
		<< "\t" << (int)data->Controller_Status[2][1]
		<< "\t" << (int)data->Controller_Status[3][1]
		<< "\t" << (int)data->Controller_Status[4][1]
		<< "\t" << (int)data->Controller_Status[5][1]
		<< "\t" << (int)data->Controller_Status[6][1]
		<< endl;
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

#endif