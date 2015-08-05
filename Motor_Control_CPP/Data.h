#include <cstdint>

#include <windows.h>
static class Data
{
public:
	int* Controller_Status[7][22];
	int* PID_Status[7][48];
};

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