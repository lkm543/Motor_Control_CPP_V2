#ifndef _AHRS_H_
#define _AHRS_H_

#include "stdafx.h"
#include "Serial_Port.h"
#include "Visual.h"
#include "Packet.h"
#include "data.h"


//TO DO  timeout handler
class AHRS{
public:
	static int number_bytes_perframe;
	static double time_per_frame;
	AHRS();
	void kick_off();
	void read_packet();
	void handle_packet(Packet *packet);
	void send_packet(Packet *packet);
	enum PName;
	enum StateName;
	struct PID_PARAM;
	struct MOTOR_MEMBER;
	struct MOTOR_PID_MEMBER;
	static const int Motor_Device_SIZE = 7;
	int getTypeIndex(byte type, int Command[]);
	Data *data;
	View *view;
	SerialPort *serial;

private:
	int Motor_Channel = 0;//Convert.to 
	/*
	MOTOR_MEMBER m_Motor_Member[Motor_Device_SIZE];
	MOTOR_PID_MEMBER m_Motor_PID_Member[Motor_Device_SIZE];
	INT16 m_Pos_SoftStart[Motor_Device_SIZE];
	INT16 m_Max_Vel_Cmd[Motor_Device_SIZE];
	INT16 m_Max_Tor_Cmd[Motor_Device_SIZE];
	INT16 m_Max_PWM_Cmd[Motor_Device_SIZE];
	INT32 m_Position_Target[Motor_Device_SIZE];
	INT16 m_Velocity_External[Motor_Device_SIZE];
	INT16 m_Torque_External[Motor_Device_SIZE];
	*/
};


#endif