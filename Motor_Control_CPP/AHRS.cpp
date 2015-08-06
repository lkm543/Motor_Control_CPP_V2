#pragma once

#include "stdafx.h"
#include "AHRS.h"
#include "CRC.h"

enum AHRS::PName
{
	WHO_AM_I = 0,
	FIRMWARE_VERSION,
	ID_STATUS,
	PWM_PARAMETER,
	CMD_CRC_FAILED,
	CMD_NO_SUPPORT,
	CMD_OVER_DATA_LENGTH,
	CMD_COMPLETE,
	REPORT_SENSOR_ADC,
	CONTROLLER_STATUS,

	RESET_CONTROLLER,
	KICK_OFF_CONTROLLER,
	REPORT_MCU_INFORMATION,
	HOME_CMD,
	PAUSE_REPORT_INFO,
	CONTINUE_REPORT_INFO,

	ALL_OF_PID_PARAM,
	POSITION_OF_PID_PARAM,
	VELOCITY_OF_PID_PARAM,
	TORQUE_OF_PID_PARAM,

	MAX_OF_POSITION_CMD,
	MAX_OF_VELOCITY_CMD,
	MAX_OF_TORQUE_CMD,
	MAX_OF_PWM_DUTYCYCLE,

	POSITION_TARGET_CMD,
	VELOCITY_EXT_CMD,
	TORQUE_EXT_CMD,
	DEBUG_MODE,
	NumberOfPnames
};

enum AHRS::StateName
{
	STATE_WHO_AM_I = 0,
	STATE_FIRMWARE_VERSION,
	STATE_ID_STATUS,
	STATE_PWM_PARAMETER,
	STATE_REPORT_SENSOR_ADC,
	STATE_CONTROLLER_STATUS,

	STATE_RESET_CONTROLLER,
	STATE_KICK_OFF_CONTROLLER,
	STATE_REPORT_MCU_INFORMATION,
	STATE_HOME_CMD,
	STATE_PAUSE_REPORT_INFO,
	STATE_CONTINUE_REPORT_INFO,

	STATE_ALL_OF_PID_PARAM,
	STATE_POSITION_OF_PID_PARAM,
	STATE_VELOCITY_OF_PID_PARAM,
	STATE_TORQUE_OF_PID_PARAM,

	STATE_MAX_OF_POSITION_CMD,
	STATE_MAX_OF_VELOCITY_CMD,
	STATE_MAX_OF_TORQUE_CMD,
	STATE_MAX_OF_PWM_DUTYCYCLE,

	STATE_POSITION_TARGET_CMD,
	STATE_VELOCITY_EXT_CMD,
	STATE_TORQUE_EXT_CMD,
	STATE_DEBUG_MODE,
};

struct AHRS::PID_PARAM
{
	public:
		int Kp,
		Ki,
		Kd,
		Int_sat; // integrator saturation
};

struct AHRS::MOTOR_MEMBER
{
	public:
		byte ticker;
		INT32 Position_Target,QEI32;
		INT16 Velocity_External,Velocity_Internal,QEI_Diff16;
		INT16 Torque_External,Torque_Internal,Motor_Current;
		INT16 PWM_Output;
};

struct AHRS::MOTOR_PID_MEMBER
{
	public :
		PID_PARAM Pos;  // position
		PID_PARAM Vel;  // velocity
		PID_PARAM Tor;  // torque
};

// Default constructor
AHRS::AHRS()
{
	//connected = false;
	serial = new SerialPort();
	serial->serial_port_name = "\\\\.\\COM4";
	serial->BaudRate = 4900000;
	serial->Open_SerialPort();
	// Fill arrays used for AHRS COM
	const int packet_count = AHRS::NumberOfPnames;
	byte Command[packet_count];

	//serialPort = new SerialPort();

	// Commands that can be sent to the AHRS device
	Command[AHRS::WHO_AM_I] = 0x00;
	Command[AHRS::FIRMWARE_VERSION] = 0x01;
	Command[AHRS::ID_STATUS] = 0x03;
	Command[AHRS::PWM_PARAMETER] = 0x04;
	Command[AHRS::CMD_CRC_FAILED] = 0x05;
	Command[AHRS::CMD_NO_SUPPORT] = 0x06;
	Command[AHRS::CMD_OVER_DATA_LENGTH] = 0x07;
	Command[AHRS::CMD_COMPLETE] = 0x08;
	Command[AHRS::REPORT_SENSOR_ADC] = 0x0E;
	Command[AHRS::CONTROLLER_STATUS] = 0x0F;

	Command[AHRS::RESET_CONTROLLER] = 0x10;
	Command[AHRS::KICK_OFF_CONTROLLER] = 0x11;
	Command[AHRS::REPORT_MCU_INFORMATION] = 0x12;
	Command[AHRS::HOME_CMD] = 0x13;
	Command[AHRS::PAUSE_REPORT_INFO] = 0x14;
	Command[AHRS::CONTINUE_REPORT_INFO] = 0x15;

	Command[AHRS::ALL_OF_PID_PARAM] = 0x20;
	Command[AHRS::POSITION_OF_PID_PARAM] = 0x21;
	Command[AHRS::VELOCITY_OF_PID_PARAM] = 0x22;
	Command[AHRS::TORQUE_OF_PID_PARAM] = 0x23;

	Command[AHRS::MAX_OF_POSITION_CMD] = 0x30;
	Command[AHRS::MAX_OF_VELOCITY_CMD] = 0x31;
	Command[AHRS::MAX_OF_TORQUE_CMD] = 0x32;
	Command[AHRS::MAX_OF_PWM_DUTYCYCLE] = 0x33;

	Command[AHRS::POSITION_TARGET_CMD] = 0x40;
	Command[AHRS::VELOCITY_EXT_CMD] = 0x41;
	Command[AHRS::TORQUE_EXT_CMD] = 0x42;
	Command[AHRS::DEBUG_MODE] = 0xFF;

	//RXbuffer = new byte[RX_BUF_SIZE];
	/*
	m_Motor_Member = new MOTOR_MEMBER[Motor_Device_SIZE];
	m_Motor_PID_Member = new MOTOR_PID_MEMBER[Motor_Device_SIZE];

	m_Pos_SoftStart = new INT16[Motor_Device_SIZE];
	m_Max_Vel_Cmd = new INT16[Motor_Device_SIZE];
	m_Max_Tor_Cmd = new INT16[Motor_Device_SIZE];
	m_Max_PWM_Cmd = new INT16[Motor_Device_SIZE];
	m_Position_Target = new UINT32[Motor_Device_SIZE];
	m_Velocity_External = new INT16[Motor_Device_SIZE];
	m_Torque_External = new INT16[Motor_Device_SIZE];
	*/

}

//讀出封包
void AHRS::read_packet(){

	int continue_parsing = 1;
	//int bytes_to_read;
	bool found_packet;
	int packet_start_index;
	int packet_index;
	DWORD   dwErrors;
	COMSTAT comStat;
	//AHRS *AHRS_Class = new AHRS();

	//取得Receive但尚未Read的數目(comStat.cbInQue)
	ClearCommError(serial->hSerial, &dwErrors, &comStat);
	//std::cout << dwErrors << ":::" << comStat.cbInQue << std::endl;


	if (ReadFile(serial->hSerial, serial->szBuff, comStat.cbInQue, (LPDWORD)serial->Read_Ptr, NULL)){
		//std::cout << szBuff << std::endl;
		//std::cout << dwErrors << ":::" << comStat.cbInQue << std::endl;


		while (continue_parsing == 1)//RXbufPtr >= 8 &&
		{
			// Search for the packet start sequence
			found_packet = false;
			packet_start_index = 0;
			for (packet_index = 0; packet_index < comStat.cbInQue; packet_index++)
			{
				if (serial->szBuff[packet_index] == 'E' && serial->szBuff[packet_index + 1] == 'C' && serial->szBuff[packet_index + 2] == 'S')
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
				DataPacket->PacketType = serial->szBuff[packet_start_index + 3];
				DataPacket->Ch_Status = serial->szBuff[packet_start_index + 4];
				DataPacket->DataLength = serial->szBuff[packet_start_index + 5];
				DataPacket->Data = new byte[DataPacket->DataLength - 1];

				// Only process packet if data_size is not too large.
				if (DataPacket->DataLength <= serial->MAX_PACKET_SIZE)
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
								DataPacket->Data[i] = serial->szBuff[packet_start_index + 6 + i];
							}
							DataPacket->CRC8 = serial->szBuff[packet_start_index + 6 + i];
							handle_packet(DataPacket);

						}

						// Copy all received bytes that weren't part of this packet into the beginning of the
						// buffer.  Then, reset RXbufPtr.

						for (int index = 0; index < (buffer_length - packet_length); index++)
						{
							serial->szBuff[index] = serial->szBuff[(packet_start_index + packet_length) + index];
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
	}
};

//解讀封包
void AHRS::handle_packet(Packet *packet){
	uint8_t channel = packet->Ch_Status;
	uint8_t command = packet->PacketType;
	uint8_t datalength = packet->DataLength;
	uint8_t CRC = packet->CRC8;
	// to do: check crc8
	if (CRC = Check_CRC8(*packet) && command == 0x0f){

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
		int *x = 0;
		short *y = 0;
		unsigned short *z = 0;
		unsigned char a[4];
		unsigned char b[2];

		//Count
		a[0] = packet->Data[0];
		*z = *(unsigned short *)a[0];
		data->Controller_Status[channel][0] = (int*) z;

		//Position Target
		a[3] = packet->Data[1];
		a[2] = packet->Data[2];
		a[1] = packet->Data[3];
		a[0] = packet->Data[4];
		*x = *(int *)a;
		data->Controller_Status[channel][1] = x;

		//Motor position
		a[3] = packet->Data[5];
		a[2] = packet->Data[6];
		a[1] = packet->Data[7];
		a[0] = packet->Data[8];
		*x = *(int *)a;
		data->Controller_Status[channel][2] = x;

		//Velocity External
		b[1] = packet->Data[9];
		b[0] = packet->Data[10];
		*y = *(short *)b;
		data->Controller_Status[channel][3] = (int*) y;

		//Velocity internal
		b[1] = packet->Data[11];
		b[0] = packet->Data[12];
		*y = *(short *)b;
		data->Controller_Status[channel][4] = (int*) y;

		//Motor velocity
		b[1] = packet->Data[13];
		b[0] = packet->Data[14];
		*y = *(short *)b;
		data->Controller_Status[channel][5] = (int*) y;
		
		//torque external
		b[1] = packet->Data[15];
		b[0] = packet->Data[16];
		*y = *(short *)b;
		data->Controller_Status[channel][6] = (int*) y;

		//torque internal
		b[1] = packet->Data[17];
		b[0] = packet->Data[18];
		*y = *(short *)b;
		data->Controller_Status[channel][7] = (int*) y;

		//Mortor torque
		b[1] = packet->Data[19];
		b[0] = packet->Data[20];
		*y = *(short *)b;
		data->Controller_Status[channel][8] = (int*) y;

		//dutycycle
		a[0] = packet->Data[21];
		*z = *(unsigned short *)a[0];
		data->Controller_Status[channel][9] = (int*) z;

		view->refresh(data);
	}
	else{
	
	}
	/*
	int *x = 0;
	unsigned char a[4];
	a[3] = packet->Data[5];
	a[2] = packet->Data[6];
	a[1] = packet->Data[7];
	a[0] = packet->Data[8];
	*x = *(int *)a;
	data->Controller_Status[channel][0] = x;
	*/
	//view->refresh(data);
};

//啟動馬達
void AHRS::kick_off(){
	Packet *packet = new Packet();
	for (int i = 0; i < 7; i++){
		//reset
		packet->Ch_Status = i;
		packet->PacketType = 0x10;
		packet->DataLength = 1;
		serial->Write_into_SerialPort(*packet);
		Sleep(100);

		//set period
		packet->Ch_Status = i;
		packet->PacketType = 0x11;
		packet->DataLength = 1;
		serial->Write_into_SerialPort(*packet);
		Sleep(100);

		//kick-off
		packet->Ch_Status = i;
		packet->PacketType = 0x43;
		packet->DataLength = 2;
		packet->Data[0] = 30;
		serial->Write_into_SerialPort(*packet);
		Sleep(100);
	}
	serial->flush();
}



int AHRS::getTypeIndex(byte type, int Command[])
{
	int type_index = -1;

	// Iterate through the PID array to determine which packet was received
	// (ie. which enum PName was received).
	for (int i = 0; i < AHRS::NumberOfPnames; i++)
	{
		if (Command[i] == type)
		{
			type_index = i;
			break;
		}
	}

	return type_index;
}

/*
private bool updateAHRS(int index)
{
	Packet AHRSPacket = new Packet();
	byte_conversion_array DConvert = new byte_conversion_array();
	AHRSPacket.Ch_Status = mChBox_CH;

	if (index == (int)StateName.STATE_WHO_AM_I)
	{
		AHRSPacket.PacketType = PID[(int)PName.REPORT_MCU_INFORMATION];
		AHRSPacket.DataLength = 2;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		AHRSPacket.Data[0] = PID[(int)PName.WHO_AM_I];

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_FIRMWARE_VERSION)
	{
		AHRSPacket.PacketType = PID[(int)PName.REPORT_MCU_INFORMATION];
		AHRSPacket.DataLength = 2;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		AHRSPacket.Data[0] = PID[(int)PName.FIRMWARE_VERSION];

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_ID_STATUS)
	{
		AHRSPacket.PacketType = PID[(int)PName.REPORT_MCU_INFORMATION];
		AHRSPacket.DataLength = 2;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		AHRSPacket.Data[0] = PID[(int)PName.ID_STATUS];

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_PWM_PARAMETER)
	{
		AHRSPacket.PacketType = PID[(int)PName.REPORT_MCU_INFORMATION];
		AHRSPacket.DataLength = 2;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		AHRSPacket.Data[0] = PID[(int)PName.PWM_PARAMETER];

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_KICK_OFF_CONTROLLER)
	{
		AHRSPacket.PacketType = PID[(int)PName.KICK_OFF_CONTROLLER];
		AHRSPacket.DataLength = 1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_RESET_CONTROLLER)
	{
		AHRSPacket.PacketType = PID[(int)PName.RESET_CONTROLLER];
		AHRSPacket.DataLength = 1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_REPORT_SENSOR_ADC)
	{
		AHRSPacket.PacketType = PID[(int)PName.REPORT_MCU_INFORMATION];
		AHRSPacket.DataLength = 2;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		AHRSPacket.Data[0] = PID[(int)PName.REPORT_SENSOR_ADC];

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_HOME_CMD)
	{
		AHRSPacket.PacketType = PID[(int)PName.HOME_CMD];
		AHRSPacket.DataLength = 1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_PAUSE_REPORT_INFO)
	{
		AHRSPacket.PacketType = PID[(int)PName.PAUSE_REPORT_INFO];
		AHRSPacket.DataLength = 1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_CONTINUE_REPORT_INFO)
	{
		AHRSPacket.PacketType = PID[(int)PName.CONTINUE_REPORT_INFO];
		AHRSPacket.DataLength = 1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_ALL_OF_PID_PARAM)
	{
		AHRSPacket.PacketType = PID[(int)PName.ALL_OF_PID_PARAM];
		AHRSPacket.DataLength = 49;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Pos.Kp;
		AHRSPacket.Data[0] = DConvert.byte0;
		AHRSPacket.Data[1] = DConvert.byte1;
		AHRSPacket.Data[2] = DConvert.byte2;
		AHRSPacket.Data[3] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Pos.Ki;
		AHRSPacket.Data[4] = DConvert.byte0;
		AHRSPacket.Data[5] = DConvert.byte1;
		AHRSPacket.Data[6] = DConvert.byte2;
		AHRSPacket.Data[7] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Pos.Kd;
		AHRSPacket.Data[8] = DConvert.byte0;
		AHRSPacket.Data[9] = DConvert.byte1;
		AHRSPacket.Data[10] = DConvert.byte2;
		AHRSPacket.Data[11] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Pos.Int_sat;
		AHRSPacket.Data[12] = DConvert.byte0;
		AHRSPacket.Data[13] = DConvert.byte1;
		AHRSPacket.Data[14] = DConvert.byte2;
		AHRSPacket.Data[15] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Vel.Kp;
		AHRSPacket.Data[16] = DConvert.byte0;
		AHRSPacket.Data[17] = DConvert.byte1;
		AHRSPacket.Data[18] = DConvert.byte2;
		AHRSPacket.Data[19] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Vel.Ki;
		AHRSPacket.Data[20] = DConvert.byte0;
		AHRSPacket.Data[21] = DConvert.byte1;
		AHRSPacket.Data[22] = DConvert.byte2;
		AHRSPacket.Data[23] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Vel.Kd;
		AHRSPacket.Data[24] = DConvert.byte0;
		AHRSPacket.Data[25] = DConvert.byte1;
		AHRSPacket.Data[26] = DConvert.byte2;
		AHRSPacket.Data[27] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Vel.Int_sat;
		AHRSPacket.Data[28] = DConvert.byte0;
		AHRSPacket.Data[29] = DConvert.byte1;
		AHRSPacket.Data[30] = DConvert.byte2;
		AHRSPacket.Data[31] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Tor.Kp;
		AHRSPacket.Data[32] = DConvert.byte0;
		AHRSPacket.Data[33] = DConvert.byte1;
		AHRSPacket.Data[34] = DConvert.byte2;
		AHRSPacket.Data[35] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Tor.Ki;
		AHRSPacket.Data[36] = DConvert.byte0;
		AHRSPacket.Data[37] = DConvert.byte1;
		AHRSPacket.Data[38] = DConvert.byte2;
		AHRSPacket.Data[39] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Tor.Kd;
		AHRSPacket.Data[40] = DConvert.byte0;
		AHRSPacket.Data[41] = DConvert.byte1;
		AHRSPacket.Data[42] = DConvert.byte2;
		AHRSPacket.Data[43] = DConvert.byte3;

		DConvert.int32 = m_Motor_PID_Member[mChBox_CH].Tor.Int_sat;
		AHRSPacket.Data[44] = DConvert.byte0;
		AHRSPacket.Data[45] = DConvert.byte1;
		AHRSPacket.Data[46] = DConvert.byte2;
		AHRSPacket.Data[47] = DConvert.byte3;
		//AHRSPacket.Data[48] = 0x00;
		//AHRSPacket.Data[49] = 0x00;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_MAX_OF_POSITION_CMD)
	{
		AHRSPacket.PacketType = PID[(int)PName.MAX_OF_POSITION_CMD];
		AHRSPacket.DataLength = 3;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		DConvert.int16_0 = m_Pos_SoftStart[mChBox_CH];
		AHRSPacket.Data[0] = DConvert.byte0;
		AHRSPacket.Data[1] = DConvert.byte1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_MAX_OF_VELOCITY_CMD)
	{
		AHRSPacket.PacketType = PID[(int)PName.MAX_OF_VELOCITY_CMD];
		AHRSPacket.DataLength = 3;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		DConvert.int16_0 = m_Max_Vel_Cmd[mChBox_CH];
		AHRSPacket.Data[0] = DConvert.byte0;
		AHRSPacket.Data[1] = DConvert.byte1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_MAX_OF_TORQUE_CMD)
	{
		AHRSPacket.PacketType = PID[(int)PName.MAX_OF_TORQUE_CMD];
		AHRSPacket.DataLength = 3;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		DConvert.int16_0 = m_Max_Tor_Cmd[mChBox_CH];
		AHRSPacket.Data[0] = DConvert.byte0;
		AHRSPacket.Data[1] = DConvert.byte1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_MAX_OF_PWM_DUTYCYCLE)
	{
		AHRSPacket.PacketType = PID[(int)PName.MAX_OF_PWM_DUTYCYCLE];
		AHRSPacket.DataLength = 3;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		DConvert.int16_0 = m_Max_PWM_Cmd[mChBox_CH];
		AHRSPacket.Data[0] = DConvert.byte0;
		AHRSPacket.Data[1] = DConvert.byte1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_POSITION_TARGET_CMD)
	{
		AHRSPacket.PacketType = PID[(int)PName.POSITION_TARGET_CMD];
		AHRSPacket.DataLength = 5;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		DConvert.uint32 = m_Position_Target[mChBox_CH];
		AHRSPacket.Data[0] = DConvert.byte0;
		AHRSPacket.Data[1] = DConvert.byte1;
		AHRSPacket.Data[2] = DConvert.byte2;
		AHRSPacket.Data[3] = DConvert.byte3;
		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_VELOCITY_EXT_CMD)
	{
		AHRSPacket.PacketType = PID[(int)PName.VELOCITY_EXT_CMD];
		AHRSPacket.DataLength = 3;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		DConvert.int16_0 = m_Velocity_External[mChBox_CH];
		AHRSPacket.Data[0] = DConvert.byte0;
		AHRSPacket.Data[1] = DConvert.byte1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}
	else if (index == (int)StateName.STATE_TORQUE_EXT_CMD)
	{
		AHRSPacket.PacketType = PID[(int)PName.TORQUE_EXT_CMD];
		AHRSPacket.DataLength = 3;

		AHRSPacket.Data = new byte[AHRSPacket.DataLength - 1];
		DConvert.int16_0 = m_Torque_External[mChBox_CH];
		AHRSPacket.Data[0] = DConvert.byte0;
		AHRSPacket.Data[1] = DConvert.byte1;

		AHRSPacket.CRC8 = Check_CRC8(AHRSPacket); // calculation CRC-8

		if (!sendPacket(AHRSPacket))
			return false;

	}



	return true;
}
*/

