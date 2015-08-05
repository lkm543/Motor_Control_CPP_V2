#include "Serial_Port.h"
#include "Visual.h"
#include "Packet.h"
#include "data.h"

//TO DO  timeout handler
class AHRS{
	public:
		void receive_packet(Packet *packet);
		void send_packet(Packet *packet);
		enum PName;
		enum StateName;
		struct PID_PARAM;
		struct MOTOR_MEMBER;
		struct MOTOR_PID_MEMBER;
		// Variables for storing the state of the Motor controller
		static const int Motor_Device_SIZE = 7;
		int getTypeIndex(byte type);

	private:
		int Motor_Channel = 0;//Convert.to 
		MOTOR_MEMBER m_Motor_Member[Motor_Device_SIZE];
		MOTOR_PID_MEMBER m_Motor_PID_Member[Motor_Device_SIZE];
		INT16 m_Pos_SoftStart[Motor_Device_SIZE];
		INT16 m_Max_Vel_Cmd[Motor_Device_SIZE];
		INT16 m_Max_Tor_Cmd[Motor_Device_SIZE];
		INT16 m_Max_PWM_Cmd[Motor_Device_SIZE];
		INT32 m_Position_Target[Motor_Device_SIZE];
		INT16 m_Velocity_External[Motor_Device_SIZE];
		INT16 m_Torque_External[Motor_Device_SIZE];
};

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
// Commands that can be sent to the AHRS device
PID[(int)PName.WHO_AM_I] = 0x00;
PID[(int)PName.FIRMWARE_VERSION] = 0x01;
PID[(int)PName.ID_STATUS] = 0x03;
PID[(int)PName.PWM_PARAMETER] = 0x04;
PID[(int)PName.CMD_CRC_FAILED] = 0x05;
PID[(int)PName.CMD_NO_SUPPORT] = 0x06;
PID[(int)PName.CMD_OVER_DATA_LENGTH] = 0x07;
PID[(int)PName.CMD_COMPLETE] = 0x08;
PID[(int)PName.REPORT_SENSOR_ADC] = 0x0E;
PID[(int)PName.CONTROLLER_STATUS] = 0x0F;

PID[(int)PName.RESET_CONTROLLER] = 0x10;
PID[(int)PName.KICK_OFF_CONTROLLER] = 0x11;
PID[(int)PName.REPORT_MCU_INFORMATION] = 0x12;
PID[(int)PName.HOME_CMD] = 0x13;
PID[(int)PName.PAUSE_REPORT_INFO] = 0x14;
PID[(int)PName.CONTINUE_REPORT_INFO] = 0x15;

PID[(int)PName.ALL_OF_PID_PARAM] = 0x20;
PID[(int)PName.POSITION_OF_PID_PARAM] = 0x21;
PID[(int)PName.VELOCITY_OF_PID_PARAM] = 0x22;
PID[(int)PName.TORQUE_OF_PID_PARAM] = 0x23;

PID[(int)PName.MAX_OF_POSITION_CMD] = 0x30;
PID[(int)PName.MAX_OF_VELOCITY_CMD] = 0x31;
PID[(int)PName.MAX_OF_TORQUE_CMD] = 0x32;
PID[(int)PName.MAX_OF_PWM_DUTYCYCLE] = 0x33;

PID[(int)PName.POSITION_TARGET_CMD] = 0x40;
PID[(int)PName.VELOCITY_EXT_CMD] = 0x41;
PID[(int)PName.TORQUE_EXT_CMD] = 0x42;
PID[(int)PName.DEBUG_MODE] = 0xFF;

m_Motor_Member = new MOTOR_MEMBER[Motor_Device_SIZE];
m_Motor_PID_Member = new MOTOR_PID_MEMBER[Motor_Device_SIZE];

m_Pos_SoftStart = new Int16[Motor_Device_SIZE];
m_Max_Vel_Cmd = new Int16[Motor_Device_SIZE];
m_Max_Tor_Cmd = new Int16[Motor_Device_SIZE];
m_Max_PWM_Cmd = new Int16[Motor_Device_SIZE];
m_Position_Target = new UInt32[Motor_Device_SIZE];
m_Velocity_External = new Int16[Motor_Device_SIZE];
m_Torque_External = new Int16[Motor_Device_SIZE];
void AHRS::receive_packet(Packet *packet){
	Data *data;
	View *view;
	byte channel = packet->Ch_Status;
	byte command = packet->PacketType;
	byte datalength = packet->DataLength;
	byte CRC = packet->CRC8;
	// to do: check crc8

	int *x;
	unsigned char a[4];
	a[3] = packet->Data[5];
	a[2] = packet->Data[6];
	a[1] = packet->Data[7];
	a[0] = packet->Data[8];
	*x = *(int *)a;
	data->Controller_Status[channel][0] = x;
	view->refresh(data);
};

int AHRS::getTypeIndex(byte type)
{
	int type_index = -1;

	// Iterate through the PID array to determine which packet was received
	// (ie. which enum PName was received).
	for (int i = 0; i < PID.Length; i++)
	{
		if (PID[i] == type)
		{
			type_index = i;
			break;
		}
	}

	return type_index;
}

private void handle_packet(Packet DataPacket)
{
	int type_index = -1;
	byte CRC8_Temp;
	byte_conversion_array DConvert = new byte_conversion_array();

	type_index = getTypeIndex(DataPacket.PacketType);

	// For the packet received, update 'dataPending' and 'ElapsedTime' flags
	if (type_index != -1)
	{

		CRC8_Temp = Check_CRC8(DataPacket); // calculation CRC-8

		if (CRC8_Temp != DataPacket.CRC8)
		{
			PacketReceivedEvent(PName.CMD_CRC_FAILED, -1);
		}
		else
		{
			updatePacketSynch((PName)type_index, DataPacket.Data);
		}
	}
	else
	{
		// Generate a COMMAND_COMPLETE event with a -1 flag.  The -1 indicates that the packet
		// wasn't recognized.
		PacketReceivedEvent(PName.CMD_COMPLETE, -1);
		return;
	}

	switch (type_index)
	{
	case (int)PName.WHO_AM_I:
		PacketLabelEvent(DataPacket.Data, 1);
		PacketReceivedEvent(PName.WHO_AM_I, type_index);
		break;

	case (int)PName.FIRMWARE_VERSION:
		PacketLabelEvent(DataPacket.Data, 2);
		PacketReceivedEvent(PName.FIRMWARE_VERSION, type_index);
		break;

	case (int)PName.REPORT_SENSOR_ADC:
		PacketLabelEvent(DataPacket.Data, 23);
		PacketReceivedEvent(PName.REPORT_SENSOR_ADC, type_index);
		break;

	case (int)PName.ID_STATUS:

		PacketReceivedEvent(PName.ID_STATUS, type_index);
		break;

	case (int)PName.PWM_PARAMETER:

		PacketReceivedEvent(PName.PWM_PARAMETER, type_index);
		break;

	case (int)PName.CMD_COMPLETE:

		type_index = getTypeIndex(DataPacket.Data[0]);

		PacketReceivedEvent(PName.CMD_COMPLETE, type_index);

		break;

	case (int)PName.CMD_NO_SUPPORT:

		PacketReceivedEvent(PName.CMD_NO_SUPPORT, DataPacket.Data[0]);

		break;

	case (int)PName.CMD_CRC_FAILED:

		PacketReceivedEvent(PName.CMD_CRC_FAILED, 0);

		break;

	case (int)PName.CMD_OVER_DATA_LENGTH:

		PacketReceivedEvent(PName.CMD_OVER_DATA_LENGTH, 0);

		break;

	case (int)PName.CONTROLLER_STATUS:

		if (DataPacket.DataLength != 23)
		{
			// this packet is wrong!!!
			return;
		}
		byte ch_tmp;
		ch_tmp = Convert.ToByte(Convert.ToByte(DataPacket.Ch_Status) & 0xf);
		switch (ch_tmp)
		{
		case 0:
			m_Motor_Member[0].ticker = DataPacket.Data[0];

			DConvert.byte0 = DataPacket.Data[1];
			DConvert.byte1 = DataPacket.Data[2];
			DConvert.byte2 = DataPacket.Data[3];
			DConvert.byte3 = DataPacket.Data[4];
			m_Motor_Member[0].Position_Target = DConvert.uint32; // command

			DConvert.byte0 = DataPacket.Data[5];
			DConvert.byte1 = DataPacket.Data[6];
			DConvert.byte2 = DataPacket.Data[7];
			DConvert.byte3 = DataPacket.Data[8];
			m_Motor_Member[0].QEI32 = DConvert.uint32;

			DConvert.byte0 = DataPacket.Data[9];
			DConvert.byte1 = DataPacket.Data[10];
			m_Motor_Member[0].Velocity_External = DConvert.int16_0; // command

			DConvert.byte0 = DataPacket.Data[11];
			DConvert.byte1 = DataPacket.Data[12];
			m_Motor_Member[0].Velocity_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[13];
			DConvert.byte1 = DataPacket.Data[14];
			m_Motor_Member[0].QEI_Diff16 = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[15];
			DConvert.byte1 = DataPacket.Data[16];
			m_Motor_Member[0].Torque_External = DConvert.int16_0; //command

			DConvert.byte0 = DataPacket.Data[17];
			DConvert.byte1 = DataPacket.Data[18];
			m_Motor_Member[0].Torque_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[19];
			DConvert.byte1 = DataPacket.Data[20];
			m_Motor_Member[0].Motor_Current = DConvert.int16_0;

			m_Motor_Member[0].PWM_Output = DataPacket.Data[21];
			break;
		case 1:
			m_Motor_Member[1].ticker = DataPacket.Data[0];

			DConvert.byte0 = DataPacket.Data[1];
			DConvert.byte1 = DataPacket.Data[2];
			DConvert.byte2 = DataPacket.Data[3];
			DConvert.byte3 = DataPacket.Data[4];
			m_Motor_Member[1].Position_Target = DConvert.uint32; // command

			DConvert.byte0 = DataPacket.Data[5];
			DConvert.byte1 = DataPacket.Data[6];
			DConvert.byte2 = DataPacket.Data[7];
			DConvert.byte3 = DataPacket.Data[8];
			m_Motor_Member[1].QEI32 = DConvert.uint32;

			DConvert.byte0 = DataPacket.Data[9];
			DConvert.byte1 = DataPacket.Data[10];
			m_Motor_Member[1].Velocity_External = DConvert.int16_0; // command

			DConvert.byte0 = DataPacket.Data[11];
			DConvert.byte1 = DataPacket.Data[12];
			m_Motor_Member[1].Velocity_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[13];
			DConvert.byte1 = DataPacket.Data[14];
			m_Motor_Member[1].QEI_Diff16 = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[15];
			DConvert.byte1 = DataPacket.Data[16];
			m_Motor_Member[1].Torque_External = DConvert.int16_0; //command

			DConvert.byte0 = DataPacket.Data[17];
			DConvert.byte1 = DataPacket.Data[18];
			m_Motor_Member[1].Torque_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[19];
			DConvert.byte1 = DataPacket.Data[20];
			m_Motor_Member[1].Motor_Current = DConvert.int16_0;

			m_Motor_Member[1].PWM_Output = DataPacket.Data[21];
			break;
		case 2:
			m_Motor_Member[2].ticker = DataPacket.Data[0];

			DConvert.byte0 = DataPacket.Data[1];
			DConvert.byte1 = DataPacket.Data[2];
			DConvert.byte2 = DataPacket.Data[3];
			DConvert.byte3 = DataPacket.Data[4];
			m_Motor_Member[2].Position_Target = DConvert.uint32; // command

			DConvert.byte0 = DataPacket.Data[5];
			DConvert.byte1 = DataPacket.Data[6];
			DConvert.byte2 = DataPacket.Data[7];
			DConvert.byte3 = DataPacket.Data[8];
			m_Motor_Member[2].QEI32 = DConvert.uint32;

			DConvert.byte0 = DataPacket.Data[9];
			DConvert.byte1 = DataPacket.Data[10];
			m_Motor_Member[2].Velocity_External = DConvert.int16_0; // command

			DConvert.byte0 = DataPacket.Data[11];
			DConvert.byte1 = DataPacket.Data[12];
			m_Motor_Member[2].Velocity_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[13];
			DConvert.byte1 = DataPacket.Data[14];
			m_Motor_Member[2].QEI_Diff16 = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[15];
			DConvert.byte1 = DataPacket.Data[16];
			m_Motor_Member[2].Torque_External = DConvert.int16_0; //command

			DConvert.byte0 = DataPacket.Data[17];
			DConvert.byte1 = DataPacket.Data[18];
			m_Motor_Member[2].Torque_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[19];
			DConvert.byte1 = DataPacket.Data[20];
			m_Motor_Member[2].Motor_Current = DConvert.int16_0;

			m_Motor_Member[2].PWM_Output = DataPacket.Data[21];
			break;
		case 3:
			m_Motor_Member[3].ticker = DataPacket.Data[0];

			DConvert.byte0 = DataPacket.Data[1];
			DConvert.byte1 = DataPacket.Data[2];
			DConvert.byte2 = DataPacket.Data[3];
			DConvert.byte3 = DataPacket.Data[4];
			m_Motor_Member[3].Position_Target = DConvert.uint32; // command

			DConvert.byte0 = DataPacket.Data[5];
			DConvert.byte1 = DataPacket.Data[6];
			DConvert.byte2 = DataPacket.Data[7];
			DConvert.byte3 = DataPacket.Data[8];
			m_Motor_Member[3].QEI32 = DConvert.uint32;

			DConvert.byte0 = DataPacket.Data[9];
			DConvert.byte1 = DataPacket.Data[10];
			m_Motor_Member[3].Velocity_External = DConvert.int16_0; // command

			DConvert.byte0 = DataPacket.Data[11];
			DConvert.byte1 = DataPacket.Data[12];
			m_Motor_Member[3].Velocity_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[13];
			DConvert.byte1 = DataPacket.Data[14];
			m_Motor_Member[3].QEI_Diff16 = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[15];
			DConvert.byte1 = DataPacket.Data[16];
			m_Motor_Member[3].Torque_External = DConvert.int16_0; //command

			DConvert.byte0 = DataPacket.Data[17];
			DConvert.byte1 = DataPacket.Data[18];
			m_Motor_Member[3].Torque_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[19];
			DConvert.byte1 = DataPacket.Data[20];
			m_Motor_Member[3].Motor_Current = DConvert.int16_0;

			m_Motor_Member[3].PWM_Output = DataPacket.Data[21];
			break;
		case 4:
			m_Motor_Member[4].ticker = DataPacket.Data[0];

			DConvert.byte0 = DataPacket.Data[1];
			DConvert.byte1 = DataPacket.Data[2];
			DConvert.byte2 = DataPacket.Data[3];
			DConvert.byte3 = DataPacket.Data[4];
			m_Motor_Member[4].Position_Target = DConvert.uint32; // command

			DConvert.byte0 = DataPacket.Data[5];
			DConvert.byte1 = DataPacket.Data[6];
			DConvert.byte2 = DataPacket.Data[7];
			DConvert.byte3 = DataPacket.Data[8];
			m_Motor_Member[4].QEI32 = DConvert.uint32;

			DConvert.byte0 = DataPacket.Data[9];
			DConvert.byte1 = DataPacket.Data[10];
			m_Motor_Member[4].Velocity_External = DConvert.int16_0; // command

			DConvert.byte0 = DataPacket.Data[11];
			DConvert.byte1 = DataPacket.Data[12];
			m_Motor_Member[4].Velocity_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[13];
			DConvert.byte1 = DataPacket.Data[14];
			m_Motor_Member[4].QEI_Diff16 = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[15];
			DConvert.byte1 = DataPacket.Data[16];
			m_Motor_Member[4].Torque_External = DConvert.int16_0; //command

			DConvert.byte0 = DataPacket.Data[17];
			DConvert.byte1 = DataPacket.Data[18];
			m_Motor_Member[4].Torque_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[19];
			DConvert.byte1 = DataPacket.Data[20];
			m_Motor_Member[4].Motor_Current = DConvert.int16_0;

			m_Motor_Member[4].PWM_Output = DataPacket.Data[21];
			break;
		case 5:
			m_Motor_Member[5].ticker = DataPacket.Data[0];

			DConvert.byte0 = DataPacket.Data[1];
			DConvert.byte1 = DataPacket.Data[2];
			DConvert.byte2 = DataPacket.Data[3];
			DConvert.byte3 = DataPacket.Data[4];
			m_Motor_Member[5].Position_Target = DConvert.uint32; // command

			DConvert.byte0 = DataPacket.Data[5];
			DConvert.byte1 = DataPacket.Data[6];
			DConvert.byte2 = DataPacket.Data[7];
			DConvert.byte3 = DataPacket.Data[8];
			m_Motor_Member[5].QEI32 = DConvert.uint32;

			DConvert.byte0 = DataPacket.Data[9];
			DConvert.byte1 = DataPacket.Data[10];
			m_Motor_Member[5].Velocity_External = DConvert.int16_0; // command

			DConvert.byte0 = DataPacket.Data[11];
			DConvert.byte1 = DataPacket.Data[12];
			m_Motor_Member[5].Velocity_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[13];
			DConvert.byte1 = DataPacket.Data[14];
			m_Motor_Member[5].QEI_Diff16 = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[15];
			DConvert.byte1 = DataPacket.Data[16];
			m_Motor_Member[5].Torque_External = DConvert.int16_0; //command

			DConvert.byte0 = DataPacket.Data[17];
			DConvert.byte1 = DataPacket.Data[18];
			m_Motor_Member[5].Torque_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[19];
			DConvert.byte1 = DataPacket.Data[20];
			m_Motor_Member[5].Motor_Current = DConvert.int16_0;

			m_Motor_Member[5].PWM_Output = DataPacket.Data[21];
			break;
		case 6:
			m_Motor_Member[6].ticker = DataPacket.Data[0];

			DConvert.byte0 = DataPacket.Data[1];
			DConvert.byte1 = DataPacket.Data[2];
			DConvert.byte2 = DataPacket.Data[3];
			DConvert.byte3 = DataPacket.Data[4];
			m_Motor_Member[6].Position_Target = DConvert.uint32; // command

			DConvert.byte0 = DataPacket.Data[5];
			DConvert.byte1 = DataPacket.Data[6];
			DConvert.byte2 = DataPacket.Data[7];
			DConvert.byte3 = DataPacket.Data[8];
			m_Motor_Member[6].QEI32 = DConvert.uint32;

			DConvert.byte0 = DataPacket.Data[9];
			DConvert.byte1 = DataPacket.Data[10];
			m_Motor_Member[6].Velocity_External = DConvert.int16_0; // command

			DConvert.byte0 = DataPacket.Data[11];
			DConvert.byte1 = DataPacket.Data[12];
			m_Motor_Member[6].Velocity_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[13];
			DConvert.byte1 = DataPacket.Data[14];
			m_Motor_Member[6].QEI_Diff16 = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[15];
			DConvert.byte1 = DataPacket.Data[16];
			m_Motor_Member[6].Torque_External = DConvert.int16_0; //command

			DConvert.byte0 = DataPacket.Data[17];
			DConvert.byte1 = DataPacket.Data[18];
			m_Motor_Member[6].Torque_Internal = DConvert.int16_0;

			DConvert.byte0 = DataPacket.Data[19];
			DConvert.byte1 = DataPacket.Data[20];
			m_Motor_Member[6].Motor_Current = DConvert.int16_0;

			m_Motor_Member[6].PWM_Output = DataPacket.Data[21];
			break;

		default:

			break;

		}
		/*
		m_Motor_Member[mChBox_CH].ticker = DataPacket.Data[0];

		DConvert.byte0 = DataPacket.Data[1];
		DConvert.byte1 = DataPacket.Data[2];
		DConvert.byte2 = DataPacket.Data[3];
		DConvert.byte3 = DataPacket.Data[4];
		m_Motor_Member[mChBox_CH].Position_Target = DConvert.uint32; // command

		DConvert.byte0 = DataPacket.Data[5];
		DConvert.byte1 = DataPacket.Data[6];
		DConvert.byte2 = DataPacket.Data[7];
		DConvert.byte3 = DataPacket.Data[8];
		m_Motor_Member[mChBox_CH].QEI32 = DConvert.uint32;

		DConvert.byte0 = DataPacket.Data[9];
		DConvert.byte1 = DataPacket.Data[10];
		m_Motor_Member[mChBox_CH].Velocity_External = DConvert.int16_0; // command

		DConvert.byte0 = DataPacket.Data[11];
		DConvert.byte1 = DataPacket.Data[12];
		m_Motor_Member[mChBox_CH].Velocity_Internal = DConvert.int16_0;

		DConvert.byte0 = DataPacket.Data[13];
		DConvert.byte1 = DataPacket.Data[14];
		m_Motor_Member[mChBox_CH].QEI_Diff16 = DConvert.int16_0;

		DConvert.byte0 = DataPacket.Data[15];
		DConvert.byte1 = DataPacket.Data[16];
		m_Motor_Member[mChBox_CH].Torque_External = DConvert.int16_0; //command

		DConvert.byte0 = DataPacket.Data[17];
		DConvert.byte1 = DataPacket.Data[18];
		m_Motor_Member[mChBox_CH].Torque_Internal = DConvert.int16_0;

		DConvert.byte0 = DataPacket.Data[19];
		DConvert.byte1 = DataPacket.Data[20];
		m_Motor_Member[mChBox_CH].Motor_Current = DConvert.int16_0;

		m_Motor_Member[mChBox_CH].PWM_Output = DataPacket.Data[21];

		*/
		PacketReceivedEvent(PName.CONTROLLER_STATUS, 0);

		bitstatue = Convert.ToByte(Convert.ToByte(DataPacket.Ch_Status) & 0xf0);
		//bitstatue +=1;

		DataReceivedEvent(ch_tmp);

		break;


	default:

		break;

	}

}

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