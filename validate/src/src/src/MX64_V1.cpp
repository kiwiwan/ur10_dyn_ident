#include "MX64_V1.h"
 #include <unistd.h>


namespace MX64_V1
{


const double MX64::PROTOCOL_VERSION = 1.0; 
// const char*  DEVICENAME = "/dev/ttyUSB0";
const int    MX64::DXL_ID = 1;
const int    MX64::BAUDRATE = 115200; //1M

const int MX64::MIN_VALUE = 0;
const int MX64::CENTER_VALUE = 2048;
const int MX64::MAX_VALUE = 4095;
const double MX64::MIN_ANGLE = -180.0; // degree
const double MX64::MAX_ANGLE = 180.0; // degree
const double MX64::RATIO_VALUE2ANGLE = 0.088; // 360 / 4096
const double MX64::RATIO_ANGLE2VALUE = 11.378; // 4096 / 360

const double MX64::RATIO_VALUE2TORQUE = 6.0*0.001;
const double MX64::RATIO_TORQUE2VALUE = 1.0/(6.0*0.001);

const int MX64::PARAM_BYTES = 7;

MX64::MX64(const char *port_name)
{
	
	getHandlerAndOpenPort(port_name);
	// portHandler->setBaudRate(333333);
	setBaudrate(BAUDRATE);
	usleep(500000);
	setReturnDelayTime(100);
	setEnable(true);

	// printf("ReturnDelayTime = %d\n", getReturnDelayTime());
	getSensorData();
}

MX64::~MX64()
{
	// Close port
	portHandler->closePort();
}

MX64 *MX64::getInstance(const char *port_name)
{
	return (MX64 *)(new MX64(port_name));
}

void MX64::getHandlerAndOpenPort(const char *port_name)
{
	// Initialize PortHandler instance
  	// Set the port path
  	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	portHandler = dynamixel::PortHandler::getPortHandler(port_name);

  	// Initialize PacketHandler instance
  	// Set the protocol version
  	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);


	// Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
	}

	// Initialize GroupBulkRead instance
	groupBulkRead = (dynamixel::GroupBulkRead*) (new dynamixel::GroupBulkRead(portHandler, packetHandler));

	// Add parameter storage for Dynamixel#1 present position value
	dxl_comm_result = groupBulkRead->addParam(DXL_ID, P_PRESENT_POSITION_L, 6);
	if (dxl_comm_result != true)
	{
		fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID);
		return ;
	}

}

void MX64::setEnable(bool enable)
{
	// Enable Dynamixel Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_TORQUE_ENABLE , enable, &dxl_error);
	
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else
	{
		printf("Dynamixel has been successfully connected \n");
	}

}

bool MX64::getEnable()
{


}


void MX64::setBaudrate(int baudrate)
{
	uint8_t val;
	
	switch(baudrate)
	{
		case 1000000:
		  val = 1;
		  break;
		case 500000:
		  val = 3;
		  break;
		case 400000:
		  val = 4;
		  break;
		case 250000:
		  val = 7;
		  break;
		case 200000:
		  val = 9;
		  break;
		case 115200:
		  val = 16;
		  break;
		case 57600:
		  val = 34;
		  break;
		case 19200:
		  val = 103;
		  break;
		case 9600:
		  val = 207;
		  break;
		case 2250000:
		  val = 250;
		  break;
		case 2500000:
		  val = 251;
		  break;
		case 3000000:
		  val = 252;
		  break;
		default:
		  printf("baudrate not konwn!\n");
		  return;
 	}

 	// printf("val = %d",val);
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_BAUD_RATE, val, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	if (portHandler->setBaudRate(baudrate))
	{
		printf("Succeeded to change the baudrate to %d!\n",baudrate);
	}
	else
	{
		printf("Failed to change the baudrate!\n");
	}


}

int MX64::getBaudrate()  //dynamixel baudrate,not the port
{
	uint8_t rval;
	// Read present position
	dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, P_BAUD_RATE, &rval, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	switch(rval)
	{
		case 1:
		  return 1000000;
		case 3:
		  return 500000;
		case 4:
		  return 400000;
		case 7:
		  return 250000;
		case 9:
		  return 200000;
		case 16:
		  return 115200;
		case 34:
		  return 57600;
		case 103:
		  return 19200;
		case 207:
		  return 9600;
		case 250:
		  return 2250000;
		case 251:
		  return 2500000;
		case 252:
		  return 3000000;
		default:
		  return -1;
 	}

}

void MX64::setReturnDelayTime(int delay)
{
	uint8_t val = delay/2;

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_RETURN_DELAY_TIME, val, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

int MX64::getReturnDelayTime()
{
	uint8_t return_time;
	// Read present position
	dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, P_RETURN_DELAY_TIME, &return_time, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	return return_time;

}

void MX64::setPGain(int pgain)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_P_GAIN, pgain, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

int  MX64::getPGain()
{


}

void MX64::setIGain( int igain)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_I_GAIN, igain, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

int  MX64::getIGain()
{


}

void MX64::setDGain(int dgain)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_D_GAIN, dgain, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

int  MX64::getDGain()
{


}


void MX64::setGoalPositon(double angle)
{

	// Write goal position
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, P_GOAL_POSITION_L, angle2Value(angle), &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

void MX64::getCurrentPositon()
{
	uint16_t present_position;
	// Read present position
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, P_PRESENT_POSITION_L, &present_position, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	curPos = value2Angle(present_position);
}

double MX64::getVel()
{
	uint16_t present_vel;
	// Read present position
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID,  P_PRESENT_SPEED_L, &present_vel, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	curVel = value2Vel(present_vel);

}


void MX64::setTorqueCtrlEnable(bool enable)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_TORQUE_CTRL_ENABLE, enable, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

bool MX64::getTorqueCtrlEnable()
{


}


void MX64::setGoalTorque(double tor)
{
	// int torq = 1025;
	// Write goal Torque
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, P_GOAL_TORQUE_L, torque2Value(tor), &dxl_error);  //
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

double MX64::getCurrentTorque()
{
	uint16_t present_torque;
	// Read present position
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, P_PRESENT_LOAD_L, &present_torque, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	curTorque = value2Torque(present_torque);

}


void MX64::getSensorData()
{

	
	dxl_comm_result = groupBulkRead->txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	dxl_comm_result = groupBulkRead->isAvailable(DXL_ID, P_PRESENT_POSITION_L, 6);
	if (dxl_comm_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID);
		return ;
	}


	uint32_t present_torque,present_vel,present_position;
	// Get Dynamixel#1 present position value
	present_position = groupBulkRead->getData(DXL_ID, P_PRESENT_POSITION_L, 2);
	present_vel = groupBulkRead->getData(DXL_ID, P_PRESENT_SPEED_L, 2);
	present_torque = groupBulkRead->getData(DXL_ID, P_PRESENT_LOAD_L, 2);	
	// printf("present_torque = %d\n",present_torque);

	curPos = value2Angle(present_position);
	curVel = value2Vel(present_vel);
	curTorque = value2Torque(present_torque);
	curTorque = -curTorque; 	//torque need multiply -1,convert to sensor of torque command,rather than load torque

}



}

