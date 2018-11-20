#include "MX64_V2.h"
 #include <unistd.h>


namespace MX64_V2
{

const double MX64::PROTOCOL_VERSION = 2.0; 
// const char*  DEVICENAME = "/dev/ttyUSB0";
const int    MX64::DXL_ID = 1;
const int    MX64::BAUDRATE = 3000000;//115200; //1M

const int MX64::MIN_VALUE = 0;
const int MX64::CENTER_VALUE = 2048;
const int MX64::MAX_VALUE = 4095;
const double MX64::MIN_ANGLE = -180.0; // degree
const double MX64::MAX_ANGLE = 180.0; // degree
const double MX64::RATIO_VALUE2ANGLE = 0.088; // 360 / 4096
const double MX64::RATIO_ANGLE2VALUE = 11.378; // 4096 / 360

const double MX64::RATIO_VALUE2VEL = 0.229*6; // degree/s
const double MX64::RATIO_VEL2VALUE = 0.7278; // 

const double MX64::RATIO_VALUE2TORQUE = 0.00336*1.1619;        //from torque-current curve(2.72,2.5),(0.28,0.4),derive 1.1619 N.m/A
const double MX64::RATIO_TORQUE2VALUE = 1.0/(0.00336*1.1619);




MX64::MX64(const char *port_name)
{
	
	getHandlerAndOpenPort(port_name);
	// portHandler->setBaudRate(4000000);
	setBaudrate(BAUDRATE);
	usleep(500000);
	setReturnDelayTime(100);
	setTemperatureLimit(50);

	setEnable(false);
	setOperateMode(POSITION_CONTROLL);
	setEnable(true);
	setGoalPositon(0);
	usleep(500000);
	
	setEnable(false);
	setOperateMode(CURRENT_CONTROLL);
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
	dxl_comm_result = groupBulkRead->addParam(DXL_ID, P_PRESENT_CURRENT_L, 10);
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
		if(enable == true)
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
		case 9600:
		  val = 0;
		  break;
		case 57600:
		  val = 1;
		  break;
		case 115200:
		  val = 2;
		  break;
		case 1000000:
		  val = 3;
		  break;
		case 2000000:
		  val = 4;
		  break;
		case 3000000:
		  val = 5;
		  break;
		case 4000000:
		  val = 6;
		  break;
		case 4500000:
		  val = 7;
		  break;
		default:
		  printf("baudrate not konwn!\n");
		  return;
 	}

 	// printf("val = %d",val);
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_BAUD_RATE, val, &dxl_error); //set the dynamixel baudrate
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	if (portHandler->setBaudRate(baudrate))				//set the port baudrate
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
		case 0:
		  return 9600;
		case 1:
		  return 57600;
		case 2:
		  return 115200;
		case 3:
		  return 1000000;
		case 4:
		  return 2000000;
		case 5:
		  return 3000000;
		case 6:
		  return 4000000;
		case 7:
		  return 4500000;
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

void MX64::setTemperatureLimit(int t)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_HIGH_LIMIT_TEMPERATURE, t, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
}

void MX64::setPositionPGain(int pgain)
{
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, P_POSITION_P_GAIN_L, pgain, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

int  MX64::getPositionPGain()
{


}

void MX64::setPositionIGain( int igain)
{
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, P_POSITION_I_GAIN_L, igain, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

int  MX64::getPositionIGain()
{


}

void MX64::setPositionDGain(int dgain)
{
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, P_POSITION_D_GAIN_L, dgain, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

int  MX64::getPositionDGain()
{


}


void MX64::setOperateMode(operateMode mode)
{
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, P_OPERATING_MODE, mode, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
}

void MX64::setGoalPositon(double angle)
{

	// Write goal position
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, P_GOAL_POSITION, angle2Value(angle), &dxl_error);
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
	uint32_t present_position;
	// Read present position
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, P_PRESENT_POSITION, &present_position, &dxl_error);
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

void MX64::setGoalVel(double vel)
{

	// Write goal position
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, P_GOAL_VELOCITY, vel2Value(vel), &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

}

double MX64::getCurrentVel()
{
	uint32_t present_vel;
	// Read present position
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID,  P_PRESENT_VELOCITY, &present_vel, &dxl_error);
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



void MX64::setGoalTorque(double tor)
{
	// double t;
	// gettimeofday(&last_tv,NULL);

	// int torq = 3;
	// Write goal Torque
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, P_GOAL_CURRENT_L, torque2Value(tor), &dxl_error);  //
	if (dxl_comm_result != COMM_SUCCESS)
	{
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0)
	{
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}

	// gettimeofday(&tv,NULL);
	// t=tv.tv_sec*1000000+tv.tv_usec-(last_tv.tv_sec*1000000+last_tv.tv_usec);
	// if(t > 4000)
	// 	printf("setGoalTorque:%f\n",t);

}

double MX64::getCurrentTorque()
{
	uint16_t present_torque;	//why need uint
	// Read present position
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, P_PRESENT_CURRENT_L, &present_torque, &dxl_error);
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

	// double t;
	// gettimeofday(&last_tv,NULL);

	
	dxl_comm_result = groupBulkRead->txRxPacket();
	


	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	dxl_comm_result = groupBulkRead->isAvailable(DXL_ID, P_PRESENT_CURRENT_L, 10);
	if (dxl_comm_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID);
		return ;
	}

	uint32_t present_position;
	int32_t present_vel;
	int16_t present_torque;
	// Get Dynamixel#1 present position value
	present_position = groupBulkRead->getData(DXL_ID, P_PRESENT_POSITION, 4);
	present_vel = groupBulkRead->getData(DXL_ID, P_PRESENT_VELOCITY, 4);
	present_torque = groupBulkRead->getData(DXL_ID, P_PRESENT_CURRENT_L, 2);	
	// printf("present_position = %d;  present_vel = %d;  present_torque = %d\n",present_position,present_vel,present_torque);

	curPos = value2Angle(present_position);
	curVel = value2Vel(present_vel);
	curTorque = value2Torque(present_torque);
	// curTorque = -curTorque; 	//torque need multiply -1,convert to sensor of torque command,rather than load torque
	// printf("curPos = %f;  curVel = %f;  curTorque = %f\n",curPos,curVel,curTorque);


	// gettimeofday(&tv,NULL);
	// t=tv.tv_sec*1000000+tv.tv_usec-(last_tv.tv_sec*1000000+last_tv.tv_usec);
	// if(t > 4000)
	// 	printf("getSensorData:%f\n",t);
}




}


