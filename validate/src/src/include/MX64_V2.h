
#ifndef _MX_64_V2_H_
#define _MX_64_V2_H_

#include "dynamixel_sdk.h" 

#include <time.h>
#include <sys/time.h>

namespace MX64_V2
{

class MX64
{
public:
    static const double PROTOCOL_VERSION;                          
    // static const char*  DEVICENAME; 
    static const int DXL_ID;

    static const int BAUDRATE;

    static const int MIN_VALUE;
    static const int CENTER_VALUE;
    static const int MAX_VALUE;
    static const double MIN_ANGLE;
    static const double MAX_ANGLE;
    static const double RATIO_VALUE2ANGLE;
    static const double RATIO_ANGLE2VALUE;
    static const double RATIO_VALUE2VEL; 
    static const double RATIO_VEL2VALUE; 
    static const double RATIO_VALUE2TORQUE;
    static const double RATIO_TORQUE2VALUE;


    MX64(const char *port_name);
    ~MX64();


    static int getMirrorValue(int value)        {
        return MAX_VALUE + 1 - value;
    }
    static double getMirrorAngle(double angle)  {
        return -angle;
    }

    static int angle2Value(double angle) {
        return (int)(angle*RATIO_ANGLE2VALUE)+CENTER_VALUE;
    }
    static double value2Angle(int value) {
        return (double)(value-CENTER_VALUE)*RATIO_VALUE2ANGLE;
    }

    static int torque2Value(double tor) {
        if(tor > 6.0 || tor < -6.0)   //7.57
            {printf("torque command too large!\n");return 0;}

            return (int)(tor*RATIO_TORQUE2VALUE);
    }
    static double value2Torque(int value) {
            return (double)(value*RATIO_VALUE2TORQUE);
    }

    static int vel2Value(double vel) {  //degree/s
            return (int)(vel*RATIO_VEL2VALUE);
    }
    static double value2Vel(int value) {  //degree/s
            return (double)(value*RATIO_VALUE2VEL);
    }

    // Address  protocol 1.0
    enum controlTableAdress
    {
        P_MODEL_NUMBER_L            = 0,
        P_MODEL_NUMBER_H            = 1,
        P_VERSION                   = 6,
        P_ID                        = 7,
        P_BAUD_RATE                 = 8,
        P_RETURN_DELAY_TIME         = 9,
        P_DRIVE_MODE                = 10,
        P_OPERATING_MODE            = 11,
        P_SECONDARY_ID              = 12,
        P_PROTOCOL_VERSION          = 13,
        P_HOMING_OFFSET             = 20,  //4 BYTE
        P_MOVING_THRESHOLD          = 24,  //4 BYTE
        P_HIGH_LIMIT_TEMPERATURE    = 31,
        P_HIGH_LIMIT_VOLTAGE_L      = 32,
        P_HIGH_LIMIT_VOLTAGE_H      = 33,
        P_LOW_LIMIT_VOLTAGE_L       = 34,
        P_LOW_LIMIT_VOLTAGE_H       = 35,
        P_PWM_LIMIT_L               = 36,
        P_PWM_LIMIT_H               = 37,
        P_CURRENT_LIMIT_L           = 38,
        P_CURRENT_LIMIT_H           = 39,
        P_ACCELERATION_LIMIT        = 40,  //4 BYTE
        P_VELOCITY_LIMIT            = 44,  //4 BYTE
        P_MAX_POSITION_LIMIT        = 48,  //4 BYTE
        P_MIN_POSITION_LIMIT        = 52,  //4 BYTE
        P_SHUTDOWN                  = 63,
        P_TORQUE_ENABLE             = 64,
        P_LED                       = 65,
        P_STATUS_RETURN_LEVEL       = 68,
        P_REGISTERED_INSTRUCTION    = 69,
        P_HARDWARE_ERROR_STATUS     = 70,
        P_VELOCITY_I_GAIN_L         = 76,
        P_VELOCITY_I_GAIN_H         = 77,
        P_VELOCITY_P_GAIN_L         = 78,
        P_VELOCITY_P_GAIN_H         = 79,
        P_POSITION_D_GAIN_L         = 80,
        P_POSITION_D_GAIN_H         = 81,
        P_POSITION_I_GAIN_L         = 82,
        P_POSITION_I_GAIN_H         = 83,
        P_POSITION_P_GAIN_L         = 84,
        P_POSITION_P_GAIN_H         = 85,
        P_FEEDFORWARD_2ND_GAIN_L    = 88,
        P_FEEDFORWARD_2ND_GAIN_H    = 89,
        P_FEEDFORWARD_1ST_GAIN_L    = 90,
        P_FEEDFORWARD_1ST_GAIN_H    = 91,
        P_BUS_WATCHDOG              = 98,
        P_GOAL_PWM_L                = 100,
        P_GOAL_PWM_H                = 101,
        P_GOAL_CURRENT_L            = 102,
        P_GOAL_CURRENT_H            = 103,
        P_GOAL_VELOCITY             = 104,  //4 BYTE
        P_PROFILE_ACCELERATION      = 108,  //4 BYTE
        P_PROFILE_VELOCITY          = 112,  //4 BYTE
        P_GOAL_POSITION             = 116,  //4 BYTE
        P_MOVING                    = 122,
        P_MOVING_STATUS             = 123,
        P_PRESENT_PWM_L             = 124,
        P_PRESENT_PWM_H             = 125,
        P_PRESENT_CURRENT_L         = 126,
        P_PRESENT_CURRENT_H         = 127,
        P_PRESENT_VELOCITY          = 128,  //4 BYTE
        P_PRESENT_POSITION          = 132,  //4 BYTE
        P_PRESENT_INPUT_VOLTAGE     = 144,
        P_PRESENT_TEMPERATURE       = 146,
        MAXNUM_ADDRESS
    };

    enum operateMode
    {
        CURRENT_CONTROLL                    = 0,
        VELOCITY_CONTROLL                   = 1,
        POSITION_CONTROLL                   = 3,
        EXTENDED_POSITION_CONTROLL          = 4,
        CURRENT_BASED_POSITION_CONTROLL     = 5,
        PWM_CONTROLL                        = 16,
    };

    double curPos;
    double goalPos;
    double curVel;
    double curTorque;
    double goalTorque;

    uint8_t dxl_error = 0;                           // Dynamixel error
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    dynamixel::GroupBulkRead* groupBulkRead;


    static MX64 *getInstance(const char *port_name);

    void getHandlerAndOpenPort(const char *port_name);

    void setEnable(bool enable);
    bool getEnable();

    void setBaudrate(int baudrate);
    int getBaudrate();

    void setReturnDelayTime(int delay);
    int getReturnDelayTime();

    void setTemperatureLimit(int t);

    void setPositionPGain(int pgain);
    int  getPositionPGain();
    void setPositionIGain( int igain);
    int  getPositionIGain();
    void setPositionDGain(int dgain);
    int  getPositionDGain();

    void setOperateMode(operateMode mode);

    void setGoalPositon(double angle);
    void getCurrentPositon();
    void setGoalVel(double vel);
    double getCurrentVel();


    void setGoalTorque(double tor);
    double getCurrentTorque();

    void getSensorData();


private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    struct  timeval    tv,last_tv;
};


}

#endif