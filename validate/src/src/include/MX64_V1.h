
#ifndef _MX_64_V1_H_
#define _MX_64_V1_H_

#include "dynamixel_sdk.h" 



namespace MX64_V1
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
    static const double RATIO_VALUE2TORQUE;
    static const double RATIO_TORQUE2VALUE;

    static const int PARAM_BYTES;

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
        if(tor > 6.138 || tor < -6.138)
            {printf("torque command too large!\n");return 0;}

        if(tor >= 0)
            return (int)(tor*RATIO_TORQUE2VALUE);
        else if(tor < 0)
            return (int)((-tor*RATIO_TORQUE2VALUE)+1024);
    }
    static double value2Torque(int value) {
        if(value < 1024)
            return (double)(value*RATIO_VALUE2TORQUE);
        else if(value > 1023)
            return (double)((1024-value)*RATIO_VALUE2TORQUE);
    }

    static double value2Vel(int value) {  //degree/s
        if(value < 1024)
            return (double)(value*0.66);
        else if(value > 1023)
            return (double)((1024-value)*0.66);
    }

    // Address  protocol 1.0
    enum controlTableAdress
    {
        P_MODEL_NUMBER_L            = 0,
        P_MODEL_NUMBER_H            = 1,
        P_VERSION                   = 2,
        P_ID                        = 3,
        P_BAUD_RATE                 = 4,
        P_RETURN_DELAY_TIME         = 5,
        P_CW_ANGLE_LIMIT_L          = 6,
        P_CW_ANGLE_LIMIT_H          = 7,
        P_CCW_ANGLE_LIMIT_L         = 8,
        P_CCW_ANGLE_LIMIT_H         = 9,
        P_SYSTEM_DATA2              = 10,
        P_HIGH_LIMIT_TEMPERATURE    = 11,
        P_LOW_LIMIT_VOLTAGE         = 12,
        P_HIGH_LIMIT_VOLTAGE        = 13,
        P_MAX_TORQUE_L              = 14,
        P_MAX_TORQUE_H              = 15,
        P_RETURN_LEVEL              = 16,
        P_ALARM_LED                 = 17,
        P_ALARM_SHUTDOWN            = 18,
        P_TORQUE_ENABLE             = 24,
        P_LED                       = 25,
        P_D_GAIN                    = 26,
        P_I_GAIN                    = 27,
        P_P_GAIN                    = 28,
        P_GOAL_POSITION_L           = 30,
        P_GOAL_POSITION_H           = 31,
        P_MOVING_SPEED_L            = 32,
        P_MOVING_SPEED_H            = 33,
        P_TORQUE_LIMIT_L            = 34,
        P_TORQUE_LIMIT_H            = 35,
        P_PRESENT_POSITION_L        = 36,
        P_PRESENT_POSITION_H        = 37,
        P_PRESENT_SPEED_L           = 38,
        P_PRESENT_SPEED_H           = 39,
        P_PRESENT_LOAD_L            = 40,
        P_PRESENT_LOAD_H            = 41,
        P_PRESENT_VOLTAGE           = 42,
        P_PRESENT_TEMPERATURE       = 43,
        P_REGISTERED_INSTRUCTION    = 44,
        P_PAUSE_TIME                = 45,
        P_MOVING                    = 46,
        P_LOCK                      = 47,
        P_PUNCH_L                   = 48,
        P_PUNCH_H                   = 49,
        P_CURRENT_L                 = 68,
        P_CURRENT_H                 = 69,
        P_TORQUE_CTRL_ENABLE        = 70,
        P_GOAL_TORQUE_L             = 71,
        P_GOAL_TORQUE_H             = 72,
        P_GOAL_ACCELERATION         = 73,
        MAXNUM_ADDRESS
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

    void setPGain(int pgain);
    int  getPGain();
    void setIGain( int igain);
    int  getIGain();
    void setDGain(int dgain);
    int  getDGain();

    void setGoalPositon(double angle);
    void getCurrentPositon();
    double getVel();

    void setTorqueCtrlEnable(bool enable);
    bool getTorqueCtrlEnable();

    void setGoalTorque(double tor);
    double getCurrentTorque();

    void getSensorData();


private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;


};


}


#endif