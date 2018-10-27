#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
// #include "Eigen.h"
#include "UR10Controller.h"
// #include "EigenUtil.h"

using namespace ljnoid;

namespace SimControll
{
	extern ros::Publisher simStart_pub;
	extern ros::Publisher simStop_pub;
	extern ros::Publisher simPause_pub;
	extern ros::Publisher simEnSync_pub;
	extern ros::Publisher simTrigNext_pub;
	extern ros::Publisher jointCmd_pub;
	extern ros::Subscriber jointAngle_sub;

	extern  bool stepDone;   //一步仿真完成标志
	extern std_msgs::Bool simCtr;


	extern std_msgs::Float64MultiArray jointCmd;


	extern Eigen::Matrix<double, UR10Controller::JOINT_NUM, 1> jointAngle,jointVel;
	extern Eigen::Matrix<double, UR10Controller::JOINT_NUM, 1> jointTorque;
	extern Eigen::Matrix<double, 6, 1> endFT;


	void simInit();

	void simControll(Eigen::Matrix<double, UR10Controller::JOINT_NUM, 1> jointValue);
}


