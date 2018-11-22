#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
// #include "Eigen.h"
#include "LjhnController.h"
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
	extern std_msgs::Float64MultiArray footCop;


	extern Eigen::Matrix<double, LjhnController::JOINT_NUM, 1> jointAngle,jointVel;
	extern Eigen::Matrix<double, LjhnController::FLOATING_CONFIG_NUM, 1> basePos;//quaternion[w x y z]  translation[x y z]
	extern Eigen::Matrix<double, 6, 1> leftFT;//torque[x y z] force[x y z]
	extern Eigen::Matrix<double, 6, 1> rightFT;//torque[x y z] force[x y z]


	void simInit();

	void simControll(Eigen::Matrix<double, LjhnController::JOINT_NUM, 1> jointValue);
}


