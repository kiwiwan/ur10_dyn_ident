// #include <ros/ros.h>
// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Int32.h>
// #include <sensor_msgs/Imu.h>
// #include "Eigen.h"
#include "SimControll.h"



namespace SimControll
{
	ros::Subscriber simStepDone_sub;
	ros::Subscriber simState_sub;
	ros::Publisher simStart_pub;
	ros::Publisher simStop_pub;
	ros::Publisher simPause_pub;
	ros::Publisher simEnSync_pub;
	ros::Publisher simTrigNext_pub;
	ros::Publisher jointCmd_pub;


	ros::Subscriber jointAngle_sub;
	ros::Subscriber jointVel_sub;
	ros::Subscriber endFT_sub;
	ros::Subscriber jointTorque_sub;


	bool stepDone;   //一步仿真完成标志
	std_msgs::Bool simCtr;


	std_msgs::Float64MultiArray jointCmd;


	Eigen::Matrix<double, UR10Controller::JOINT_NUM, 1> jointAngle,jointVel;
	Eigen::Matrix<double, UR10Controller::JOINT_NUM, 1> jointTorque;
	Eigen::Matrix<double, 6, 1> endFT;



	void simStateCallback(const std_msgs::Int32::ConstPtr& state)
	{
		// std_msgs::Bool start;
		// std::cout<<"here:" << std::endl;
		// if(state->data == 0)
		// {
		// 	start.data=1;
		// 	start_pub.publish(start);
		// }
	}

	void simStepDoneCallback(const std_msgs::Bool::ConstPtr& done)
	{
		stepDone=true;
	}

	void jointAngleCallback(const std_msgs::Float64MultiArray::ConstPtr& joint)
	{
		for(int i = 0; i < 6; ++i)
			jointAngle[i]=joint->data[i];
		// std::cout<< jointAngle[2] <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}

	void jointVelCallback(const std_msgs::Float64MultiArray::ConstPtr& vel)
	{
		for(int i = 0; i < 6; ++i)
			jointVel[i]=vel->data[i];
		// std::cout<< jointSensor[1] <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}

	void endFTCallback(const std_msgs::Float64MultiArray::ConstPtr& force)
	{
		for(int i = 0; i < 6; ++i)
			endFT[i]=force->data[i];
		// std::cout<< endFT <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}

	void jointTorqueCallback(const std_msgs::Float64MultiArray::ConstPtr& torq)
	{
		for(int i = 0; i < 6; ++i)
			jointTorque[i]=torq->data[i];
		// std::cout<< jointTorque <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}


	
	void simInit()
	{
	    ros::NodeHandle node;

	    simStepDone_sub=node.subscribe<std_msgs::Bool>("simulationStepDone",1,&simStepDoneCallback);
		simState_sub=node.subscribe<std_msgs::Int32>("simulationState",1,&simStateCallback);
		simStart_pub=node.advertise<std_msgs::Bool>("startSimulation",1);
		simStop_pub=node.advertise<std_msgs::Bool>("stopSimulation",1);
		simPause_pub=node.advertise<std_msgs::Bool>("pauseSimulation",1);
		simEnSync_pub=node.advertise<std_msgs::Bool>("enableSyncMode",1);
		simTrigNext_pub=node.advertise<std_msgs::Bool>("triggerNextStep",1);
	    jointCmd_pub=node.advertise<std_msgs::Float64MultiArray>("joint_command",1);


		jointAngle_sub=node.subscribe<std_msgs::Float64MultiArray>("joint_angle",10,&jointAngleCallback);
		jointVel_sub=node.subscribe<std_msgs::Float64MultiArray>("joint_velocity",10,&jointVelCallback);
		endFT_sub=node.subscribe<std_msgs::Float64MultiArray>("endFT",10,&endFTCallback);
		jointTorque_sub=node.subscribe<std_msgs::Float64MultiArray>("joint_torque",10,&jointTorqueCallback);

	    while(simStart_pub.getNumSubscribers() <= 0 ||simEnSync_pub.getNumSubscribers() <= 0 ||simTrigNext_pub.getNumSubscribers() <= 0);   //等待发布者与接收者建立连接
		
	}

	void simControll(Eigen::Matrix<double, UR10Controller::JOINT_NUM, 1> jointValue)//
	{
	    for(unsigned i=0;i<6;i++)
		{
			jointCmd.data[i]=jointValue[i];
		}
	   
		stepDone=false;
		simTrigNext_pub.publish(simCtr);   //vrep仿真一步  //大约要150ms才能收到simulationStepDone消息
		while(jointAngle_sub.getNumPublishers () <=0);
		while(endFT_sub.getNumPublishers () <=0);
		while(jointVel_sub.getNumPublishers () <=0);
		while(jointTorque_sub.getNumPublishers () <=0);
		while(jointCmd_pub.getNumSubscribers() <= 0);
		jointCmd_pub.publish(jointCmd);
		while(stepDone == false && ros::ok())  ros::spinOnce();
	}
}

