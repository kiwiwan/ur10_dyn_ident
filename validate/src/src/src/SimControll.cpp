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
	ros::Publisher footCop_pub;


	ros::Subscriber jointAngle_sub;
	ros::Subscriber jointVel_sub;
	ros::Subscriber basePos_sub;
	ros::Subscriber leftFT_sub;
	ros::Subscriber rightFT_sub;


	bool stepDone;   //一步仿真完成标志
	std_msgs::Bool simCtr;


	std_msgs::Float64MultiArray jointCmd;
	std_msgs::Float64MultiArray footCop;


	Eigen::Matrix<double, LjhnController::JOINT_NUM, 1> jointAngle,jointVel;
	Eigen::Matrix<double, LjhnController::FLOATING_CONFIG_NUM, 1> basePos;//quaternion[w x y z]  translation[x y z]
	Eigen::Matrix<double, 6, 1> leftFT;//torque[x y z] force[x y z]
	Eigen::Matrix<double, 6, 1> rightFT;//torque[x y z] force[x y z]



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
		for(int i = 0; i < LjhnController::JOINT_NUM; ++i)
			jointAngle[i]=joint->data[i];
		// std::cout<< jointAngle[2] <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}

	void jointVelCallback(const std_msgs::Float64MultiArray::ConstPtr& vel)
	{
		for(int i = 0; i < LjhnController::JOINT_NUM; ++i)
			jointVel[i]=vel->data[i];
		// std::cout<< jointSensor[1] <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}


	void basePosCallback(const std_msgs::Float64MultiArray::ConstPtr& pos)
	{
		for(int i = 0; i < LjhnController::FLOATING_CONFIG_NUM; ++i)
			basePos[i]=pos->data[i];
		// std::cout<< basePos <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}

	void leftFTCallback(const std_msgs::Float64MultiArray::ConstPtr& force)
	{
		for(int i = 0; i < 6; ++i)
			leftFT[i]=force->data[i];
		// std::cout<< leftFT <<std::endl;
		// std::cout<< "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" <<std::endl;
	}

	void rightFTCallback(const std_msgs::Float64MultiArray::ConstPtr& force)
	{
		for(int i = 0; i < 6; ++i)
			rightFT[i]=force->data[i];
		// std::cout<< rightFT <<std::endl;
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
	    footCop_pub=node.advertise<std_msgs::Float64MultiArray>("foot_cop",1);


		jointAngle_sub=node.subscribe<std_msgs::Float64MultiArray>("joint_angle",10,&jointAngleCallback);
		jointVel_sub=node.subscribe<std_msgs::Float64MultiArray>("joint_velocity",10,&jointVelCallback);
		basePos_sub=node.subscribe<std_msgs::Float64MultiArray>("base_pos",10,&basePosCallback);
		leftFT_sub=node.subscribe<std_msgs::Float64MultiArray>("left_FT",10,&leftFTCallback);
		rightFT_sub=node.subscribe<std_msgs::Float64MultiArray>("right_FT",10,&rightFTCallback);

	    while(simStart_pub.getNumSubscribers() <= 0 ||simEnSync_pub.getNumSubscribers() <= 0 ||simTrigNext_pub.getNumSubscribers() <= 0);   //等待发布者与接收者建立连接
		
	    jointCmd.data.resize(LjhnController::JOINT_NUM);
		footCop.data.resize(2);
		simCtr.data=1;                  //仿真控制变量
		simEnSync_pub.publish(simCtr);  //开启vrep同步模式
		simStart_pub.publish(simCtr);  //开始vrep仿真

	}

	void simControll(Eigen::Matrix<double, LjhnController::JOINT_NUM, 1> jointValue)//
	{
	    for(int i = 0; i < LjhnController::JOINT_NUM; ++i)
		{
			jointCmd.data[i]=jointValue[i];
		}

		footCop.data[0]=LjhnController::GetInstance()->pl[0];
		footCop.data[1]=LjhnController::GetInstance()->pl[1];
	   
		stepDone=false;
		simTrigNext_pub.publish(simCtr);   //vrep仿真一步  //大约要150ms才能收到simulationStepDone消息
		while(jointAngle_sub.getNumPublishers () <=0);
		while(basePos_sub.getNumPublishers () <=0);
		while(leftFT_sub.getNumPublishers () <=0);
		// while(rightFT_sub.getNumPublishers () <=0);
		while(jointCmd_pub.getNumSubscribers() <= 0);
		while(footCop_pub.getNumSubscribers() <= 0);
		jointCmd_pub.publish(jointCmd);
		footCop_pub.publish(footCop);
		while(stepDone == false && ros::ok())  ros::spinOnce();
	}
}

