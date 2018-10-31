/**
   \file
   \author 
*/


#include "main.h"
#include <time.h>



// #include "Coriolis.h"

// #include "FD.h"
#include "FK.h"
#include "FV.h"
#include "FA.h"
#include "IK.h"
// #include "CoM.h"
// #include "Momentum.h"
// #include "EulerIntegration.h"
#include "ID.h"
// #include "IDIM.h"

// #include <iostream>

// // RBDyn
// #include "Body.h"
// #include "Joint.h"
// #include "MultiBody.h"
#include "MultiBodyConfig.h"
// #include "MultiBodyGraph.h"

#include "Conversions.h"
#include "Util.h"

using namespace std;
using namespace ljnoid;




int main(int argc, char **argv)
{
	ros::init(argc,argv,"ros_node");


	advertise();

	onedofCon=OnedofController::getInstance();

	



	SimControll::simInit();	
	SimControll::jointCmd.data.resize(UR10Controller::JOINT_NUM);
	SimControll::simCtr.data=1;                  //仿真控制变量
	SimControll::simEnSync_pub.publish(SimControll::simCtr);  //开启vrep同步模式
	SimControll::simStart_pub.publish(SimControll::simCtr);  //开始vrep仿真



	

	pthread_t thread_LIPM;
	pthread_create(&thread_LIPM, NULL, control_thread, NULL);


	while(ros::ok())
	{
		usleep(100*1000);
		char cmd = getchar();
		if (cmd == 'q')
		{
			exit(0);
		}
	}
	




    return 0;
}




void* control_thread(void* ptr)
{
	rbd::InverseDynamics onedofIDy(onedofCon->onedof);
	rbd::forwardKinematics(onedofCon->onedof, onedofCon->onedofmbc);
	rbd::forwardVelocity(onedofCon->onedof,onedofCon->onedofmbc);
	rbd::forwardAcceleration(onedofCon->onedof,onedofCon->onedofmbc);
	onedofCon->onedofmbc.gravity=Eigen::Vector3d(0.,0.,9.81);
	onedofIDy.inverseDynamics(onedofCon->onedof,onedofCon->onedofmbc);
	printf("init jointTorques:\n %10f\n",onedofCon->onedofmbc.jointTorque[1][0]);
// exit(0);	
	for(int j=0;j<2;j++)    //wait the sensor value norm
	{
		onedofCon->jointCommand=sDofToVector(onedofCon->onedof,onedofCon->onedofmbc.jointTorque);
		SimControll::simControll(onedofCon->jointCommand);  
	}


	


	
	int count=0;

	
	while(true)
	{


		if(count <= 3240)
		{
			Eigen::Matrix<double, 1, 1> q;
			q(0)=q_excit[count];
			// q(0)=90*sin(PI*(cos(count*PI/50+PI)+1))*PI/180.;//sin(2*count*PI/90);//2*PI*
			onedofCon->onedofmbc.q=rbd::sVectorToParam(onedofCon->onedof,q);

			// tarPos.translation().y()=y0+0.25*sin(count*PI/180); //linear increase

			// tarPos.translation().z()=z0+0.25*cos(PI*(cos(count*PI/90+PI)+1));

		}
		



		onedofCon->modelJointTrajControll();


		SimControll::simControll(onedofCon->jointCommand);  



		count++;
	}
}


void debug_publish()
{
	std_msgs::Float64 msg_val;

	// msg_val.data = onedofCon->q;//mWalk.FB_GYRO;
	// q_pub.publish(msg_val);


	// std_msgs::Float64MultiArray msg_arr;
	// msg_arr.data.resize(UR10Controller::JOINT_NUM);
	// for(int i = 0; i < 6; ++i)
	// 		msg_arr.data[i]=onedofCon->q(i);
	// q_pub.publish(msg_arr);	
	// for(int i = 0; i < 6; ++i)
	// 		msg_arr.data[i]=onedofCon->dq(i);
	// dq_pub.publish(msg_arr);
	// for(int i = 0; i < 6; ++i)
	// 		msg_arr.data[i]=onedofCon->ddq(i);
	// ddq_pub.publish(msg_arr);
}

void advertise()
{
	ros::NodeHandle n;
	// q_pub =n.advertise<std_msgs::Float64>("q",1000);


	q_pub =n.advertise<std_msgs::Float64MultiArray>("q",1000);
	dq_pub =n.advertise<std_msgs::Float64MultiArray>("dq",1000);
	ddq_pub =n.advertise<std_msgs::Float64MultiArray>("ddq",1000);
}






















