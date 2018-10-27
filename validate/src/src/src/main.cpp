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

	ur10Con=UR10Controller::getInstance();

	



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
	rbd::InverseDynamics ur10IDy(ur10Con->ur10);
	rbd::forwardKinematics(ur10Con->ur10, ur10Con->ur10mbc);
	rbd::forwardVelocity(ur10Con->ur10,ur10Con->ur10mbc);
	rbd::forwardAcceleration(ur10Con->ur10,ur10Con->ur10mbc);
	ur10Con->ur10mbc.gravity=Eigen::Vector3d(0.,0.,9.81);
	ur10IDy.inverseDynamics(ur10Con->ur10,ur10Con->ur10mbc);
	printf("init jointTorques:\n %10f %10f %10f %10f %10f %10f\n",ur10Con->ur10mbc.jointTorque[1][0],ur10Con->ur10mbc.jointTorque[2][0],ur10Con->ur10mbc.jointTorque[3][0],ur10Con->ur10mbc.jointTorque[4][0],ur10Con->ur10mbc.jointTorque[5][0],ur10Con->ur10mbc.jointTorque[6][0]);
	
	for(int j=0;j<2;j++)    //wait the sensor value norm
	{
		ur10Con->jointCommand=sDofToVector(ur10Con->ur10,ur10Con->ur10mbc.jointTorque);
		SimControll::simControll(ur10Con->jointCommand);  
	}


	


	
	int count=0;

	rbd::InverseKinematics ur10Ik(ur10Con->ur10,ur10Con->ur10.bodyIndexByName("UR10_Link6"));
	sva::PTransformd tarPos(ur10Con->ur10mbc.bodyPosW[6].rotation(),ur10Con->ur10mbc.bodyPosW[6].translation());
	double x0=ur10Con->ur10mbc.bodyPosW[6].translation().x();
	double y0=ur10Con->ur10mbc.bodyPosW[6].translation().y();
	double z0=ur10Con->ur10mbc.bodyPosW[6].translation().z()-0.25;
	// cout<< "x0: "<<endl<<x0 <<endl<<endl;
	// cout<< "y0: "<<endl<<y0 <<endl<<endl;
	// cout<< "z0: "<<endl<<z0 <<endl<<endl;
	while(true)
	{


		if(count < 90)
		{
			// tarPos.translation().y()=y0+0.25*sin(count*PI/180); //linear increase
			// tarPos.translation().z()=z0+0.25*cos(count*PI/180);
			// tarPos.translation().y()=y0+0.25*sin(PI*(cos(count*PI/90+PI)+1));  //cos acc and dec

			// tarPos.translation().x()=x0+0.25*sin(PI*(cos(count*PI/90+PI)+1));  //cos acc and dec
			tarPos.translation().y()=y0+0.25*sin(PI*(cos(count*PI/90+PI)+1));  //cos acc and dec
			tarPos.translation().z()=z0+0.25*cos(PI*(cos(count*PI/90+PI)+1));


			if(ur10Ik.inverseKinematics(ur10Con->ur10,ur10Con->ur10mbc,tarPos)) 
				;//printf("ik succeed.\n");
			else
				printf("ik failed!\n");
		}
		

// 		if(count > 359 && count <720)
// 		{
// 			if(count == 360) z0=z0+0.3;
// // cout<< "tarPos.translation().x()="<<endl<<tarPos.translation().x() <<endl<<endl;
// 			tarPos.translation().y()=y0-0.05*sin(PI*(cos((count-360)*PI/180/2+PI)+1));
// 			tarPos.translation().z()=z0-0.05*cos(PI*(cos((count-360)*PI/180/2+PI)+1));
// 			if(ur10Con->UR10JointPath.calcInverseKinematics(tarPos)) 
// 				;//printf("ik succeed.\n");
// 			else
// 				printf("ik failed!\n");
// 		}


		ur10Con->modelJointTrajControll();
		// ur10Con->modelCartesianTrajControll(tarPos);
		// ur10Con->impedanceControll(tarPos);
		// ur10Con->admittanceControll(tarPos);

		SimControll::simControll(ur10Con->jointCommand);  



		// if(count < 720)
		// {
		// 	Eigen::Matrix<double, UR10Controller::JOINT_NUM, 1> q,q0;
		// 	q0 << 0.*TO_RADIAN,0.*TO_RADIAN,0.*TO_RADIAN,0.*TO_RADIAN,0.*TO_RADIAN,0.*TO_RADIAN;
		// 	for(int i = 0; i < 6; ++i)
		// 	{
		// 		q(i)=PI*sin(PI*(cos(count*PI/720+PI)+1))/2+PI*sin(2*PI*(cos(count*PI/720+PI)+1))/2+q0(i);//+PI*sin(3*PI*(cos(count*PI/360+PI)+1))/2;
		// 		// q(i)=PI*sin(count*PI/180)/2+PI*sin(2*count*PI/180)/2;
		// 	}
			

		// 	ur10Con->ur10mbc.q=sVectorToParam(ur10Con->ur10,q);
		// 	// SimControll::simControll(q);

		// 	ur10Con->modelJointTrajControll();
		// 	SimControll::simControll(ur10Con->jointCommand); 
		// 	ur10Con->idDynPara();
		// 	debug_publish();
		// }
		

		// for(int i = 0; i < ur10Con->ur10.joints().size(); ++i)
		// {
		// 	// std::cout << "bodyPosW["<<i<<"]->p : " <<  std::endl<<  UR10mbc.bodyPosW[i].translation() << std::endl;
		// 	// std::cout << "bodyPosW["<<i<<"]->R : " <<  std::endl<<  UR10mbc.bodyPosW[i].rotation() << std::endl;
		 
		 	
		// 	// std::cout << "bodyVelB["<<i<<"] : " <<  std::endl<<  ur10Con->ur10mbc.bodyVelB[i] << std::endl;
		// 	// std::cout << "bodyPosW["<<i<<"] : " <<  std::endl<<  conversions::toHomogeneous(UR10mbc.bodyPosW[i]) << std::endl;
		// 	// std::cout << "bodyVelW["<<i<<"] : " <<  std::endl<<  UR10mbc.bodyVelW[i] << std::endl;
		// 	// std::cout << "bodyAccB["<<i<<"] : " <<  std::endl<<  ur10Con->ur10mbc.bodyAccB[i] << std::endl;
		// 	// std::cout << "bodyAccW["<<i<<"] : " <<  std::endl<<  UR10mbc.bodyAccW[i] << std::endl;


		// }



		count++;
	}
}


void debug_publish()
{
	std_msgs::Float64 msg_val;

	// msg_val.data = ur10Con->q;//mWalk.FB_GYRO;
	// q_pub.publish(msg_val);


	std_msgs::Float64MultiArray msg_arr;
	msg_arr.data.resize(UR10Controller::JOINT_NUM);
	for(int i = 0; i < 6; ++i)
			msg_arr.data[i]=ur10Con->q(i);
	q_pub.publish(msg_arr);	
	for(int i = 0; i < 6; ++i)
			msg_arr.data[i]=ur10Con->dq(i);
	dq_pub.publish(msg_arr);
	for(int i = 0; i < 6; ++i)
			msg_arr.data[i]=ur10Con->ddq(i);
	ddq_pub.publish(msg_arr);
}

void advertise()
{
	ros::NodeHandle n;
	// q_pub =n.advertise<std_msgs::Float64>("q",1000);


	q_pub =n.advertise<std_msgs::Float64MultiArray>("q",1000);
	dq_pub =n.advertise<std_msgs::Float64MultiArray>("dq",1000);
	ddq_pub =n.advertise<std_msgs::Float64MultiArray>("ddq",1000);
}






















