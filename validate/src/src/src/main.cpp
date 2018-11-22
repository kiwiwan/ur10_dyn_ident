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
// #include "MultiBodyConfig.h"
// #include "MultiBodyGraph.h"

#include "Conversions.h"
#include "Util.h"

using namespace std;
using namespace ljnoid;



int main(int argc, char **argv)
{
	ros::init(argc,argv,"ros_node");
	advertise();

	ljhnCon=LjhnController::GetInstance();
	

	// ljhnCon->outputDynParamFile();
	// exit(0);

	ljhnCon->simStart();
	

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
	rbd::InverseDynamics ljhnIDy(ljhnCon->ljhn);
	rbd::forwardKinematics(ljhnCon->ljhn, ljhnCon->ljhnmbc);
	rbd::forwardVelocity(ljhnCon->ljhn,ljhnCon->ljhnmbc);
	rbd::forwardAcceleration(ljhnCon->ljhn,ljhnCon->ljhnmbc);
	ljhnCon->ljhnmbc.gravity=Eigen::Vector3d(0.,0.,9.81);
	ljhnIDy.inverseDynamics(ljhnCon->ljhn,ljhnCon->ljhnmbc);
	printf("init leftLeg jointTorques:\n %10f %10f %10f %10f %10f %10f\n",ljhnCon->ljhnmbc.jointTorque[1][0],ljhnCon->ljhnmbc.jointTorque[2][0],ljhnCon->ljhnmbc.jointTorque[3][0],ljhnCon->ljhnmbc.jointTorque[4][0],ljhnCon->ljhnmbc.jointTorque[5][0],ljhnCon->ljhnmbc.jointTorque[6][0]);
	printf("init rightLeg jointTorques:\n %10f %10f %10f %10f %10f %10f\n",ljhnCon->ljhnmbc.jointTorque[8][0],ljhnCon->ljhnmbc.jointTorque[9][0],ljhnCon->ljhnmbc.jointTorque[10][0],ljhnCon->ljhnmbc.jointTorque[11][0],ljhnCon->ljhnmbc.jointTorque[12][0],ljhnCon->ljhnmbc.jointTorque[13][0]);
	
// std::cout << "bodyPosW[6] : " <<  std::endl<<  sva::conversions::toHomogeneous(ljhnCon->ljhnmbc.bodyPosW[6]) << std::endl;
// 	std::cout << "bodyPosW[13] : " <<  std::endl<<  sva::conversions::toHomogeneous(ljhnCon->ljhnmbc.bodyPosW[13]) << std::endl;
// exit(0);

	for(int j=0;j<2;j++)    //wait the sensor value norm
	{
		ljhnCon->jointCommand=sDofToVector(ljhnCon->ljhn,ljhnCon->ljhnmbc.jointTorque);
		SimControll::simControll(ljhnCon->jointCommand);  
	}

	// printf("leftLegCommands:\n %10f %10f %10f %10f %10f %10f\n",ljhnCon->jointCommand[0],ljhnCon->jointCommand[1],ljhnCon->jointCommand[2],ljhnCon->jointCommand[3],ljhnCon->jointCommand[4],ljhnCon->jointCommand[5]);
	// printf("rightLegCommands:\n %10f %10f %10f %10f %10f %10f\n",ljhnCon->jointCommand[6],ljhnCon->jointCommand[7],ljhnCon->jointCommand[8],ljhnCon->jointCommand[9],ljhnCon->jointCommand[10],ljhnCon->jointCommand[11]);

	// exit(0);

	
	int count=0;


	while(true)
	{

		if(count <= 400)
		{
			Eigen::Matrix<double, LjhnController::JOINT_NUM, 1> q = Eigen::Matrix<double, LjhnController::JOINT_NUM, 1>::Zero();
			// q = Eigen::Matrix<double, LjhnController::JOINT_NUM, 1>::Constant(0.);
			q = Eigen::Matrix<double, LjhnController::JOINT_NUM, 1>::Constant(1.)*90*sin(PI*(cos(count*PI/200+PI)+1))*PI/180.;
			// q.block<6,1>(6,0) = Eigen::Matrix<double, 6, 1>::Constant(1.)*90*sin(PI*(cos(count*PI/100+PI)+1))*PI/180.;
			// q(1,0) = 90*sin(PI*(cos(count*PI/100+PI)+1))*PI/180.;
			ljhnCon->ljhnmbc.q=rbd::sVectorToParam(ljhnCon->ljhn,q);
			// std::cout<< "q:\n"<<q <<std::endl<<std::endl;
		}
		
		// // //two leg torque command 
		ljhnCon->modelJointTrajControll();
		SimControll::simControll(ljhnCon->jointCommand); 

		 
		
		count++;
		printf("count=%i\n",count);
		

	}
}


void debug_publish()
{
	std_msgs::Float64 msg_val;

	msg_val.data = err[2]*180/PI;
	err_pub.publish(msg_val);
	msg_val.data = errV[2]*180/PI;
	errV_pub.publish(msg_val);

}

void advertise()
{
	ros::NodeHandle n;
	err_pub =n.advertise<std_msgs::Float64>("error",1000);
	errV_pub =n.advertise<std_msgs::Float64>("error_v",1000);
	

}






















