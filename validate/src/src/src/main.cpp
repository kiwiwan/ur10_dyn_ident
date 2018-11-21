/**
   \file
   \author 
*/


#include "main.h"
#include <time.h>
#include <signal.h>

#include <math.h>


// // #include "Coriolis.h"

// // #include "FD.h"
// #include "FK.h"
// #include "FV.h"
// #include "FA.h"
// #include "IK.h"
// // #include "CoM.h"
// // #include "Momentum.h"
// // #include "EulerIntegration.h"
// #include "ID.h"
// // #include "IDIM.h"

// // #include <iostream>

// // // RBDyn
// // #include "Body.h"
// // #include "Joint.h"
// // #include "MultiBody.h"
// #include "MultiBodyConfig.h"
// // #include "MultiBodyGraph.h"

// #include "Conversions.h"
#include "Util.h"

using namespace std;
using namespace ljnoid;
using namespace MX64_V2;


int main(int argc, char **argv)
{
	ros::init(argc,argv,"ros_node");


	advertise();
	call_count = 0;

	mx64 = MX64::getInstance("/dev/ttyUSB0");

	// int i = 0;
	// while(i < 1000)   //return to zero point
	// {
	// 	mx64->setGoalPositon(0);
	// 	i++;
	// }
	
	// readExciteTraj();

	initTimer();


	while(ros::ok())
	{
		usleep(100*1000);
		char cmd = getchar();
		if (cmd == 'q')
		{
			mx64->setEnable(false);
			mx64->setBaudrate(57600);
			exit(0);
		}
	}
	




    return 0;
}




// struct  timeval    tv,last_tv;
void timerCallback(int in)
{

	//  double t;
	// gettimeofday(&last_tv,NULL);

	//  double t;
	// gettimeofday(&tv,NULL);

	// t=tv.tv_sec*1000000+tv.tv_usec-(last_tv.tv_sec*1000000+last_tv.tv_usec);
	// // printf("tv_sec:%f\n",t);
	// last_tv=tv;

	// trajExcite();

	if(call_count < 5000)
	{
		double pos = 90*sin(PI*(cos(call_count*PI/200+PI)+1))*PI/180.;//0;//q_excit[call_count];//
		torqueControll(pos);
	}
	
	// if(call_count <= 5000)
	// {
	// 	mx64->goalPos = 90*sin(PI*(cos(call_count*PI/200+PI)+1));
	// 	mx64->setGoalPositon(mx64->goalPos);
	// 	mx64->getSensorData();
	// }
	
	debug_publish();

	call_count++;

	// gettimeofday(&tv,NULL);
	// t=tv.tv_sec*1000000+tv.tv_usec-(last_tv.tv_sec*1000000+last_tv.tv_usec);
	// if(t > 7000)
	// 	printf("tv_sec:%f\n",t);
}

void initTimer()
{
	timer_t timer;
	struct sigevent evp;
	struct itimerspec ts;
	memset(&evp, 0, sizeof(evp));
	evp.sigev_notify = SIGEV_SIGNAL;
	evp.sigev_signo = SIGUSR1;
	signal(SIGUSR1, timerCallback);
	timer_create(CLOCK_REALTIME, &evp, &timer);
	ts.it_interval.tv_sec = 0;
	ts.it_interval.tv_nsec = timeStep*1000*1000*1000;//20*1000*1000
	// ts.it_interval.tv_nsec = 2*1000*1000;//for vrep simulation

	ts.it_value.tv_sec = 0;
	ts.it_value.tv_nsec = 10;
	timer_settime(timer, 0, &ts, NULL);
}

void torqueControll(double pos)
{
	mx64->goalPos = pos*TO_DEGREE;
	mx64->getSensorData();
	mx64->goalTorque = modelJointTrajControll();
	mx64->setGoalTorque(mx64->goalTorque);


}

void trajExcite()
{
	mx64->getSensorData();

	if(call_count < q_excit_count)
	{
		// mx64->goalPos = q_excit[call_count]*TO_DEGREE;
		// mx64->setGoalPositon(mx64->goalPos);

		mx64->goalPos = q_excit[call_count]*TO_DEGREE;
		mx64->goalTorque = modelJointTrajControll();
		mx64->setGoalTorque(mx64->goalTorque);
	}
	// printf("timeCount= %d\n",mWalk.timeCount);


    	

	if(call_count > 250 && call_count <= q_excit_count)
	mx64_traj_results	<<"  "<<mx64->curPos*TO_RADIAN<<",  "<<mx64->curVel*TO_RADIAN
						<<",  "<<mx64->curTorque 		
						<<endl;
}

void readExciteTraj()
{
	ifstream trajFile("/home/kiwi/soft/learn/IDIM/sausa/ur10_dyn_ident/data/trajectories/mx64_traj2.csv", ios::in);

	string lineStr;
	int i = 0;
	while (getline(trajFile, lineStr))
	{
		stringstream ss(lineStr);
		string str;
		vector<string> lineArray;

		
		getline(ss, str, ',');

		istringstream iss(str);
		iss >> q_excit[i];
		// cout << q_excit[i] << endl;
		i++;

		
		
	}
	q_excit_count = i;
	// cout << q_excit_count << endl;

	mx64_traj_results.open("/home/kiwi/soft/learn/IDIM/sausa/ur10_dyn_ident/data/recdata/mx64_traj2_results.csv", ios::out);
}


// model-base joint controll
double modelJointTrajControll()
{
	//not rod
	double L_1xx    =    0.0;
	double L_1xy    =    0.0;
	double L_1xz    =    0.0;
	double L_1yy    =    0.0;
	double L_1yz    =    0.0;
	double L_1zz    =    0.00307866691760475;
	double l_1x    =    -0.0009926558066106653;
	double l_1y    =    0.0009010195275339221;
	double l_1z    =    0.0;
	double m_1    =    0.0;
	double Ia_1    =    0.00307866691760475;
	double fv_1    =    0.051778085492912435;
	double fc_1    =    0.014173811653391058;
	double fo_1    =    -0.013180292286075014;

	// //rod position
	// double L_1xx    =    0.0;
	// double L_1xy    =    0.0;
	// double L_1xz    =    0.0;
	// double L_1yy    =    0.0;
	// double L_1yz    =    0.0;
	// double L_1zz    =    0.0004462486742905737;
	// double l_1x    =    0.00044303093429114924;
	// double l_1y    =    -0.008881753670799363;
	// double l_1z    =    0.0;
	// double m_1    =    0.0;
	// double Ia_1    =    0.0004462486742905737;
	// double fv_1    =    0.013037716175157945;
	// double fc_1    =    0.08064649035951603;
	// double fo_1    =    0.003962271197137713;


	// // //rod torque
	// double L_1xx    =    0.0;
	// double L_1xy    =    0.0;
	// double L_1xz    =    0.0;
	// double L_1yy    =    0.0;
	// double L_1yz    =    0.0;
	// double L_1zz    =    0.003109900136909342;
	// double l_1x    =    0.00043356208022872483;
	// double l_1y    =    -0.013987717373923505;
	// double l_1z    =    0.0;
	// double m_1    =    0.0;
	// double Ia_1    =    0.003109900136909342;
	// double fv_1    =    0.003969420734859417;
	// double fc_1    =    0.08492773985115729;
	// double fo_1    =    -0.011732120955107452;

	// // //rod cad  and friction identify
	// double L_1xx    =    0.0;
	// double L_1xy    =    0.0;
	// double L_1xz    =    0.0;
	// double L_1yy    =    0.0;
	// double L_1yz    =    0.0;
	// double L_1zz    =    0.005513919;
	// double l_1x    =    0.0;
	// double l_1y    =    -0.01475;
	// double l_1z    =    0.0;
	// double m_1    =    0.0;
	// double Ia_1    =    0.002138725039663255060;
	// double fv_1    =    0.05257446069515255699;
	// double fc_1    =    0.01304203703264694356;
	// double fo_1    =    -0.009416510700965186401;


	double kp=500;  		//500
	double kd=kp/15;  	//kp/15

	double kp1=8;       	//8
	double kd1=kp1/20.0;	//kp1/10.0

	double q,qref;
	double dq,dqref;
	double ddq,ddqref;
	// double timeStep = 0.01;


	q=mx64->curPos*TO_RADIAN;
	dq=(q-qold)/timeStep;
	ddq=(dq-dqold)/timeStep;
	qold=q;
	dqold=dq;


	qref=mx64->goalPos*TO_RADIAN;
	dqref=(qref-qrefOld)/timeStep;
	ddqref=(dqref-dqrefOld)/timeStep;
	qrefOld=qref;
	dqrefOld=dqref;

	double err,errV;
	err=qref-q;
	errV=dqref-dq;//SimControll::jointVel[i];
	double uPD=ddqref+kp*err+kd*errV;  //servo-base part
	
 
	double C =  dq*fv_1 + fc_1*sign(dq) + fo_1 - 9.81*l_1x*cos(q) + 9.81*l_1y*sin(q);
	double jointCommand= (Ia_1 + L_1zz)*uPD+C;//Bq*uPD+c_qdq+gq;  model-base part

	// double jointCommand=kp1*err+kd1*errV;  //servo-base part

	// jointCommand = -jointCommand;
	if(jointCommand > 6.0) 
		jointCommand = 6.0;
	else if(jointCommand < -6.0)
		jointCommand = -6.0;
	// printf("jointCommands:\n %10f\n",jointCommand);

	return jointCommand;

}




void debug_publish()
{
	std_msgs::Float64 msg_val;

	msg_val.data = mx64->goalPos;
	goalPos_pub.publish(msg_val);

	msg_val.data = mx64->curPos;
	curPos_pub.publish(msg_val);

	msg_val.data = mx64->curVel;
	curVel_pub.publish(msg_val);

	msg_val.data = mx64->goalTorque;
	goalTorque_pub.publish(msg_val);

	msg_val.data = mx64->curTorque;
	curTorque_pub.publish(msg_val);

	// std_msgs::Float64MultiArray msg_arr;
	// msg_arr.data.resize(UR10Controller::JOINT_NUM);
	// for(int i = 0; i < 6; ++i)
	// 		msg_arr.data[i]=onedofCon->q(i);
	// q_pub.publish(msg_arr);	
}

void advertise()
{
	ros::NodeHandle n;
	// q_pub =n.advertise<std_msgs::Float64>("q",1000);


	goalPos_pub =n.advertise<std_msgs::Float64>("goalPos",1000);
	curPos_pub =n.advertise<std_msgs::Float64>("curPos",1000);
	curVel_pub =n.advertise<std_msgs::Float64>("curVel",1000);
	goalTorque_pub =n.advertise<std_msgs::Float64>("goalTorque",1000);
	curTorque_pub =n.advertise<std_msgs::Float64>("curTorque",1000);

}






















