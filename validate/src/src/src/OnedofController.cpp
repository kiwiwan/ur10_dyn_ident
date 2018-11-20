#include <iostream>
#include "OnedofController.h"

#include "MultiBodyGraph.h"
#include "Conversions.h"
#include "Util.h"


#include "Coriolis.h"
#include "FD.h"
#include "FK.h"
#include "FV.h"
#include "FA.h"
#include "IK.h"
#include "IDIM.h"
// #include "Momentum.h"
#include "ID.h"


#include "SimControll.h"



using namespace ljnoid;
using namespace std;


OnedofController* OnedofController::m_UniqueInstance= new OnedofController();   //why   but delete will fault

OnedofController::OnedofController()
{

	timeStep=0.02;//s
	onedof_initRobot();

	for(int i = 0; i < JOINT_NUM; ++i)  //init some variate
	{
		uPD[i]=0;
		err[i]=0;
		errV[i]=0;
	}
	
	jointParaInit();

	double kp=1000;
	double kd=kp/20;

	Kp<< kp;
	Kd<< kd;

	// Kp<<  kp,0,0,0,0,0,
	// 	  0,kp,0,0,0,0,
	// 	  0,0,kp,0,0,0,
	// 	  0,0,0,kp,0,0,
	// 	  0,0,0,0,kp,0,
	// 	  0,0,0,0,0,kp;
	// Kd<<  kd,0,0,0,0,0,
	// 	  0,kd,0,0,0,0,
	// 	  0,0,kd,0,0,0,
	// 	  0,0,0,kd,0,0,
	// 	  0,0,0,0,kd,0,
	// 	  0,0,0,0,0,kd;
 

}



OnedofController::~OnedofController()
{


}



// model-base joint controll
void OnedofController::modelJointTrajControll()
{

   ////////compute H and C(c(q,qd) and g(q))
	rbd::ForwardDynamics fd(onedof);
	rbd::MultiBodyConfig mbcSensor(onedof);
	mbcSensor.zero(onedof);


	q=SimControll::jointAngle;
	dq=(q-qold)/timeStep;
	mbcSensor.q=sVectorToParam(onedof,q);
	mbcSensor.alpha=sVectorToDof(onedof,dq);
	qold=q;


	rbd::forwardKinematics(onedof, mbcSensor);
	rbd::forwardVelocity(onedof,mbcSensor);
	mbcSensor.gravity=Vector3d(0.,0.,9.81);
	fd.computeH(onedof, mbcSensor);
	fd.computeC(onedof, mbcSensor);
	
	

	qref=sParamToVector(onedof,onedofmbc.q);
	onedofmbc.alpha=sVectorToDof(onedof,(qref-qrefOld)/timeStep);
	dqref=sDofToVector(onedof,onedofmbc.alpha);
	onedofmbc.alphaD=sVectorToDof(onedof,(dqref-dqrefOld)/timeStep);

	err=qref-q;
	errV=dqref-dq;//SimControll::jointVel[i];
	uPD=sDofToVector(onedof,onedofmbc.alphaD)+Kp*err+Kd*errV;  //servo-base part
	

	qrefOld=qref;
	dqrefOld=dqref;


	jointCommand=fd.H()*uPD+fd.C();//Bq*uPD+c_qdq+gq;  model-base part

	printf("jointCommands:\n %10f\n",jointCommand[0]);

	// printf("jointCommands:\n %10f  %10f\n",jointCommand[0],fd.H()*sDofToVector(onedof,onedofmbc.alphaD)+fd.C());


}



void OnedofController::jointParaInit()
{


	onedofmbc.zero(onedof);
	onedofmbc.gravity=Vector3d(0.,0.,9.81);
	onedofmbc.q={{},{0.*TO_RADIAN}};
	// onedofmbc.alpha={{},{170.*TO_RADIAN}};
	// onedofmbc.alphaD={{},{43.*TO_RADIAN}};
	// onedofmbc.q={{},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN}};
	// onedofmbc.q={{},{30.*TO_RADIAN},{75.*TO_RADIAN},{130.*TO_RADIAN},{-50.*TO_RADIAN},{-160.*TO_RADIAN},{80.*TO_RADIAN}};
	// onedofmbc.alpha={{},{30.*TO_RADIAN},{75.*TO_RADIAN},{130.*TO_RADIAN},{-50.*TO_RADIAN},{-160.*TO_RADIAN},{80.*TO_RADIAN}};
	// onedofmbc.alphaD={{},{30.*TO_RADIAN},{75.*TO_RADIAN},{130.*TO_RADIAN},{-50.*TO_RADIAN},{-160.*TO_RADIAN},{80.*TO_RADIAN}};
	

	rbd::forwardKinematics(onedof, onedofmbc);

	qref=sParamToVector(onedof,onedofmbc.q);
	dqref=sDofToVector(onedof,onedofmbc.alpha);
	qrefOld=qref;
	qold=qref;
	dqrefOld=dqref;
	dqold=dqref;


}




// ///from vrep
// void OnedofController::onedof_initRobot()
// {
// 	rbd::MultiBodyGraph mbg;


// 	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

// 	double mass0=0.0;
// 	Eigen::Matrix3d linkI0;
// 	linkI0 << 0.,0.,0.,
// 			  0.,0.,0.,
// 			  0.,0.,0.;
// 	Eigen::Vector3d com0 =Eigen::Vector3d(0.,0.,0.);	  
// 	Eigen::Matrix3d I0=sva::inertiaToOrigin(linkI0,mass0, com0,iden3d);
// 	sva::RBInertiad rb0(mass0,mass0*com0,I0);
// 	rbd::Body Onedof_Link_Base(rb0,"Onedof_Link_Base");


// 	double mass1=2.;
// 	Eigen::Matrix3d linkI1;
// 	linkI1 << 2.731*mass1,0,0,
// 			  0,7.1*mass1,0,
// 			  0,0,1.42*mass1;
// 	Eigen::Vector3d com1 =Eigen::Vector3d(0.,0.,0.04);	
// 	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, com1,iden3d);	  
// 	sva::RBInertiad rb1(mass1,mass1*com1,I1);
// 	rbd::Body Onedof_Link1(rb1,"Onedof_Link1");


// 	std::cout<< "rb1:\n"<<rb1 <<std::endl<<std::endl;

// 	mbg.addBody(Onedof_Link_Base);
// 	mbg.addBody(Onedof_Link1);



// 	rbd::Joint j1(rbd::Joint::RevX,true,"joint1");


// 	mbg.addJoint(j1);



// 	sva::PTransformd to1(Eigen::Vector3d(0.,0.,0.));
// 	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("Onedof_Link_Base",to1,"Onedof_Link1",from1,"joint1");

	


// 	onedof=mbg.makeMultiBody("Onedof_Link_Base",true);

// 	onedofmbc=rbd::MultiBodyConfig(onedof);

// 	// jac=rbd::Jacobian(onedof,"Onedof_Link6");

// }


///sousa 2014 IROS result(wls)
void OnedofController::onedof_initRobot()
{
	rbd::MultiBodyGraph mbg;

	///sousa 2014 IROS result(wls)
	/////robot dynamic param in D-H coordinate
	////from using vrep joint pid controll excit and mesured 
	// double L_1xx    =    0.0;
	// double L_1xy    =    0.0;
	// double L_1xz    =    0.0;
	// double L_1yy    =    0.0;
	// double L_1yz    =    0.0;
	// double L_1zz    =    4.632148991886592;
	// double l_1x    =    -0.0006605847348112476;
	// double l_1y    =    -0.07980563043671064;
	// double l_1z    =    0.0;
	// double m_1    =    0.0;


	////from using RBDyn model control excit and mesured in vrep 
	double L_1xx    =    0.0;
	double L_1xy    =    0.0;
	double L_1xz    =    0.0;
	double L_1yy    =    0.0;
	double L_1yz    =    0.0;
	double L_1zz    =    5.464440794180246;
	double l_1x    =    -0.0004756487605652551;
	double l_1y    =    -0.07982580549261459;
	double l_1z    =    0.0;
	double m_1    =    0.0;



	/////convert robot dynamic param from D-H coordinate to my kinematic coordinate
	/////my kinematic coordinate coinside with D-H coordinate
	double mass0=0.0;
	Eigen::Matrix3d linkI0;
	linkI0 << 0.,0.,0.,
			  0.,0.,0.,
			  0.,0.,0.;
	Eigen::Matrix3d Irot0=Eigen::Matrix3d::Identity();	
	Eigen::Vector3d h0 =Irot0*Eigen::Vector3d(0.,0.,0.);  
	Eigen::Matrix3d I0=sva::inertiaToOrigin(linkI0,mass0, Eigen::Vector3d(0.,0.,0.),Irot0);
	sva::RBInertiad rb0(mass0,h0,I0);
	rbd::Body Onedof_Link_Base(rb0,"Onedof_Link_Base");


	double mass1=m_1;
	Eigen::Matrix3d linkI1;
	linkI1 << L_1xx,L_1xy,L_1xz,
			  L_1xy,L_1yy,L_1yz,
			  L_1xz,L_1yz,L_1zz;
	Eigen::Matrix3d Irot1=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
	// std::cout<< "Irot1:\n"<<Irot1 <<std::endl<<std::endl;
	Eigen::Vector3d h1 =Irot1*Eigen::Vector3d(l_1x,l_1y,l_1z);	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, Eigen::Vector3d(0.,0.,0.),Irot1);	  
	sva::RBInertiad rb1(mass1,h1,I1);
	rbd::Body Onedof_Link1(rb1,"Onedof_Link1");

	// std::cout<< "rb1:\n"<<rb1 <<std::endl<<std::endl;

	mbg.addBody(Onedof_Link_Base);
	mbg.addBody(Onedof_Link1);



	rbd::Joint j1(rbd::Joint::RevX,true,"joint1");


	mbg.addJoint(j1);



	sva::PTransformd to1(Eigen::Vector3d(0.,0.,0.));
	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("Onedof_Link_Base",to1,"Onedof_Link1",from1,"joint1");

	


	onedof=mbg.makeMultiBody("Onedof_Link_Base",true);

	onedofmbc=rbd::MultiBodyConfig(onedof);

	// jac=rbd::Jacobian(onedof,"Onedof_Link6");

}






