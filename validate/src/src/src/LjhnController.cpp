#include <iostream>
#include "LjhnController.h"
#include "RobotDimensions.h"


//////RBDyn
#include "Conversions.h"
#include "Util.h"
#include "EigenUtility.h"

#include "Jacobian.h"
#include "Coriolis.h"
#include "FD.h"
#include "FK.h"
#include "FV.h"
#include "FA.h"
#include "IK.h"
#include "CoM.h"
#include "Momentum.h"
#include "ID.h"
#include "Body.h"

//////

#include "SimControll.h"



using namespace ljnoid;
using namespace std;


LjhnController* LjhnController::m_UniqueInstance= new LjhnController();   //why   but delete will fault

LjhnController::LjhnController()
{

	timeStep=0.02;//s
	constructRobot();
	jointParaInit();

}


LjhnController::~LjhnController()
{

}

void LjhnController::simStart()
{
	SimControll::simInit();
}


void LjhnController::modelJointTrajControll()
{


	Eigen::Matrix<double, JOINT_NUM, JOINT_NUM> kp;
	Eigen::Matrix<double, JOINT_NUM, JOINT_NUM> kd;
	kp=Eigen::Matrix<double, JOINT_NUM, JOINT_NUM>::Zero();
	kd=Eigen::Matrix<double, JOINT_NUM, JOINT_NUM>::Zero();

	Eigen::Matrix<double, LLEG_JOINT_NUM, LLEG_JOINT_NUM> kpLLeg;
	Eigen::Matrix<double, LLEG_JOINT_NUM, LLEG_JOINT_NUM> kdLLeg;
	double kp1=1000.;  //500
	double kd1=50.;	//10

	kpLLeg<< kp1,0,0,0,0,0,
		  0,kp1,0,0,0,0,
		  0,0,kp1,0,0,0,
		  0,0,0,kp1,0,0,
		  0,0,0,0,kp1,0,
		  0,0,0,0,0,kp1;


	kdLLeg<< kd1,0,0,0,0,0,
		  0,kd1,0,0,0,0,
		  0,0,kd1,0,0,0,
		  0,0,0,kd1,0,0,
		  0,0,0,0,kd1,0,
		  0,0,0,0,0,kd1;	

	kp.block<6,6>(0,0)=kpLLeg;
	kd.block<6,6>(0,0)=kdLLeg;


	Eigen::Matrix<double, LLEG_JOINT_NUM, LLEG_JOINT_NUM> kpRLeg;
	Eigen::Matrix<double, LLEG_JOINT_NUM, LLEG_JOINT_NUM> kdRLeg;
	double kp2=300;
	double kd2=5;//5

	kpRLeg<< kp2,0,0,0,0,0,
		  0,kp2,0,0,0,0,
		  0,0,kp2,0,0,0,
		  0,0,0,kp2,0,0,
		  0,0,0,0,kp2,0,
		  0,0,0,0,0,kp2;


	kdRLeg<< kd2,0,0,0,0,0,
		  0,kd2,0,0,0,0,
		  0,0,kd2,0,0,0,
		  0,0,0,kd2,0,0,
		  0,0,0,0,kd2,0,
		  0,0,0,0,0,kd2;	

	kp.block<6,6>(6,6)=kpRLeg;
	kd.block<6,6>(6,6)=kdRLeg;

  ////////compute H and C(c(q,qd) and g(q))
	rbd::ForwardDynamics fd(ljhn);
	rbd::MultiBodyConfig mbcSensor(ljhn);
	mbcSensor.zero(ljhn);


	q=SimControll::jointAngle;
	// q=Eigen::Matrix<double, JOINT_NUM, 1>::Zero();
	dq=(q-qold)/timeStep;
	mbcSensor.q=sVectorToParam(ljhn,q);
	mbcSensor.alpha=sVectorToDof(ljhn,dq);
	qold=q;


	rbd::forwardKinematics(ljhn, mbcSensor);
	rbd::forwardVelocity(ljhn,mbcSensor);
	mbcSensor.gravity=Vector3d(0.,0.,9.81);
	fd.computeH(ljhn, mbcSensor);
	fd.computeC(ljhn, mbcSensor);
	
	
  ////////derive joint trajectory error
	qref=sParamToVector(ljhn,ljhnmbc.q);
	// qref=Eigen::Matrix<double, JOINT_NUM, 1>::Zero();
	ljhnmbc.alpha=sVectorToDof(ljhn,(qref-qrefOld)/timeStep);
	dqref=sDofToVector(ljhn,ljhnmbc.alpha);
	ljhnmbc.alphaD=sVectorToDof(ljhn,(dqref-dqrefOld)/timeStep);

	err=qref-q;
	errV=dqref-dq;//SimControll::jointVel[i];
	uPD=sDofToVector(ljhn,ljhnmbc.alphaD)+kp*err+kd*errV; //servo-base part
	

	qrefOld=qref;
	dqrefOld=dqref;

  ////////controll law
	jointCommand=fd.H()*uPD+fd.C();//Bq*uPD+c_qdq+gq;  model-base part

	// printf("leftLegCommands:\n %10f %10f %10f %10f %10f %10f\n",jointCommand[0],jointCommand[1],jointCommand[2],jointCommand[3],jointCommand[4],jointCommand[5]);
	printf("rightLegCommands:\n %10f %10f %10f %10f %10f %10f\n",jointCommand[6],jointCommand[7],jointCommand[8],jointCommand[9],jointCommand[10],jointCommand[11]);

}



void LjhnController::jointParaInit()
{

	ljhnmbc.zero(ljhn);
	ljhnmbc.q={{},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{}    //leftLeg 
	,{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{}};    //rightLeg
	// ljhnmbc.q={{},{30.*TO_RADIAN},{120.*TO_RADIAN},{40.*TO_RADIAN},{70.*TO_RADIAN},{150.*TO_RADIAN},{70.*TO_RADIAN},{}    //leftLeg 
	// ,{30.*TO_RADIAN},{120.*TO_RADIAN},{40.*TO_RADIAN},{70.*TO_RADIAN},{150.*TO_RADIAN},{70.*TO_RADIAN},{}};    //rightLeg
	
	// ljhnmbc.alpha={{},{30.*TO_RADIAN},{120.*TO_RADIAN},{40.*TO_RADIAN},{70.*TO_RADIAN},{150.*TO_RADIAN},{70.*TO_RADIAN},{}    //leftLeg 
	// ,{30.*TO_RADIAN},{120.*TO_RADIAN},{40.*TO_RADIAN},{70.*TO_RADIAN},{150.*TO_RADIAN},{70.*TO_RADIAN},{}};    //rightLeg
	
	// ljhnmbc.alphaD={{},{30.*TO_RADIAN},{120.*TO_RADIAN},{40.*TO_RADIAN},{70.*TO_RADIAN},{150.*TO_RADIAN},{70.*TO_RADIAN},{}    //leftLeg 
	// ,{30.*TO_RADIAN},{120.*TO_RADIAN},{40.*TO_RADIAN},{70.*TO_RADIAN},{150.*TO_RADIAN},{70.*TO_RADIAN},{}};    //rightLeg

	rbd::forwardKinematics(ljhn, ljhnmbc);
	// for(int i = 0; i < ljhn.joints().size(); ++i)
	// {
		
	// 	std::cout << "direction["<<i<<"] : " <<  std::endl<<  ljhn.joint(i).direction() << std::endl;
	// 	// std::cout << "bodyPosW["<<i<<"] : " <<  std::endl<<  sva::conversions::toHomogeneous(ljhnmbc.bodyPosW[i]) << std::endl;
	// }

	qref=sParamToVector(ljhn,ljhnmbc.q);
	dqref=sDofToVector(ljhn,ljhnmbc.alpha);
	qrefOld=qref;
	qold=qref;
	dqrefOld=dqref;


}

///vrep dyn param to D-H frame,output for checkout the identified dyn param
void LjhnController::outputDynParamFile()
{
	///D-H frame in kinematic frame
	Eigen::Matrix3d Irot[6];
	Eigen::Matrix3d Irot1=Eigen::Matrix3d(Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitZ())
							*Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX()));
	Eigen::Matrix3d Irot2=Eigen::Matrix3d(Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY()));
	Eigen::Matrix3d Irot3=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitZ()));
	Eigen::Matrix3d Irot4=Eigen::Matrix3d(Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitZ()));
	Eigen::Matrix3d Irot5=Eigen::Matrix3d(Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitZ()));
	Eigen::Matrix3d Irot6=Eigen::Matrix3d(Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitY()));
	
	Eigen::Vector3d cf[6];
	cf[0]=Eigen::Vector3d(0.,0.,0.);
	cf[1]=Eigen::Vector3d(-0.02496,0.,0.);
	cf[2]=Eigen::Vector3d(0.,0.,0.);
	cf[3]=Eigen::Vector3d(0.,0.,0.);
	cf[4]=Eigen::Vector3d(0.,0.,0.);
	cf[5]=Eigen::Vector3d(0.,0.,0.);

	///kinematic frame in D-H frame 
	Irot[0]= Irot1.inverse();
	Irot[1]= Irot2.inverse();
	Irot[2]= Irot3.inverse();
	Irot[3]= Irot4.inverse();
	Irot[4]= Irot5.inverse();
	Irot[5]= Irot6.inverse();

	double m[6];
	Eigen::Vector3d c[6];
	Eigen::Matrix3d IrotCom[6];
	for(int i=0;i<6;i++)
	{
		m[i]=ljhn.body(i+1).inertia().mass();
		c[i]=ljhn.body(i+1).inertia().momentum()/m[i];
		IrotCom[i]=ljhn.body(i+1).inertia().inertia()-m[i]*hat(c[i])*hat(c[i]).transpose();  //inertia in the com
		c[i]=c[i]+cf[i];

		std::cout<< "L_" << i+1 << "xx: " << (Irot[i]*(IrotCom[i]+m[i]*hat(c[i])*hat(c[i]).transpose())*Irot[i].transpose())(0,0) <<std::endl;
		std::cout<< "L_" << i+1 << "xy: " << (Irot[i]*(IrotCom[i]+m[i]*hat(c[i])*hat(c[i]).transpose())*Irot[i].transpose())(0,1) <<std::endl;
		std::cout<< "L_" << i+1 << "xz: " << (Irot[i]*(IrotCom[i]+m[i]*hat(c[i])*hat(c[i]).transpose())*Irot[i].transpose())(0,2) <<std::endl;
		std::cout<< "L_" << i+1 << "yy: " << (Irot[i]*(IrotCom[i]+m[i]*hat(c[i])*hat(c[i]).transpose())*Irot[i].transpose())(1,1) <<std::endl;
		std::cout<< "L_" << i+1 << "yz: " << (Irot[i]*(IrotCom[i]+m[i]*hat(c[i])*hat(c[i]).transpose())*Irot[i].transpose())(1,2) <<std::endl;
		std::cout<< "L_" << i+1 << "zz: " << (Irot[i]*(IrotCom[i]+m[i]*hat(c[i])*hat(c[i]).transpose())*Irot[i].transpose())(2,2) <<std::endl;
		std::cout<< "l_" << i+1 << "x: " << (Irot[i]*(m[i]*c[i]))(0) <<std::endl;
		std::cout<< "l_" << i+1 << "y: " << (Irot[i]*(m[i]*c[i]))(1) <<std::endl;
		std::cout<< "l_" << i+1 << "z: " << (Irot[i]*(m[i]*c[i]))(2) <<std::endl;
		std::cout<< "m_" << i+1 << ": " << m[i] <<std::endl;

	}
	

}

void LjhnController::constructRobot()
{
	constructTorso();
	constructLeftLeg();
	constructRightLeg();
	constructLeftArm();
	constructRightArm();


	ljhn=mbg.makeMultiBody("Torso",true);

	ljhnmbc=rbd::MultiBodyConfig(ljhn);



	ljhnFloat=mbg.makeMultiBody("Torso",rbd::Joint::Free);
	ljhnFloatmbc=rbd::MultiBodyConfig(ljhnFloat);




	leftLeg=leftLegMbg.makeMultiBody("Torso",rbd::Joint::Free);

	leftLegMbc=rbd::MultiBodyConfig(leftLeg);


}

void LjhnController::constructTorso()
{
	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

	double mass1=25;
	Eigen::Matrix3d linkI1;
	linkI1 << 2.564e-02*mass1,0,0,
			  0,2.477e-02*mass1,0,
			  0,0,2.467e-02*mass1;
	Eigen::Vector3d com1 =Eigen::Vector3d(0.,0.,0.);	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, com1,iden3d);	  
	sva::RBInertiad rb1(mass1,mass1*com1,I1);
	rbd::Body link1(rb1,"Torso");

	mbg.addBody(link1);


	leftLegMbg.addBody(link1);
}




////from vrep
void LjhnController::constructLeftLeg()
{
	
	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

	double mass1=2.;
	Eigen::Matrix3d linkI1;
	linkI1 << 3.677e-03*mass1,0,0,
			  0,3.262e-03*mass1,0,
			  0,0,1.73e-03*mass1;
	Eigen::Vector3d com1 =Eigen::Vector3d(-8.6475265561603e-05,0.00017283856868744,0.044496834278107);	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, com1,iden3d);	  
	sva::RBInertiad rb1(mass1,mass1*com1,I1);
	rbd::Body link1(rb1,"leftLegLink1");


	double mass2=4.;
	Eigen::Matrix3d linkI2;
	linkI2 << 4.579e-03*mass2,0,0,
			  0,4.332e-03*mass2,0,
			  0,0,2.073e-03*mass2;
	Eigen::Vector3d com2 =Eigen::Vector3d(0.022901486605406,0.0044404268264771,-0.062264978885651);	
	Eigen::Matrix3d I2=sva::inertiaToOrigin(linkI2,mass2, com2,iden3d);	  
	sva::RBInertiad rb2(mass2,mass2*com2,I2);
	rbd::Body link2(rb2,"leftLegLink2");
	// std::cout << "rb2 : " <<  std::endl<<  rb2 << std::endl;

	double mass3=5.;
	Eigen::Matrix3d linkI3;
	linkI3 << 1.144e-02*mass3,0,0,
			  0,1.13e-02*mass3,0,
			  0,0,2.437e-03*mass3;
	Eigen::Vector3d com3 =Eigen::Vector3d(-0.000197,0.019220694899559,-0.14144551753998);
	Eigen::Matrix3d I3=sva::inertiaToOrigin(linkI3,mass3, com3,iden3d);		  
	sva::RBInertiad rb3(mass3,mass3*com3,I3);
	rbd::Body link3(rb3,"leftLegLink3");


	double mass4=4.;
	Eigen::Matrix3d linkI4;
	linkI4 << 8.708e-03*mass4,0,0,
			  0,8.421e-03*mass4,0,
			  0,0,1.72e-03*mass4;
	Eigen::Vector3d com4 =Eigen::Vector3d(-6.9178640842438e-06,0.011718079447746,-0.12409943342209);
	Eigen::Matrix3d I4=sva::inertiaToOrigin(linkI4,mass4, com4,iden3d);	  
	sva::RBInertiad rb4(mass4,mass4*com4,I4);
	rbd::Body link4(rb4,"leftLegLink4");


	double mass5=0.00354;
	Eigen::Matrix3d linkI5;
	linkI5 << 6.971e-05*mass5,0,0,
			  0,5.663e-05*mass5,0,
			  0,0,5.663e-05*mass5;
	Eigen::Vector3d com5 =Eigen::Vector3d(0.022018659859896,0.00040411949157715,9.9420547485352e-05);	
	Eigen::Matrix3d I5=sva::inertiaToOrigin(linkI5,mass5, com5,iden3d);  
	sva::RBInertiad rb5(mass5,mass5*com5,I5);
	rbd::Body link5(rb5,"leftLegLink5");


	double mass6=2.;
	Eigen::Matrix3d linkI6;
	linkI6 << 5.544e-03*mass6,0,0,   //in center of mass 
			  0,4.769e-03*mass6,0,
			  0,0,2.551e-03*mass6;
	Eigen::Vector3d com6 =Eigen::Vector3d(0.038024064153433,-0.0001995861530304,-0.047664612531662);
	Eigen::Matrix3d I6=sva::inertiaToOrigin(linkI6,mass6, com6,iden3d);	//in body frame orgin	  
	sva::RBInertiad rb6(mass6,mass6*com6,I6);
	rbd::Body link6(rb6,"leftLegLink6");

	double mass7=0.0;
	Eigen::Matrix3d linkI7;
	linkI7 << 0.0,0,0,   //in center of mass 
			  0,0.0,0,
			  0,0,0.0;
	Eigen::Vector3d com7 =Eigen::Vector3d(0.0,0.0,0.0);
	Eigen::Matrix3d I7=sva::inertiaToOrigin(linkI7,mass7, com7,iden3d);	//in body frame orgin	  
	sva::RBInertiad rb7(mass7,mass7*com7,I7);
	rbd::Body link7(rb7,"leftLegLinkSole");


	mbg.addBody(link1);
	mbg.addBody(link2);
	mbg.addBody(link3);
	mbg.addBody(link4);
	mbg.addBody(link5);
	mbg.addBody(link6);
	mbg.addBody(link7);



	leftLegMbg.addBody(link1);
	leftLegMbg.addBody(link2);
	leftLegMbg.addBody(link3);
	leftLegMbg.addBody(link4);
	leftLegMbg.addBody(link5);
	leftLegMbg.addBody(link6);
	leftLegMbg.addBody(link7);

	rbd::Joint j1(rbd::Joint::Rev,Eigen::Vector3d(0.,0.,-1.), true,"leftLegJoint1");
	rbd::Joint j2(rbd::Joint::Rev,Eigen::Vector3d(-1.,0.,0.), true,"leftLegJoint2");
	rbd::Joint j3(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.), true,"leftLegJoint3");
	rbd::Joint j4(rbd::Joint::Rev,Eigen::Vector3d(0.,-1.,0.), true,"leftLegJoint4");
	rbd::Joint j5(rbd::Joint::Rev,Eigen::Vector3d(0.,-1.,0.), true,"leftLegJoint5");
	rbd::Joint j6(rbd::Joint::Rev,Eigen::Vector3d(1.,0.,0.), true,"leftLegJoint6");
	rbd::Joint j7(rbd::Joint::Fixed,true,"leftLegJointSole");

	mbg.addJoint(j1);
	mbg.addJoint(j2);
	mbg.addJoint(j3);
	mbg.addJoint(j4);
	mbg.addJoint(j5);
	mbg.addJoint(j6);
	mbg.addJoint(j7);


	leftLegMbg.addJoint(j1);
	leftLegMbg.addJoint(j2);
	leftLegMbg.addJoint(j3);
	leftLegMbg.addJoint(j4);
	leftLegMbg.addJoint(j5);
	leftLegMbg.addJoint(j6);
	leftLegMbg.addJoint(j7);



	sva::PTransformd to1(Eigen::Vector3d(-Hip_X,Hip_Y,-Hip_Z));
	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("Torso",to1,"leftLegLink1",from1,"leftLegJoint1");

	sva::PTransformd to2(Eigen::Vector3d(0.,0.,0.));
	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink1",to2,"leftLegLink2",from2,"leftLegJoint2");

	sva::PTransformd to3(Eigen::Vector3d(Hip_Thigh_X,Hip_Ankle_Y,-Hip_Thigh_Z));
	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink2",to3,"leftLegLink3",from3,"leftLegJoint3");

	sva::PTransformd to4(Eigen::Vector3d(Thigh_Knee_X,0.0,-Thigh_Knee_Z));
	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink3",to4,"leftLegLink4",from4,"leftLegJoint4");

	sva::PTransformd to5(Eigen::Vector3d(-Knee_AnkleP_X,0,-Knee_AnkleP_Z));
	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink4",to5,"leftLegLink5",from5,"leftLegJoint5");

	sva::PTransformd to6(Eigen::Vector3d(0.0,AnkleP_R_Y,-AnkleP_R_Z));
	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink5",to6,"leftLegLink6",from6,"leftLegJoint6");

	sva::PTransformd to7(Eigen::Vector3d(Foot_X,Foot_Y,-Foot_Z));
	sva::PTransformd from7(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink6",to7,"leftLegLinkSole",from7,"leftLegJointSole");
	



	leftLegMbg.linkBodies("Torso",to1,"leftLegLink1",from1,"leftLegJoint1");
	leftLegMbg.linkBodies("leftLegLink1",to2,"leftLegLink2",from2,"leftLegJoint2");
	leftLegMbg.linkBodies("leftLegLink2",to3,"leftLegLink3",from3,"leftLegJoint3");
	leftLegMbg.linkBodies("leftLegLink3",to4,"leftLegLink4",from4,"leftLegJoint4");
	leftLegMbg.linkBodies("leftLegLink4",to5,"leftLegLink5",from5,"leftLegJoint5");
	leftLegMbg.linkBodies("leftLegLink5",to6,"leftLegLink6",from6,"leftLegJoint6");
	leftLegMbg.linkBodies("leftLegLink6",to7,"leftLegLinkSole",from7,"leftLegJointSole");

	
}  


// ////from sousa 2014
// void LjhnController::constructLeftLeg()
// {

// 	///sousa 2014 IROS result (wls)
// 	/////robot dynamic param in D-H coordinate

// 	double L_1xx    =    0.;
// 	double L_1xy    =    0;
// 	double L_1xz    =    0;
// 	double L_1yy    =    0.;
// 	double L_1yz    =    0;
// 	double L_1zz    =    1.00018330326246e-6;
// 	double l_1x    =    0;
// 	double l_1y    =    0;
// 	double l_1z    =    0;
// 	double m_1    =    1.84641911095404;
// 	double L_2xx    =    0.000993796194111218;
// 	double L_2xy    =    0.000905215469353252;
// 	double L_2xz    =    -0.00465119713512309;
// 	double L_2yy    =    0.0221761876285668;
// 	double L_2yz    =    0.0316761256415542;
// 	double L_2zz    =    0.0851636227062729;
// 	double l_2x    =    0.199774735365134;
// 	double l_2y    =    -0.0449989876537795;
// 	double l_2z    =    0.0449989839516478;
// 	double m_2    =    4.49999891648078;
// 	double L_3xx    =    0.114704173137468;
// 	double L_3xy    =    -0.0649421496268148;
// 	double L_3xz    =    -0.0133402687602906;
// 	double L_3yy    =    0.121969108152042;
// 	double L_3yz    =    -0.00113743565943898;
// 	double L_3zz    =    0.0829682861203963;
// 	double l_3x    =    0.675352622163562;
// 	double l_3y    =    0.00937934187753452;
// 	double l_3z    =    0.121930424412324;
// 	double m_3    =    5.49999899495981;
// 	double L_4xx    =    0.0623026773722308;
// 	double L_4xy    =    0.0170733599708324;
// 	double L_4xz    =    -0.00500043744884359;
// 	double L_4yy    =    0.0936097376808420;
// 	double L_4yz    =    0.000697558216721389;
// 	double L_4zz    =    0.0897260358235396;
// 	double l_4x    =    0.634430251358882;
// 	double l_4y    =    -0.0174813970296112;
// 	double l_4z    =    0.0504070961223520;
// 	double m_4    =    4.49299392502249;
// 	double L_5xx    =    0.0129870222680546;
// 	double L_5xy    =    0.00197441081442854;
// 	double L_5xz    =    -0.000178101944977239;
// 	double L_5yy    =    0.000301192557501012;
// 	double L_5yz    =    -2.70786616280359e-5;
// 	double L_5zz    =    3.44272667060497e-6;
// 	double l_5x    =    -1.04316226980986e-8;
// 	double l_5y    =    -1.01651468631392e-8;
// 	double l_5z    =    6.11361695482522e-9;
// 	double m_5    =    0.000101048349120792;
// 	double L_6xx    =    0.00714385741353338;
// 	double L_6xy    =    0.000146950159580914;
// 	double L_6xz    =    0.00335723553882893;
// 	double L_6yy    =    0.0103654722786105;
// 	double L_6yz    =    0.00188124939455720;
// 	double L_6zz    =    0.0131324517197398;
// 	double l_6x    =    -0.0954906319060648;
// 	double l_6y    =    0.000334888894832268;
// 	double l_6z    =    0.0744649336732348;
// 	double m_6    =    1.50000102901346;

// 	// double L_1xx    =    1.1449713719949776e-17;
// 	// double L_1xy    =    1.1227774039498878e-16;
// 	// double L_1xz    =    1.0587952711793113e-17;
// 	// double L_1yy    =    -1.8397840156045998e-17;
// 	// double L_1yz    =    0.0;
// 	// double L_1zz    =    0.05924739711743074;
// 	// double l_1x    =    -7.166513620305257e-17;
// 	// double l_1y    =    1.3952265216050925e-17;
// 	// double l_1z    =    8.806053437855415e-17;
// 	// double m_1    =    0.0;
// 	// double L_2xx    =    -0.041129068499415396;
// 	// double L_2xy    =    0.009841967112937118;
// 	// double L_2xz    =    -0.004880950930211855;
// 	// double L_2yy    =    0.10037646561684646;
// 	// double L_2yz    =    0.0314076956163465;
// 	// double L_2zz    =    0.2240173841812685;
// 	// double l_2x    =    1.288998653550916;
// 	// double l_2y    =    -0.19753772977793704;
// 	// double l_2z    =    -0.002957630064102143;
// 	// double m_2    =    3.691122319998755e-05;
// 	// double L_3xx    =    -0.06509747787998893;
// 	// double L_3xy    =    -0.0661155827983546;
// 	// double L_3xz    =    0.11925542032316852;
// 	// double L_3yy    =    0.2479857935618418;
// 	// double L_3yz    =    0.0023575073699505927;
// 	// double L_3zz    =    0.3884956891998633;
// 	// double l_3x    =    1.6975984083472935;
// 	// double l_3y    =    0.010245644586890488;
// 	// double l_3z    =    0.22679852154483754;
// 	// double m_3    =    0.18268673324920603;
// 	// double L_4xx    =    0.09201514469853926;
// 	// double L_4xy    =    0.017429744151811446;
// 	// double L_4xz    =    -0.05460698311796631;
// 	// double L_4yy    =    0.09087317098331368;
// 	// double L_4yz    =    0.000459053344332167;
// 	// double L_4zz    =    0.060172660804112765;
// 	// double l_4x    =    0.5314027314488883;
// 	// double l_4y    =    -0.01778115563719687;
// 	// double l_4z    =    -0.19103501354412253;
// 	// double m_4    =    0.7490208533005929;
// 	// double L_5xx    =    0.08634268138370838;
// 	// double L_5xy    =    0.0020204413072134725;
// 	// double L_5xz    =    -0.00031983118193377465;
// 	// double L_5yy    =    0.09654563429814453;
// 	// double L_5yz    =    -4.945611253481829e-05;
// 	// double L_5zz    =    -0.022947728290677025;
// 	// double l_5x    =    -1.6117148854081148e-07;
// 	// double l_5y    =    -0.03723913196961327;
// 	// double l_5z    =    -0.17465892537687555;
// 	// double m_5    =    0.9219673760984246;
// 	// double L_6xx    =    0.030067795803880325;
// 	// double L_6xy    =    0.00014897159447647477;
// 	// double L_6xz    =    0.003357089001958659;
// 	// double L_6yy    =    0.03332715728915105;
// 	// double L_6yz    =    0.0018882293972144518;
// 	// double L_6zz    =    0.013162947288663141;
// 	// double l_6x    =    -0.09549538478188754;
// 	// double l_6y    =    0.0003467635394448294;
// 	// double l_6z    =    0.03723913196961327;
// 	// double m_6    =    0.9219673760984246;


// 	// //from vrep
// 	// double L_1xx= 0.010484;
// 	// double L_1xy= 2.98925e-08;
// 	// double L_1xz= 1.53815e-05;
// 	// double L_1yy= 0.011314;
// 	// double L_1yz= -7.69575e-06;
// 	// double L_1zz= 0.00346007;
// 	// double l_1x= 0.000345677;
// 	// double l_1y= -0.000172951;
// 	// double l_1z= -0.0889937;
// 	// double m_1= 2;
// 	// double L_2xx= 0.00838782;
// 	// double L_2xy= 0.00110593;
// 	// double L_2xz= -0.000512693;
// 	// double L_2yy= 0.0328527;
// 	// double L_2yz= 3.65627e-05;
// 	// double L_2zz= 0.0339026;
// 	// double l_2x= 0.24906;
// 	// double l_2y= -0.0177617;
// 	// double l_2z= 0.00823405;
// 	// double m_2= 4;
// 	// double L_3xx= 0.0140324;
// 	// double L_3xy= -0.000139324;
// 	// double L_3xz= -0.0135934;
// 	// double L_3yy= 0.159081;
// 	// double L_3yz= -1.89324e-05;
// 	// double L_3zz= 0.156534;
// 	// double l_3x= 0.707228;
// 	// double l_3y= 0.000985;
// 	// double l_3z= 0.0961035;
// 	// double m_3= 5;
// 	// double L_4xx= 0.00742925;
// 	// double L_4xy= 3.43401e-06;
// 	// double L_4xz= 0.00581683;
// 	// double L_4yy= 0.0969839;
// 	// double L_4yz= -3.24256e-07;
// 	// double L_4zz= 0.0952867;
// 	// double l_4x= 0.496398;
// 	// double l_4y= -2.76715e-05;
// 	// double l_4z= -0.0468723;
// 	// double m_4= 4;
// 	// double L_5xx= 1.91732e-06;
// 	// double L_5xy= 7.74944e-09;
// 	// double L_5xz= 1.42229e-10;
// 	// double L_5yy= 2.47387e-07;
// 	// double L_5yz= -3.14995e-08;
// 	// double L_5zz= 1.91677e-06;
// 	// double l_5x= 3.51949e-07;
// 	// double l_5y= -7.79461e-05;
// 	// double l_5z= -1.43058e-06;
// 	// double m_5= 0.00354;
// 	// double L_6xx= 0.00799374;
// 	// double L_6xy= 1.90264e-05;
// 	// double L_6xz= 0.0036248;
// 	// double L_6yy= 0.0169735;
// 	// double L_6yz= -1.51782e-05;
// 	// double L_6zz= 0.0156319;
// 	// double l_6x= -0.0953292;
// 	// double l_6y= 0.000399172;
// 	// double l_6z= 0.0760481;
// 	// double m_6= 2;


	
// 	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

// 	double mass1=m_1;
// 	Eigen::Matrix3d linkI1;
// 	linkI1 << L_1xx,L_1xy,L_1xz,
// 			  L_1xy,L_1yy,L_1yz,
// 			  L_1xz,L_1yz,L_1zz;
// 	Eigen::Vector3d com1 =Eigen::Vector3d(0.,0.,0.);//Eigen::Vector3d(l_1x,l_1y,l_1z)/mass1;//    //mass1 is zero
// 	Eigen::Matrix3d ICom1=linkI1-mass1*hat(com1)*hat(com1).transpose();
// 	Eigen::Vector3d frameTrans1=Eigen::Vector3d(0.,0.,0.);
// 	Eigen::Matrix3d frameRot1=Eigen::Matrix3d(Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitZ())
// 							*Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX()));
// 	com1=com1+frameTrans1;	
// 	Eigen::Matrix3d I1=sva::inertiaToOrigin(ICom1,mass1,com1,frameRot1);
// 	Eigen::Vector3d h1 =frameRot1*(Eigen::Vector3d(l_1x,l_1y,l_1z));	  
// 	sva::RBInertiad rb1(mass1,h1,I1);
// 	rbd::Body link1(rb1,"leftLegLink1");


// 	double mass2=m_2;
// 	Eigen::Matrix3d linkI2;
// 	linkI2 << L_2xx,L_2xy,L_2xz,
// 			  L_2xy,L_2yy,L_2yz,
// 			  L_2xz,L_2yz,L_2zz;	
// 	Eigen::Vector3d com2 =Eigen::Vector3d(l_2x,l_2y,l_2z)/mass2;
// 	Eigen::Matrix3d ICom2=linkI2-mass2*hat(com2)*hat(com2).transpose();
// 	Eigen::Vector3d frameTrans2=Eigen::Vector3d(0.,0.,-0.02496);//
// 	Eigen::Matrix3d frameRot2=Eigen::Matrix3d(Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX())
// 							*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY()));
// 	com2=com2+frameTrans2;	
// 	Eigen::Matrix3d I2=sva::inertiaToOrigin(ICom2,mass2, com2,frameRot2);
// 	Eigen::Vector3d h2 =frameRot2*(mass2*com2);	 	  
// 	sva::RBInertiad rb2(mass2,h2,I2);
// 	rbd::Body link2(rb2,"leftLegLink2");
// 	// std::cout << "rb2 : " <<  std::endl<<  rb2 << std::endl;


// 	double mass3=m_3;
// 	Eigen::Matrix3d linkI3;
// 	linkI3 << L_3xx,L_3xy,L_3xz,
// 			  L_3xy,L_3yy,L_3yz,
// 			  L_3xz,L_3yz,L_3zz;
// 	Eigen::Vector3d com3 =Eigen::Vector3d(l_3x,l_3y,l_3z)/mass3;
// 	Eigen::Matrix3d ICom3=linkI3-mass3*hat(com3)*hat(com3).transpose();
// 	Eigen::Vector3d frameTrans3=Eigen::Vector3d(0.,0.,0.);
// 	Eigen::Matrix3d frameRot3=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX())
// 							*Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitZ()));
// 	com3=com3+frameTrans3;	
// 	Eigen::Matrix3d I3=sva::inertiaToOrigin(ICom3,mass3, com3,frameRot3);
// 	Eigen::Vector3d h3 =frameRot3*(mass3*com3);		  
// 	sva::RBInertiad rb3(mass3,h3,I3);
// 	rbd::Body link3(rb3,"leftLegLink3");


// 	double mass4=m_4;
// 	Eigen::Matrix3d linkI4;
// 	linkI4 << L_4xx,L_4xy,L_4xz,
// 			  L_4xy,L_4yy,L_4yz,
// 			  L_4xz,L_4yz,L_4zz;
// 	Eigen::Vector3d com4 =Eigen::Vector3d(l_4x,l_4y,l_4z)/mass4;
// 	Eigen::Matrix3d ICom4=linkI4-mass4*hat(com4)*hat(com4).transpose();
// 	Eigen::Vector3d frameTrans4=Eigen::Vector3d(0.,0.,0.);
// 	Eigen::Matrix3d frameRot4=Eigen::Matrix3d(Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitX())
// 							*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitZ()));
// 	com4=com4+frameTrans4;	
// 	Eigen::Matrix3d I4=sva::inertiaToOrigin(ICom4,mass4, com4,frameRot4);
// 	Eigen::Vector3d h4 =frameRot4*(mass4*com4);	  
// 	sva::RBInertiad rb4(mass4,h4,I4);
// 	rbd::Body link4(rb4,"leftLegLink4");


// 	double mass5=m_5;
// 	Eigen::Matrix3d linkI5;
// 	linkI5 << L_5xx,L_5xy,L_5xz,
// 			  L_5xy,L_5yy,L_5yz,
// 			  L_5xz,L_5yz,L_5zz;	
// 	Eigen::Vector3d com5 =Eigen::Vector3d(l_5x,l_5y,l_5z)/mass5;
// 	Eigen::Matrix3d ICom5=linkI5-mass5*hat(com5)*hat(com5).transpose();
// 	Eigen::Vector3d frameTrans5=Eigen::Vector3d(0.,0.,0.);
// 	Eigen::Matrix3d frameRot5=Eigen::Matrix3d(Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitX())
// 							*Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitZ()));
// 	com5=com5+frameTrans5;	
// 	Eigen::Matrix3d I5=sva::inertiaToOrigin(ICom5,mass5, com5,frameRot5);
// 	Eigen::Vector3d h5 =frameRot5*(mass5*com5);  
// 	sva::RBInertiad rb5(mass5,h5,I5);
// 	rbd::Body link5(rb5,"leftLegLink5");


// 	double mass6=m_6;
// 	Eigen::Matrix3d linkI6;
// 	linkI6 << L_6xx,L_6xy,L_6xz,
// 			  L_6xy,L_6yy,L_6yz,
// 			  L_6xz,L_6yz,L_6zz;
// 	Eigen::Vector3d com6 =Eigen::Vector3d(l_6x,l_6y,l_6z)/mass6;
// 	Eigen::Matrix3d ICom6=linkI6-mass6*hat(com6)*hat(com6).transpose();
// 	Eigen::Vector3d frameTrans6=Eigen::Vector3d(0.,0.,0.);
// 	Eigen::Matrix3d frameRot6=Eigen::Matrix3d(Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX())
// 							*Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitY()));
// 	com6=com6+frameTrans6;	
// 	Eigen::Matrix3d I6=sva::inertiaToOrigin(ICom6,mass6, com6,frameRot6);
// 	Eigen::Vector3d h6 =frameRot6*(mass6*com6);	  
// 	sva::RBInertiad rb6(mass6,h6,I6);
// 	rbd::Body link6(rb6,"leftLegLink6");


// 	double mass7=0.0;
// 	Eigen::Matrix3d linkI7;
// 	linkI7 << 0.0,0,0,   //in center of mass 
// 			  0,0.0,0,
// 			  0,0,0.0;
// 	Eigen::Vector3d com7 =Eigen::Vector3d(0.0,0.0,0.0);
// 	Eigen::Matrix3d I7=sva::inertiaToOrigin(linkI7,mass7, com7,iden3d);	//in body frame orgin	  
// 	sva::RBInertiad rb7(mass7,mass7*com7,I7);
// 	rbd::Body link7(rb7,"leftLegLinkSole");


// 	mbg.addBody(link1);
// 	mbg.addBody(link2);
// 	mbg.addBody(link3);
// 	mbg.addBody(link4);
// 	mbg.addBody(link5);
// 	mbg.addBody(link6);
// 	mbg.addBody(link7);



// 	leftLegMbg.addBody(link1);
// 	leftLegMbg.addBody(link2);
// 	leftLegMbg.addBody(link3);
// 	leftLegMbg.addBody(link4);
// 	leftLegMbg.addBody(link5);
// 	leftLegMbg.addBody(link6);
// 	leftLegMbg.addBody(link7);

// 	rbd::Joint j1(rbd::Joint::Rev,Eigen::Vector3d(0.,0.,-1.), true,"leftLegJoint1");
// 	rbd::Joint j2(rbd::Joint::Rev,Eigen::Vector3d(-1.,0.,0.), true,"leftLegJoint2");
// 	rbd::Joint j3(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.), true,"leftLegJoint3");
// 	rbd::Joint j4(rbd::Joint::Rev,Eigen::Vector3d(0.,-1.,0.), true,"leftLegJoint4");
// 	rbd::Joint j5(rbd::Joint::Rev,Eigen::Vector3d(0.,-1.,0.), true,"leftLegJoint5");
// 	rbd::Joint j6(rbd::Joint::Rev,Eigen::Vector3d(1.,0.,0.), true,"leftLegJoint6");
// 	rbd::Joint j7(rbd::Joint::Fixed,true,"leftLegJointSole");

// 	mbg.addJoint(j1);
// 	mbg.addJoint(j2);
// 	mbg.addJoint(j3);
// 	mbg.addJoint(j4);
// 	mbg.addJoint(j5);
// 	mbg.addJoint(j6);
// 	mbg.addJoint(j7);


// 	leftLegMbg.addJoint(j1);
// 	leftLegMbg.addJoint(j2);
// 	leftLegMbg.addJoint(j3);
// 	leftLegMbg.addJoint(j4);
// 	leftLegMbg.addJoint(j5);
// 	leftLegMbg.addJoint(j6);
// 	leftLegMbg.addJoint(j7);



// 	sva::PTransformd to1(Eigen::Vector3d(-Hip_X,Hip_Y,-Hip_Z));
// 	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("Torso",to1,"leftLegLink1",from1,"leftLegJoint1");

// 	sva::PTransformd to2(Eigen::Vector3d(0.,0.,0.));
// 	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("leftLegLink1",to2,"leftLegLink2",from2,"leftLegJoint2");

// 	sva::PTransformd to3(Eigen::Vector3d(Hip_Thigh_X,Hip_Ankle_Y,-Hip_Thigh_Z));
// 	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("leftLegLink2",to3,"leftLegLink3",from3,"leftLegJoint3");

// 	sva::PTransformd to4(Eigen::Vector3d(Thigh_Knee_X,0.0,-Thigh_Knee_Z));
// 	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("leftLegLink3",to4,"leftLegLink4",from4,"leftLegJoint4");

// 	sva::PTransformd to5(Eigen::Vector3d(-Knee_AnkleP_X,0,-Knee_AnkleP_Z));
// 	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("leftLegLink4",to5,"leftLegLink5",from5,"leftLegJoint5");

// 	sva::PTransformd to6(Eigen::Vector3d(0.0,AnkleP_R_Y,-AnkleP_R_Z));
// 	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("leftLegLink5",to6,"leftLegLink6",from6,"leftLegJoint6");

// 	sva::PTransformd to7(Eigen::Vector3d(Foot_X,Foot_Y,-Foot_Z));
// 	sva::PTransformd from7(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("leftLegLink6",to7,"leftLegLinkSole",from7,"leftLegJointSole");
	



// 	leftLegMbg.linkBodies("Torso",to1,"leftLegLink1",from1,"leftLegJoint1");
// 	leftLegMbg.linkBodies("leftLegLink1",to2,"leftLegLink2",from2,"leftLegJoint2");
// 	leftLegMbg.linkBodies("leftLegLink2",to3,"leftLegLink3",from3,"leftLegJoint3");
// 	leftLegMbg.linkBodies("leftLegLink3",to4,"leftLegLink4",from4,"leftLegJoint4");
// 	leftLegMbg.linkBodies("leftLegLink4",to5,"leftLegLink5",from5,"leftLegJoint5");
// 	leftLegMbg.linkBodies("leftLegLink5",to6,"leftLegLink6",from6,"leftLegJoint6");
// 	leftLegMbg.linkBodies("leftLegLink6",to7,"leftLegLinkSole",from7,"leftLegJointSole");

	
// } 


// ////from sousa 2014
void LjhnController::constructRightLeg()
{
	///sousa 2014 IROS result (wls)
	/////robot dynamic param in D-H coordinate
	double L_1xx    =    2.2430170718571447e-17;
	double L_1xy    =    1.0330703958022457e-16;
	double L_1xz    =    -1.0417892834014457e-17;
	double L_1yy    =    -1.838848051013073e-17;
	double L_1yz    =    0.0;
	double L_1zz    =    0.05732048047026843;
	double l_1x    =    -7.157612108266939e-17;
	double l_1y    =    -1.2992777358483022e-17;
	double l_1z    =    8.574016752850091e-17;
	double m_1    =    0.0;
	double L_2xx    =    -0.03652325943727739;
	double L_2xy    =    -0.021626390222468563;
	double L_2xz    =    -0.007610642999438165;
	double L_2yy    =    0.09384373990754608;
	double L_2yz    =    0.0028556341700136704;
	double L_2zz    =    0.18696542646278377;
	double l_2x    =    1.2840889154707584;
	double l_2y    =    0.19676582940139284;
	double l_2z    =    -0.0028614383850758002;
	double m_2    =    3.571075104573789e-05;
	double L_3xx    =    -0.11336648938660424;
	double L_3xy    =    -0.004887436498756467;
	double L_3xz    =    -0.10976250086951297;
	double L_3yy    =    0.2638086564121103;
	double L_3yz    =    -0.00035848667779758795;
	double L_3zz    =    0.386753019400404;
	double l_3x    =    1.6900501664605423;
	double l_3y    =    0.002625069553142663;
	double l_3z    =    -0.2225282736004551;
	double m_3    =    0.18125236543684803;
	double L_4xx    =    0.052664851131348334;
	double L_4xy    =    0.01414444403477113;
	double L_4xz    =    0.04898648687256533;
	double L_4yy    =    0.09777731589415782;
	double L_4yz    =    -0.014503644719225193;
	double L_4zz    =    0.06808804697888252;
	double l_4x    =    0.5268636823643502;
	double l_4y    =    0.015489951070807478;
	double l_4z    =    0.1896115972146969;
	double m_4    =    0.7465891314810188;
	double L_5xx    =    0.07080734481851776;
	double L_5xy    =    -0.0018944617557600418;
	double L_5xz    =    -0.0020130297638147427;
	double L_5yy    =    0.0796348222069885;
	double L_5yz    =    -0.001855353999388646;
	double L_5zz    =    -0.016506150765974892;
	double l_5x    =    9.898997233102145e-05;
	double l_5y    =    -0.03773862750960299;
	double l_5z    =    0.17492103966648317;
	double m_5    =    0.9195072187368584;
	double L_6xx    =    0.0267209822214234;
	double L_6xy    =    0.00062866537392535;
	double L_6xz    =    0.003930363731368943;
	double L_6yy    =    0.027580211831119455;
	double L_6yz    =    -0.00024194532886227118;
	double L_6zz    =    0.014871773856168393;
	double l_6x    =    -0.09546302146835012;
	double l_6y    =    0.0011744570781885072;
	double l_6z    =    0.03773862750960299;
	double m_6    =    0.9195072187368584;



	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

	double mass1=m_1;
	Eigen::Matrix3d linkI1;
	linkI1 << L_1xx,L_1xy,L_1xz,
			  L_1xy,L_1yy,L_1yz,
			  L_1xz,L_1yz,L_1zz;
	Eigen::Vector3d com1 =Eigen::Vector3d(0.,0.,0.);//Eigen::Vector3d(l_1x,l_1y,l_1z)/mass1;//    //mass1 is zero
	Eigen::Matrix3d ICom1=linkI1-mass1*hat(com1)*hat(com1).transpose();
	Eigen::Vector3d frameTrans1=Eigen::Vector3d(0.,0.,0.);
	Eigen::Matrix3d frameRot1=Eigen::Matrix3d(Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitZ())
							*Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX()));
	com1=com1+frameTrans1;	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(ICom1,mass1,com1,frameRot1);
	Eigen::Vector3d h1 =frameRot1*(Eigen::Vector3d(l_1x,l_1y,l_1z));		  
	sva::RBInertiad rb1(mass1,h1,linkI1);
	rbd::Body link1(rb1,"rightLegLink1");


	double mass2=m_2;
	Eigen::Matrix3d linkI2;
	linkI2 << L_2xx,L_2xy,L_2xz,
			  L_2xy,L_2yy,L_2yz,
			  L_2xz,L_2yz,L_2zz;
	Eigen::Vector3d com2 =Eigen::Vector3d(l_2x,l_2y,l_2z)/mass2;
	Eigen::Matrix3d ICom2=linkI2-mass2*hat(com2)*hat(com2).transpose();
	Eigen::Vector3d frameTrans2=Eigen::Vector3d(0.,0.,-0.02496);//
	Eigen::Matrix3d frameRot2=Eigen::Matrix3d(Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY()));
	com2=com2+frameTrans2;	
	Eigen::Matrix3d I2=sva::inertiaToOrigin(ICom2,mass2, com2,frameRot2);
	Eigen::Vector3d h2 =frameRot2*(mass2*com2);		  
	sva::RBInertiad rb2(mass2,h2,linkI2);
	rbd::Body link2(rb2,"rightLegLink2");


	double mass3=m_3;
	Eigen::Matrix3d linkI3;
	linkI3 << L_3xx,L_3xy,L_3xz,
			  L_3xy,L_3yy,L_3yz,
			  L_3xz,L_3yz,L_3zz;
	Eigen::Vector3d com3 =Eigen::Vector3d(l_3x,l_3y,l_3z)/mass3;
	Eigen::Matrix3d ICom3=linkI3-mass3*hat(com3)*hat(com3).transpose();
	Eigen::Vector3d frameTrans3=Eigen::Vector3d(0.,0.,0.);
	Eigen::Matrix3d frameRot3=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitZ()));
	com3=com3+frameTrans3;	
	Eigen::Matrix3d I3=sva::inertiaToOrigin(ICom3,mass3, com3,frameRot3);
	Eigen::Vector3d h3 =frameRot3*(mass3*com3);	  
	sva::RBInertiad rb3(mass3,h3,linkI3);
	rbd::Body link3(rb3,"rightLegLink3");


	double mass4=m_4;
	Eigen::Matrix3d linkI4;
	linkI4 << L_4xx,L_4xy,L_4xz,
			  L_4xy,L_4yy,L_4yz,
			  L_4xz,L_4yz,L_4zz;
	Eigen::Vector3d com4 =Eigen::Vector3d(l_4x,l_4y,l_4z)/mass4;
	Eigen::Matrix3d ICom4=linkI4-mass4*hat(com4)*hat(com4).transpose();
	Eigen::Vector3d frameTrans4=Eigen::Vector3d(0.,0.,0.);
	Eigen::Matrix3d frameRot4=Eigen::Matrix3d(Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitZ()));
	com4=com4+frameTrans4;	
	Eigen::Matrix3d I4=sva::inertiaToOrigin(ICom4,mass4, com4,frameRot4);
	Eigen::Vector3d h4 =frameRot4*(mass4*com4); 
	sva::RBInertiad rb4(mass4,h4,linkI4);
	rbd::Body link4(rb4,"rightLegLink4");


	double mass5=m_5;
	Eigen::Matrix3d linkI5;
	linkI5 << L_5xx,L_5xy,L_5xz,
			  L_5xy,L_5yy,L_5yz,
			  L_5xz,L_5yz,L_5zz;
	Eigen::Vector3d com5 =Eigen::Vector3d(l_5x,l_5y,l_5z)/mass5;
	Eigen::Matrix3d ICom5=linkI5-mass5*hat(com5)*hat(com5).transpose();
	Eigen::Vector3d frameTrans5=Eigen::Vector3d(0.,0.,0.);
	Eigen::Matrix3d frameRot5=Eigen::Matrix3d(Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitZ()));
	com5=com5+frameTrans5;	
	Eigen::Matrix3d I5=sva::inertiaToOrigin(ICom5,mass5, com5,frameRot5);
	Eigen::Vector3d h5 =frameRot5*(mass5*com5);	
	sva::RBInertiad rb5(mass5,h5,linkI5);
	rbd::Body link5(rb5,"rightLegLink5");


	double mass6=m_6;
	Eigen::Matrix3d linkI6;
	linkI6 << L_6xx,L_6xy,L_6xz,
			  L_6xy,L_6yy,L_6yz,
			  L_6xz,L_6yz,L_6zz;
	Eigen::Vector3d com6 =Eigen::Vector3d(l_6x,l_6y,l_6z)/mass6;
	Eigen::Matrix3d ICom6=linkI6-mass6*hat(com6)*hat(com6).transpose();
	Eigen::Vector3d frameTrans6=Eigen::Vector3d(0.,0.,0.);
	Eigen::Matrix3d frameRot6=Eigen::Matrix3d(Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX())
							*Eigen::AngleAxisd(PI_2, Eigen::Vector3d::UnitY()));
	com6=com6+frameTrans6;	
	Eigen::Matrix3d I6=sva::inertiaToOrigin(ICom6,mass6, com6,frameRot6);
	Eigen::Vector3d h6 =frameRot6*(mass6*com6);	  
	sva::RBInertiad rb6(mass6,h6,linkI6);
	rbd::Body link6(rb6,"rightLegLink6");

	double mass7=0.0;
	Eigen::Matrix3d linkI7;
	linkI7 << 0.0,0,0,   //in center of mass 
			  0,0.0,0,
			  0,0,0.0;
	Eigen::Vector3d com7 =Eigen::Vector3d(0.0,0.0,0.0);
	Eigen::Matrix3d I7=sva::inertiaToOrigin(linkI7,mass7, com7,iden3d);	//in body frame orgin	  
	sva::RBInertiad rb7(mass7,mass7*com7,I7);
	rbd::Body link7(rb7,"rightLegLinkSole");


	mbg.addBody(link1);
	mbg.addBody(link2);
	mbg.addBody(link3);
	mbg.addBody(link4);
	mbg.addBody(link5);
	mbg.addBody(link6);
	mbg.addBody(link7);



	rbd::Joint j1(rbd::Joint::Rev,Eigen::Vector3d(0.,0.,-1.), true,"rightLegJoint1");
	rbd::Joint j2(rbd::Joint::Rev,Eigen::Vector3d(-1.,0.,0.), true,"rightLegJoint2");
	rbd::Joint j3(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.),true,"rightLegJoint3");
	rbd::Joint j4(rbd::Joint::Rev,Eigen::Vector3d(0.,-1.,0.), true,"rightLegJoint4");
	rbd::Joint j5(rbd::Joint::Rev,Eigen::Vector3d(0.,-1.,0.), true,"rightLegJoint5");
	rbd::Joint j6(rbd::Joint::Rev,Eigen::Vector3d(1.,0.,0.), true,"rightLegJoint6");
	rbd::Joint j7(rbd::Joint::Fixed,true,"rightLegJointSole");


	mbg.addJoint(j1);
	mbg.addJoint(j2);
	mbg.addJoint(j3);
	mbg.addJoint(j4);
	mbg.addJoint(j5);
	mbg.addJoint(j6);
	mbg.addJoint(j7);



	sva::PTransformd to1(Eigen::Vector3d(-Hip_X,-Hip_Y,-Hip_Z));
	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("Torso",to1,"rightLegLink1",from1,"rightLegJoint1");

	sva::PTransformd to2(Eigen::Vector3d(0.,0.,0.));
	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink1",to2,"rightLegLink2",from2,"rightLegJoint2");

	sva::PTransformd to3(Eigen::Vector3d(Hip_Thigh_X,-Hip_Ankle_Y,-Hip_Thigh_Z));
	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink2",to3,"rightLegLink3",from3,"rightLegJoint3");

	sva::PTransformd to4(Eigen::Vector3d(Thigh_Knee_X,0.0,-Thigh_Knee_Z));
	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink3",to4,"rightLegLink4",from4,"rightLegJoint4");

	sva::PTransformd to5(Eigen::Vector3d(-Knee_AnkleP_X,0,-Knee_AnkleP_Z));
	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink4",to5,"rightLegLink5",from5,"rightLegJoint5");

	sva::PTransformd to6(Eigen::Vector3d(0.0,-AnkleP_R_Y,-AnkleP_R_Z));
	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink5",to6,"rightLegLink6",from6,"rightLegJoint6");

	sva::PTransformd to7(Eigen::Vector3d(Foot_X,-Foot_Y,-Foot_Z));
	sva::PTransformd from7(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink6",to7,"rightLegLinkSole",from7,"rightLegJointSole");


}


// // ////from vrep
// void LjhnController::constructRightLeg()
// {
// 	///read from vrep in kinematic frame
// 	double L_1xx =	0.011819135397673	;
// 	double L_1xy =	-3.9261554007908e-08	;
// 	double L_1xz =	-2.8663738703472e-05	;
// 	double L_1yy =	0.010989315807819	;
// 	double L_1yz =	4.2275246414647e-06	;
// 	double L_1zz =	0.0034601900260895	;
// 	double l_1x =	0.00060943746939301	;
// 	double l_1y =	-9.4637274742126e-05	;
// 	double l_1z =	0.094500064849854	;
// 	double m_1 = 	2	;
// 	double L_2xx =	0.034299068152905	;
// 	double L_2xy =	0.0004091163573321	;
// 	double L_2xz =	0.0058164070360363	;
// 	double L_2yy =	0.035358853638172	;
// 	double L_2yz =	-0.0011185515904799	;
// 	double L_2zz =	0.010497213341296	;
// 	double l_2x =	0.09222799539566	;
// 	double l_2y =	-0.01774400472641	;
// 	double l_2z =	-0.25222492218018	;
// 	double m_2 = 	4	;
// 	double L_3xx =	0.15908165276051	;
// 	double L_3xy =	1.7292686038672e-07	;
// 	double L_3xz =	-1.8295536392543e-06	;
// 	double L_3yy =	0.15653255581856	;
// 	double L_3yz =	-0.013597444631159	;
// 	double L_3zz =	0.014034084975719	;
// 	double l_3x =	6.5378844738007e-06	;
// 	double l_3y =	-0.096153169870377	;
// 	double l_3z =	-0.70722192525864	;
// 	double m_3 = 	5	;
// 	double L_4xx =	0.098464474081993	;
// 	double L_4xy =	0.00015164092474151	;
// 	double L_4xz =	0.0016229993198067	;
// 	double L_4yy =	0.096809864044189	;
// 	double L_4yz =	-0.0058799241669476	;
// 	double L_4zz =	0.0074703618884087	;
// 	double l_4x =	0.012942582368851	;
// 	double l_4y =	-0.046839594841003	;
// 	double l_4z =	-0.50233054161072	;
// 	double m_4 = 	4	;
// 	double L_5xx =	2.4692710098861e-07	;
// 	double L_5xy =	-3.0280507268277e-11	;
// 	double L_5xz =	1.6271625824515e-08	;
// 	double L_5yy =	1.9241954305471e-06	;
// 	double L_5yz =	3.1851119506633e-13	;
// 	double L_5zz =	1.9240417259425e-06	;
// 	double l_5x =	7.8111736237212e-05	;
// 	double l_5y =	1.5297531852365e-09	;
// 	double l_5z =	-7.3755203572334e-07	;
// 	double m_5 = 	0.0035399999469519	;
// 	double L_6xx =	0.015582595020533	;
// 	double L_6xy =	1.7611364455661e-05	;
// 	double L_6xz =	0.0036089294590056	;
// 	double L_6yy =	0.01693120598793	;
// 	double L_6yz =	-2.2142594389152e-05	;
// 	double L_6zz =	0.0080005135387182	;
// 	double l_6x =	0.076140910387039	;
// 	double l_6y =	-0.00046736001968384	;
// 	double l_6z =	-0.09481018781662	;
// 	double m_6 = 	2	;



// 	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

// 	double mass1=m_1;
// 	Eigen::Matrix3d linkI1;
// 	linkI1 << L_1xx,L_1xy,L_1xz,
// 			  L_1xy,L_1yy,L_1yz,
// 			  L_1xz,L_1yz,L_1zz;
// 	Eigen::Vector3d h1 =Eigen::Vector3d(l_1x,l_1y,l_1z);		  
// 	sva::RBInertiad rb1(mass1,h1,linkI1);
// 	rbd::Body link1(rb1,"rightLegLink1");


// 	double mass2=m_2;
// 	Eigen::Matrix3d linkI2;
// 	linkI2 << L_2xx,L_2xy,L_2xz,
// 			  L_2xy,L_2yy,L_2yz,
// 			  L_2xz,L_2yz,L_2zz;
// 	Eigen::Vector3d h2 =Eigen::Vector3d(l_2x,l_2y,l_2z);	  
// 	sva::RBInertiad rb2(mass2,h2,linkI2);
// 	rbd::Body link2(rb2,"rightLegLink2");


// 	double mass3=m_3;
// 	Eigen::Matrix3d linkI3;
// 	linkI3 << L_3xx,L_3xy,L_3xz,
// 			  L_3xy,L_3yy,L_3yz,
// 			  L_3xz,L_3yz,L_3zz;
// 	Eigen::Vector3d h3 =Eigen::Vector3d(l_3x,l_3y,l_3z);	  
// 	sva::RBInertiad rb3(mass3,h3,linkI3);
// 	rbd::Body link3(rb3,"rightLegLink3");


// 	double mass4=m_4;
// 	Eigen::Matrix3d linkI4;
// 	linkI4 << L_4xx,L_4xy,L_4xz,
// 			  L_4xy,L_4yy,L_4yz,
// 			  L_4xz,L_4yz,L_4zz;
// 	Eigen::Vector3d h4 =Eigen::Vector3d(l_4x,l_4y,l_4z); 
// 	sva::RBInertiad rb4(mass4,h4,linkI4);
// 	rbd::Body link4(rb4,"rightLegLink4");


// 	double mass5=m_5;
// 	Eigen::Matrix3d linkI5;
// 	linkI5 << L_5xx,L_5xy,L_5xz,
// 			  L_5xy,L_5yy,L_5yz,
// 			  L_5xz,L_5yz,L_5zz;
// 	Eigen::Vector3d h5 =Eigen::Vector3d(l_5x,l_5y,l_5z);	
// 	sva::RBInertiad rb5(mass5,h5,linkI5);
// 	rbd::Body link5(rb5,"rightLegLink5");


// 	double mass6=m_6;
// 	Eigen::Matrix3d linkI6;
// 	linkI6 << L_6xx,L_6xy,L_6xz,
// 			  L_6xy,L_6yy,L_6yz,
// 			  L_6xz,L_6yz,L_6zz;
// 	Eigen::Vector3d h6 =Eigen::Vector3d(l_6x,l_6y,l_6z);	  
// 	sva::RBInertiad rb6(mass6,h6,linkI6);
// 	rbd::Body link6(rb6,"rightLegLink6");

// 	double mass7=0.0;
// 	Eigen::Matrix3d linkI7;
// 	linkI7 << 0.0,0,0,   //in center of mass 
// 			  0,0.0,0,
// 			  0,0,0.0;
// 	Eigen::Vector3d com7 =Eigen::Vector3d(0.0,0.0,0.0);
// 	Eigen::Matrix3d I7=sva::inertiaToOrigin(linkI7,mass7, com7,iden3d);	//in body frame orgin	  
// 	sva::RBInertiad rb7(mass7,mass7*com7,I7);
// 	rbd::Body link7(rb7,"rightLegLinkSole");


// 	mbg.addBody(link1);
// 	mbg.addBody(link2);
// 	mbg.addBody(link3);
// 	mbg.addBody(link4);
// 	mbg.addBody(link5);
// 	mbg.addBody(link6);
// 	mbg.addBody(link7);



// 	rbd::Joint j1(rbd::Joint::Rev,Eigen::Vector3d(0.,0.,-1.), true,"rightLegJoint1");
// 	rbd::Joint j2(rbd::Joint::Rev,Eigen::Vector3d(-1.,0.,0.), true,"rightLegJoint2");
// 	rbd::Joint j3(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.),true,"rightLegJoint3");
// 	rbd::Joint j4(rbd::Joint::Rev,Eigen::Vector3d(0.,-1.,0.), true,"rightLegJoint4");
// 	rbd::Joint j5(rbd::Joint::Rev,Eigen::Vector3d(0.,-1.,0.), true,"rightLegJoint5");
// 	rbd::Joint j6(rbd::Joint::Rev,Eigen::Vector3d(1.,0.,0.), true,"rightLegJoint6");
// 	rbd::Joint j7(rbd::Joint::Fixed,true,"rightLegJointSole");


// 	mbg.addJoint(j1);
// 	mbg.addJoint(j2);
// 	mbg.addJoint(j3);
// 	mbg.addJoint(j4);
// 	mbg.addJoint(j5);
// 	mbg.addJoint(j6);
// 	mbg.addJoint(j7);



// 	sva::PTransformd to1(Eigen::Vector3d(-Hip_X,-Hip_Y,-Hip_Z));
// 	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("Torso",to1,"rightLegLink1",from1,"rightLegJoint1");

// 	sva::PTransformd to2(Eigen::Vector3d(0.,0.,0.));
// 	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("rightLegLink1",to2,"rightLegLink2",from2,"rightLegJoint2");

// 	sva::PTransformd to3(Eigen::Vector3d(Hip_Thigh_X,-Hip_Ankle_Y,-Hip_Thigh_Z));
// 	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("rightLegLink2",to3,"rightLegLink3",from3,"rightLegJoint3");

// 	sva::PTransformd to4(Eigen::Vector3d(Thigh_Knee_X,0.0,-Thigh_Knee_Z));
// 	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("rightLegLink3",to4,"rightLegLink4",from4,"rightLegJoint4");

// 	sva::PTransformd to5(Eigen::Vector3d(-Knee_AnkleP_X,0,-Knee_AnkleP_Z));
// 	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("rightLegLink4",to5,"rightLegLink5",from5,"rightLegJoint5");

// 	sva::PTransformd to6(Eigen::Vector3d(0.0,-AnkleP_R_Y,-AnkleP_R_Z));
// 	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("rightLegLink5",to6,"rightLegLink6",from6,"rightLegJoint6");

// 	sva::PTransformd to7(Eigen::Vector3d(Foot_X,-Foot_Y,-Foot_Z));
// 	sva::PTransformd from7(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("rightLegLink6",to7,"rightLegLinkSole",from7,"rightLegJointSole");


// }


void LjhnController::constructLeftArm()
{



	
}

void LjhnController::constructRightArm()
{




}












