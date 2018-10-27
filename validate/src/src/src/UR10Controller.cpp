#include <iostream>
#include "UR10Controller.h"

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


UR10Controller* UR10Controller::m_UniqueInstance= new UR10Controller();   //why   but delete will fault

UR10Controller::UR10Controller()
{

	timeStep=0.02;//s
	ur10_initRobot();

	for(int i = 0; i < JOINT_NUM; ++i)  //init some variate
	{
		uPD[i]=0;
		err[i]=0;
		errV[i]=0;
	}
	for(int i = 0; i <6; ++i)  //init some variate
	{
		errx(i)=0;
		errxV(i)=0;
		uxPD(i)=0;
	}
	jointParaInit();

	double kp=1000;
	double kd=kp/20;


	Kp<<  kp,0,0,0,0,0,
		  0,kp,0,0,0,0,
		  0,0,kp,0,0,0,
		  0,0,0,kp,0,0,
		  0,0,0,0,kp,0,
		  0,0,0,0,0,kp;
	Kd<<  kd,0,0,0,0,0,
		  0,kd,0,0,0,0,
		  0,0,kd,0,0,0,
		  0,0,0,kd,0,0,
		  0,0,0,0,kd,0,
		  0,0,0,0,0,kd;

	kp=400;
	kd=kp/30;

	kpx<< kp,0,0,0,0,0,
		  0,kp,0,0,0,0,
		  0,0,kp,0,0,0,
		  0,0,0,kp,0,0,
		  0,0,0,0,kp,0,
		  0,0,0,0,0,kp;



	kdx<< kd,0,0,0,0,0,
		  0,kd,0,0,0,0,
		  0,0,kd,0,0,0,
		  0,0,0,kd,0,0,
		  0,0,0,0,kd,0,
		  0,0,0,0,0,kd;


	samplesCount=0;	 
	samples=720; 
	regressors.resize(samples*ur10.nrDof(), (ur10.nrBodies()-1)*10);
	torques.resize(samples*ur10.nrDof());
	inertiaVec.resize((ur10.nrBodies()-1)*10);	  

}



UR10Controller::~UR10Controller()
{


}




void UR10Controller::idDynPara()
{
	if(samplesCount < samples)
	{
		rbd::IDIM idim(ur10);

		rbd::MultiBodyConfig mbcSensor(ur10);
		mbcSensor.zero(ur10);
		mbcSensor.gravity=Vector3d(0.,0.,9.81);

		// q=SimControll::jointAngle;
		// dq=(q-qold)/timeStep;	
		ddq=(dq-dqold)/timeStep;
		mbcSensor.q=sVectorToParam(ur10,q);
		mbcSensor.alpha=sVectorToDof(ur10,dq);
		mbcSensor.alphaD=sVectorToDof(ur10,ddq);
		// qold=q;
		dqold=dq;
		// std::cout << "q : " <<  std::endl<<  q << std::endl;
		rbd::forwardKinematics(ur10, mbcSensor);
		rbd::forwardVelocity(ur10,mbcSensor);
		rbd::forwardAcceleration(ur10,mbcSensor);

		rbd::forwardKinematics(ur10, ur10mbc);
		rbd::forwardVelocity(ur10,ur10mbc);
		rbd::forwardAcceleration(ur10,ur10mbc);

		idim.computeY(ur10, mbcSensor);
	    regressors.block(samplesCount*ur10.nrDof(), 0, ur10.nrDof(), (ur10.nrBodies()-1)*10) = idim.Y().block(0, 6, ur10.nrDof(), (ur10.nrBodies()-1)*10);
	    torques.segment(samplesCount*ur10.nrDof(), ur10.nrDof()) = SimControll::jointTorque;
	    // std::cout << "idim.Y() : " <<  std::endl<<  idim.Y().block(0,10,6,30) << std::endl;
	    // std::cout <<  idim.Y().block(0,40,6,30) << std::endl;
	    
	}

	samplesCount++;

	if(samplesCount == samples)
    {
    	inertiaVec = regressors.jacobiSvd(ComputeThinU | ComputeThinV).solve(torques);
    	std::cout << "inertiaVec : " <<  std::endl<<  inertiaVec.segment(0,10) << std::endl;
    	std::cout <<  std::endl<< inertiaVec.segment(10,10) << std::endl;
    	std::cout <<  std::endl<< inertiaVec.segment(20,10) << std::endl;
    	std::cout <<  std::endl<< inertiaVec.segment(30,10) << std::endl;
    	std::cout <<  std::endl<< inertiaVec.segment(40,10) << std::endl;
    	std::cout <<  std::endl<< inertiaVec.segment(50,10) << std::endl;
    	// std::cout <<  std::endl<< inertiaVec.segment(60,10) << std::endl;
    	// // std::cout << "torques : " <<  std::endl<<  torques << std::endl;
    }

}

// impedance controll
void UR10Controller::admittanceControll(sva::PTransformd tarPos)
{
	Eigen::MatrixXd Bxq,Sxqdq,gxq;

	Eigen::Matrix<double, JOINT_NUM, 1> F,Fa;
	Eigen::Matrix<double, JOINT_NUM, JOINT_NUM> Bm,Dm,Km;



	double bm=0.6;
	double km=15;
	double dm=5;



	Bm<< bm,0,0,0,0,0,
		  0,bm,0,0,0,0,
		  0,0,bm,0,0,0,
		  0,0,0,bm,0,0,
		  0,0,0,0,bm,0,
		  0,0,0,0,0,bm;



	Dm<< dm,0,0,0,0,0,
		  0,dm,0,0,0,0,
		  0,0,dm,0,0,0,
		  0,0,0,dm,0,0,
		  0,0,0,0,dm,0,
		  0,0,0,0,0,dm;


	Km<< km,0,0,0,0,0,
		  0,km,0,0,0,0,
		  0,0,km,0,0,0,
		  0,0,0,km,0,0,
		  0,0,0,0,km,0,
		  0,0,0,0,0,km;



	Eigen::Matrix<double, 6, 6> kp,kd;
	double kp1=50;
	double kd1=55;

	kp<< kp1,0,0,0,0,0,
		  0,kp1,0,0,0,0,
		  0,0,kp1,0,0,0,
		  0,0,0,kp1,0,0,
		  0,0,0,0,kp1,0,
		  0,0,0,0,0,kp1;


	kd<< kd1,0,0,0,0,0,
		  0,kd1,0,0,0,0,
		  0,0,kd1,0,0,0,
		  0,0,0,kd1,0,0,
		  0,0,0,0,kd1,0,
		  0,0,0,0,0,kd1;



	F=SimControll::endFT;
	// F<<0,0,0,0,0,0;

  ////////compute H and C(c(q,qd) and g(q))
	rbd::ForwardDynamics fd(ur10);
	rbd::MultiBodyConfig mbcSensor(ur10);
	mbcSensor.zero(ur10);


	q=SimControll::jointAngle;
	dq=(q-qold)/timeStep;
	ddq=(dq-dqold)/timeStep;
	mbcSensor.q=sVectorToParam(ur10,q);
	mbcSensor.alpha=sVectorToDof(ur10,dq);
	qold=q;
	dqold=dq;

	rbd::forwardKinematics(ur10, mbcSensor);
	rbd::forwardVelocity(ur10,mbcSensor);
	mbcSensor.gravity=Vector3d(0.,0.,9.81);
	fd.computeH(ur10, mbcSensor);
	fd.computeC(ur10, mbcSensor);


  ////////compute Jacobi and JacobiDot
	rbd::Jacobian jacSensor(ur10,"UR10_Link6");
  	Eigen::MatrixXd jacMat,jacMatDot;
	jacMat=jacSensor.jacobian(ur10,mbcSensor);
	jacMatDot=jacSensor.jacobianDot(ur10,mbcSensor);
	
	// cout<< "jacMat="<<endl<<jacMat <<endl<<endl;


  ////////controll law
    ////////Admittance controll	  use xyz euler angle
	xrefAd.head<3>()=xyzEulerFromRot(tarPos.rotation().transpose());
	xrefAd.tail<3>()=tarPos.translation();


	dxrefAd=(xrefAd-xrefAdOld)/timeStep;
	xrefAdOld=xrefAd;

	ddxrefAd=(dxrefAd-dxrefAdOld)/timeStep;
	dxrefAdOld=dxrefAd;


	errxAdm=xrefAd-xa;
	errxVAdm=dxrefAd-dxa;


	Fa=(GeometricJ2AnalyticalJ(mbcSensor.bodyPosW[6].rotation().transpose()).inverse()).transpose()*F;//relate to euler angle

	
	ddxa=ddxrefAd+Bm.inverse()*(Dm*errxVAdm+Km*errxAdm+Fa);
	dxa+=ddxa*timeStep;
	xa+=dxa*timeStep;

    ////////Position controll
  ////////derive cartesian trajectory error	
	sva::PTransformd xaX;
	xaX.rotation()=rotFromXyzEuler(xa[0], xa[1], xa[2]).transpose();
	xaX.translation()=xa.tail<3>();

	dx=sva::transformError(xSenOld,mbcSensor.bodyPosW[6]).vector()/timeStep;
	xSenOld=mbcSensor.bodyPosW[6];

	dxref=sva::transformError(xtarOld,xaX).vector()/timeStep;
	xtarOld=xaX;

	ddxref=(dxref-dxrefOld)/timeStep;
	dxrefOld=dxref;

	errx=sva::transformError(mbcSensor.bodyPosW[6], xaX).vector();
	errxV=dxref-dx;



  ////////controll law
  	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  	svd.compute(jacMat, Eigen::ComputeThinU | Eigen::ComputeThinV);	

	uxPD=ddxref+kp*errx+kd*errxV;//servo-base part
	ux_ddq=svd.solve(uxPD-jacMatDot*dq);//model-base part
	jointCommand=fd.H()*ux_ddq+fd.C();//model-base part


	printf("jointCommands:\n %10f %10f %10f %10f %10f %10f\n",jointCommand[0],jointCommand[1],jointCommand[2],jointCommand[3],jointCommand[4],jointCommand[5]);

	// printf("jointAngles:\n %10f %10f %10f %10f %10f %10f\n",ur10->link(1)->q()*180/PI,ur10->link(2)->q()*180/PI,ur10->link(3)->q()*180/PI,ur10->link(4)->q()*180/PI,ur10->link(5)->q()*180/PI,ur10->link(6)->q()*180/PI);
	// printf("jointSensors:\n %10f %10f %10f %10f %10f %10f\n\n",SimControll::jointAngle[0]*180/PI,SimControll::jointAngle[1]*180/PI,SimControll::jointAngle[2]*180/PI,SimControll::jointAngle[3]*180/PI,SimControll::jointAngle[4]*180/PI,SimControll::jointAngle[5]*180/PI);

}



// impedance controll
void UR10Controller::impedanceControll(sva::PTransformd tarPos)
{
	Eigen::MatrixXd Bxq,Sxqdq,gxq;

	Eigen::Matrix<double, JOINT_NUM, 1> F;
	Eigen::Matrix<double, JOINT_NUM, JOINT_NUM> Bm,Dm,Km;

	// double bm=0.5;
	// double km=35;
	// double dm=1;

	double bm=0.6;
	double km=15;
	double dm=5;


	Bm<< bm,0,0,0,0,0,
		  0,bm,0,0,0,0,
		  0,0,bm,0,0,0,
		  0,0,0,bm,0,0,
		  0,0,0,0,bm,0,
		  0,0,0,0,0,bm;



	Dm<< dm,0,0,0,0,0,
		  0,dm,0,0,0,0,
		  0,0,dm,0,0,0,
		  0,0,0,dm,0,0,
		  0,0,0,0,dm,0,
		  0,0,0,0,0,dm;


	Km<< km,0,0,0,0,0,
		  0,km,0,0,0,0,
		  0,0,km,0,0,0,
		  0,0,0,km,0,0,
		  0,0,0,0,km,0,
		  0,0,0,0,0,km;



	F=SimControll::endFT;

  ////////compute H and C(c(q,qd) and g(q))
	rbd::ForwardDynamics fd(ur10);
	rbd::MultiBodyConfig mbcSensor(ur10);
	mbcSensor.zero(ur10);


	q=SimControll::jointAngle;
	dq=(q-qold)/timeStep;
	ddq=(dq-dqold)/timeStep;
	mbcSensor.q=sVectorToParam(ur10,q);
	mbcSensor.alpha=sVectorToDof(ur10,dq);
	qold=q;
	dqold=dq;

	rbd::forwardKinematics(ur10, mbcSensor);
	rbd::forwardVelocity(ur10,mbcSensor);
	mbcSensor.gravity=Vector3d(0.,0.,9.81);
	fd.computeH(ur10, mbcSensor);
	fd.computeC(ur10, mbcSensor);


  ////////////Coriolis Sqdq*dq
	rbd::Coriolis coriolis(ur10);
	Eigen::MatrixXd Sqdq = coriolis.coriolis(ur10, mbcSensor);


  ////////compute Jacobi and JacobiDot
	rbd::Jacobian jacSensor(ur10,"UR10_Link6");
  	Eigen::MatrixXd jacMat,jacMatDot;
	jacMat=jacSensor.jacobian(ur10,mbcSensor);
	jacMatDot=jacSensor.jacobianDot(ur10,mbcSensor);
	
	// cout<< "jacMat="<<endl<<jacMat <<endl<<endl;



  ////////Bxq Sxqdq gxq  --cartesian manipulate formula

	Bxq=jacMat.transpose().inverse()*fd.H()*jacMat.inverse();
	Sxqdq=jacMat.transpose().inverse()*Sqdq*jacMat.inverse()-Bxq*jacMatDot*jacMat.inverse();
	gxq=jacMat.transpose().inverse()*(fd.C()-Sqdq*dq);




  ////////derive cartesian trajectory error	

	dx=sva::transformError(xSenOld,mbcSensor.bodyPosW[6]).vector()/timeStep;
	xSenOld=mbcSensor.bodyPosW[6];


	dxref=sva::transformError(xtarOld,tarPos).vector()/timeStep;
	xtarOld=tarPos;

	ddxref=(dxref-dxrefOld)/timeStep;
	dxrefOld=dxref;

	errx=sva::transformError(mbcSensor.bodyPosW[6], tarPos).vector();
	errxV=dxref-dx;


	
  ////////controll law

	
	/////could get sqdq
	uxPD=ddxref+Bm.inverse()*(Dm*errxV+Km*errx+F);// Bm-desired inertia   Dm-desired damping   Km-desired stiffness  
	jointCommand=jacMat.transpose()*(Bxq*uxPD+Sxqdq*dx+gxq-F);//



	// ////not sqdq
	// uxPD=ddxref+Bm.inverse()*(Dm*errxV+Km*errx+F);//Bm-desired inertia   Dm-desired damping   Km-desired stiffness  
	// ux_ddq=jacMat.inverse()*(uxPD-jacMatDot*dq);
	// jointCommand=fd.H()*ux_ddq+fd.C()-jacMat.transpose()*F;//


	// Bm=Bxq;
	// uxPD=ddxref+Bm.inverse()*(Dm*errxV+Km*errx+F);//Bm-desired inertia   Dm-desired damping   Km-desired stiffness  
	// jointCommand=jacMat.transpose()*(Bxq*uxPD+Sxqdq*dx+gxq-F);
	// // jointCommand=fd.H()*jacMat.inverse()*(ddxref-jacMatDot*dq)+Sqdq*dq+(fd.C()-Sqdq*dq)+jacMat.transpose()*(Dm*errxV+Km*errx);


	// Bm=Bxq;
	// uxPD=ddxref+Bm.inverse()*((Sxqdq+Dm)*errxV+Km*errx+F);//Bm-desired inertia   Dm-desired damping   Km-desired stiffness  
	// jointCommand=jacMat.transpose()*(Bxq*uxPD+Sxqdq*dx+gxq-F);
	// // jointCommand=fd.H()*jacMat.inverse()*(ddxref-jacMatDot*jacMat.inverse()*dxref)+Sqdq*jacMat.inverse()*dxref+(fd.C()-Sqdq*dq)+jacMat.transpose()*(Dm*errxV+Km*errx);//



	printf("jointCommands:\n %10f %10f %10f %10f %10f %10f\n",jointCommand[0],jointCommand[1],jointCommand[2],jointCommand[3],jointCommand[4],jointCommand[5]);

	// printf("jointAngles:\n %10f %10f %10f %10f %10f %10f\n",ur10->link(1)->q()*180/PI,ur10->link(2)->q()*180/PI,ur10->link(3)->q()*180/PI,ur10->link(4)->q()*180/PI,ur10->link(5)->q()*180/PI,ur10->link(6)->q()*180/PI);
	// printf("jointSensors:\n %10f %10f %10f %10f %10f %10f\n\n",SimControll::jointAngle[0]*180/PI,SimControll::jointAngle[1]*180/PI,SimControll::jointAngle[2]*180/PI,SimControll::jointAngle[3]*180/PI,SimControll::jointAngle[4]*180/PI,SimControll::jointAngle[5]*180/PI);

}




// model-base cartesian controll
void UR10Controller::modelCartesianTrajControll(sva::PTransformd tarPos)
{

  ////////compute H and C(c(q,qd) and g(q))
	rbd::ForwardDynamics fd(ur10);
	rbd::MultiBodyConfig mbcSensor(ur10);
	mbcSensor.zero(ur10);


	q=SimControll::jointAngle;
	dq=(q-qold)/timeStep;
	mbcSensor.q=sVectorToParam(ur10,q);
	mbcSensor.alpha=sVectorToDof(ur10,dq);
	qold=q;


	rbd::forwardKinematics(ur10, mbcSensor);
	rbd::forwardVelocity(ur10,mbcSensor);
	mbcSensor.gravity=Vector3d(0.,0.,9.81);
	fd.computeH(ur10, mbcSensor);
	fd.computeC(ur10, mbcSensor);


  ////////compute Jacobi and JacobiDot
	rbd::Jacobian jacSensor(ur10,"UR10_Link6");
  	Eigen::MatrixXd jacMat,jacMatDot;
	jacMat=jacSensor.jacobian(ur10,mbcSensor);
	jacMatDot=jacSensor.jacobianDot(ur10,mbcSensor);
	
	// cout<< "jacMat="<<endl<<jacMat <<endl<<endl;



  ////////derive cartesian trajectory error	

	dx=sva::transformError(xSenOld,mbcSensor.bodyPosW[6]).vector()/timeStep;
	xSenOld=mbcSensor.bodyPosW[6];

	dxref=sva::transformError(xtarOld,tarPos).vector()/timeStep;
	xtarOld=tarPos;

	ddxref=(dxref-dxrefOld)/timeStep;
	dxrefOld=dxref;

	errx=sva::transformError(mbcSensor.bodyPosW[6], tarPos).vector();
	errxV=dxref-dx;



  ////////controll law
  	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  	svd.compute(jacMat, Eigen::ComputeThinU | Eigen::ComputeThinV);	

	uxPD=ddxref+kpx*errx+kdx*errxV;//servo-base part
	ux_ddq=svd.solve(uxPD-jacMatDot*dq);//model-base part
	jointCommand=fd.H()*ux_ddq+fd.C();//model-base part
	
	// printf("jointCommands\n: %10f %10f %10f %10f %10f %10f\n\n",jointCommand[0],jointCommand[1],jointCommand[2],jointCommand[3],jointCommand[4],jointCommand[5]);

	// printf("jointAngles:\n %10f %10f %10f %10f %10f %10f\n",ur10->link(1)->q()*180/PI,ur10->link(2)->q()*180/PI,ur10->link(3)->q()*180/PI,ur10->link(4)->q()*180/PI,ur10->link(5)->q()*180/PI,ur10->link(6)->q()*180/PI);
	// printf("jointSensors:\n %10f %10f %10f %10f %10f %10f\n\n",SimControll::jointAngle[0]*180/PI,SimControll::jointAngle[1]*180/PI,SimControll::jointAngle[2]*180/PI,SimControll::jointAngle[3]*180/PI,SimControll::jointAngle[4]*180/PI,SimControll::jointAngle[5]*180/PI);

}



// model-base joint controll
void UR10Controller::modelJointTrajControll()
{

   ////////compute H and C(c(q,qd) and g(q))
	rbd::ForwardDynamics fd(ur10);
	rbd::MultiBodyConfig mbcSensor(ur10);
	mbcSensor.zero(ur10);


	q=SimControll::jointAngle;
	dq=(q-qold)/timeStep;
	mbcSensor.q=sVectorToParam(ur10,q);
	mbcSensor.alpha=sVectorToDof(ur10,dq);
	qold=q;


	rbd::forwardKinematics(ur10, mbcSensor);
	rbd::forwardVelocity(ur10,mbcSensor);
	mbcSensor.gravity=Vector3d(0.,0.,9.81);
	fd.computeH(ur10, mbcSensor);
	fd.computeC(ur10, mbcSensor);
	
	

	qref=sParamToVector(ur10,ur10mbc.q);
	ur10mbc.alpha=sVectorToDof(ur10,(qref-qrefOld)/timeStep);
	dqref=sDofToVector(ur10,ur10mbc.alpha);
	ur10mbc.alphaD=sVectorToDof(ur10,(dqref-dqrefOld)/timeStep);

	err=qref-q;
	errV=dqref-dq;//SimControll::jointVel[i];
	uPD=sDofToVector(ur10,ur10mbc.alphaD)+Kp*err+Kd*errV;  //servo-base part
	

	qrefOld=qref;
	dqrefOld=dqref;


	jointCommand=fd.H()*uPD+fd.C();//Bq*uPD+c_qdq+gq;  model-base part

	printf("jointCommands:\n %10f %10f %10f %10f %10f %10f\n",jointCommand[0],jointCommand[1],jointCommand[2],jointCommand[3],jointCommand[4],jointCommand[5]);


}


void UR10Controller::computerJointCommand(std::vector<Eigen::Matrix<double, 6, 1>> pos,std::vector<Eigen::Matrix<double, 6, 1>> f,std::vector<Eigen::Matrix<double, 6, 1>>& torq)
{
	
	// if(pos.size() != f.size() || pos.size() != torq.size() )
	// 	printf("the array num not match!");

	// Vector6 qold,qdold;
	// for(int i = 0; i < ur10JointPath.numJoints(); ++i)  //init some variate
	// {
	// 	qold[i]=ur10->link(i+1)->q();
	// 	qdold[i]=ur10->link(i+1)->dq();
	// }

	// std::vector<Vector6>::iterator it;
	// for(it=pos.begin();it!=pos.end();it++)
	// {
	// 	Position tarPos;
	// 	tarPos.translation()=pos[it-pos.begin()].head<3>();
	// 	tarPos.linear()=rotFromXyzEuler(pos[it-pos.begin()](4), pos[it-pos.begin()](5), pos[it-pos.begin()](6));


	// 	if(ur10JointPath.calcInverseKinematics(tarPos)) 
	// 		;//printf("ik succeed.\n");
	// 	else
	// 		printf("ik failed!\n");

	// 	for(int i = 0; i < ur10JointPath.numJoints(); ++i)  //init some variate
	// 	{
	// 		ur10->link(i+1)->dq()=ur10->link(i+1)->q()-qold[i];
	// 		ur10->link(i+1)->ddq()=ur10->link(i+1)->dq()-qdold[i];
	// 		qold[i]=ur10->link(i+1)->q();
	// 		qdold[i]=ur10->link(i+1)->dq();

	// 	}

	// 	calcInverseDynamics(ur10->link(0));
	// 	// printf("jointTorques:\n %10f %10f %10f %10f %10f %10f\n",ur10->link(1)->u(),ur10->link(2)->u(),ur10->link(3)->u(),ur10->link(4)->u(),ur10->link(5)->u(),ur10->link(6)->u());
	// 	Eigen::MatrixXd J;
	// 	ur10JointPath.calcJacobian(J);

	// 	Vector6 uf;
	// 	uf=J.transpose()*f[it-pos.begin()];

	// 	for(int i = 0; i < ur10JointPath.numJoints(); ++i)  //init some variate
	// 		uf[i]+=ur10->link(i+1)->u();

	//     torq[it-pos.begin()]=uf;

	// }

}


void UR10Controller::jointParaInit()
{


	ur10mbc.zero(ur10);
	ur10mbc.gravity=Vector3d(0.,0.,9.81);
	ur10mbc.q={{},{0.*TO_RADIAN},{0.*TO_RADIAN},{90.*TO_RADIAN},{-90.*TO_RADIAN},{-90.*TO_RADIAN},{0.*TO_RADIAN}};
	// ur10mbc.q={{},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN}};
	// ur10mbc.q={{},{30.*TO_RADIAN},{75.*TO_RADIAN},{130.*TO_RADIAN},{-50.*TO_RADIAN},{-160.*TO_RADIAN},{80.*TO_RADIAN}};
	// ur10mbc.alpha={{},{30.*TO_RADIAN},{75.*TO_RADIAN},{130.*TO_RADIAN},{-50.*TO_RADIAN},{-160.*TO_RADIAN},{80.*TO_RADIAN}};
	// ur10mbc.alphaD={{},{30.*TO_RADIAN},{75.*TO_RADIAN},{130.*TO_RADIAN},{-50.*TO_RADIAN},{-160.*TO_RADIAN},{80.*TO_RADIAN}};
	

	rbd::forwardKinematics(ur10, ur10mbc);

	qref=sParamToVector(ur10,ur10mbc.q);
	dqref=sDofToVector(ur10,ur10mbc.alpha);
	qrefOld=qref;
	qold=qref;
	dqrefOld=dqref;
	dqold=dqref;




	dxrefOld=Eigen::Matrix<double, 6, 1>::Zero();


	xtarOld=ur10mbc.bodyPosW[6];
	xSenOld=xtarOld;


	//admittance controll  use xyz euler angle
	xa.head<3>()=xyzEulerFromRot(ur10mbc.bodyPosW[6].rotation().transpose());
	xa.tail<3>()=ur10mbc.bodyPosW[6].translation();
	dxa=Eigen::Matrix<double, 6, 1>::Zero();
	ddxa=Eigen::Matrix<double, 6, 1>::Zero();

	dxrefAdOld=Eigen::Matrix<double, 6, 1>::Zero();
	xrefAdOld=xa;


}


///vrep dyn param to D-H frame,output for checkout the identified dyn param
void UR10Controller::outputDynParamFile()
{
	///D-H frame in kinematic frame
	Eigen::Matrix3d Irot[6];
	Eigen::Matrix3d Irot1=Eigen::Matrix3d::Identity();
	Eigen::Matrix3d Irot2=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY())
							*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
	Eigen::Matrix3d Irot3=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY())
							*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
	Eigen::Matrix3d Irot4=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
	Eigen::Matrix3d Irot5=Eigen::Matrix3d::Identity();
	Eigen::Matrix3d Irot6=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
	
	///kinematic frame in D-H frame 
	Irot[0]= Irot1.inverse();
	Irot[1]= Irot2.inverse();
	Irot[2]= Irot3.inverse();
	Irot[3]= Irot4.inverse();
	Irot[4]= Irot5.inverse();
	Irot[5]= Irot6.inverse();

	for(int i=0;i<6;i++)
	{
		std::cout<< "L" << i+1 << "xx: " << (Irot[i]*ur10.body(i+1).inertia().inertia()*Irot[i].transpose())(0,0) <<std::endl;
		std::cout<< "L" << i+1 << "xy: " << (Irot[i]*ur10.body(i+1).inertia().inertia()*Irot[i].transpose())(0,1) <<std::endl;
		std::cout<< "L" << i+1 << "xz: " << (Irot[i]*ur10.body(i+1).inertia().inertia()*Irot[i].transpose())(0,2) <<std::endl;
		std::cout<< "L" << i+1 << "yy: " << (Irot[i]*ur10.body(i+1).inertia().inertia()*Irot[i].transpose())(1,1) <<std::endl;
		std::cout<< "L" << i+1 << "yz: " << (Irot[i]*ur10.body(i+1).inertia().inertia()*Irot[i].transpose())(1,2) <<std::endl;
		std::cout<< "L" << i+1 << "zz: " << (Irot[i]*ur10.body(i+1).inertia().inertia()*Irot[i].transpose())(2,2) <<std::endl;
		std::cout<< "l" << i+1 << "x: " << (Irot[i]*ur10.body(i+1).inertia().momentum())(0) <<std::endl;
		std::cout<< "l" << i+1 << "y: " << (Irot[i]*ur10.body(i+1).inertia().momentum())(1) <<std::endl;
		std::cout<< "l" << i+1 << "z: " << (Irot[i]*ur10.body(i+1).inertia().momentum())(2) <<std::endl;
		std::cout<< "m" << i+1 << ": " << ur10.body(i+1).inertia().mass() <<std::endl;

	}
	

}

// ///from vrep
// void UR10Controller::ur10_initRobot()
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
// 	rbd::Body UR10_Link_Base(rb0,"UR10_Link_Base");


// 	double mass1=3.9;
// 	Eigen::Matrix3d linkI1;
// 	linkI1 << 0.04*3.9,0,0,
// 			  0,0.04*3.9,0,
// 			  0,0,0.04*3.9;
// 	Eigen::Vector3d com1 =Eigen::Vector3d(0.000097,0.0052305,-0.00141);	
// 	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, com1,iden3d);	  
// 	sva::RBInertiad rb1(mass1,mass1*com1,I1);
// 	rbd::Body UR10_Link1(rb1,"UR10_Link1");


// 	double mass2=8.9;
// 	Eigen::Matrix3d linkI2;
// 	linkI2 << 0.06*8.9,0,0,
// 			  0,0.06*8.9,0,
// 			  0,0,0.04*8.9;
// 	Eigen::Vector3d com2 =Eigen::Vector3d(-0.0002055,-0.00122,0.299280545);	
// 	Eigen::Matrix3d I2=sva::inertiaToOrigin(linkI2,mass2, com2,iden3d);	  
// 	sva::RBInertiad rb2(mass2,mass2*com2,I2);
// 	rbd::Body UR10_Link2(rb2,"UR10_Link2");


// 	double mass3=6.1;
// 	Eigen::Matrix3d linkI3;
// 	linkI3 << 0.055*6.1,0,0,
// 			  0,0.055*6.1,0,
// 			  0,0,0.04*6.1;
// 	Eigen::Vector3d com3 =Eigen::Vector3d(0.0001174,-0.003402,0.27910829);
// 	Eigen::Matrix3d I3=sva::inertiaToOrigin(linkI3,mass3, com3,iden3d);		  
// 	sva::RBInertiad rb3(mass3,mass3*com3,I3);
// 	rbd::Body UR10_Link3(rb3,"UR10_Link3");


// 	double mass4=3.9;
// 	Eigen::Matrix3d linkI4;
// 	linkI4 << 0.04*3.9,0,0,
// 			  0,0.04*3.9,0,
// 			  0,0,0.04*3.9;
// 	Eigen::Vector3d com4 =Eigen::Vector3d(-0.00015,-0.00421,0.0022);
// 	Eigen::Matrix3d I4=sva::inertiaToOrigin(linkI4,mass4, com4,iden3d);	  
// 	sva::RBInertiad rb4(mass4,mass4*com4,I4);
// 	rbd::Body UR10_Link4(rb4,"UR10_Link4");


// 	double mass5=3.9;
// 	Eigen::Matrix3d linkI5;
// 	linkI5 << 0.04*3.9,0,0,
// 			  0,0.04*3.9,0,
// 			  0,0,0.04*3.9;
// 	Eigen::Vector3d com5 =Eigen::Vector3d(0.00010107,0.00205,-0.0041);	
// 	Eigen::Matrix3d I5=sva::inertiaToOrigin(linkI5,mass5, com5,iden3d);  
// 	sva::RBInertiad rb5(mass5,mass5*com5,I5);
// 	rbd::Body UR10_Link5(rb5,"UR10_Link5");


// 	double mass6=1.1;
// 	Eigen::Matrix3d linkI6;
// 	linkI6 << 0.01*1.1,0,0,   //in center of mass 
// 			  0,0.01*1.1,0,
// 			  0,0,0.01*1.1;
// 	Eigen::Vector3d com6 =Eigen::Vector3d(-0.000069,-0.01894,0.0002);
// 	Eigen::Matrix3d I6=sva::inertiaToOrigin(linkI6,mass6, com6,iden3d);	//in body frame orgin	  
// 	sva::RBInertiad rb6(mass6,mass6*com6,I6);
// 	rbd::Body UR10_Link6(rb6,"UR10_Link6");


	
// 	// std::cout<< "rb1:\n"<<rb1 <<std::endl<<std::endl;
// 	// std::cout<< "rb2:\n"<<rb2 <<std::endl<<std::endl;
// 	// std::cout<< "rb3:\n"<<rb3 <<std::endl<<std::endl;
// 	// std::cout<< "rb4:\n"<<rb4 <<std::endl<<std::endl;
// 	// std::cout<< "rb5:\n"<<rb5 <<std::endl<<std::endl;
// 	// std::cout<< "rb6:\n"<<rb6 <<std::endl<<std::endl;

// 	mbg.addBody(UR10_Link_Base);
// 	mbg.addBody(UR10_Link1);
// 	mbg.addBody(UR10_Link2);
// 	mbg.addBody(UR10_Link3);
// 	mbg.addBody(UR10_Link4);
// 	mbg.addBody(UR10_Link5);
// 	mbg.addBody(UR10_Link6);


// 	rbd::Joint j1(rbd::Joint::RevZ,true,"joint1");
// 	rbd::Joint j2(rbd::Joint::RevY,true,"joint2");
// 	rbd::Joint j3(rbd::Joint::RevY,true,"joint3");
// 	rbd::Joint j4(rbd::Joint::RevY,true,"joint4");
// 	rbd::Joint j5(rbd::Joint::RevZ,true,"joint5");
// 	rbd::Joint j6(rbd::Joint::RevY,true,"joint6");


// 	mbg.addJoint(j1);
// 	mbg.addJoint(j2);
// 	mbg.addJoint(j3);
// 	mbg.addJoint(j4);
// 	mbg.addJoint(j5);
// 	mbg.addJoint(j6);



// 	sva::PTransformd to1(Eigen::Vector3d(0.,0.,0.128));
// 	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link_Base",to1,"UR10_Link1",from1,"joint1");

// 	sva::PTransformd to2(Eigen::Vector3d(0.,0.176,0.));
// 	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link1",to2,"UR10_Link2",from2,"joint2");

// 	sva::PTransformd to3(Eigen::Vector3d(0.,-0.128,0.612));
// 	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link2",to3,"UR10_Link3",from3,"joint3");

// 	sva::PTransformd to4(Eigen::Vector3d(0,0.116,0.572));
// 	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link3",to4,"UR10_Link4",from4,"joint4");

// 	sva::PTransformd to5(Eigen::Vector3d(0.,0.,0.116));
// 	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link4",to5,"UR10_Link5",from5,"joint5");

// 	sva::PTransformd to6(Eigen::Vector3d(0.,0.092,0.));
// 	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link5",to6,"UR10_Link6",from6,"joint6");


// 	ur10=mbg.makeMultiBody("UR10_Link_Base",true);

// 	ur10mbc=rbd::MultiBodyConfig(ur10);

// 	jac=rbd::Jacobian(ur10,"UR10_Link6");

// }

// ///wangyan result
// void UR10Controller::ur10_initRobot()
// {
// 	rbd::MultiBodyGraph mbg;

// 	///wangyan result
// 	/////robot dynamic param in D-H coordinate
// 	double L1xx    =    1.1003423223150088e-05;
// 	double L1xy    =    0.0;
// 	double L1xz    =    0.0;
// 	double L1yy    =    1.1003423223150088e-05;
// 	double L1yz    =    0.0;
// 	double L1zz    =    8.582174094290116e-06;
// 	double l1x    =    0.0;
// 	double l1y    =    0.0;
// 	double l1z    =    0.0;
// 	double m1    =    1.744631704434645;
// 	double L2xx    =    0.807358612403818;
// 	double L2xy    =    -0.06220182497167579;
// 	double L2xz    =    0.30178427042140343;
// 	double L2yy    =    0.7752457790010959;
// 	double L2yz    =    -0.22801909743430632;
// 	double L2zz    =    1.0968976668876123;
// 	double l2x    =    2.399745840032498;
// 	double l2y    =    -0.03834163574240626;
// 	double l2z    =    -1.0000013557705723;
// 	double m2    =    10.00001190896946;
// 	double L3xx    =    0.17187255420270162;
// 	double L3xy    =    -0.11565409427256765;
// 	double L3xz    =    -0.09385572289302249;
// 	double L3yy    =    1.468171638945253;
// 	double L3yz    =    -0.1033138844658493;
// 	double L3zz    =    1.4163943972449773;
// 	double l3x    =    3.6335655732542587;
// 	double l3y    =    -0.021329412619258987;
// 	double l3z    =    0.5282705383453288;
// 	double m3    =    10.000010301376188;
// 	double L4xx    =    0.24661220985961588;
// 	double L4xy    =    0.003296400422864102;
// 	double L4xz    =    0.023081150452335483;
// 	double L4yy    =    0.003345155380628983;
// 	double L4yz    =    -0.025631679231644208;
// 	double L4zz    =    0.25448404723663526;
// 	double l4x    =    0.0011744582444715892;
// 	double l4y    =    -0.3093607930970952;
// 	double l4z    =    -0.07273729258343008;
// 	double m4    =    3.1079658181370853;
// 	double L5xx    =    0.013241409913143735;
// 	double L5xy    =    -0.005590867674539648;
// 	double L5xz    =    0.009743338424445636;
// 	double L5yy    =    0.002958461399853967;
// 	double L5yz    =    0.0001553360743826459;
// 	double L5zz    =    0.13824317267024688;
// 	double l5x    =    -0.0020243594557412103;
// 	double l5y    =    0.03895871536069664;
// 	double l5z    =    -0.010171222176613431;
// 	double m5    =    1.1744607113046133;
// 	double L6xx    =    0.01768658323254602;
// 	double L6xy    =    -0.001316941622243443;
// 	double L6xz    =    0.0050679996034952495;
// 	double L6yy    =    0.004538530829771588;
// 	double L6yz    =    0.0027227323247468508;
// 	double L6zz    =    0.008345537253667832;
// 	double l6x    =    -0.0003552830082578914;
// 	double l6y    =    -0.0004276517434320727;
// 	double l6z    =    -0.0572217442014842;
// 	double m6    =    1.1444347934031547;


// 	/////convert robot dynamic param from D-H coordinate to my kinematic coordinate
// 	/////my kinematic coordinate coinside with D-H coordinate
// 	double mass0=0.0;
// 	Eigen::Matrix3d linkI0;
// 	linkI0 << 0.,0.,0.,
// 			  0.,0.,0.,
// 			  0.,0.,0.;
// 	Eigen::Matrix3d Irot0=Eigen::Matrix3d::Identity();	
// 	Eigen::Vector3d h0 =Irot0*Eigen::Vector3d(0.,0.,0.);  
// 	Eigen::Matrix3d I0=sva::inertiaToOrigin(linkI0,mass0, Eigen::Vector3d(0.,0.,0.),Irot0);
// 	sva::RBInertiad rb0(mass0,h0,I0);
// 	rbd::Body UR10_Link_Base(rb0,"UR10_Link_Base");


// 	double mass1=m1;
// 	Eigen::Matrix3d linkI1;
// 	linkI1 << L1xx,L1xy,L1xz,
// 			  L1xy,L1yy,L1yz,
// 			  L1xz,L1yz,L1zz;
// 	Eigen::Matrix3d Irot1=Eigen::Matrix3d::Identity();
// 	Eigen::Vector3d h1 =Irot1*Eigen::Vector3d(l1x,l1y,l1z);	
// 	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, Eigen::Vector3d(0.,0.,0.),Irot1);	  
// 	sva::RBInertiad rb1(mass1,h1,I1);
// 	rbd::Body UR10_Link1(rb1,"UR10_Link1");


// 	double mass2=m2;
// 	Eigen::Matrix3d linkI2;
// 	linkI2 << L2xx,L2xy,L2xz,
// 			  L2xy,L2yy,L2yz,
// 			  L2xz,L2yz,L2zz;	
// 	Eigen::Matrix3d Irot2=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY())
// 								*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
// 	Eigen::Vector3d h2 =Irot2*Eigen::Vector3d(l2x,l2y,l2z);
// 	Eigen::Matrix3d I2=sva::inertiaToOrigin(linkI2,mass2, Eigen::Vector3d(0.,0.,0.),Irot2);	  
// 	sva::RBInertiad rb2(mass2,h2,I2);
// 	rbd::Body UR10_Link2(rb2,"UR10_Link2");

	
// 	double mass3=m3;
// 	Eigen::Matrix3d linkI3;
// 	linkI3 << L3xx,L3xy,L3xz,
// 			  L3xy,L3yy,L3yz,
// 			  L3xz,L3yz,L3zz;
// 	Eigen::Matrix3d Irot3=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY())
// 								*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
// 	Eigen::Vector3d h3 =Irot3*Eigen::Vector3d(l3x,l3y,l3z);
// 	Eigen::Matrix3d I3=sva::inertiaToOrigin(linkI3,mass3, Eigen::Vector3d(0.,0.,0.),Irot3);		  
// 	sva::RBInertiad rb3(mass3,h3,I3);
// 	rbd::Body UR10_Link3(rb3,"UR10_Link3");


// 	double mass4=m4;
// 	Eigen::Matrix3d linkI4;
// 	linkI4 << L4xx,L4xy,L4xz,
// 			  L4xy,L4yy,L4yz,
// 			  L4xz,L4yz,L4zz;
// 	Eigen::Matrix3d Irot4=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
// 	Eigen::Vector3d h4 =Irot4*Eigen::Vector3d(l4x,l4y,l4z);
// 	Eigen::Matrix3d I4=sva::inertiaToOrigin(linkI4,mass4, Eigen::Vector3d(0.,0.,0.),Irot4);	  
// 	sva::RBInertiad rb4(mass4,h4,I4);
// 	rbd::Body UR10_Link4(rb4,"UR10_Link4");


// 	double mass5=m5;
// 	Eigen::Matrix3d linkI5;
// 	linkI5 << L5xx,L5xy,L5xz,
// 			  L5xy,L5yy,L5yz,
// 			  L5xz,L5yz,L5zz;	
// 	Eigen::Matrix3d Irot5=Eigen::Matrix3d::Identity();
// 	Eigen::Vector3d h5 =Irot5*Eigen::Vector3d(l5x,l5y,l5z);	
// 	Eigen::Matrix3d I5=sva::inertiaToOrigin(linkI5,mass5, Eigen::Vector3d(0.,0.,0.),Irot5);  
// 	sva::RBInertiad rb5(mass5,h5,I5);
// 	rbd::Body UR10_Link5(rb5,"UR10_Link5");


// 	double mass6=m6;
// 	Eigen::Matrix3d linkI6;
// 	linkI6 << L6xx,L6xy,L6xz,
// 			  L6xy,L6yy,L6yz,
// 			  L6xz,L6yz,L6zz;
// 	Eigen::Matrix3d Irot6=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
// 	Eigen::Vector3d h6 =Irot6*Eigen::Vector3d(l6x,l6y,l6z);
// 	Eigen::Matrix3d I6=sva::inertiaToOrigin(linkI6,mass6, Eigen::Vector3d(0.,0.,0.),Irot6);	//in body frame orgin	  
// 	sva::RBInertiad rb6(mass6,h6,I6);
// 	rbd::Body UR10_Link6(rb6,"UR10_Link6");


// 	mbg.addBody(UR10_Link_Base);
// 	mbg.addBody(UR10_Link1);
// 	mbg.addBody(UR10_Link2);
// 	mbg.addBody(UR10_Link3);
// 	mbg.addBody(UR10_Link4);
// 	mbg.addBody(UR10_Link5);
// 	mbg.addBody(UR10_Link6);


// 	rbd::Joint j1(rbd::Joint::RevZ,true,"joint1");
// 	rbd::Joint j2(rbd::Joint::RevY,true,"joint2");
// 	rbd::Joint j3(rbd::Joint::RevY,true,"joint3");
// 	rbd::Joint j4(rbd::Joint::RevY,true,"joint4");
// 	rbd::Joint j5(rbd::Joint::RevZ,true,"joint5");
// 	rbd::Joint j6(rbd::Joint::RevY,true,"joint6");


// 	mbg.addJoint(j1);
// 	mbg.addJoint(j2);
// 	mbg.addJoint(j3);
// 	mbg.addJoint(j4);
// 	mbg.addJoint(j5);
// 	mbg.addJoint(j6);



// 	sva::PTransformd to1(Eigen::Vector3d(0.,0.,0.128));
// 	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link_Base",to1,"UR10_Link1",from1,"joint1");

// 	sva::PTransformd to2(Eigen::Vector3d(0.,0.176,0.));
// 	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link1",to2,"UR10_Link2",from2,"joint2");

// 	sva::PTransformd to3(Eigen::Vector3d(0.,-0.128,0.612));
// 	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link2",to3,"UR10_Link3",from3,"joint3");

// 	sva::PTransformd to4(Eigen::Vector3d(0,0.116,0.572));
// 	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link3",to4,"UR10_Link4",from4,"joint4");

// 	sva::PTransformd to5(Eigen::Vector3d(0.,0.,0.116));
// 	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link4",to5,"UR10_Link5",from5,"joint5");

// 	sva::PTransformd to6(Eigen::Vector3d(0.,0.092,0.));
// 	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link5",to6,"UR10_Link6",from6,"joint6");


// 	ur10=mbg.makeMultiBody("UR10_Link_Base",true);

// 	ur10mbc=rbd::MultiBodyConfig(ur10);

// 	jac=rbd::Jacobian(ur10,"UR10_Link6");

// } 


///sousa 2013 IROS result
void UR10Controller::ur10_initRobot()
{
	rbd::MultiBodyGraph mbg;

	///sousa 2013 IROS result(wls)
	/////robot dynamic param in D-H coordinate
	double L_1xx    =    0.0;
	double L_1xy    =    -0.0;
	double L_1xz    =    0.0;
	double L_1yy    =    -0.0;
	double L_1yz    =    0.0;
	double L_1zz    =    0.4479708855;
	double l_1x    =    -0.0;
	double l_1y    =    0.0;
	double l_1z    =    0.0;
	double m_1    =    0.0;
	double L_2xx    =    0.0097432346;
	double L_2xy    =    -0.3009010304;
	double L_2xz    =    0.330541869;
	double L_2yy    =    0.4382276509;
	double L_2yz    =    -0.1575147075;
	double L_2zz    =    0.976339128;
	double l_2x    =    3.7949949668;
	double l_2y    =    -0.0038248544;
	double l_2z    =    0.1576857517;
	double m_2    =    0.0138763462;
	double L_3xx    =    0.3131728862;
	double L_3xy    =    0.3984243248;
	double L_3xz    =    -0.0532199979;
	double L_3yy    =    0.1347979993;
	double L_3yz    =    0.0694089453;
	double L_3zz    =    0.0174162035;
	double l_3x    =    0.8965495725;
	double l_3y    =    -0.017890234;
	double l_3z    =    -0.1592864188;
	double m_3    =    2.879279872;
	double L_4xx    =    0.208351743;
	double L_4xy    =    -0.0120490611;
	double L_4xz    =    0.0493086107;
	double L_4yy    =    0.2396191425;
	double L_4yz    =    0.0126485542;
	double L_4zz    =    0.066845265;
	double l_4x    =    0.0025156473;
	double l_4y    =    0.1425670149;
	double l_4z    =    -0.0249153345;
	double m_4    =    3.4329902042;
	double L_5xx    =    0.1060537783;
	double L_5xy    =    0.0015847604;
	double L_5xz    =    0.0132681099;
	double L_5yy    =    0.1691432297;
	double L_5yz    =    0.0031687922;
	double L_5zz    =    0.0638314337;
	double l_5x    =    -0.0009913572;
	double l_5y    =    -0.1277578215;
	double l_5z    =    -0.078721309;
	double m_5    =    3.4201554814;
	double L_6xx    =    0.0896622857;
	double L_6xy    =    0.0001941699;
	double L_6xz    =    0.0076538361;
	double L_6yy    =    0.0802229262;
	double L_6yz    =    0.0042075726;
	double L_6zz    =    0.0106841665;
	double l_6x    =    -0.0016311847;
	double l_6y    =    0.0024764142;
	double l_6z    =    -0.0964989425;
	double m_6    =    3.4098396703;


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
	rbd::Body UR10_Link_Base(rb0,"UR10_Link_Base");


	double mass1=m_1;
	Eigen::Matrix3d linkI1;
	linkI1 << L_1xx,L_1xy,L_1xz,
			  L_1xy,L_1yy,L_1yz,
			  L_1xz,L_1yz,L_1zz;
	Eigen::Matrix3d Irot1=Eigen::Matrix3d::Identity();
	Eigen::Vector3d h1 =Irot1*Eigen::Vector3d(l_1x,l_1y,l_1z);	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, Eigen::Vector3d(0.,0.,0.),Irot1);	  
	sva::RBInertiad rb1(mass1,h1,I1);
	rbd::Body UR10_Link1(rb1,"UR10_Link1");


	double mass2=m_2;
	Eigen::Matrix3d linkI2;
	linkI2 << L_2xx,L_2xy,L_2xz,
			  L_2xy,L_2yy,L_2yz,
			  L_2xz,L_2yz,L_2zz;	
	Eigen::Matrix3d Irot2=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY())
								*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
	Eigen::Vector3d h2 =Irot2*Eigen::Vector3d(l_2x,l_2y,l_2z);
	Eigen::Matrix3d I2=sva::inertiaToOrigin(linkI2,mass2, Eigen::Vector3d(0.,0.,0.),Irot2);	  
	sva::RBInertiad rb2(mass2,h2,I2);
	rbd::Body UR10_Link2(rb2,"UR10_Link2");


	double mass3=m_3;
	Eigen::Matrix3d linkI3;
	linkI3 << L_3xx,L_3xy,L_3xz,
			  L_3xy,L_3yy,L_3yz,
			  L_3xz,L_3yz,L_3zz;
	Eigen::Matrix3d Irot3=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY())
								*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
	Eigen::Vector3d h3 =Irot3*Eigen::Vector3d(l_3x,l_3y,l_3z);
	Eigen::Matrix3d I3=sva::inertiaToOrigin(linkI3,mass3, Eigen::Vector3d(0.,0.,0.),Irot3);		  
	sva::RBInertiad rb3(mass3,h3,I3);
	rbd::Body UR10_Link3(rb3,"UR10_Link3");


	double mass4=m_4;
	Eigen::Matrix3d linkI4;
	linkI4 << L_4xx,L_4xy,L_4xz,
			  L_4xy,L_4yy,L_4yz,
			  L_4xz,L_4yz,L_4zz;
	Eigen::Matrix3d Irot4=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
	Eigen::Vector3d h4 =Irot4*Eigen::Vector3d(l_4x,l_4y,l_4z);
	Eigen::Matrix3d I4=sva::inertiaToOrigin(linkI4,mass4, Eigen::Vector3d(0.,0.,0.),Irot4);	  
	sva::RBInertiad rb4(mass4,h4,I4);
	rbd::Body UR10_Link4(rb4,"UR10_Link4");


	double mass5=m_5;
	Eigen::Matrix3d linkI5;
	linkI5 << L_5xx,L_5xy,L_5xz,
			  L_5xy,L_5yy,L_5yz,
			  L_5xz,L_5yz,L_5zz;	
	Eigen::Matrix3d Irot5=Eigen::Matrix3d::Identity();
	Eigen::Vector3d h5 =Irot5*Eigen::Vector3d(l_5x,l_5y,l_5z);	
	Eigen::Matrix3d I5=sva::inertiaToOrigin(linkI5,mass5, Eigen::Vector3d(0.,0.,0.),Irot5);  
	sva::RBInertiad rb5(mass5,h5,I5);
	rbd::Body UR10_Link5(rb5,"UR10_Link5");


	double mass6=m_6;
	Eigen::Matrix3d linkI6;
	linkI6 << L_6xx,L_6xy,L_6xz,
			  L_6xy,L_6yy,L_6yz,
			  L_6xz,L_6yz,L_6zz;
	Eigen::Matrix3d Irot6=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
	Eigen::Vector3d h6 =Irot6*Eigen::Vector3d(l_6x,l_6y,l_6z);
	Eigen::Matrix3d I6=sva::inertiaToOrigin(linkI6,mass6, Eigen::Vector3d(0.,0.,0.),Irot6);	//in body frame orgin	  
	sva::RBInertiad rb6(mass6,h6,I6);
	rbd::Body UR10_Link6(rb6,"UR10_Link6");


	mbg.addBody(UR10_Link_Base);
	mbg.addBody(UR10_Link1);
	mbg.addBody(UR10_Link2);
	mbg.addBody(UR10_Link3);
	mbg.addBody(UR10_Link4);
	mbg.addBody(UR10_Link5);
	mbg.addBody(UR10_Link6);


	rbd::Joint j1(rbd::Joint::RevZ,true,"joint1");
	rbd::Joint j2(rbd::Joint::RevY,true,"joint2");
	rbd::Joint j3(rbd::Joint::RevY,true,"joint3");
	rbd::Joint j4(rbd::Joint::RevY,true,"joint4");
	rbd::Joint j5(rbd::Joint::RevZ,true,"joint5");
	rbd::Joint j6(rbd::Joint::RevY,true,"joint6");


	mbg.addJoint(j1);
	mbg.addJoint(j2);
	mbg.addJoint(j3);
	mbg.addJoint(j4);
	mbg.addJoint(j5);
	mbg.addJoint(j6);



	sva::PTransformd to1(Eigen::Vector3d(0.,0.,0.128));
	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("UR10_Link_Base",to1,"UR10_Link1",from1,"joint1");

	sva::PTransformd to2(Eigen::Vector3d(0.,0.176,0.));
	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("UR10_Link1",to2,"UR10_Link2",from2,"joint2");

	sva::PTransformd to3(Eigen::Vector3d(0.,-0.128,0.612));
	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("UR10_Link2",to3,"UR10_Link3",from3,"joint3");

	sva::PTransformd to4(Eigen::Vector3d(0,0.116,0.572));
	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("UR10_Link3",to4,"UR10_Link4",from4,"joint4");

	sva::PTransformd to5(Eigen::Vector3d(0.,0.,0.116));
	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("UR10_Link4",to5,"UR10_Link5",from5,"joint5");

	sva::PTransformd to6(Eigen::Vector3d(0.,0.092,0.));
	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("UR10_Link5",to6,"UR10_Link6",from6,"joint6");


	ur10=mbg.makeMultiBody("UR10_Link_Base",true);

	ur10mbc=rbd::MultiBodyConfig(ur10);

	jac=rbd::Jacobian(ur10,"UR10_Link6");

} 


// ///sousa 2014 IROS result
// void UR10Controller::ur10_initRobot()
// {
// 	rbd::MultiBodyGraph mbg;

// 	///sousa 2014 IROS result (wls)
// 	/////robot dynamic param in D-H coordinate
	

// 	double L_1xx    =    2.582239198925221e-16;
// 	double L_1xy    =    -1.094839181539446e-16;
// 	double L_1xz    =    0.0;
// 	double L_1yy    =    -8.340031004485599e-17;
// 	double L_1yz    =    0.0;
// 	double L_1zz    =    0.501285947507327;
// 	double l_1x    =    -3.4089770308621807e-17;
// 	double l_1y    =    0.0;
// 	double l_1z    =    5.837720472499818e-17;
// 	double m_1    =    2.0462264333873076e-18;
// 	double L_2xx    =    0.0010432940087180542;
// 	double L_2xy    =    -0.20847279333561713;
// 	double L_2xz    =    0.31514073150964883;
// 	double L_2yy    =    0.5002426534986084;
// 	double L_2yz    =    -0.054927706071427954;
// 	double L_2zz    =    1.1799777140794525;
// 	double l_2x    =    3.7012264275753104;
// 	double l_2y    =    0.01358675333054423;
// 	double l_2z    =    0.17645265352257958;
// 	double m_2    =    0.015527833509986967;
// 	double L_3xx    =    0.3362186147497067;
// 	double L_3xy    =    0.35261522989191574;
// 	double L_3xz    =    -0.023077014311984;
// 	double L_3yy    =    0.16506733275762042;
// 	double L_3yz    =    0.03331134914798354;
// 	double L_3zz    =    0.0006494857579790736;
// 	double l_3x    =    0.8490282570511556;
// 	double l_3y    =    -0.025038885340891163;
// 	double l_3z    =    -0.1447426767232019;
// 	double m_3    =    2.9203088581968455;
// 	double L_4xx    =    0.22888533814279288;
// 	double L_4xy    =    -0.010700389360604471;
// 	double L_4xz    =    0.0480001825387179;
// 	double L_4yy    =    0.27240060936453464;
// 	double L_4yz    =    0.016513788810248578;
// 	double L_4zz    =    0.058991562425868505;
// 	double l_4x    =    0.0024703585303843555;
// 	double l_4y    =    0.14617391588080386;
// 	double l_4z    =    -0.0152442847150467;
// 	double m_4    =    3.4516592720427104;
// 	double L_5xx    =    0.11145547239505896;
// 	double L_5xy    =    0.0015688801005760116;
// 	double L_5xz    =    0.013277496126345654;
// 	double L_5yy    =    0.17642142817360257;
// 	double L_5yz    =    0.0031628036567820797;
// 	double L_5zz    =    0.06195854227005597;
// 	double l_5x    =    -0.0009996594182182462;
// 	double l_5y    =    -0.12893200488019022;
// 	double l_5z    =    -0.07938647494887448;
// 	double m_5    =    3.4385767693745892;
// 	double L_6xx    =    0.09142604618608917;
// 	double L_6xy    =    0.0001935493606393923;
// 	double L_6xz    =    0.0076531215494778666;
// 	double L_6yy    =    0.08198796847902587;
// 	double L_6yz    =    0.004206780000129019;
// 	double L_6zz    =    0.010684308275062785;
// 	double l_6x    =    -0.0016315592548608211;
// 	double l_6y    =    0.002476337015479084;
// 	double l_6z    =    -0.09702382618180909;
// 	double m_6    =    3.4281828011457374;


// 	/////convert robot dynamic param from D-H coordinate to my kinematic coordinate
// 	/////my kinematic coordinate coinside with D-H coordinate
// 	double mass0=0.0;
// 	Eigen::Matrix3d linkI0;
// 	linkI0 << 0.,0.,0.,
// 			  0.,0.,0.,
// 			  0.,0.,0.;
// 	Eigen::Matrix3d Irot0=Eigen::Matrix3d::Identity();	
// 	Eigen::Vector3d h0 =Irot0*Eigen::Vector3d(0.,0.,0.);  
// 	Eigen::Matrix3d I0=sva::inertiaToOrigin(linkI0,mass0, Eigen::Vector3d(0.,0.,0.),Irot0);
// 	sva::RBInertiad rb0(mass0,h0,I0);
// 	rbd::Body UR10_Link_Base(rb0,"UR10_Link_Base");


// 	double mass1=m_1;
// 	Eigen::Matrix3d linkI1;
// 	linkI1 << L_1xx,L_1xy,L_1xz,
// 			  L_1xy,L_1yy,L_1yz,
// 			  L_1xz,L_1yz,L_1zz;
// 	Eigen::Matrix3d Irot1=Eigen::Matrix3d::Identity();
// 	Eigen::Vector3d h1 =Irot1*Eigen::Vector3d(l_1x,l_1y,l_1z);	
// 	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, Eigen::Vector3d(0.,0.,0.),Irot1);	  
// 	sva::RBInertiad rb1(mass1,h1,I1);
// 	rbd::Body UR10_Link1(rb1,"UR10_Link1");


// 	double mass2=m_2;
// 	Eigen::Matrix3d linkI2;
// 	linkI2 << L_2xx,L_2xy,L_2xz,
// 			  L_2xy,L_2yy,L_2yz,
// 			  L_2xz,L_2yz,L_2zz;	
// 	Eigen::Matrix3d Irot2=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY())
// 								*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
// 	Eigen::Vector3d h2 =Irot2*Eigen::Vector3d(l_2x,l_2y,l_2z);
// 	Eigen::Matrix3d I2=sva::inertiaToOrigin(linkI2,mass2, Eigen::Vector3d(0.,0.,0.),Irot2);	  
// 	sva::RBInertiad rb2(mass2,h2,I2);
// 	rbd::Body UR10_Link2(rb2,"UR10_Link2");


// 	double mass3=m_3;
// 	Eigen::Matrix3d linkI3;
// 	linkI3 << L_3xx,L_3xy,L_3xz,
// 			  L_3xy,L_3yy,L_3yz,
// 			  L_3xz,L_3yz,L_3zz;
// 	Eigen::Matrix3d Irot3=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitY())
// 								*Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
// 	Eigen::Vector3d h3 =Irot3*Eigen::Vector3d(l_3x,l_3y,l_3z);
// 	Eigen::Matrix3d I3=sva::inertiaToOrigin(linkI3,mass3, Eigen::Vector3d(0.,0.,0.),Irot3);		  
// 	sva::RBInertiad rb3(mass3,h3,I3);
// 	rbd::Body UR10_Link3(rb3,"UR10_Link3");


// 	double mass4=m_4;
// 	Eigen::Matrix3d linkI4;
// 	linkI4 << L_4xx,L_4xy,L_4xz,
// 			  L_4xy,L_4yy,L_4yz,
// 			  L_4xz,L_4yz,L_4zz;
// 	Eigen::Matrix3d Irot4=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
// 	Eigen::Vector3d h4 =Irot4*Eigen::Vector3d(l_4x,l_4y,l_4z);
// 	Eigen::Matrix3d I4=sva::inertiaToOrigin(linkI4,mass4, Eigen::Vector3d(0.,0.,0.),Irot4);	  
// 	sva::RBInertiad rb4(mass4,h4,I4);
// 	rbd::Body UR10_Link4(rb4,"UR10_Link4");


// 	double mass5=m_5;
// 	Eigen::Matrix3d linkI5;
// 	linkI5 << L_5xx,L_5xy,L_5xz,
// 			  L_5xy,L_5yy,L_5yz,
// 			  L_5xz,L_5yz,L_5zz;	
// 	Eigen::Matrix3d Irot5=Eigen::Matrix3d::Identity();
// 	Eigen::Vector3d h5 =Irot5*Eigen::Vector3d(l_5x,l_5y,l_5z);	
// 	Eigen::Matrix3d I5=sva::inertiaToOrigin(linkI5,mass5, Eigen::Vector3d(0.,0.,0.),Irot5);  
// 	sva::RBInertiad rb5(mass5,h5,I5);
// 	rbd::Body UR10_Link5(rb5,"UR10_Link5");


// 	double mass6=m_6;
// 	Eigen::Matrix3d linkI6;
// 	linkI6 << L_6xx,L_6xy,L_6xz,
// 			  L_6xy,L_6yy,L_6yz,
// 			  L_6xz,L_6yz,L_6zz;
// 	Eigen::Matrix3d Irot6=Eigen::Matrix3d(Eigen::AngleAxisd(-PI_2, Eigen::Vector3d::UnitX()));
// 	Eigen::Vector3d h6 =Irot6*Eigen::Vector3d(l_6x,l_6y,l_6z);
// 	Eigen::Matrix3d I6=sva::inertiaToOrigin(linkI6,mass6, Eigen::Vector3d(0.,0.,0.),Irot6);	//in body frame orgin	  
// 	sva::RBInertiad rb6(mass6,h6,I6);
// 	rbd::Body UR10_Link6(rb6,"UR10_Link6");


// 	mbg.addBody(UR10_Link_Base);
// 	mbg.addBody(UR10_Link1);
// 	mbg.addBody(UR10_Link2);
// 	mbg.addBody(UR10_Link3);
// 	mbg.addBody(UR10_Link4);
// 	mbg.addBody(UR10_Link5);
// 	mbg.addBody(UR10_Link6);


// 	rbd::Joint j1(rbd::Joint::RevZ,true,"joint1");
// 	rbd::Joint j2(rbd::Joint::RevY,true,"joint2");
// 	rbd::Joint j3(rbd::Joint::RevY,true,"joint3");
// 	rbd::Joint j4(rbd::Joint::RevY,true,"joint4");
// 	rbd::Joint j5(rbd::Joint::RevZ,true,"joint5");
// 	rbd::Joint j6(rbd::Joint::RevY,true,"joint6");


// 	mbg.addJoint(j1);
// 	mbg.addJoint(j2);
// 	mbg.addJoint(j3);
// 	mbg.addJoint(j4);
// 	mbg.addJoint(j5);
// 	mbg.addJoint(j6);



// 	sva::PTransformd to1(Eigen::Vector3d(0.,0.,0.128));
// 	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link_Base",to1,"UR10_Link1",from1,"joint1");

// 	sva::PTransformd to2(Eigen::Vector3d(0.,0.176,0.));
// 	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link1",to2,"UR10_Link2",from2,"joint2");

// 	sva::PTransformd to3(Eigen::Vector3d(0.,-0.128,0.612));
// 	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link2",to3,"UR10_Link3",from3,"joint3");

// 	sva::PTransformd to4(Eigen::Vector3d(0,0.116,0.572));
// 	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link3",to4,"UR10_Link4",from4,"joint4");

// 	sva::PTransformd to5(Eigen::Vector3d(0.,0.,0.116));
// 	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link4",to5,"UR10_Link5",from5,"joint5");

// 	sva::PTransformd to6(Eigen::Vector3d(0.,0.092,0.));
// 	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
// 	mbg.linkBodies("UR10_Link5",to6,"UR10_Link6",from6,"joint6");


// 	ur10=mbg.makeMultiBody("UR10_Link_Base",true);

// 	ur10mbc=rbd::MultiBodyConfig(ur10);

// 	jac=rbd::Jacobian(ur10,"UR10_Link6");

// }





