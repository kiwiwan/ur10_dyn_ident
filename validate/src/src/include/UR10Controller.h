
#ifndef UR10_CONTROLLER_H
#define  UR10_CONTROLLER_H


#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "Jacobian.h"



  
namespace ljnoid {

class UR10Controller
{

public:
	UR10Controller();
	virtual ~UR10Controller();

	void ur10_initRobot();
	
	void outputDynParamFile();///vrep dyn param to D-H frame,output for checkout the identified dyn param

	void idDynPara();
	void modelJointTrajControll();
	void modelCartesianTrajControll(sva::PTransformd tarPos);

	void impedanceControll(sva::PTransformd tarPos);
	void admittanceControll(sva::PTransformd tarPos);


	void computerJointCommand(std::vector<Eigen::Matrix<double, 6, 1>> pos,std::vector<Eigen::Matrix<double, 6, 1>> f,std::vector<Eigen::Matrix<double, 6, 1>>& torq);
	void jointParaInit();
	static UR10Controller* getInstance()
	{
        return m_UniqueInstance;
    }




	rbd::MultiBody ur10;
	rbd::MultiBodyConfig ur10mbc;
	


	enum jointId
	{
		JOINT_START=0,
		JOINT_END=6,
		

		JOINT_NUM=6,
	};

	double timeStep;
	Eigen::Matrix<double, JOINT_NUM, 1> jointCommand;

	

	enum linkId
	{
		LINK_BASE=0,

		LINK_1=1,
		LINK_2=2,
		LINK_3=3,
		LINK_4=4,
		LINK_5=5,
		LINK_6=6,


		LINK_NUM=7,
	};



	rbd::Jacobian jac;

	Eigen::Matrix<double, JOINT_NUM, JOINT_NUM> Kp;
	Eigen::Matrix<double, JOINT_NUM, JOINT_NUM> Kd;
	Eigen::Matrix<double, JOINT_NUM, 1> uPD;
	Eigen::Matrix<double, JOINT_NUM, 1> err;
	Eigen::Matrix<double, JOINT_NUM, 1> errV;


	
	Eigen::Matrix<double, JOINT_NUM, 1> qref;
	Eigen::Matrix<double, JOINT_NUM, 1> qrefOld;
	Eigen::Matrix<double, JOINT_NUM, 1> q;
	Eigen::Matrix<double, JOINT_NUM, 1> qold;
	Eigen::Matrix<double, JOINT_NUM, 1> dqref;
	Eigen::Matrix<double, JOINT_NUM, 1> dqrefOld;
	Eigen::Matrix<double, JOINT_NUM, 1> dq;
	Eigen::Matrix<double, JOINT_NUM, 1> ddq;
	Eigen::Matrix<double, JOINT_NUM, 1> dqold;


private:

	static UR10Controller* m_UniqueInstance;

	Eigen::Matrix<double, 6, 6> kpx,kdx;
	Eigen::Matrix<double, 6, 1> uxPD,ux_ddq;
	Eigen::Matrix<double, 6, 1> errx,errxV;



	Eigen::Matrix<double, 6, 1> dxref;
	Eigen::Matrix<double, 6, 1> dxrefOld;
	Eigen::Matrix<double, 6, 1> dx;
	Eigen::Matrix<double, 6, 1> ddxref;



	//admittance controll  use xyz euler angle
	Eigen::Matrix<double, 6, 1> dxrefAd;
	Eigen::Matrix<double, 6, 1> dxrefAdOld;
	Eigen::Matrix<double, 6, 1> xrefAd;
	Eigen::Matrix<double, 6, 1> xrefAdOld;
	Eigen::Matrix<double, 6, 1> ddxrefAd;
	Eigen::Matrix<double, 6, 1> xa,dxa,ddxa;
	Eigen::Matrix<double, 6, 1> errxAdm,errxVAdm;

	sva::PTransformd xSenOld,xtarOld;


	Eigen::MatrixXd regressors;
	Eigen::VectorXd torques;
	Eigen::VectorXd inertiaVec;
	int samplesCount;
	int samples;


};







}










#endif

