
#ifndef ONEDOF_CONTROLLER_H
#define  ONEDOF_CONTROLLER_H


#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "Jacobian.h"



  
namespace ljnoid {

class OnedofController
{

public:
	OnedofController();
	virtual ~OnedofController();

	void onedof_initRobot();
	
	void outputDynParamFile();///vrep dyn param to D-H frame,output for checkout the identified dyn param

	void idDynPara();
	void modelJointTrajControll();
	void jointParaInit();
	static OnedofController* getInstance()
	{
        return m_UniqueInstance;
    }




	rbd::MultiBody onedof;
	rbd::MultiBodyConfig onedofmbc;
	


	enum jointId
	{
		JOINT_START=0,
		JOINT_END=1,
		

		JOINT_NUM=1,
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


		LINK_NUM=1,
	};



	// rbd::Jacobian jac;

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

	static OnedofController* m_UniqueInstance;

	


};







}










#endif

