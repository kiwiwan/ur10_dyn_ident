
#ifndef  MINI_CONTROLLER_H
#define  MINI_CONTROLLER_H

#include "MultiBody.h"
#include "MultiBodyConfig.h"
#include "MultiBodyGraph.h"


  
namespace ljnoid {

class LjhnController
{

public:
	LjhnController();
	virtual ~LjhnController();

	void constructRobot();
	void constructTorso();
	void constructLeftLeg();
	void constructRightLeg();
	void constructLeftArm();
	void constructRightArm();
	void outputDynParamFile();


	void modelJointTrajControll();
	void jointParaInit();
	void simStart();
	static LjhnController* GetInstance()
	{
        return m_UniqueInstance;
    }



    rbd::MultiBodyGraph mbg;
    rbd::MultiBody ljhn;
	rbd::MultiBodyConfig ljhnmbc;

	rbd::MultiBodyGraph ljhnFloatMbg;
	rbd::MultiBody ljhnFloat;
	rbd::MultiBodyConfig ljhnFloatmbc;

	rbd::MultiBodyGraph leftLegMbg;
	rbd::MultiBody leftLeg;
	rbd::MultiBodyConfig leftLegMbc;



	enum jointId
	{
		LLEG_JOINT_START=0,
		LLEG_JOINT_END=6,
		LLEG_JOINT_NUM=6,

		RLEG_JOINT_START=6,
		RLEG_JOINT_END=12,
		RLEG_JOINT_NUM=6,

		LARM_JOINT_START=12,
		LARM_JOINT_END=15,
		LARM_JOINT_NUM=3,

		RARM_JOINT_START=15,
		RARM_JOINT_END=18,
		RARM_JOINT_NUM=3,

		JOINT_NUM=12,

		FLOATING_FREEDOM_NUM=6,
		FLOATING_CONFIG_NUM=7,
		JOINT_FREEDOM_NUM=18,
		JOINT_CONFIG_NUM=19,

		LLEG_JOINT_FREEDOM_NUM=12,
		LLEG_JOINT_CONFIG_NUM=13,


	};

	double timeStep;
	Eigen::Matrix<double, JOINT_NUM, 1> jointCommand;

	Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_jointCommand;

	Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_jointCommand;

	Eigen::Vector3d pl;

	enum linkId
	{
		TORSO_LINK=0,

		LLEG_LINK_1=1,
		LLEG_LINK_2=2,
		LLEG_LINK_3=3,
		LLEG_LINK_4=4,
		LLEG_LINK_5=5,
		LLEG_LINK_6=6,
		LLEG_LINK_SOLE=7,
		LLEG_LINK_NUM=7,

		RLEG_LINK_1=8,
		RLEG_LINK_2=9,
		RLEG_LINK_3=10,
		RLEG_LINK_4=11,
		RLEG_LINK_5=12,
		RLEG_LINK_6=13,
		RLEG_LINK_SOLE=14,
		RLEG_LINK_NUM=7,

		LARM_LINK_1=15,
		LARM_LINK_2=16,
		LARM_LINK_3=17,
		LARM_LINK_HAND=18,
		LARM_LINK_NUM=4,

		RARM_LINK_1=19,
		RARM_LINK_2=20,
		RARM_LINK_3=21,
		RARM_LINK_HAND=22,
		RARM_LINK_NUM=4,

		LINK_NUM=23,
	};


private:
	
  ///fixed base 
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




	static LjhnController* m_UniqueInstance;



  ///left leg debug robot
	Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_uPD;
	Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_err;
	Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_errV;


	
	Eigen::Matrix<double, LLEG_JOINT_CONFIG_NUM, 1> leftLeg_qref;
	Eigen::Matrix<double, LLEG_JOINT_CONFIG_NUM, 1> leftLeg_qrefOld;
	Eigen::Matrix<double, LLEG_JOINT_CONFIG_NUM, 1> leftLeg_q;
	Eigen::Matrix<double, LLEG_JOINT_CONFIG_NUM, 1> leftLeg_qold;
	Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_dqref;
	Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_dqrefOld;
	Eigen::Matrix<double, LLEG_JOINT_FREEDOM_NUM, 1> leftLeg_dq;

	Eigen::Matrix<double, LLEG_JOINT_NUM, 1> dqd,qd;



  ///floating base
	Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_uPD;
	Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_err;
	Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_errV;


	
	Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_qref;
	Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_qrefOld;
	Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_q;
	Eigen::Matrix<double, JOINT_CONFIG_NUM, 1> float_qold;
	Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_dqref;
	Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_dqrefOld;
	Eigen::Matrix<double, JOINT_FREEDOM_NUM, 1> float_dq;


};







}










#endif

