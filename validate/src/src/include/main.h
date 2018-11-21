// #include "OnedofController.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float64MultiArray.h>

// #include "SimControll.h"
#include <fstream>  
#include <iostream> 
#include "MX64_V2.h"


using namespace std;
using namespace MX64_V2;
// using namespace ljnoid;



MX64* mx64;

int call_count;

double qold = 0.;
double dqold = 0.;
double qrefOld = 0.;
double dqrefOld = 0.;

double timeStep = 0.02;//20 ms

ros::Publisher goalPos_pub;
ros::Publisher curPos_pub;
ros::Publisher curVel_pub;
ros::Publisher goalTorque_pub;
ros::Publisher curTorque_pub;

void timerCallback(int in);
void initTimer();

void trajExcite();
double modelJointTrajControll();
void torqueControll(double pos);

void advertise();

void debug_publish();

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}


ofstream mx64_traj_results; 

void readExciteTraj();
double q_excit[8000];
int q_excit_count;




