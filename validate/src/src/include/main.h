#include "UR10Controller.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "SimControll.h"
#include <iostream> 


using namespace std;
using namespace ljnoid;



UR10Controller* ur10Con;

double err[6];
double errV[6];


ros::Publisher q_pub;
ros::Publisher dq_pub;
ros::Publisher ddq_pub;

void* control_thread(void* ptr);

void advertise();

void debug_publish();

sva::RBInertiad randomInertia();







