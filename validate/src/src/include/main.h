#include "LjhnController.h"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "SimControll.h"
#include <iostream> 


using namespace std;
using namespace ljnoid;



LjhnController* ljhnCon;

double err[6];
double errV[6];


ros::Publisher err_pub;
ros::Publisher errV_pub;


void* control_thread(void* ptr);

void advertise();

void debug_publish();








