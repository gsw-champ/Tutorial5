#include <ros/ros.h>
#include "GraspClass.h"
using namespace Grasp;

int main(int argc, char**argv){
    ros::init(argc, argv, "Grasp_node");
    ros::NodeHandle n;
    ros::MultiThreadedSpinner spinner(3);
    GraspClass grasp("Grasp_node",n); 
    spinner.spin();   

}
