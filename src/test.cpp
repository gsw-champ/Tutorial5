#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "GraspClass.h"
#include <cstdlib>
#include <iostream>

using namespace std;


int main(int argc, char**argv){

    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::ServiceClient test_client = n.serviceClient<tutorial5::Grasp>("Grasp_node");
    tutorial5::Grasp test;
    double x,y,z;
    std::cout<<"x"<<endl;
    std::cin>> x;
    std::cout<<"y"<<endl;
    std::cin>> y;
    std::cout<<"z"<<endl;
    std::cin>> z;
    test.request.point.header.frame_id = "base_link";
    test.request.point.point.x = x;
    test.request.point.point.y = y;
    test.request.point.point.z = z;
    if(test_client.call(test))
	ROS_INFO("SUCCESS");
    else
	ROS_INFO("FALSE");
    

}
