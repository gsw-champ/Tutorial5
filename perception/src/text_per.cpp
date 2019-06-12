#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "perception/perc.h"
#include <cstdlib>
#include <iostream>

using namespace std;


int main(int argc, char**argv){

    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::ServiceClient test_client = n.serviceClient<perception::perc>("find_box");
    perception::perc test;
    std::string class_name;
    class_name = "bottle";
    test.request.class_name = class_name;
    if(test_client.call(test)){
	ROS_INFO("SUCCESS");
    ROS_INFO("X: %f, Y: %f, Z: %f", test.response.point.point.x, test.response.point.point.y, test.response.point.point.z);
    }
    else
	ROS_INFO("FALSE");
    

}
