#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "GraspClass.h"
#include <cstdlib>




int main(int argc, char**argv){

    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::ServiceClient test_client = n.serviceClient<tutorial5::Grasp>("Grasp_node");
    tutorial5::Grasp test;
    test.request.point.header.frame_id = "base_link";
    test.request.point.point.x = 0.4;
    test.request.point.point.y = -0.3;
    test.request.point.point.z = 0.26;
    if(test_client.call(test))
	ROS_INFO("SUCCESS");
    else
	ROS_INFO("FALSE");
    

}
