#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <pose_tensorflow/SaveImage.h>
#include <geometry_msgs/Twist.h>


void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    static cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::stringstream sstream;
    sstream << "image/a.png";
    cv::imwrite(sstream.str(), cv_ptr->image);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_image");
    ros::NodeHandle nh;

    ros::ServiceClient pose_client = nh.serviceClient<pose_tensorflow::SaveImage>("person_pose");
    pose_tensorflow::SaveImage srv;
    image_transport::ImageTransport it(nh);
    ros::Rate r(5);
    image_transport::Subscriber image_sub = it.subscribe("xtion/rgb/image_raw", 1, image_callback);
    //if (pose_client.call(srv))
    //{
        // geometry_msgs::Twist msg;
        // ros::Publisher turn_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel",1);
        // msg.angular.z = 0.1;
        // ros::Time start_time = ros::Time::now();
        // while ((ros::Time::now() - start_time) < ros::Duration(10)){
        //     turn_pub.publish(msg);
        //     ROS_INFO("Turning...");
        // }
        //image_transport::Subscriber image_sub_twice = it.subscribe("xtion/rgb/image_raw", 1, image_callback);

    //}
    std_msgs::Empty empty;
    srv.request.request = empty;


    while(ros::ok()){

        if (pose_client.call(srv)){
            ROS_INFO_STREAM("SUCCESS");
        }
        else{
            ROS_INFO_STREAM("FALSE");
        }

        ros::spinOnce();
        r.sleep();        

    }

    return 0;
}
