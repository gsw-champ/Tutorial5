#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <save_image/image.h>

// #include <save_image/image.h>


void image_callback(const sensor_msgs::ImageConstPtr &msg, save_image::image::Request &req, save_image::image::Response &res)
{
    static cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    ROS_INFO_STREAM("#######");
    std::stringstream sstream;
    bool do_save, save_status;
    save_status = false;
    do_save = req.do_save;
    //TODO!!!!!: save as a service input

    while (do_save)
    {
        ROS_INFO_STREAM(do_save);
        sstream << "image/image.png" ;
        cv::imwrite( sstream.str(),  cv_ptr->image );
        ROS_INFO("############");
        save_status = res.save_status;
    }
}



int main(int argc, char **argv){
    
    ros::init(argc, argv, "save_image");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("xtion/rgb/image_raw", 1, image_callback);
    ros::ServiceServer image_service = nh.advertiseService("image_saved", image_callback);

    ros::spin();

    return 0;
}
