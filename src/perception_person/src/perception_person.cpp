#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Char.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <vector>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>

// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

//#include <image_geometry/pinhole_camera_model.h>

#include <perception_person/PercepPerson.h>

using namespace std;
using namespace cv;

class Perception
{

private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Subscriber sub_cloud;
  // ros::Subscriber sub_PP;
  // ros::Publisher pub_point3D_person;
  ros::ServiceServer person_pose_service;
      // ros::ServiceServer srv_Perception;

      pcl::PointCloud<pcl::PointXYZRGB>
          pc_point3D;

  tf::TransformListener listener_;

  // string type_obj, class_obj;
  int CenterPoints[2];

  geometry_msgs::PointStamped point3D;
  geometry_msgs::PointStamped point3D_base;
  geometry_msgs::PointStamped person_base_point;

  //------------------ Callbacks -------------------
  // process point cloud
  void processCloud(const sensor_msgs::PointCloud2ConstPtr &pc);
  // process 2D to 3D point
  bool processPersonPos(perception_person::PercepPerson::Request &req, perception_person::PercepPerson::Response &res);
  // transform to tf base
  geometry_msgs::PointStamped transformPoint(const tf::TransformListener &listener, const geometry_msgs::PointStamped laser_point);

public:
  Perception(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
  {
    // sub the point cloud topic
    sub_cloud = nh_.subscribe("/xtion/depth_registered/points", 10, &Perception::processCloud, this);
    // service 3D point
    person_pose_service = nh_.advertiseService("position_base/point3D", &Perception::processPersonPos, this);

    ROS_INFO("Perception initialized ...");
  }

  ~Perception() {}
};


void Perception::processCloud(const sensor_msgs::PointCloud2ConstPtr &pc)
{
  // store local data copy or shared, depending on the message
  pcl::fromROSMsg(*pc, pc_point3D);
}

// TO DO : change the type "darknet_ros_msgs::BoundingBoxesConstPtr"
bool Perception::processPersonPos(perception_person::PercepPerson::Request &req, perception_person::PercepPerson::Response &res)
{

  if (pc_point3D.isOrganized() != 1)
  {
    ROS_INFO_STREAM(" Point Cloud is not organized, waiting... ");
  }
  else
  {
    ROS_INFO_STREAM("Point Cloud is already organized:");

    // load point from client
    CenterPoints[0] = req.x;
    CenterPoints[1] = req.y;
    point3D.point.x = pc_point3D.at(CenterPoints[0], CenterPoints[1]).x;
    point3D.point.y = pc_point3D.at(CenterPoints[0], CenterPoints[1]).y;
    point3D.point.z = pc_point3D.at(CenterPoints[0], CenterPoints[1]).z;

    while (isnan(pc_point3D.at(CenterPoints[0], CenterPoints[1]).x))
    {
      float d = -0.5;
      //int d = -20;
      CenterPoints[0] = CenterPoints[0] + d;
    }
    //std::cout << " CenterPoints[0] = " << CenterPoints[0] << endl;

    point3D.point.x = pc_point3D.at(CenterPoints[0], CenterPoints[1]).x;
    point3D.point.y = pc_point3D.at(CenterPoints[0], CenterPoints[1]).y;
    point3D.point.z = pc_point3D.at(CenterPoints[0], CenterPoints[1]).z;
    point3D.header.frame_id = pc_point3D.header.frame_id;

    point3D_base = transformPoint(listener_, point3D);
    //point3D_base.header.frame_id = pc_point3D.header.frame_id;

    res.point = point3D_base;

    ROS_INFO_STREAM("The 3D coordinate of "
                    << " is x: " << point3D.point.x
                    << " y: " << point3D.point.y
                    << " z: " << point3D.point.z
                    << " x': " << point3D_base.point.x
                    << " y': " << point3D_base.point.y
                    << " z': " << point3D_base.point.z);
  }
  return 1;
}

geometry_msgs::PointStamped Perception::transformPoint(const tf::TransformListener &listener, const geometry_msgs::PointStamped laser_point)
{

  try
  {

    listener.transformPoint("base_link", laser_point, person_base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
             laser_point.point.x, laser_point.point.y, laser_point.point.z,
             person_base_point.point.x, person_base_point.point.y, person_base_point.point.z, person_base_point.header.stamp.toSec());
  }

  catch (tf::TransformException &ex)
  {

    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
  return person_base_point;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_person");
  ros::NodeHandle nh;
  Perception node(nh);
  ros::spin();
  return 0;
}
