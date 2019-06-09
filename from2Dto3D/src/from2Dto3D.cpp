#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Char.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
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


//#include <perception_msgs/Rect.h>



using namespace std;
using namespace cv;


class From2Dto3D
{

    private:
      //! The node handle
      ros::NodeHandle nh_;
      //! Node handle in the private namespace
      ros::NodeHandle priv_nh_;

      //! Define publishers and subscribers
      ros::Subscriber sub_cloud;
      ros::Subscriber sub_BB;
      ros::Publisher pub_point3D;
      //! Define the pointcloud structure and the bounding box local copy
      pcl::PointCloud< pcl::PointXYZRGB > pc_point3D;
      //! A tf transform listener if needed

      tf::TransformListener listener_;
      // rosrun tf tf_echo /xtion_rgb_optical_frame /base_link
      int temp, Object_num = 10, CenterPoints[2];
      geometry_msgs::Point point3D;



      //------------------ Callbacks -------------------

      //! Process clusters
      void processCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
      //! Process bounding boxes
      void processBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& r);
      void transformPoint(const tf::TransformListener& listener);

    public:
      //! Subscribes to and advertises topics
      From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
      {
        // subscribers to the bounding boxes and the point cloud
        // format:
        // sub_name = nh_.subscribe<Type>("topic", queuesize, Function_of_the_class, this);
        sub_cloud = nh_.subscribe( "/xtion/depth_registered/points", 10, &From2Dto3D::processCloud, this);
        sub_BB = nh_.subscribe( "/darknet_ros/bounding_boxes", 10, &From2Dto3D::processBoundingBoxes, this);
        // Publishers
        // format:
        //pub_name = nh_.advertise< Type >("topic", queuesize);
        pub_point3D = nh_.advertise< geometry_msgs::Point >("/segmentation/point3D", 10);
        ROS_INFO("from2Dto3D initialized ...");

      }

      ~From2Dto3D() {}
};


void From2Dto3D::processCloud(const sensor_msgs::PointCloud2ConstPtr& pc)
{
    // store local data copy or shared, depending on the message
    pcl::fromROSMsg(*pc, pc_point3D);

}

void From2Dto3D::processBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& r)
{

    // process bounding box and send 3D position to the topic
    // tip: take a look at the pcl::PointXYZRGB structure
    // bounding_boxes: Class: "bottle", probability: 0.953633725643, xmin: 261, ymin: 60, xmax: 335, ymax: 315.
    //ros::Duration(5).sleep();

    if (Object_num == 10)
    {
        std::cout << " There are " << r->bounding_boxes.size() << " objects on the table." << endl;
        for(int j = 0; j < r->bounding_boxes.size(); j++ )
        {
          std::cout << " Number " << j << " is " << r->bounding_boxes[j].Class << endl;
        }
        std::cout << " Object to recognize, Please enter the object's number: " << endl;
        std::cin >> temp;
        Object_num = temp;
        while( temp >= r->bounding_boxes.size())
        {
          std::cout << " The object's number is wrong, please enter the new number: \n"
                    << "the current Object_num/temp is: " << temp << endl;
          std::cin >> temp;
          Object_num = temp;
        }
    }

    // while( pc_point3D.isOrganized() )
    if (pc_point3D.isOrganized() != 1)
    {
        ROS_INFO_STREAM(" Point Cloud is not organized, waiting... ");
    }
    else
    {
        CenterPoints[0] = int ((r->bounding_boxes[Object_num].xmin + r->bounding_boxes[Object_num].xmax) / 2 );
        CenterPoints[1] = int ((r->bounding_boxes[Object_num].ymin + r->bounding_boxes[Object_num].ymax) / 2 );

        point3D.x = pc_point3D.at(CenterPoints[0], CenterPoints[1]).x;
        point3D.y = pc_point3D.at(CenterPoints[0], CenterPoints[1]).y;
        point3D.z = pc_point3D.at(CenterPoints[0], CenterPoints[1]).z;

        while (isnan(pc_point3D.at(CenterPoints[0], CenterPoints[1]).x))
        {
            float d = -0.5;
            //int d = -20;
            CenterPoints[0] = CenterPoints[0] + d;
            //CenterPoints[1] = CenterPoints[1] + d;
            d = d + 0.1;
            //std::cout << " JUST CHECK IF ENTER THE WHILE  " << endl;
            //std::cout << " CenterPoints[0] = " << CenterPoints[0] << endl;
            //std::cout << " CenterPoints[1] = " << CenterPoints[1] << endl;
            if ( d == 0.5 )
            {
                //std::cout << " d = " << d;
                break;
            }
        }
        //std::cout << " CenterPoints[0] = " << CenterPoints[0] << endl;

        point3D.x = pc_point3D.at(CenterPoints[0], CenterPoints[1]).x;
        point3D.y = pc_point3D.at(CenterPoints[0], CenterPoints[1]).y;
        point3D.z = pc_point3D.at(CenterPoints[0], CenterPoints[1]).z;

        pub_point3D.publish(point3D);

        std::cout << " The Number " << Object_num
                  << " " << r->bounding_boxes[Object_num].Class
                  << ":  CenterPoints[0] : " << CenterPoints[0]
                  << "   CenterPoints[1] : " << CenterPoints[1]
                  << " \n \t \t    position is x: " << point3D.x
                  << "  y: " << point3D.y
                  << "  z: " << point3D.z << endl;
    }

}

void transformPoint(const tf::TransformListener& listener){

  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame

  geometry_msgs::PointStamped laser_point;

  laser_point.header.frame_id = "base_laser";



  //we'll just use the most recent transform available for our simple example

  laser_point.header.stamp = ros::Time();
  //just an arbitrary point in space
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

  try{

    geometry_msgs::PointStamped base_point;

    listener.transformPoint("base_link", laser_point, base_point);



    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",

        laser_point.point.x, laser_point.point.y, laser_point.point.z,

        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());

  }

  catch(tf::TransformException& ex){

    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());

  }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "from2Dto3D");
    ros::NodeHandle nh;
    From2Dto3D node(nh);
    ros::spin();
    return 0;
}


