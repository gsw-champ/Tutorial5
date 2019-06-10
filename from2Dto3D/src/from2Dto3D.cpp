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
      string type_obj, class_obj;
      int temp, CenterPoints[2];
      int Object_num;
      geometry_msgs::PointStamped point3D;
      geometry_msgs::PointStamped point3D_base;
      geometry_msgs::PointStamped base_point;

      //------------------ Callbacks -------------------

      //! Process clusters
      void processCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
      //! Process bounding boxes
      void processBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& r);
      geometry_msgs::PointStamped transformPoint(const tf::TransformListener& listener, const geometry_msgs::PointStamped laser_point);

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
        pub_point3D = nh_.advertise< geometry_msgs::PointStamped >("/segmentation/point3D", 10);
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

//    if (Object_num == 10)
//    {
//        std::cout << " There are " << r->bounding_boxes.size() << " objects on the table." << endl;
//        for(int j = 0; j < r->bounding_boxes.size(); j++ )
//        {
//          std::cout << " Number " << j << " is " << r->bounding_boxes[j].Class << endl;
//        }
//        std::cout << " Object to recognize, Please enter the object's number: " << endl;
//        std::cin >> temp;
//        Object_num = temp;
//        while( temp >= r->bounding_boxes.size())
//        {
//          std::cout << " The object's number is wrong, please enter the new number: \n"
//                    << "the current Object_num/temp is: " << temp << endl;
//          std::cin >> temp;
//          Object_num = temp;
//        }
//    }

    // while( pc_point3D.isOrganized() )

    Object_num = r->bounding_boxes.size();

    if (pc_point3D.isOrganized() != 1)
    {
        ROS_INFO_STREAM(" Point Cloud is not organized, waiting... ");
    }
    else
    {
        ROS_INFO_STREAM("There are " << Object_num << " in this picture:");
        ROS_INFO_STREAM("They are: ");

        for (int i_obj = 0; i_obj < Object_num; i_obj++) {
          ROS_INFO_STREAM(r->bounding_boxes[i_obj].Class);
        }

        //std::cin >> class_obj;
        class_obj = "bottle";

        for (int j_obj = 0; j_obj < Object_num; j_obj++) {
              type_obj = r->bounding_boxes[j_obj].Class;

              if (!type_obj.compare(class_obj)) {

                    CenterPoints[0] = int ((r->bounding_boxes[j_obj].xmin +
                                            r->bounding_boxes[j_obj].xmax) / 2 );
                    CenterPoints[1] = int ((r->bounding_boxes[j_obj].ymin +
                                            r->bounding_boxes[j_obj].ymax) / 2 );

                    point3D.point.x = pc_point3D.at(CenterPoints[0], CenterPoints[1]).x;
                    point3D.point.y = pc_point3D.at(CenterPoints[0], CenterPoints[1]).y;
                    point3D.point.z = pc_point3D.at(CenterPoints[0], CenterPoints[1]).z;

                    while (isnan(pc_point3D.at(CenterPoints[0], CenterPoints[1]).x))
                    {
                        float d = -0.5;
                        //int d = -20;
                        CenterPoints[0] = CenterPoints[0] + d;
                        //CenterPoints[1] = CenterPoints[1] + d;
                        d = d + 0.1;
                            if ( d == 0.5 )
                             {
                                //std::cout << " d = " << d;
                                break;
                             }
                     }
        //std::cout << " CenterPoints[0] = " << CenterPoints[0] << endl;

                    point3D.point.x = pc_point3D.at(CenterPoints[0], CenterPoints[1]).x;
                    point3D.point.y = pc_point3D.at(CenterPoints[0], CenterPoints[1]).y;
                    point3D.point.z = pc_point3D.at(CenterPoints[0], CenterPoints[1]).z;
                    point3D.header.frame_id = pc_point3D.header.frame_id;
                    point3D_base = transformPoint(listener_, point3D);
                    //point3D_base.header.frame_id = pc_point3D.header.frame_id;
                    pub_point3D.publish(point3D_base);
                    //pub_point3D.publish(point3D);

                    ROS_INFO_STREAM("The 3D coordinate of " << type_obj
                                    << " is x: " << point3D.point.x
                                    << " y: " << point3D.point.y
                                    << " z: " << point3D.point.z
                                    << " x': " << point3D_base.point.x
                                    << " y': " << point3D_base.point.y
                                    << " z': " << point3D_base.point.z);

//                    std::cout  << " The Number " << Object_num
//                               << " " << r->bounding_boxes[Object_num].Class
//                               << ":  CenterPoints[0] : " << CenterPoints[0]
//                               << "   CenterPoints[1] : " << CenterPoints[1]
//                               << " \n \t \t    position is x: " << point3D.point.x
//                               << "  y: " << point3D.point.y
//                               << "  z: " << point3D.point.z
//                               << " \n \t \t   base_link position is x' : " << point3D_base.point.x
//                               << "  y': " << point3D_base.point.y
//                               << "  z': " << point3D_base.point.z << endl;

                     }
                }
             }
}

geometry_msgs::PointStamped From2Dto3D::transformPoint(const tf::TransformListener& listener, const geometry_msgs::PointStamped laser_point)
{
  //geometry_msgs::PointStamped laser_point;

  try{

    listener.transformPoint("base_link", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }

  catch(tf::TransformException& ex){

    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());

  }
  return base_point;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "from2Dto3D");
    ros::NodeHandle nh;
    From2Dto3D node(nh);
    ros::spin();
    return 0;
}


