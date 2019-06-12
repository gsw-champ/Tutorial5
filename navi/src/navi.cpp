#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <pal_navigation_msgs/Acknowledgment.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>

/******************************
 Srv header
******************************/
#include <manipulation/Grasp.h>
#include <perception/perc.h>




typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

// callback function
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
}

// callback function, called once when the goal becomes active
void activeCb() { ROS_INFO("Goal just went active"); }





int main(int argc, char** argv) {
    ros::init(argc, argv, "navi");
    ros::NodeHandle n;
    ros::Rate r(10);

    std_srvs::Empty client_srv;
    geometry_msgs::Twist msg; 
    geometry_msgs::PointStamped object; 
    pal_navigation_msgs::Acknowledgment pal_srv;

    //TODO  add two service client
    perception::perc percep_req;
    manipulation::Grasp manipu_req;

    ros::ServiceClient client_globallization =
        n.serviceClient<std_srvs::Empty>("/global_localization");
    ros::ServiceClient client_clear =
        n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    ros::ServiceClient client_pal_navigation =
        n.serviceClient<pal_navigation_msgs::Acknowledgment>("/pal_navigation_sm");
    ros::ServiceClient client_changeMap =
        n.serviceClient<pal_navigation_msgs::Acknowledgment>("/pal_map_manager/change_map");
    ros::ServiceClient client_perception =
        n.serviceClient<perception::perc>("find_box");
    ros::ServiceClient client_manipulation =
        n.serviceClient<manipulation::Grasp>("Grasp_node");



    ros::Duration(10).sleep();  // wait for arm tucked

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    //pal_srv.request.input = "LOC";
   // while (!client_pal_navigation.call(pal_srv)) {
   //  ROS_INFO("pal_navigation_sm.");
    //  r.sleep();
   // }


   // pal_srv.request.input = "gsw_champ";
   // while (!client_changeMap.call(pal_srv)) {
   //   ROS_INFO("Change costmaps.");
   //   r.sleep();
   // }

    while (!client_globallization.call(client_srv)) {
      ROS_INFO("Wait for global_localization.");
      r.sleep();
    }

    ros::Publisher pub =
        n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
    msg.angular.z = 0.5;
    ros::Time start = ros::Time::now();
    while ((ros::Time::now() - start) < ros::Duration(25)) {
      pub.publish(msg);
      //    ROS_INFO("Localizing...");
    }


    while (!client_clear.call(client_srv)) {
      ROS_INFO("Clear costmaps.");
      r.sleep();
    }

    std::vector<geometry_msgs::Pose> points;

   /*geometry_msgs::Pose point1;
   point1.position.x = -1.2214;
   point1.position.y = 1.11677;
   point1.position.z = 0.000;
   point1.orientation.x = 0;
   point1.orientation.y = 0;
   point1.orientation.z = 0.43753;
   point1.orientation.w = 0.8992;
   points.push_back(point1);*/

   geometry_msgs::Pose point1;
   point1.position.x = 2.679;
   point1.position.y = -0.215;
   point1.position.z = 0.000;
   point1.orientation.x = 0;
   point1.orientation.y = 0;
   point1.orientation.z = 0.707;
   point1.orientation.w = 0.707;
   points.push_back(point1);

   geometry_msgs::Pose point2;
   point2.position.x = -2.36213;
   point2.position.y = 1.6746;
   point2.position.z = 0.000;
   point2.orientation.x = 0;
   point2.orientation.y = 0;
   point2.orientation.z = 0.43694;
   point2.orientation.w = 0.8994;
   points.push_back(point2);


   percep_req.response.point.point.x = 0.0;
   percep_req.response.point.point.y = 0.0;
   percep_req.response.point.point.z = 0.0;


    move_base_msgs::MoveBaseGoal goal, goal_temp;

      goal_temp.target_pose.header.stamp = ros::Time::now();     
      goal_temp.target_pose.header.frame_id = "base_link";
      goal_temp.target_pose.pose.position.x = 0.1;
      goal_temp.target_pose.pose.position.y = 0.0;
      goal_temp.target_pose.pose.position.z = 0.0;
      goal_temp.target_pose.pose.orientation.x = 0.0;
      goal_temp.target_pose.pose.orientation.y = 0.0;
      goal_temp.target_pose.pose.orientation.z = 0.0;
      goal_temp.target_pose.pose.orientation.w = 1.0;


    // set target pose frame of coordinates
    goal.target_pose.header.frame_id = "map";

    int i = 0;

    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = points.at(i);
    ROS_INFO("Sending goal %d", i + 1);
    // send goal and register callback handler
    ac.sendGoal(goal, &doneCb, &activeCb);  // &feedbackCb
    ac.waitForResult();                     // wait for goal result

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
      ROS_INFO("Successfully moved to goal %d", i + 1);
    else {
      ROS_INFO("The base failed to move to goal %d for some reason", i + 1);
    }

	//first goal done

    manipu_req.request.motion_name = "prepare";
    while(!client_manipulation.call(manipu_req)){
      ROS_INFO("Waiting for prepare");
    }

    ros::Duration(3.0).sleep();

        //call perception
    percep_req.request.class_name = "cup";
    while(percep_req.response.point.point.x == 0.0){
    	client_perception.call(percep_req);
	ROS_INFO("no object detected");
    }
    object = percep_req.response.point;
    manipu_req.request.point = object;
    manipu_req.request.motion_name = "pick";

      //point3d
      //call manipulation

    while(!client_manipulation.call(manipu_req)){

      ac.sendGoal(goal_temp, &doneCb, &activeCb);  // go forward 0.1m
      ac.waitForResult();  
      ROS_INFO("Go forward 0.1m");

      //perception one more time
      percep_req.response.point.point.x = 0.0;
      percep_req.response.point.point.y = 0.0;
      percep_req.response.point.point.z = 0.0;
      while(percep_req.response.point.point.x == 0.0){
    	  client_perception.call(percep_req);
	  ROS_INFO("no object detected");
      }
      
      //reset action goal
      object = percep_req.response.point;
      manipu_req.request.point = object;
    }
      //output bool
    ROS_INFO("Catch object");


    
    return 0;
}
