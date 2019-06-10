#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>

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

  ros::Duration(15).sleep();  // wait for arm tucked

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // ros::ServiceClient client =
  //    n.serviceClient<std_msgs::Bool>("ReachedTable");

  ros::ServiceClient client_global =
      n.serviceClient<std_srvs::Empty>("/global_localization");
  ros::ServiceClient client_clear =
      n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  while (!client_global.call(client_srv)) {
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

  std::vector<geometry_msgs::Pose> points;

  geometry_msgs::Pose point1;
  point1.position.x = -0.896;
  point1.position.y = 2.396;
  point1.position.z = 0.000;
  point1.orientation.x = 0;
  point1.orientation.y = 0;
  point1.orientation.z = 0.55;
  point1.orientation.w = 0.835;
  points.push_back(point1);

  geometry_msgs::Pose point2;
  point2.position.x = -1.77;
  point2.position.y = 2.907;
  point2.position.z = 0.000;
  point2.orientation.x = 0;
  point2.orientation.y = 0;
  point2.orientation.z = 0.562;
  point2.orientation.w = 0.827;
  points.push_back(point2);

  move_base_msgs::MoveBaseGoal goal;

  // set target pose frame of coordinates
  goal.target_pose.header.frame_id = "map";



  int i = 0;
  while (ros::ok()) {
    if (i < points.size()) {
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose = points.at(i);
      ROS_INFO("Sending goal %d", i + 1);
      // send goal and register callback handler
      ac.sendGoal(goal, &doneCb, &activeCb);  // &feedbackCb
      ac.waitForResult();                     // wait for goal result

      if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The base successfully moved to goal %d", i + 1);
        i++;
      } else {
        ROS_INFO("The base failed to move to goal %d for some reason", i + 1);
        continue;
      }

    } else {
      i = 0;
      continue;
    }
  }
  return 0;
}
