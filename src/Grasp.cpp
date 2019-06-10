#include "GraspClass.h"
#include <tf/tf.h>
#include <map>
#include <control_msgs/FollowJointTrajectoryAction.h>

using namespace Grasp;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  control_client;
typedef boost::shared_ptr<control_client> control_client_ptr;

void GraspClass::graspActionCB(){
    goal_ = grasp_.acceptNewGoal() ->goal;
}

void GraspClass::preemptCB(){
    ROS_INFO("%s:Preemted", action_name_.c_str());
    grasp_.setPreempted();
}


bool GraspClass::graspServerCB(tutorial5::Grasp::Request &req, tutorial5::Grasp::Response &res){

    GraspClass::prepare_robot();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = req.point.header.frame_id;
    goal.pose.position.x = req.point.point.x;
    goal.pose.position.y = req.point.point.y;
    goal.pose.position.z = req.point.point.z;
    goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.011,1.57,0.037);
    ROS_INFO("Start motion planning to x:%f y:%f z:%f", goal.pose.position.x,
							goal.pose.position.y,
							goal.pose.position.z);

    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    group_arm_torso.setPlannerId("SBLkConfigDefault");
    //ROS_INFO("%s",req.point.header.frame_id.c_str());
    group_arm_torso.setPoseReferenceFrame(req.point.header.frame_id);
    group_arm_torso.setPoseTarget(goal);

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(1.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    group_arm_torso.setPlanningTime(5.0);
    group_arm_torso.plan(my_plan);
    bool success;
    success = bool(group_arm_torso.plan(my_plan));
     if (!success)
         throw std::runtime_error("no plan found");
    moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
    if (!bool(e))
        throw std::runtime_error("Error executing plan");
    

    return 0;
}

void GraspClass::prepare_robot(){
    ROS_INFO("Preparing robot");
    GraspClass::lift_torso();

}

void GraspClass::lift_torso(){
    ROS_INFO("Moving torso up");
    // moveit::planning_interface::MoveGroupInterface torso_lift("arm_torso");

    // torso_lift.setPlannerId("SBLkConfigDefault");
    // torso_lift.setStartStateToCurrentState();
    // torso_lift.setMaxVelocityScalingFactor(1.0);
    // torso_lift.setJointValueTarget("torso_lift_joint", 0.35);

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // torso_lift.setPlanningTime(5.0);
    // bool success;
    // success = bool(torso_lift.plan(my_plan));
    // if ( !success )
    //     throw std::runtime_error("No plan found");

    // moveit::planning_interface::MoveItErrorCode e=torso_lift.move();
    // if(!e){
    //     throw std::runtime_error("Can't finish");
    // }

    trajectory_msgs::JointTrajectory jt;
    trajectory_msgs::JointTrajectoryPoint jtp;
    jt.joint_names.push_back("torso_lift_joint");
    jtp.positions.push_back(0.34);
    jtp.time_from_start = ros::Duration(2);
    jt.points.push_back(jtp);
    lift_torso_.publish(jt);

    ROS_INFO("Torso lift done!");
}


void GraspClass::head_look_around(){
    ROS_INFO("Head look around");
    // control_client_ptr head_control_client;
    // head_control_client->reset(new control_client("/head_controller/follow_joint_trajectory"));
    // head_control_client->waitForServer(ros::Duration(2.0));

    // control_msgs::FollowJointTrajectoryGoal head_goal;
    // head_goal.trajectory.joint_names.push_back("head_1_joint");
    // head_goal.trajectory.joint_names.push_back("head_2_joint");

    // head_goal.trajectory.points.resize(9);

    // head_goal.trajectory.points[0].resize(2);
    // head_goal.trajectory.points[0].position[0]=0.0;
    // head_goal.trajectory.points[0].position[1]=0.0;
    // head_goal.trajectory.points[0].time_from_start = ros::Duration(0.1);
    // head_goal.trajectory.points[1].resize(2);
    // head_goal.trajectory.points[1].position[0]=0.0;
    // head_goal.trajectory.points[1].position[1]=-0.7;
    // head_goal.trajectory.points[1].time_from_start = ros::Duration(3);
    // head_goal.trajectory.points[2].resize(2);
    // head_goal.trajectory.points[2].position[0]=-0.7;
    // head_goal.trajectory.points[2].position[1]=-0.7;
    // head_goal.trajectory.points[2].time_from_start = ros::Duration(6);
    // head_goal.trajectory.points[3].resize(2);
    // head_goal.trajectory.points[3].position[0]=-0.7;
    // head_goal.trajectory.points[3].position[1]=-0.5;
    // head_goal.trajectory.points[3].time_from_start = ros::Duration(9);
    // head_goal.trajectory.points[4].resize(2);
    // head_goal.trajectory.points[4].position[0]=0.0;
    // head_goal.trajectory.points[4].position[1]=-0.5;
    // head_goal.trajectory.points[4].time_from_start = ros::Duration(12);
    // head_goal.trajectory.points[5].resize(2);
    // head_goal.trajectory.points[5].position[0]=0.7;
    // head_goal.trajectory.points[5].position[1]=-0.5;
    // head_goal.trajectory.points[5].time_from_start = ros::Duration(15);
    // head_goal.trajectory.points[6].resize(2);
    // head_goal.trajectory.points[6].position[0]=0.7;
    // head_goal.trajectory.points[6].position[1]=-0.7;
    // head_goal.trajectory.points[6].time_from_start = ros::Duration(18);
    // head_goal.trajectory.points[7].resize(2);
    // head_goal.trajectory.points[7].position[0]=0.0;
    // head_goal.trajectory.points[7].position[1]=-0.7;
    // head_goal.trajectory.points[7].time_from_start = ros::Duration(21);
    // head_goal.trajectory.points[8].resize(2);
    // head_goal.trajectory.points[8].position[0]=0.0;
    // head_goal.trajectory.points[8].position[1]=-0.5;
    // head_goal.trajectory.points[8].time_from_start = ros::Duration(24);
 
    // head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    // head_control_client->sendGoal(head_goal);

    // if(head_control_client.getState().isDone()){
    //     ROS_INFO("head looking around is done!");
    // }


}
 