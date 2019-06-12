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


bool GraspClass::graspServerCB(manipulation::Grasp::Request &req, manipulation::Grasp::Response &res){

    if(req.motion_name == "pick"){
        geometry_msgs::PoseStamped goal;
        geometry_msgs::PoseStamped pre_goal;
        goal.header.frame_id = req.point.header.frame_id;
        goal.pose.position.x = req.point.point.x-0.2;
        goal.pose.position.y = req.point.point.y;
        goal.pose.position.z = req.point.point.z;
        goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57,0,0);
        ROS_INFO("Start motion planning to x:%f y:%f z:%f", goal.pose.position.x,
                                                            goal.pose.position.y,
                                                            goal.pose.position.z);
        pre_goal = goal;
        pre_goal.pose.position.z += 0.1;
        pre_goal.pose.position.x = 0.3;
        
        GraspClass::arm_control(pre_goal);
        GraspClass::arm_control(goal);
        GraspClass::gripper_control(0.04);
        GraspClass::arm_control(pre_goal);
        GraspClass::arm_turked();
    }
    else if (req.motion_name == "prepare"){
        GraspClass::prepare_robot();
    }
    else if (req.motion_name == "place"){
        ROS_INFO("Place");
    }
    else{
        ROS_INFO("Please input a valid command");
        return 0;
    }

    ROS_INFO("Grasp service done");
    return 1;
}

void GraspClass::prepare_robot(){
    ROS_INFO("Preparing robot");
    GraspClass::arm_turked();
    GraspClass::head_look_around();
}

void GraspClass::lift_torso(double height){
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



    //using torson controller with publisher, it will cause several motions executed in same time.
    trajectory_msgs::JointTrajectory jt;
    trajectory_msgs::JointTrajectoryPoint jtp;
    jt.joint_names.push_back("torso_lift_joint");
    jtp.positions.push_back(height);
    jtp.velocities.push_back(0.25);
    jtp.time_from_start = ros::Duration(5);
    jt.points.push_back(jtp);
    lift_torso_.publish(jt);
    ros::Duration(2.0).sleep();
    ROS_INFO("Torso lift done!");
}


void GraspClass::head_look_around(){
    ROS_INFO("Head look around");
    trajectory_msgs::JointTrajectory jt;
    trajectory_msgs::JointTrajectoryPoint jtp;
    jt.joint_names.push_back("head_1_joint");
    jt.joint_names.push_back("head_2_joint");
    jtp.positions.push_back(0.0);
    jtp.positions.push_back(-0.3);
    jtp.velocities.push_back(0.2);
    jtp.velocities.push_back(0.2);
    jtp.time_from_start = ros::Duration(5);
    jt.points.push_back(jtp);
    head_control_.publish(jt);
    ros::Duration(5.0).sleep();
    ROS_INFO("Head look around done");

}

void GraspClass::arm_control(geometry_msgs::PoseStamped goal){
    ROS_INFO("Arm control");
    moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
    //group_arm_torso.setPlannerId("SBLkConfigDefault");
    group_arm_torso.setPlannerId("RRTConnectkConfigDefault");

    //ROS_INFO("%s",req.point.header.frame_id.c_str());
    group_arm_torso.setPoseReferenceFrame(goal.header.frame_id);
    group_arm_torso.setPoseTarget(goal);

    group_arm_torso.setStartStateToCurrentState();
    group_arm_torso.setMaxVelocityScalingFactor(0.5);
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

    ROS_INFO("Arm control done");
}

void GraspClass::gripper_control(double width){
    ROS_INFO("grasping");
    moveit::planning_interface::MoveGroupInterface gripper_control("gripper");
    //group_arm_torso.setPlannerId("SBLkConfigDefault");
    gripper_control.setPlannerId("RRTConnectkConfigDefault");

    //ROS_INFO("%s",req.point.header.frame_id.c_str());
    gripper_control.setJointValueTarget("gripper_left_finger_joint", width/2);
    gripper_control.setJointValueTarget("gripper_right_finger_joint", width/2);
    gripper_control.setStartStateToCurrentState();
    gripper_control.setMaxVelocityScalingFactor(0.5);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    gripper_control.setPlanningTime(5.0);
    gripper_control.plan(my_plan);
    bool success;
    success = bool(gripper_control.plan(my_plan));
     if (!success)
         throw std::runtime_error("no plan found");
    moveit::planning_interface::MoveItErrorCode e = gripper_control.move();
    if (!bool(e))
        throw std::runtime_error("Error executing plan");

    ROS_INFO("grasping done!");
}
 
void GraspClass::arm_turked(){
    ROS_INFO("Turk arm");
    moveit::planning_interface::MoveGroupInterface arm_turk("arm_torso");

    arm_turk.setPlannerId("SBLkConfigDefault");
    arm_turk.setStartStateToCurrentState();
    arm_turk.setMaxVelocityScalingFactor(0.8);
    arm_turk.setJointValueTarget("torso_lift_joint", 0.14);
    arm_turk.setJointValueTarget("arm_1_joint", 0.20);
    arm_turk.setJointValueTarget("arm_2_joint", -1.34);
    arm_turk.setJointValueTarget("arm_3_joint", -0.20);
    arm_turk.setJointValueTarget("arm_4_joint", 1.94);
    arm_turk.setJointValueTarget("arm_5_joint", -1.57);
    arm_turk.setJointValueTarget("arm_6_joint", 1.37);
    arm_turk.setJointValueTarget("arm_7_joint", 0.00);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    arm_turk.setPlanningTime(5.0);
    bool success;
    success = bool(arm_turk.plan(my_plan));
    if ( !success )
        throw std::runtime_error("No plan found turking arm");

    moveit::planning_interface::MoveItErrorCode e=arm_turk.move();
    if(!e){
        throw std::runtime_error("Can't finish arm turking");
    }

    ROS_INFO("Turk arm done");
}
