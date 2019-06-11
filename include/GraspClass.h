/****** ROS HEADER *********/
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

/****** STD HEADER *********/
#include <std_msgs/String.h>
#include <boost/bind.hpp>
#include <iostream>
/****** Moveit HEADER *********/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/PickupAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
/****** Msg HEADER *********/
#include "tutorial5/Point.h"

/****** Srv HEADER *********/
#include "tutorial5/Grasp.h"

/****** Action HEADER *********/
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "tutorial5/GraspAction.h"



namespace Grasp{
    class GraspClass
    {

        private:
            ros::NodeHandle nh_;


            actionlib::SimpleActionServer<tutorial5::GraspAction> grasp_;
            std::string action_name_;
            tutorial5::GraspActionFeedback feedback_;
            geometry_msgs::PointStamped goal_;
            
            ros::ServiceServer grasp_srv_ ;
            ros::Publisher lift_torso_;
            ros::Publisher gripper_control_;
	    ros::Publisher head_control_;

        public:
            GraspClass(std::string name, ros::NodeHandle n):nh_(n),grasp_(nh_,name, false),action_name_(name){

                // grasp_.registerGoalCallback(boost::bind(&GraspClass::graspActionCB, this));
                // grasp_.registerPreemptCallback(boost::bind(&GraspClass::preemptCB, this));
                // grasp_.start();
                grasp_srv_ = nh_.advertiseService(name, &GraspClass::graspServerCB, this);
                lift_torso_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command",1);
                gripper_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/gripper_controller/command",1);
		head_control_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_controller/command",1);
            };
            ~GraspClass(){};

            void graspActionCB();
            void preemptCB();
            bool graspServerCB(tutorial5::Grasp::Request &req, tutorial5::Grasp::Response &res);
            void prepare_robot();
            void lift_torso(double height);
            void head_look_around();
            void arm_control(geometry_msgs::PoseStamped goal);
            void gripper_control(double width);
            void arm_turked();

    }; //Class def

} //namespace
