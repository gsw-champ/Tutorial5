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




        public:
            GraspClass(std::string name, ros::NodeHandle n):nh_(n),grasp_(nh_,name, false),action_name_(name){

                // grasp_.registerGoalCallback(boost::bind(&GraspClass::graspActionCB, this));
                // grasp_.registerPreemptCallback(boost::bind(&GraspClass::preemptCB, this));
                // grasp_.start();
                grasp_srv_ = nh_.advertiseService(name, &GraspClass::graspServerCB, this);
            };
            ~GraspClass(){};

            void graspActionCB();
            void preemptCB();
            bool graspServerCB(tutorial5::Grasp::Request &req, tutorial5::Grasp::Response &res);
            void prepare_robot();
            void lift_torso();
            void head_look_around();

    }; //Class def

} //namespace
