/****** ROS HEADER *********/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

/****** STD HEADER *********/
#include <std_msgs/String.h>
#include <boost/bind.hpp>
/****** Moveit HEADER *********/
#include <moveit/move_group_interface/move_group_interface.h>

/****** Msg HEADER *********/
#include "tutorial5/Point.h"

/****** Srv HEADER *********/
#include "tutorial5/Grasp.h"

/****** Action HEADER *********/
#include <actionlib/server/simple_action_server.h>
#include "tutorial5/GraspAction.h"




class Grasp
{

    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<tutorial5::GraspAction> grasp_;
        std::string action_name_;
        tutorial5::GraspActionFeedback feedback_;
        geometry_msgs::PointStamped goal_;




    public:
        Grasp(std::string name):grasp_(nh_,name, false),action_name_(name){
            grasp_.registerGoalCallback(boost::bind(&Grasp::Grasp_CB, this));

            grasp_.start();
        };
        ~Grasp();

        void Grasp_CB();




};

