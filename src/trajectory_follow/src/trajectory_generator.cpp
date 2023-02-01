//All the function is integrated in a class
/*
use rosservice to achieve the desired position and orientation
return the motion plan by respond of rosservice
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include <trajectory_follow/desired_position.h>

class Trajectory
{
  public:
    Trajectory(ros::NodeHandle& nodehandle);
  
  private:
    //with nh_ connect main function and constructor 
    ros::NodeHandle nh_;
    //get desired position
    ros::ServiceServer server_;
    //publish move_plan, use server respond rather than publich
    // ros::Publisher pub_;

    // set the controled part's name. Should be changed, when using UR robot 
    const std::string planning_group_ = "panda_arm";

    // create a empty move group interface at first, will reset later; 
    // with this can set the pose target; and set the defined the controled part
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_=nullptr;

    geometry_msgs::Pose target_pose1_;
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_;

    //check wheather get plan 
    bool success_;
    // print trajectory
    moveit_msgs::RobotTrajectory msg_;

    //initialize-----------------------------
    void initializeMoveGroup();
    void initializeServices();
    // void initializePublicher();

    bool serviceCallback(trajectory_follow::desired_position::Request& req,
          trajectory_follow::desired_position::Response& resp);
};

Trajectory::Trajectory(ros::NodeHandle& nodehandle):nh_(nodehandle) //initialize nh_ with passing nodehandle
{
  ROS_INFO("start to construct a class.....");
  initializeMoveGroup();
  // initializePublicher();
  initializeServices();
  
}

void Trajectory::initializeMoveGroup()
{
  ROS_INFO("Start to initialize move group");
  //planning_group_ = "panda_arm";
  move_group_interface_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_));
  ROS_INFO("Finish initialize");
}

// void Trajectory::initializePublicher()
// {
//   ROS_INFO("Initializing Publisher");
//   pub_ = nh_.advertise<moveit_msgs::RobotTrajectory>("trajectory_plan",1);
//   ROS_INFO("A Publisher is created");
// }

void Trajectory::initializeServices()
{
  ROS_INFO("Initializing Services");
  server_ = nh_.advertiseService("desired_position", &Trajectory::serviceCallback,this);
  ROS_INFO("A server is created");
}

bool Trajectory::serviceCallback(trajectory_follow::desired_position::Request& req,
          trajectory_follow::desired_position::Response& resp)
{
  target_pose1_ = req.targetpose;

  move_group_interface_->setPoseTarget(target_pose1_);
  move_group_interface_->setGoalTolerance(0.1);

  std::string ee= move_group_interface_->getEndEffectorLink();

  ROS_INFO("Recived a pose goal");
  ROS_INFO("End effector link %s", ee.c_str());
  ROS_INFO("%.2f",target_pose1_.position.x);
  ROS_INFO("%.2f",target_pose1_.position.y);
  ROS_INFO("%.2f",target_pose1_.position.z);
  ROS_INFO("%.2f",target_pose1_.orientation.x);
  ROS_INFO("%.2f",target_pose1_.orientation.y);
  ROS_INFO("%.2f",target_pose1_.orientation.z);
  ROS_INFO("%.2f",target_pose1_.orientation.w);

  // move_group_interface_->plan(this->my_plan_); 
  success_ = (move_group_interface_->plan(this->my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // only need the motion plan, no need to excute motion 
  // move_group_interface_->move();
  ROS_INFO("Plan pose goal %s",success_ ? "Success": "FAILED");

  msg_ = my_plan_.trajectory_;
  std::vector<int>::size_type size1 = msg_.joint_trajectory.points.size();  
  ROS_INFO("The trajectory points size is %.ld", size1);

  resp.s = "Success";
  resp.trajectory = msg_;
  return success_;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_test");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.

  ros::AsyncSpinner spinner(2); //at least two thread to run and publish the motion planung 
  spinner.start();
  
  ROS_INFO("service is starting.......");

  // use reference to pass argument. It refer to the same variable
  Trajectory trajectory(node_handle);

  //ros::spin();

  ros::waitForShutdown();
  
  return 0;
}
