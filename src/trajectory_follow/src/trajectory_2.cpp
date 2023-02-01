#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_follow/desired_position.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_getParam");
//   nh_ = ros::NodeHandlePtr(new ros::NodeHandle);
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  double p_w = 0.0, p_x = 0.0, p_y = 0.0, p_z = 0.0;
  

  // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
  // are used interchangeably.
  static const std::string PLANNING_GROUP = "panda_arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);


  nh.getParam("pose_w", p_w);
  nh.getParam("pose_x", p_x);
  nh.getParam("pose_y", p_y);
  nh.getParam("pose_z", p_z);

  // ros::param::get("pose_w", p_w);
  // ros::param::get("pose_x", p_x);
  // ros::param::get("pose_y", p_y);
  // ros::param::get("pose_z", p_z);

  

  ROS_INFO("x:%.3f, y:%.3f, z:%.3f, w:%.3f", 
            p_w,p_x,p_y,p_z
  );

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = p_w;
  target_pose1.position.x = p_x;
  target_pose1.position.y = p_y;
  target_pose1.position.z = p_z;

  move_group_interface.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Note that this can lead to problems if the robot moved in the meanwhile.
  // move_group_interface.execute(my_plan);

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // If you do not want to inspect the planned trajectory,
  // the following is a more robust combination of the two-step plan+execute pattern shown above
  // and should be preferred. Note that the pose goal we had set earlier is still active,
  // so the robot will try to move to that goal.

  // move_group_interface.move();
  std::string ee= move_group_interface.getEndEffectorLink();
  ROS_INFO("End effector %s", ee.c_str());


  // store data
  moveit_msgs::RobotTrajectory msg;
  msg = my_plan.trajectory_;

  ros::Publisher pub = nh.advertise<moveit_msgs::RobotTrajectory>("trajectory_plan",1);

  std::vector<int>::size_type size1 = msg.joint_trajectory.points.size();  
  
  ros::Rate r(1);
  int count = 0;
  while(count<=1)
  {
    ROS_INFO("size %.2ld", size1);
    pub.publish(msg);
    r.sleep();
    count++;
  }
  

  ros::waitForShutdown();
  // ros::shutdown();
  

    
  return 0;
}