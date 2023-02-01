#include "ros/ros.h"
#include <moveit_msgs/RobotTrajectory.h>

void doMsg(const moveit_msgs::RobotTrajectory::ConstPtr& msg)
{
  std::vector<int>::size_type size1 = msg->joint_trajectory.points.size();  
  ROS_INFO("size %.2ld", size1);
  for (unsigned i=0; i<size1; i++)  
  {  
    ROS_INFO("point %d:joint1: %.3f,joint2: %.3f,joint3: %.3f,joint4: %.3f,joint5: %.3f,joint6: %.3f,joint7: %.3f",
    i, msg->joint_trajectory.points[i].positions[0],
    msg->joint_trajectory.points[i].positions[1],
    msg->joint_trajectory.points[i].positions[2],
    msg->joint_trajectory.points[i].positions[3],
    msg->joint_trajectory.points[i].positions[4],
    msg->joint_trajectory.points[i].positions[5],
    msg->joint_trajectory.points[i].positions[6]);
    //msg->joint_trajectory.points[i].velocities[0]
  }  

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "listener_trajectory");

    ros::NodeHandle nh;

    ROS_INFO("Subscriber is setup");
    ros::Subscriber sub = nh.subscribe<moveit_msgs::RobotTrajectory>("trajectory_plan",1,doMsg);
    //ros::Subscriber sub = nh.subscribe<moveit_msgs::RobotTrajectory>("trajectory_getParam/trajectory_plan",1,doMsg);


    ros::spin();
    return 0;
}