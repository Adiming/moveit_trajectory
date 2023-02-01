// publish desired position to moveit
#include "ros/ros.h"
// #include "trajectory_follow/position.h"
#include "trajectory_follow/desired_position.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "position_publlisher");

    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<trajectory_follow::desired_position>("desired_position");

    ros::service::waitForService("desired_position");

    trajectory_follow::desired_position pos;

    pos.request.targetpose.position.x = atof(argv[1]);
    pos.request.targetpose.position.y = atof(argv[2]);
    pos.request.targetpose.position.z = atof(argv[3]);

    // pos.request.targetpose.orientation.x = atof(argv[4]);
    // pos.request.targetpose.orientation.y = atof(argv[5]);
    // pos.request.targetpose.orientation.z = atof(argv[6]);
    pos.request.targetpose.orientation.w = atof(argv[4]);

    bool flag = client.call(pos);

    if(flag)
    {
        ROS_INFO("Request is succeeded");
        std::vector<int>::size_type size1 = pos.response.trajectory.joint_trajectory.points.size();  
        ROS_INFO("%d", size1);
        for (unsigned i=0; i<size1; i++)  
        {  
            ROS_INFO("point %d:joint1: %.3f,joint2: %.3f,joint3: %.3f,joint4: %.3f,joint5: %.3f,joint6: %.3f,joint7: %.3f",
            i, pos.response.trajectory.joint_trajectory.points[i].positions[0],
            pos.response.trajectory.joint_trajectory.points[i].positions[1],
            pos.response.trajectory.joint_trajectory.points[i].positions[2],
            pos.response.trajectory.joint_trajectory.points[i].positions[3],
            pos.response.trajectory.joint_trajectory.points[i].positions[4],
            pos.response.trajectory.joint_trajectory.points[i].positions[5],
            pos.response.trajectory.joint_trajectory.points[i].positions[6]);
            //pos.response.trajectory.joint_trajectory.points[i].velocities[0]
        }
    }
    else
    {
        ROS_ERROR("Request is failed");
        return 1;
    }
    

    return 0;
}