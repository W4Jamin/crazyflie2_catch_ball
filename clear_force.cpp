//
// Created by yuanbao on 11/19/18.
//

#include <ros/ros.h>
#include "gazebo_msgs/ApplyBodyWrench.h"
#include "std_msgs/Duration.h"

using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc,argv,"cancel_force0_node");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    ros::Duration dur;
    dur = ros::Duration(1000000000);

    geometry_msgs::Vector3 vec3;
    vec3.x = 0;
    vec3.y = 0;
    vec3.z = 0;

    geometry_msgs::Wrench wrench_msg;
    wrench_msg.force = vec3;

    gazebo_msgs::ApplyBodyWrench srv;
    srv.request.body_name = "ball_0::ball_0::link";
    srv.request.wrench = wrench_msg;
    srv.request.duration = dur;

    if(client.call(srv))
    {
        ROS_INFO("Cancel force 0 success!!");
    }
    else
    {
        ROS_ERROR("Failed to throw the ball! Error msg:%s",srv.response.status_message.c_str());
    }
    return 0;
}

