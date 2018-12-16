#include <ros/ros.h>
#include "gazebo_msgs/ApplyBodyWrench.h"
#include "std_msgs/Duration.h"
#include <time.h>
#include <cstdlib>

using namespace std;

float generate_rand(int a, int b) {
    srand((unsigned)time(NULL)+2);
    return ((rand()%(b-a+1))+a)/100.0;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"throw_ball2_node");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    ros::Duration dur;
    dur = ros::Duration(1000000000);


    geometry_msgs::Vector3 vec3;
    vec3.x = generate_rand(-100, -50);
    vec3.y = generate_rand(-20, 20);
    vec3.z = 0;

    geometry_msgs::Wrench wrench_msg;
    wrench_msg.force = vec3;

    gazebo_msgs::ApplyBodyWrench srv;
    srv.request.body_name = "ball_2::ball_2::link";
    srv.request.wrench = wrench_msg;
    srv.request.duration = dur;

    if(client.call(srv))
    {
        ROS_INFO("Throw ball 2 success!!");
    }
    else
    {
        ROS_ERROR("Failed to throw the ball! Error msg:%s",srv.response.status_message.c_str());
    }
    return 0;
}