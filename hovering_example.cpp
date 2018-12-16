//
// Created by yuanbao on 12/13/18.
//
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Vector3.h>

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Topic you want to publish
        trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        ROS_INFO("Started hovering.");

        std_srvs::Empty srv;
        bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        unsigned int i = 0;

        // Trying to unpause Gazebo for 10 seconds.
        while (i <= 10 && !unpaused) {
            ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            unpaused = ros::service::call("/gazebo/unpause_physics", srv);
            ++i;
        }

        if (!unpaused) {
            ROS_FATAL("Could not wake up Gazebo.");
        }
        else {
            ROS_INFO("Unpaused the Gazebo simulation.");
        }

        ros::Duration(5.0).sleep();

        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        trajectory_msg.header.stamp = ros::Time::now();
        trajectory_msg = trajectory_msg = setPosition(0.0, 1.0, 1.0, 0.0);
        trajectory_pub.publish(trajectory_msg);
        ros::Duration(5.0).sleep();
        ROS_INFO("start subscribing");
        //Topic you want to subscribe
        sub = nh.subscribe("/ball_0_coor", 100, &SubscribeAndPublish::callback, this);
    }

    trajectory_msgs::MultiDOFJointTrajectory setPosition(float x, float y, float z, double yaw) {
        Eigen::Vector3d desired_position(x, y, z);
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
        mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, yaw, &trajectory_msg);
        ROS_INFO("Publishing waypoint on namespace: [%f, %f, %f].",
                 desired_position.x(),
                 desired_position.y(),
                 desired_position.z());
        return trajectory_msg;

    }

    void callback(const geometry_msgs::Vector3 input)
    {
        ROS_INFO("running callback");
        float x = input.x;
        float y = input.y;
        float z = input.z;
        trajectory_msgs::MultiDOFJointTrajectory trajectory_msg = setPosition(x, y, z, 0.0);
        trajectory_pub.publish(trajectory_msg);
        ros::Duration(5.0).sleep();
    }



private:
    ros::NodeHandle nh;
    ros::Publisher trajectory_pub;
    ros::Subscriber sub;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "rec_pub_fly");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}

