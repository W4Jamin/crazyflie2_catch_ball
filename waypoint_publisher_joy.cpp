/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/DeleteModel.h>
#include <ros/topic.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>

const float DEG_2_RAD = M_PI / 180.0;

struct Commands {
  int seq;
  double x;
  double y;
  double z;
  double yaw;
};

class WaypointJoy {

private:
  ros::NodeHandle nh_;
  ros::Publisher trajectory_pub_;
  ros::Subscriber joy_sub_;
  ros::Timer timer_;
  Eigen::Vector3d desired_position_;
  double desired_yaw_ = 0.0;
  Commands cmds_ = { 0, 0, 0, 0, 0 };
  Commands last_cmds_;
  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void TimerCallback(const ros::TimerEvent& event);

public:
  WaypointJoy();
  void Start();
};

WaypointJoy::WaypointJoy() {
  trajectory_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
    mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
}

void WaypointJoy::Start() {
  while (trajectory_pub_.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO("There is no subscriber available, trying again in 1 second.");
    ros::Duration(1.0).sleep();
  }
  
  nav_msgs::OdometryConstPtr odometry_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(mav_msgs::default_topics::ODOMETRY, ros::Duration(30.0));
  geometry_msgs::Point position = odometry_msg->pose.pose.position;

  desired_position_ = Eigen::Vector3d(position.x, position.y, position.z);
  desired_yaw_ = 0.0;
  ROS_INFO("Initial position: [%f, %f, %f].",
           desired_position_.x(),
           desired_position_.y(),
           desired_position_.z());

  last_cmds_ = cmds_;

  joy_sub_ = nh_.subscribe("joy", 10, &WaypointJoy::JoyCallback, this);
  timer_ = nh_.createTimer(ros::Rate(30.0).expectedCycleTime(), &WaypointJoy::TimerCallback, this);
}

void WaypointJoy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  cmds_ = (Commands){ cmds_.seq + 1,
    -msg->axes[0] * 0.01,
    msg->axes[1] * 0.01,
    msg->axes[4] * 0.01,
    msg->axes[3] * DEG_2_RAD * 180 };
  ROS_DEBUG("cmds_ %d %lf %lf %lf %lf", cmds_.seq, cmds_.x, cmds_.y, cmds_.z, cmds_.yaw);
}

void WaypointJoy::TimerCallback(const ros::TimerEvent& event) {
  if (cmds_.seq > last_cmds_.seq) {
    last_cmds_ = cmds_;
  }

  // if (Eigen::Vector3d(last_cmds_.x, last_cmds_.y, last_cmds_.z).norm() < 0.00001) {
  //   // ROS_INFO("No commands");
  //   return;
  // }
  // ROS_INFO("Cmds norm: %f", Eigen::Vector3d(last_cmds_.x, last_cmds_.y, last_cmds_.z).norm());

  desired_position_.x() += last_cmds_.x;
  desired_position_.y() += last_cmds_.y;
  desired_position_.z() += last_cmds_.z;
  
  // desired_yaw_ = last_cmds_.yaw;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position_,
      desired_yaw_, &trajectory_msg);

  ROS_DEBUG("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh_.getNamespace().c_str(),
           desired_position_.x(),
           desired_position_.y(),
           desired_position_.z());
  trajectory_pub_.publish(trajectory_msg);
}

void spawn_mav(const std::string &mav_name, const std::string &model_name, double x, double y, double z) {
  char buffer[1001];
  // snprintf(buffer, 1000, "env ROS_NAMESPACE=%s roslaunch rotors_gazebo spawn_mav_with_state_publishers.launch\
  //   x:=%f\
  //   y:=%f\
  //   z:=%f\
  //   mav_name:=%s\
  //   model_name:=%s",
  //   ros::this_node::getNamespace().c_str(), x, y, z, mav_name.c_str(), model_name.c_str());
  snprintf(buffer, 1000, "env ROS_NAMESPACE=%s rosrun gazebo_ros spawn_model\
    -param robot_description\
    -urdf\
    -x %f\
    -y %f\
    -z %f\
    -model %s",
    ros::this_node::getNamespace().c_str(), x, y, z, model_name.c_str());
  system(buffer);
}

void spawn_mav(const std::string &mav_name, const std::string &model_name) {
  spawn_mav(mav_name, model_name, 0.0, 0.0, 0.1);
}

bool despawn_mav(const std::string &mav_name) {
  gazebo_msgs::DeleteModelRequest req;
  gazebo_msgs::DeleteModelResponse res;
  req.model_name = mav_name;
  bool result = ros::service::call("/gazebo/delete_model", req, res);
  ROS_INFO("delete model %d %s", res.success, res.status_message.c_str());
  return res.success;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "waypoint_publisher_joy");

  ROS_INFO("Started waypoint_publisher_joy.");

  WaypointJoy waypoint_joy;

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

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
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  std::string mav_name;
  std::string model_name;
  ros::param::get("~mav_name", mav_name);
  ros::param::get("~model_name", model_name);
  ROS_INFO("Spawning mav_name=%s, model_name=%s", mav_name.c_str(), model_name.c_str());

  if (!despawn_mav(model_name)) {
    ROS_INFO("Cannot despawn mav %s", mav_name.c_str());
  }

  double x, y, z;
  ros::param::get("~spawn_x", x);
  ros::param::get("~spawn_y", y);
  ros::param::get("~spawn_z", z);

  spawn_mav(mav_name, model_name, x, y, z);

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  waypoint_joy.Start();

  ros::spin();

  return 0;
}
