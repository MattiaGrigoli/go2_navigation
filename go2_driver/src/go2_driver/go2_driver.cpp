// BSD 3-Clause License

// Copyright (c) 2024, Intelligent Robotics Lab
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <go2_driver/go2_driver.hpp>

namespace go2_driver
{

Go2Driver::Go2Driver(
  const rclcpp::NodeOptions & options)
: Node("go2_driver", options),
  tf_broadcaster_(this)
{
  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", qos_profile);
  imu_pub_ = create_publisher<unitree_go::msg::IMUState>("imu", 10);
  request_pub_ = create_publisher<unitree_api::msg::Request>("api/sport/request", 10);

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/cloud", 10,
    std::bind(&Go2Driver::publish_lidar, this, std::placeholders::_1));

  robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/utlidar/robot_pose", 10,
    std::bind(&Go2Driver::publish_pose_stamped, this, std::placeholders::_1));

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&Go2Driver::joy_callback, this, std::placeholders::_1));

  low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
    "lowstate", 10,
    std::bind(&Go2Driver::publish_joint_states, this, std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&Go2Driver::cmd_vel_callback, this, std::placeholders::_1));

  set_body_height_service_ =
    this->create_service<go2_interfaces::srv::BodyHeight>(
    "body_height",
    std::bind(
      &Go2Driver::handleBodyHeight, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_continuous_gait_service_ =
    this->create_service<go2_interfaces::srv::ContinuousGait>(
    "continuous_gait",
    std::bind(
      &Go2Driver::handleContinuousGait, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_euler_service_ =
    this->create_service<go2_interfaces::srv::Euler>(
    "euler",
    std::bind(
      &Go2Driver::handleEuler, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_foot_raise_height_service_ =
    this->create_service<go2_interfaces::srv::FootRaiseHeight>(
    "foot_raise_height",
    std::bind(
      &Go2Driver::handleFootRaiseHeight, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_mode_service_ =
    this->create_service<go2_interfaces::srv::Mode>(
    "mode",
    std::bind(
      &Go2Driver::handleMode, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_pose_service_ =
    this->create_service<go2_interfaces::srv::Pose>(
    "pose",
    std::bind(
      &Go2Driver::handlePose, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_speed_level_service_ =
    this->create_service<go2_interfaces::srv::SpeedLevel>(
    "speed_level",
    std::bind(
      &Go2Driver::handleSpeedLevel, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_switch_gait_service_ =
    this->create_service<go2_interfaces::srv::SwitchGait>(
    "switch_gait",
    std::bind(
      &Go2Driver::handleSwitchGait, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  set_switch_joystick_service_ =
    this->create_service<go2_interfaces::srv::SwitchJoystick>(
    "switch_joystick",
    std::bind(
      &Go2Driver::handleSwitchJoystick, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void Go2Driver::publish_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  msg->header.stamp = now();
  msg->header.frame_id = "radar";
  pointcloud_pub_->publish(*msg);
}

void Go2Driver::publish_pose_stamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = msg->pose.position.x;
  transform.transform.translation.y = msg->pose.position.y;
  transform.transform.translation.z = msg->pose.position.z + 0.07;
  transform.transform.rotation.x = msg->pose.orientation.x;
  transform.transform.rotation.y = msg->pose.orientation.y;
  transform.transform.rotation.z = msg->pose.orientation.z;
  transform.transform.rotation.w = msg->pose.orientation.w;
  tf_broadcaster_.sendTransform(transform);

  if (!odom_published_) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = msg->pose.position.x;
    odom.pose.pose.position.y = msg->pose.position.y;
    odom.pose.pose.position.z = msg->pose.position.z + 0.07;
    odom.pose.pose.orientation.x = msg->pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.orientation.z;
    odom.pose.pose.orientation.w = msg->pose.orientation.w;
    odom_pub_->publish(odom);
    odom_published_ = true;
  }
}

void Go2Driver::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joy_state_ = *msg;
}

void Go2Driver::publish_joint_states(const unitree_go::msg::LowState::SharedPtr msg)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now();
  joint_state.name = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"};

  joint_state.position = {msg->motor_state[3].q, msg->motor_state[4].q, msg->motor_state[5].q,
    msg->motor_state[0].q, msg->motor_state[1].q, msg->motor_state[2].q,
    msg->motor_state[9].q, msg->motor_state[10].q, msg->motor_state[11].q,
    msg->motor_state[6].q, msg->motor_state[7].q, msg->motor_state[8].q};

  joint_state_pub_->publish(joint_state);
}

void Go2Driver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  nlohmann::json js;
  js["x"] = msg->linear.x;
  js["y"] = msg->linear.y;
  js["z"] = msg->angular.z;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Move);

  request_pub_->publish(req);
}

void Go2Driver::handleBodyHeight(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::BodyHeight::Request> request,
  const std::shared_ptr<go2_interfaces::srv::BodyHeight::Response> response)
{
  (void)request_header;

  if (request->height < -0.18 || request->height > 0.03) {
    response->success = false;
    response->message = "Height value is out of range [0.3 ~ 0.5]";
    return;
  }

  nlohmann::json js;
  js["data"] = request->height;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::BodyHeight);

  request_pub_->publish(req);
  response->success = true;
}

void Go2Driver::handleContinuousGait(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::ContinuousGait::Request> request,
  const std::shared_ptr<go2_interfaces::srv::ContinuousGait::Response> response)
{
  (void)request_header;

  nlohmann::json js;
  js["data"] = request->flag;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::ContinuousGait);

  request_pub_->publish(req);
  response->success = true;
}

void Go2Driver::handleEuler(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::Euler::Request> request,
  const std::shared_ptr<go2_interfaces::srv::Euler::Response> response)
{
  (void)request_header;

  nlohmann::json js;
  if (request->roll < -0.75 || request->roll > 0.75) {
    response->success = false;
    response->message = "Roll value is out of range [-0.75 ~ 0.75]";
    return;
  } else if (request->pitch < -0.75 || request->pitch > 0.75) {
    response->success = false;
    response->message = "Pitch value is out of range [-0.75 ~ 0.75]";
    return;
  } else if (request->yaw < -0.6 || request->yaw > 0.6) {
    response->success = false;
    response->message = "Yaw value is out of range [-1.5 ~ 1.5]";
    return;
  }

  js["x"] = request->roll;
  js["y"] = request->pitch;
  js["z"] = request->yaw;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Euler);

  request_pub_->publish(req);
  response->success = true;
}

void Go2Driver::handleFootRaiseHeight(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::FootRaiseHeight::Request> request,
  const std::shared_ptr<go2_interfaces::srv::FootRaiseHeight::Response> response)
{
  (void)request_header;

  if (request->height < 0 || request->height > 0.1) {
    response->success = false;
    response->message = "Height value is out of range [-0.06 ~ 0.03]";
    return;
  }

  nlohmann::json js;
  js["data"] = request->height;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FootRaiseHeight);

  request_pub_->publish(req);
  response->success = true;
}

void Go2Driver::handleMode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::Mode::Request> request,
  const std::shared_ptr<go2_interfaces::srv::Mode::Response> response)
{
  (void)request_header;
  std::string mode = request->mode;

  unitree_api::msg::Request req;

  if (mode == "damp") {
    response->message = "Change the mode to Damp";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Damp);
  } else if (mode == "balance_stand") {
    response->message = "Change the mode to BalanceStand";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::BalanceStand);
  } else if (mode == "stop_move") {
    response->message = "Change the mode to StopMove";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::StopMove);
  } else if (mode == "stand_up") {
    response->message = "Change the mode to StandUp";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::StandUp);
  } else if (mode == "stand_down") {
    response->message = "Change the mode to StandDown";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::StandDown);
  } else if (mode == "sit") {
    response->message = "Change the mode to Sit";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Sit);
  } else if (mode == "rise_sit") {
    response->message = "Change the mode to RiseSit";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::RiseSit);
  } else if (mode == "hello") {
    response->message = "Change the mode to Hello. Say hello to your robot!";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Hello);
  } else if (mode == "stretch") {
    response->message = "Change the mode to Stretch";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Stretch);
  } else if (mode == "wallow") {
    response->message = "Change the mode to Wallow";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Wallow);
  } else if (mode == "scrape") {
    response->message = "Change the mode to Scrape";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Scrape);
  } else if (mode == "front_flip") {
    response->message = "Front flip??? Really? You want to break your robot? Crazy!";
    // req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FrontFlip);
  } else if (mode == "front_jump") {
    response->message = "Change the mode to Front Jump";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FrontJump);
  } else if (mode == "front_pounce") {
    response->message = "Change the mode to Front Pounce";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FrontPounce);
  } else if (mode == "dance1") {
    response->message = "Change the mode to Dance 1. Let's dance!";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Dance1);
  } else if (mode == "dance2") {
    response->message = "Change the mode to Dance 2. Let's dance!";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Dance2);
  } else if (mode == "finger_heart") {
    response->message = "Change the mode to Finger Heart";
    req.header.identity.api_id = static_cast<int>(go2_driver::Mode::FingerHeart);
  } else {
    response->success = false;
    response->message = "Invalid mode";
    return;
  }

  request_pub_->publish(req);
  response->success = true;
}

void Go2Driver::handlePose(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::Pose::Request> request,
  const std::shared_ptr<go2_interfaces::srv::Pose::Response> response)
{
  (void)request_header;

  nlohmann::json js;
  js["data"] = request->flag;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::Pose);

  request_pub_->publish(req);
  response->success = true;
}

void Go2Driver::handleSpeedLevel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SpeedLevel::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SpeedLevel::Response> response)
{
  (void)request_header;

  if (request->level < -1 || request->level > 1) {
    response->success = false;
    response->message = "Speed level is out of range [-1 ~ 1]";
    return;
  }

  nlohmann::json js;
  js["data"] = request->level;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::SpeedLevel);

  request_pub_->publish(req);
  response->success = true;
}

void Go2Driver::handleSwitchGait(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SwitchGait::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SwitchGait::Response> response)
{
  (void)request_header;

  if (request->d < 0 || request->d > 4) {
    response->success = false;
    response->message = "Invalid gait type [0 - 4]";
    return;
  }

  nlohmann::json js;
  js["data"] = request->d;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::SwitchGait);

  request_pub_->publish(req);
  response->success = true;
}

void Go2Driver::handleSwitchJoystick(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Request> request,
  const std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Response> response)
{
  (void)request_header;

  nlohmann::json js;
  js["data"] = request->flag;

  unitree_api::msg::Request req;
  req.parameter = js.dump();
  req.header.identity.api_id = static_cast<int>(go2_driver::Mode::SwitchJoystick);

  request_pub_->publish(req);
  response->success = true;
}

}  // namespace go2_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(go2_driver::Go2Driver)
