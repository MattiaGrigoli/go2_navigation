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

#ifndef GO2_DRIVER__GO2_DRIVER_HPP_
#define GO2_DRIVER__GO2_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nlohmann/json.hpp"
#include "unitree_api/msg/request.hpp"
#include "go2_driver/go2_api_id.hpp"
#include "go2_interfaces/srv/body_height.hpp"
#include "go2_interfaces/srv/continuous_gait.hpp"
#include "go2_interfaces/srv/euler.hpp"
#include "go2_interfaces/srv/foot_raise_height.hpp"
#include "go2_interfaces/srv/mode.hpp"
#include "go2_interfaces/srv/pose.hpp"
#include "go2_interfaces/srv/speed_level.hpp"
#include "go2_interfaces/srv/switch_gait.hpp"
#include "go2_interfaces/srv/switch_joystick.hpp"

namespace go2_driver
{

class Go2Driver : public rclcpp::Node
{
public:
  Go2Driver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void publish_lidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_pose_stamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publish_joint_states(const unitree_go::msg::LowState::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void handleBodyHeight(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::BodyHeight::Request> request,
    const std::shared_ptr<go2_interfaces::srv::BodyHeight::Response> response);

  void handleContinuousGait(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::ContinuousGait::Request> request,
    const std::shared_ptr<go2_interfaces::srv::ContinuousGait::Response> response);

  void handleEuler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::Euler::Request> request,
    const std::shared_ptr<go2_interfaces::srv::Euler::Response> response);

  void handleFootRaiseHeight(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::FootRaiseHeight::Request> request,
    const std::shared_ptr<go2_interfaces::srv::FootRaiseHeight::Response> response);

  void handleMode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::Mode::Request> request,
    const std::shared_ptr<go2_interfaces::srv::Mode::Response> response);

  void handlePose(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::Pose::Request> request,
    const std::shared_ptr<go2_interfaces::srv::Pose::Response> response);

  void handleSpeedLevel(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SpeedLevel::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SpeedLevel::Response> response);

  void handleSwitchGait(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SwitchGait::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SwitchGait::Response> response);

  void handleSwitchJoystick(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Request> request,
    const std::shared_ptr<go2_interfaces::srv::SwitchJoystick::Response> response);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<unitree_go::msg::IMUState>::SharedPtr imu_pub_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;

  rclcpp::Service<go2_interfaces::srv::BodyHeight>::SharedPtr set_body_height_service_;
  rclcpp::Service<go2_interfaces::srv::ContinuousGait>::SharedPtr set_continuous_gait_service_;
  rclcpp::Service<go2_interfaces::srv::Euler>::SharedPtr set_euler_service_;
  rclcpp::Service<go2_interfaces::srv::FootRaiseHeight>::SharedPtr set_foot_raise_height_service_;
  rclcpp::Service<go2_interfaces::srv::Mode>::SharedPtr set_mode_service_;
  rclcpp::Service<go2_interfaces::srv::Pose>::SharedPtr set_pose_service_;
  rclcpp::Service<go2_interfaces::srv::SpeedLevel>::SharedPtr set_speed_level_service_;
  rclcpp::Service<go2_interfaces::srv::SwitchGait>::SharedPtr set_switch_gait_service_;
  rclcpp::Service<go2_interfaces::srv::SwitchJoystick>::SharedPtr set_switch_joystick_service_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_lidar_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  sensor_msgs::msg::Joy joy_state_;

  bool odom_published_{false};
};

}  // namespace go2_driver

#endif  // GO2_DRIVER__GO2_DRIVER_HPP_
