// Copyright 2024 Team Reboot.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#include "op3_driver/op3_driver.hpp"

namespace op3_driver
{

OP3Driver::OP3Driver() {}

void OP3Driver::init(
  webots_ros2_driver::WebotsNode * node, std::unordered_map<std::string, std::string> & parameters)
{
  node_ = node;
  robot_ = std::make_shared<webots::Robot>();

  time_step_ = robot_->getBasicTimeStep();

  initialize_joint();

  joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  target_joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "target_joint_states", 10,
    std::bind(&OP3Driver::callback_target_joint_state, this, std::placeholders::_1));
}

void OP3Driver::initialize_joint()
{
  motor_joint_table_[0] = "r_hip_yaw";
  motor_joint_table_[1] = "r_hip_roll";
  motor_joint_table_[2] = "r_hip_pitch";
  motor_joint_table_[3] = "r_knee";
  motor_joint_table_[4] = "r_ank_pitch";
  motor_joint_table_[5] = "r_ank_roll";

  motor_joint_table_[6] = "l_hip_yaw";
  motor_joint_table_[7] = "l_hip_roll";
  motor_joint_table_[8] = "l_hip_pitch";
  motor_joint_table_[9] = "l_knee";
  motor_joint_table_[10] = "l_ank_pitch";
  motor_joint_table_[11] = "l_ank_roll";

  motor_joint_table_[12] = "r_sho_pitch";
  motor_joint_table_[13] = "l_sho_pitch";

  motor_joint_table_[14] = "r_sho_roll";
  motor_joint_table_[15] = "l_sho_roll";
  motor_joint_table_[16] = "r_el";
  motor_joint_table_[17] = "l_el";
  motor_joint_table_[18] = "head_pan";
  motor_joint_table_[19] = "head_tilt";

  for (std::size_t idx = 0; idx < motor_joint_table_.size(); idx++) {
    const std::string motor_joint_name = motor_joint_table_[idx];
    const std::string sensor_joint_name = motor_joint_name + "S";

    motor_[motor_joint_name] = std::make_shared<webots::Motor>(motor_joint_name);
    motor_[motor_joint_name]->setPosition(0.0);

    joint_sensor_[sensor_joint_name] = std::make_shared<webots::PositionSensor>(sensor_joint_name);
    joint_sensor_[sensor_joint_name]->enable(time_step_);
  }
}

void OP3Driver::step()
{
  sensor_msgs::msg::JointState joint_state_msgs;
  for (std::size_t idx = 0; idx < motor_joint_table_.size(); idx++) {
    const std::string motor_joint_name = motor_joint_table_[idx];
    const std::string sensor_joint_name = motor_joint_name + "S";
    joint_state_msgs.name.emplace_back(motor_joint_name);
    joint_state_msgs.position.emplace_back(joint_sensor_[sensor_joint_name]->getValue());
  }
  joint_state_msgs.header.stamp = node_->get_clock()->now();
  joint_state_publisher_->publish(joint_state_msgs);
}

void OP3Driver::write_all_motors(const sensor_msgs::msg::JointState::SharedPtr target_joint_state)
{
  for (std::size_t idx = 0; idx < target_joint_state->name.size(); idx++) {
    const std::string motor_joint_name = target_joint_state->name[idx];
    motor_[motor_joint_name]->setPosition(target_joint_state->position[idx]);
  }
}

void OP3Driver::write_motor(std::string joint_name, double joint_angle)
{
  motor_[joint_name]->setPosition(joint_angle);
}

void OP3Driver::callback_target_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  write_all_motors(msg);
}

}  // namespace op3_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(op3_driver::OP3Driver, webots_ros2_driver::PluginInterface)
