#ifndef _OP3_WEBOTS__OP3_DRIVER_HPP_
#define _OP3_WEBOTS__OP3_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace op3_driver
{
class OP3Driver : public webots_ros2_driver::PluginInterface
{
public:
  OP3Driver();
  ~OP3Driver() = default;

  void init(
    webots_ros2_driver::WebotsNode * node,
    std::unordered_map<std::string, std::string> & parameters) override;
  void step() override;

  void initialize_joint();

  void write_all_motors(const sensor_msgs::msg::JointState::SharedPtr target_joint_state);
  void write_motor(std::string joint_name, double joint_angle);
  void read_motor();

  void callback_target_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_subscriber_;

  std::shared_ptr<webots::Robot> robot_;

  webots_ros2_driver::WebotsNode * node_;

  double time_step_;

  std::map<int, std::string> motor_joint_table_;
  std::map<std::string, std::shared_ptr<webots::Motor>> motor_;
  std::map<std::string, std::shared_ptr<webots::PositionSensor>> joint_sensor_;
};

}  // namespace op3_driver

#endif
