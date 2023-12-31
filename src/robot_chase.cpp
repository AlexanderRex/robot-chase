#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class RobotChase : public rclcpp::Node {
public:
  RobotChase() : Node("robot_chase"), min_stop_distance_(0.4) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RobotChase::calculate_and_publish_velocity, this));
  }

private:
  void calculate_and_publish_velocity() {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
      transformStamped = tf_buffer_->lookupTransform(
          "rick/base_link", "morty/base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    double error_distance =
        std::sqrt(std::pow(transformStamped.transform.translation.x, 2) +
                  std::pow(transformStamped.transform.translation.y, 2) +
                  std::pow(transformStamped.transform.translation.z, 2));

    if (error_distance <= min_stop_distance_) {
      geometry_msgs::msg::Twist twist;
      twist.angular.z = 0.0;
      twist.linear.x = 0.0;
      publisher_->publish(twist);
      RCLCPP_INFO(this->get_logger(),
                  "Rick is close enough to Morty, stopping.");
      return;
    }

    tf2::Quaternion q(transformStamped.transform.rotation.x,
                      transformStamped.transform.rotation.y,
                      transformStamped.transform.rotation.z,
                      transformStamped.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double kp_yaw = 1.0;
    double kp_distance = 1.0;

    double angular_velocity = std::min(kp_yaw * yaw, max_angular_velocity_);
    double linear_velocity =
        std::min(kp_distance * error_distance, max_linear_velocity_);

    geometry_msgs::msg::Twist twist;
    twist.angular.z = angular_velocity;
    twist.linear.x = linear_velocity;
    publisher_->publish(twist);

    RCLCPP_INFO(this->get_logger(),
                "Published Twist with linear.x: %f, angular.z: %f",
                linear_velocity, angular_velocity);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  double min_stop_distance_;
  const double max_linear_velocity_ = 1.0;
  const double max_angular_velocity_ = 1.0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}