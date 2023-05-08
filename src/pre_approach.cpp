#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Topics : public rclcpp::Node {
public:
  Topics() : Node("pre_approach_node") {

    this->declare_parameter("obstacle", 0.5);
    this->declare_parameter("degrees", 45);

    getting_params();

    // RCLCPP_INFO(this->get_logger(), "Degrees: %f", degrees);

    degrees = (degrees * 3.1416) / 180;

    tiempo = degrees / 0.5;
    // RCLCPP_INFO(this->get_logger(), "Tiempo 1 : %fs", tiempo);

    tiempo += 1;

    // RCLCPP_INFO(this->get_logger(), "Tiempo 2 :  %fs", tiempo);

    // degrees = (degrees * 3.1416) / 180;

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&Topics::scan_callback, this, _1));

    timer_ = this->create_wall_timer(100ms,
                                     std::bind(&Topics::timer_callback, this));
    this->laser_left = 0.0;
    this->laser_right = 0.0;
    this->laser_forward = 0.0;
    RCLCPP_INFO(this->get_logger(), "Topics approach READY");
  }

private:
  geometry_msgs::msg::Twist twist;
  float obstacle;
  float degrees;
  bool obs = false;
  bool ignore = false;
  float tiempo;
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->laser_left = msg->ranges[180];
    this->laser_right = msg->ranges[540];
    this->laser_forward = msg->ranges[360];
    // RCLCPP_INFO(this->get_logger(), "[LEFT] = '%f'", this->laser_left);
    // RCLCPP_INFO(this->get_logger(), "[RIGHT] = '%f'", this->laser_right);
    // RCLCPP_INFO(this->get_logger(), "[FORWARD] = '%f'", this->laser_forward);
  }

  void getting_params() {
    obstacle =
        this->get_parameter("obstacle").get_parameter_value().get<float>();
    degrees = this->get_parameter("degrees").get_parameter_value().get<int>();
  }

  void timer_callback() {

    if (!ignore) {

      if (this->laser_forward > obstacle && obs == false) {

        this->twist_msg.angular.z = 0.0;
        this->twist_msg.linear.x = 0.3;
        publisher_->publish(this->twist_msg);

      } else {
        obs = true;
        // Stop
        this->twist_msg.angular.z = 0.0;
        this->twist_msg.linear.x = 0.0;
        publisher_->publish(this->twist_msg);
        ignore = true;
      }
    }

    if (obs) {

      if (degrees > 0.0) {
        this->twist_msg.angular.z = -0.5;
        publisher_->publish(this->twist_msg);
        RCLCPP_INFO(this->get_logger(), ">0");
      } else {
        this->twist_msg.angular.z = 0.5;
        publisher_->publish(this->twist_msg);
        RCLCPP_INFO(this->get_logger(), "<0");
      }

      // Esperar el tiempo calculado antes de detener el robot
      std::this_thread::sleep_for(
          std::chrono::microseconds(int(tiempo * 1000 * 1000)));
      RCLCPP_INFO(this->get_logger(), "STOP");

      this->twist_msg.angular.z = 0.0;
      publisher_->publish(this->twist_msg);
      timer_->cancel();
    }

    publisher_->publish(this->twist_msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  geometry_msgs::msg::Twist twist_msg;
  float laser_left;
  float laser_right;
  float laser_forward;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Topics>());
  rclcpp::shutdown();
  return 0;
}