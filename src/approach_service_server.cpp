#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rmw/types.h"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <vector>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <future>
#include <inttypes.h>
#include <iostream>
#include <memory>
#include <sstream>

using GoToLoadingMsg = attach_shelf::srv::GoToLoading;

class StaticTfPublisherNode : public rclcpp::Node {
public:
  float dist;

  StaticTfPublisherNode() : Node("static_tf_publisher") {
    static_transform_.header.frame_id = "cart_frame";
    static_transform_.child_frame_id = "scan";

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

private:
  void create_timer() {

    timer_ = this->create_wall_timer(
        500ms, std::bind(&StaticTfPublisherNode::publish_frame, this));
  }
  void publish_frame() {
    RCLCPP_INFO_ONCE(this->get_logger(), "ON");

    if (!initialized_) {
      // If this is the first scan message, save the initial position of scan
      initial_scan_pose_.position.x = scan_msg->ranges[0] + X;
      initial_scan_pose_.position.y = 0.0;
      initial_scan_pose_.position.z = 0.0;
      initial_scan_pose_.orientation.x = 0.0;
      initial_scan_pose_.orientation.y = 0.0;
      initial_scan_pose_.orientation.z = 0.0;
      initial_scan_pose_.orientation.w = 1.0;
      initialized_ = true;
    }

    // Update the position of static_transform_ based on the current
    // position of scan
    static_transform_.header.stamp = this->now();
    static_transform_.transform.translation.x =
        initial_scan_pose_.position.x - dist;
    static_transform_.transform.translation.y = initial_scan_pose_.position.y;
    static_transform_.transform.translation.z = initial_scan_pose_.position.z;
    static_transform_.transform.rotation.x = initial_scan_pose_.orientation.x;
    static_transform_.transform.rotation.y = initial_scan_pose_.orientation.y;
    static_transform_.transform.rotation.z = initial_scan_pose_.orientation.z;
    static_transform_.transform.rotation.w = initial_scan_pose_.orientation.w;

    // Publish the static transform
    broadcaster_->sendTransform(static_transform_);
  }

  rclcpp::TimerBase::SharedPtr timer_move;
  bool initialized_{false};
  geometry_msgs::msg::Pose initial_scan_pose_;
  tf2_ros::StaticTransformBroadcaster::SharedPtr broadcaster_;
  geometry_msgs::msg::TransformStamped static_transform_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  double X{0.2}; // Set the distance to 0.2 meters in front of the initial
                 // position of scan
};

class AppService : public rclcpp::Node {

public:
  std::shared_ptr<StaticTfPublisherNode> staticPublisher;

  AppService() : Node("services_quiz_node") {

    this->laser_range = {0};

    // Callback Group
    callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // LaserScan G1
    laser_callback_G1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // LaserScan G1
    laser_callback_G2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = laser_callback_G1;

    rclcpp::SubscriptionOptions options2;
    options2.callback_group = laser_callback_G2;

    // Subscribers
    // sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "odom", 10,
    //     std::bind(&AppService::odom_callback, this, std::placeholders::_1),
    //     options2);

    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&AppService::scan_callback, this, std::placeholders::_1),
        options1);

    // Server
    srv_ = this->create_service<GoToLoadingMsg>(
        "approach_shelf",
        std::bind(&AppService::service_callback, this, std::placeholders::_1,
                  std::placeholders::_2),
        ::rmw_qos_profile_default, callback_group_);

    // Publisher
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "GoToLoadingMsg Server is READY!");
  }

private:
  std::vector<float> intensities_above_threshold;
  bool start = true;
  int changes = 0;
  std::vector<int> index_intensities;
  std::vector<float> ranges;
  float angle_min;
  float angle_increment;
  bool read_intensity = false;
  rclcpp::Service<GoToLoadingMsg>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr laser_callback_G1;
  rclcpp::CallbackGroup::SharedPtr laser_callback_G2;

  geometry_msgs::msg::Twist velocity;

  float min_left_laser;
  float min_right_laser;
  float min_front_laser;

  std::vector<float> laser_range;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    if (read_intensity) {

      intensities_above_threshold = msg->intensities;
      ranges = msg->ranges;
      angle_min = msg->angle_min;
      angle_increment = msg->angle_increment;
    }
  }
  void
  service_callback(const std::shared_ptr<GoToLoadingMsg::Request> request,
                   const std::shared_ptr<GoToLoadingMsg::Response> response) {

    RCLCPP_INFO(this->get_logger(), "Server has been called!");

    read_intensity = true;
    sleep(3);
    read_intensity = false;

    for (size_t i = 0; i < intensities_above_threshold.size(); ++i) {

      if (intensities_above_threshold[i] !=
          intensities_above_threshold[i - 1]) {

        changes += 1;
        if (changes % 2 == 0) // par
        {
          index_intensities.push_back(i - 1);
        } else {
          index_intensities.push_back(i);
        }
      }
    }
    if (changes <= 3) {

      RCLCPP_INFO(this->get_logger(), "One or none Leg!");
      response->complete = false;
    }
    if (changes == 4) {

      int index_d1 = (index_intensities[1] + index_intensities[2]) / 2;
      int index_d2 = (index_intensities[3] + index_intensities[4]) / 2;

      float d1 = ranges[index_d1];
      float d2 = ranges[index_d2];

      float distance = (d1 + d2) / 2;

      float angle1 = angle_min + index_d1 * angle_increment;
      float angle2 = angle_min + index_d2 * angle_increment;
      float theta = (angle1 + angle2) / 2;

      float x = distance * cos(theta);
      float y = distance * sin(theta);

      staticPublisher->create_timer();
      // Alcanzarlo mediante el frame Checkpoint 4
      //  Avanzar 30cm mas para terminar en el centro

      // Publish cart frame &&  the robot will use this TF to move towards the
      // shelf (using the transform coordinates). It will move forward 30 cm
      // more (to end up right underneath the shelf). Send true
      response->complete = true;
    } else {

      response->complete = false;
    }
  }
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto app_server = std::make_shared<AppService>();
  auto StaticPublisher = std::make_shared<StaticTfPublisherNode>();

  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(app_server);
  executor.add_node(StaticPublisher);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
